#include "slam/visual_odometry.h"
#include <opencv2/features2d.hpp>
#include <opencv2/calib3d.hpp>

namespace slam {

VisualOdometry::VisualOdometry() {
    // Inizializza il rilevatore di feature ORB
    detector_ = cv::ORB::create(
        1000,                           // numero massimo di feature
        1.2f,                          // fattore di scala
        8,                              // numero di livelli della piramide
        31,                             // dimensione del bordo
        0,                              // primo livello della piramide
        2,                              // WTA_K
        cv::ORB::HARRIS_SCORE,          // punteggio per la selezione
        31,                             // dimensione del patch
        20                              // fast threshold
    );
    
    // Inizializza il matcher di feature
    matcher_ = cv::DescriptorMatcher::create("BruteForce-Hamming");
    
    // Parametri di default per la telecamera (da calibrare)
    K_ = (cv::Mat_<double>(3, 3) << 
        500, 0, 320,
        0, 500, 240,
        0, 0, 1);
    dist_coeffs_ = cv::Mat::zeros(4, 1, CV_64F);
}

void VisualOdometry::setCameraParameters(const cv::Mat& K, const cv::Mat& dist_coeffs) {
    K.copyTo(K_);
    dist_coeffs.copyTo(dist_coeffs_);
}

VisualOdometryData VisualOdometry::processFrame(const cv::Mat& frame, uint64_t timestamp) {
    VisualOdometryData result;
    result.timestamp = timestamp;
    
    // Converti in scala di grigi se necessario
    cv::Mat gray;
    if (frame.channels() == 3) {
        cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    } else {
        gray = frame;
    }
    
    // Rileva e traccia le feature
    std::vector<cv::Point2f> curr_keypoints = detectAndTrack(gray);
    
    if (prev_keypoints_.empty() || curr_keypoints.size() < 10) {
        // Prima frame o troppo poche feature, inizializza
        prev_frame_ = gray.clone();
        prev_keypoints_ = curr_keypoints;
        // Converti i punti in KeyPoint per il calcolo dei descrittori
        std::vector<cv::KeyPoint> keypoints;
        for (const auto& pt : prev_keypoints_) {
            keypoints.emplace_back(pt, 1.0f);
        }
        detector_->compute(prev_frame_, keypoints, prev_descriptors_);
        return result;
    }
    
    // Stima il movimento
    cv::Mat R, t;
    if (estimateMotion(prev_keypoints_, curr_keypoints, R, t)) {
        // Conversione in formato di ritorno
        result.translation = Eigen::Vector3d(t.at<double>(0,0), t.at<double>(1,0), t.at<double>(2,0));
        
        // Converte la matrice di rotazione in quaternione
        Eigen::Matrix3d R_eigen;
        R_eigen << R.at<double>(0,0), R.at<double>(0,1), R.at<double>(0,2),
                  R.at<double>(1,0), R.at<double>(1,1), R.at<double>(1,2),
                  R.at<double>(2,0), R.at<double>(2,1), R.at<double>(2,2);
        result.rotation = Eigen::Quaterniond(R_eigen);
        
        // Calcola la confidenza in base al numero di inlier
        result.confidence = static_cast<double>(curr_keypoints.size()) / prev_keypoints_.size();
    }
    
    // Aggiorna lo stato per il prossimo frame
    prev_frame_ = gray.clone();
    prev_keypoints_ = curr_keypoints;
    
    // Converti i punti in KeyPoint per il calcolo dei descrittori
    if (!prev_keypoints_.empty()) {
        std::vector<cv::KeyPoint> keypoints;
        for (const auto& pt : prev_keypoints_) {
            keypoints.emplace_back(pt, 1.0f);
        }
        detector_->compute(prev_frame_, keypoints, prev_descriptors_);
    }
    
    return result;
}

std::vector<cv::Point2f> VisualOdometry::detectAndTrack(const cv::Mat& frame) {
    std::vector<cv::Point2f> result;
    
    if (prev_frame_.empty()) {
        // Prima frame, rileva nuove feature
        std::vector<cv::KeyPoint> keypoints;
        detector_->detect(frame, keypoints);
        
        // Converti in Point2f
        cv::KeyPoint::convert(keypoints, result);
    } else {
        // Traccia le feature dal frame precedente
        std::vector<uchar> status;
        std::vector<float> err;
        cv::Size win_size(21, 21);  // Dimensione della finestra di ricerca
        int max_level = 3;          // Livelli della piramide
        
        cv::calcOpticalFlowPyrLK(
            prev_frame_, frame,
            prev_keypoints_, result,
            status, err, win_size, max_level,
            cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01)
        );
        
        // Filtra i punti tracciati con successo
        std::vector<cv::Point2f> good_points;
        for (size_t i = 0; i < status.size(); i++) {
            if (status[i] && err[i] < 20.0) {  // Soglia di errore
                good_points.push_back(result[i]);
            }
        }
        
        result = good_points;
    }
    
    return result;
}

bool VisualOdometry::estimateMotion(
    const std::vector<cv::Point2f>& prev_pts,
    const std::vector<cv::Point2f>& curr_pts,
    cv::Mat& R, cv::Mat& t) {
    
    if (prev_pts.size() < 8 || curr_pts.size() < 8) {
        return false;
    }
    
    // Calcola la matrice essenziale
    cv::Mat E, mask;
    E = cv::findEssentialMat(
        curr_pts, prev_pts, K_,
        cv::RANSAC, 0.999, 1.0, mask);
    
    if (E.empty() || E.rows != 3 || E.cols != 3) {
        return false;
    }
    
    // Recupera la rotazione e la traslazione
    cv::recoverPose(E, curr_pts, prev_pts, K_, R, t, mask);
    
    return true;
}

} // namespace slam
