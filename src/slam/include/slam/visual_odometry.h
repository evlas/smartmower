#pragma once

#include <opencv2/opencv.hpp>
#include "slam/types.h"

namespace slam {

/**
 * @brief Classe per il calcolo dell'odometria visiva
 */
class VisualOdometry {
public:
    VisualOdometry();
    ~VisualOdometry() = default;

    /**
     * @brief Elabora un nuovo frame e restituisce il movimento relativo
     */
    VisualOdometryData processFrame(const cv::Mat& frame, uint64_t timestamp);

    /**
     * @brief Imposta i parametri della telecamera
     */
    void setCameraParameters(const cv::Mat& K, const cv::Mat& dist_coeffs);

private:
    // Parametri della telecamera
    cv::Mat K_;                 // Matrice di calibrazione
    cv::Mat dist_coeffs_;       // Coefficienti di distorsione
    
    // Stato interno
    cv::Mat prev_frame_;
    std::vector<cv::Point2f> prev_keypoints_;
    cv::Mat prev_descriptors_;
    
    // Rilevatore e descrittore di feature
    cv::Ptr<cv::Feature2D> detector_;
    cv::Ptr<cv::DescriptorMatcher> matcher_;
    
    // Metodi ausiliari
    std::vector<cv::Point2f> detectAndTrack(const cv::Mat& frame);
    bool estimateMotion(const std::vector<cv::Point2f>& prev_pts, 
                       const std::vector<cv::Point2f>& curr_pts,
                       cv::Mat& R, cv::Mat& t);
};

} // namespace slam
