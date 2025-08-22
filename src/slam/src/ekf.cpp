#include "slam/ekf.h"
#include <iostream>
#include <cmath>

namespace slam {

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    // Inizializza le matrici di covarianza
    constexpr int state_size = 16;  // 3 pos + 3 vel + 4 quat + 3 gb + 3 ab
    P_ = Eigen::MatrixXd::Identity(state_size, state_size) * 0.1;
    
    // Inizializza la matrice di rumore di processo
    Q_ = Eigen::MatrixXd::Identity(state_size, state_size) * 1e-4;
    
    // Inizializza la matrice di rumore delle misure GPS
    R_gps_ = Eigen::Matrix3d::Identity() * 0.1;
    
    // Inizializza la matrice di rumore dell'odometria visiva
    R_vo_ = Eigen::Matrix<double, 6, 6>::Identity() * 0.05;
}

void ExtendedKalmanFilter::updateMagnetometer(double yaw_heading, double weight) {
    // Vincola weight in [0,1]
    weight = std::max(0.0, std::min(1.0, weight));
    if (weight == 0.0) return;

    // Estrai roll, pitch, yaw correnti dall'orientamento
    const Eigen::Matrix3d R = state_.orientation.toRotationMatrix();
    double roll = std::atan2(R(2,1), R(2,2));
    double pitch = std::asin(-R(2,0));
    double yaw = std::atan2(R(1,0), R(0,0));

    // Normalizza differenza angolare in [-pi, pi]
    auto wrapPi = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    double dyaw = wrapPi(yaw_heading - yaw);

    // Correzione morbida
    double yaw_new = wrapPi(yaw + weight * dyaw);

    // Ricostruisci quaternion mantenendo roll/pitch
    Eigen::AngleAxisd Rx(roll, Eigen::Vector3d::UnitX());
    Eigen::AngleAxisd Ry(pitch, Eigen::Vector3d::UnitY());
    Eigen::AngleAxisd Rz(yaw_new, Eigen::Vector3d::UnitZ());
    state_.orientation = (Rx * Ry * Rz).toRotationMatrix();
    state_.orientation.normalize();

    // Aggiorna (in modo semplice) la covarianza per riflettere riduzione in yaw
    // Nota: senza una mappatura esplicita degli indici, applichiamo una riduzione globale
    P_ *= (1.0 - 0.05 * weight);
}

void ExtendedKalmanFilter::initialize(const State& initial_state) {
    state_ = initial_state;
    std::cout << "EKF initialized with position: " 
              << state_.position.transpose() << std::endl;
}

void ExtendedKalmanFilter::predict(const IMUData& imu, double dt) {
    if (dt <= 0.0) return;
    
    // Estrai lo stato corrente
    Eigen::Vector3d p = state_.position;
    Eigen::Vector3d v = state_.velocity;
    Eigen::Quaterniond q = state_.orientation;
    
    // Rimuovi i bias dalle misure IMU
    Eigen::Vector3d acc = imu.accel - state_.bias_accel;
    Eigen::Vector3d gyro = imu.gyro - state_.bias_gyro;
    
    // Calcola la matrice di rotazione dal frame body a world
    Eigen::Matrix3d R = q.toRotationMatrix();
    
    // Predici la nuova posizione (integrazione di velocità e accelerazione)
    Eigen::Vector3d g(0, 0, -g_);  // Vettore gravità nel frame world
    Eigen::Vector3d acc_world = R * acc + g;  // Accelerazione nel frame world
    
    // Aggiorna la posizione e la velocità
    state_.position = p + v * dt + 0.5 * acc_world * dt * dt;
    state_.velocity = v + acc_world * dt;
    
    // Aggiorna l'orientamento (integrazione del giroscopio)
    Eigen::Vector3d delta_angle = gyro * dt;
    double angle = delta_angle.norm();
    if (angle > 1e-6) {
        Eigen::Quaterniond delta_q(
            Eigen::AngleAxisd(angle, delta_angle.normalized()));
        state_.orientation = (q * delta_q).normalized();
    }
    
    // Calcola la matrice di transizione di stato F
    // (semplificata, andrebbe calcolata la jacobiana del modello)
    F_ = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    
    // Aggiorna la covarianza dello stato
    P_ = F_ * P_ * F_.transpose() + Q_;
}

void ExtendedKalmanFilter::updateGPS(const GPSData& gps) {
    // Misura di posizione diretta
    Eigen::Vector3d z = gps.position;
    
    // Matrice di osservazione H (osserviamo solo la posizione)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, P_.rows());
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
    
    // Residuo (errore tra misura e predizione)
    Eigen::Vector3d y = z - state_.position;
    
    // Calcola la matrice di Kalman
    Eigen::Matrix3d S = H * P_ * H.transpose() + R_gps_;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Aggiorna lo stato e la covarianza
    Eigen::VectorXd dx = K * y;
    state_.position += dx.segment<3>(0);
    state_.velocity += dx.segment<3>(3);
    
    // Aggiorna la covarianza
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    P_ = (I - K * H) * P_;
}

void ExtendedKalmanFilter::updateVisualOdometry(const VisualOdometryData& vo) {
    // Misura di movimento relativo
    Eigen::Vector3d delta_p = vo.translation;
    Eigen::Quaterniond delta_q = vo.rotation;
    
    // Aggiorna la posizione e l'orientamento
    state_.position += state_.orientation * delta_p;
    state_.orientation = (state_.orientation * delta_q).normalized();
    
    // Qui andrebbe implementato l'aggiornamento completo della covarianza
    // (semplificato per brevità)
    P_ *= 0.95;  // Riduci l'incertezza
}

void ExtendedKalmanFilter::updatePicoOdometry(const PicoOdometryData& pico_odom, double dt) {
    if (dt <= 0.0) return;
    
    // Estrai la misura di posizione e velocità da Pico
    Eigen::Vector3d z_pos = pico_odom.position;
    Eigen::Vector3d z_vel = pico_odom.velocity;
    
    // Matrice di osservazione H (osserviamo posizione e velocità)
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, P_.rows());
    H.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Posizione
    H.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();  // Velocità
    
    // Vettore di misura
    Eigen::VectorXd z(6);
    z.segment<3>(0) = z_pos;
    z.segment<3>(3) = z_vel;
    
    // Vettore di stato previsto (solo posizione e velocità)
    Eigen::VectorXd h(6);
    h.segment<3>(0) = state_.position;
    h.segment<3>(3) = state_.velocity;
    
    // Residuo (errore tra misura e predizione)
    Eigen::VectorXd y = z - h;
    
    // Matrice di covarianza del rumore di misura
    Eigen::MatrixXd R_pico = Eigen::MatrixXd::Zero(6, 6);
    R_pico.block<3, 3>(0, 0) = pico_odom.covariance;  // Covarianza posizione
    R_pico.block<3, 3>(3, 3) = pico_odom.covariance * 0.1;  // Covarianza velocità (scalata)
    
    // Calcola la matrice di Kalman
    Eigen::MatrixXd S = H * P_ * H.transpose() + R_pico;
    Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();
    
    // Aggiorna lo stato
    Eigen::VectorXd dx = K * y;
    state_.position += dx.segment<3>(0);
    state_.velocity += dx.segment<3>(3);
    
    // Aggiorna la covarianza
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    P_ = (I - K * H) * P_ * (I - K * H).transpose() + K * R_pico * K.transpose();
}

} // namespace slam
