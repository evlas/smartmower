#include "fusion/ekf.h"
#include <iostream>
#include <cmath>

namespace fusion {

// Costanti
constexpr double GRAVITY = 9.80665;  // m/s²
constexpr size_t STATE_DIM = 18;      // Dimensione dello stato

ExtendedKalmanFilter::ExtendedKalmanFilter() {
    // Inizializza la matrice di covarianza con valori ragionevoli
    P_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;
    
    // Inizializza le matrici di rumore
    initializeNoiseMatrices();
    
    // Inizializza le matrici di osservazione
    H_gps_ = Eigen::MatrixXd::Zero(6, STATE_DIM);
    H_gps_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Posizione
    H_gps_.block<3, 3>(3, 6) = getRotationMatrix();           // Velocità
    
    H_odom_ = Eigen::MatrixXd::Zero(3, STATE_DIM);
    H_odom_.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();  // Posizione
}

void ExtendedKalmanFilter::initialize(const State& initial_state) {
    state_ = initial_state;
    state_.timestamp = std::chrono::system_clock::now();
}

void ExtendedKalmanFilter::predict(const ImuData& imu, double dt) {
    if (dt <= 0.0) return;
    
    // 1. Aggiorna l'orientamento usando il giroscopio
    Eigen::Vector3d gyro_corrected = imu.gyro - state_.gyro_bias;
    Eigen::Quaterniond dq = Eigen::Quaterniond(
        1.0, 
        0.5 * gyro_corrected.x() * dt,
        0.5 * gyro_corrected.y() * dt,
        0.5 * gyro_corrected.z() * dt
    ).normalized();
    
    state_.orientation = (state_.orientation * dq).normalized();
    
    // 2. Calcola l'accelerazione nel sistema di riferimento globale
    Eigen::Vector3d accel_corrected = imu.accel - state_.accel_bias;
    Eigen::Vector3d accel_global = state_.orientation * accel_corrected;
    accel_global.z() -= GRAVITY;  // Sottrai la gravità
    
    // 3. Aggiorna posizione e velocità
    state_.position += state_.velocity_body * dt + 0.5 * accel_global * dt * dt;
    state_.velocity_body += accel_global * dt;
    
    // 4. Calcola la matrice jacobiana del modello di predizione
    Eigen::MatrixXd F = computePredictionJacobian(imu, dt);
    
    // 5. Calcola la matrice di rumore di processo
    Eigen::MatrixXd Q = computeProcessNoise(dt);
    
    // 6. Aggiorna la matrice di covarianza
    P_ = F * P_ * F.transpose() + Q;
    
    // 7. Aggiorna il timestamp
    state_.timestamp = imu.timestamp;
}

void ExtendedKalmanFilter::updateGps(const GpsData& gps) {
    // TODO: Implementa la correzione GPS
    // 1. Calcola l'innovazione
    // 2. Calcola la matrice di guadagno di Kalman
    // 3. Aggiorna lo stato e la covarianza
}

void ExtendedKalmanFilter::updateOdom(const OdomData& odom) {
    // TODO: Implementa la correzione odometrica
    // 1. Calcola l'innovazione
    // 2. Calcola la matrice di guadagno di Kalman
    // 3. Aggiorna lo stato e la covarianza
}

void ExtendedKalmanFilter::initializeNoiseMatrices() {
    // Inizializza le matrici di rumore con valori di default
    // Questi verranno sovrascritti dalla configurazione
    Q_ = Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM) * 0.1;
    R_gps_ = Eigen::MatrixXd::Identity(6, 6) * 0.5;
    R_odom_ = Eigen::MatrixXd::Identity(3, 3) * 0.1;
}

Eigen::Matrix3d ExtendedKalmanFilter::getRotationMatrix() const {
    return state_.orientation.toRotationMatrix();
}

Eigen::MatrixXd ExtendedKalmanFilter::computePredictionJacobian(const ImuData& imu, double dt) const {
    // TODO: Implementa il calcolo della matrice jacobiana del modello di predizione
    // Questa è una semplificazione - dovrebbe essere calcolata numericamente o simbolicamente
    return Eigen::MatrixXd::Identity(STATE_DIM, STATE_DIM);
}

Eigen::MatrixXd ExtendedKalmanFilter::computeProcessNoise(double dt) const {
    // TODO: Calcola la matrice di rumore di processo in base a dt
    return Q_ * dt;
}

} // namespace fusion
