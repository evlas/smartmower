#ifndef FUSION_EKF_H
#define FUSION_EKF_H

#include "fusion/state.h"
#include <Eigen/Dense>

namespace fusion {

class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    
    // Inizializza il filtro con lo stato iniziale
    void initialize(const State& initial_state);
    
    // Predizione dello stato basata su dati IMU
    void predict(const ImuData& imu, double dt);
    
    // Correzione basata su dati GPS
    void updateGps(const GpsData& gps);
    
    // Correzione basata su odometria
    void updateOdom(const OdomData& odom);
    
    // Restituisce lo stato stimato corrente
    const State& getState() const { return state_; }
    
    // Restituisce la matrice di covarianza
    const Eigen::MatrixXd& getCovariance() const { return P_; }
    
private:
    // Stato corrente
    State state_;
    
    // Matrice di covarianza dello stato
    Eigen::MatrixXd P_;
    
    // Matrice di rumore di processo
    Eigen::MatrixXd Q_;
    
    // Matrice di rumore delle misure GPS
    Eigen::MatrixXd R_gps_;
    
    // Matrice di rumore delle misure odometriche
    Eigen::MatrixXd R_odom_;
    
    // Matrice di osservazione GPS
    Eigen::MatrixXd H_gps_;
    
    // Matrice di osservazione odometria
    Eigen::MatrixXd H_odom_;
    
    // Inizializza le matrici di rumore
    void initializeNoiseMatrices();
    
    // Calcola la matrice di rotazione dal sistema di riferimento del corpo a quello globale
    Eigen::Matrix3d getRotationMatrix() const;
    
    // Calcola la matrice jacobiana del modello di predizione
    Eigen::MatrixXd computePredictionJacobian(const ImuData& imu, double dt) const;
    
    // Calcola la matrice di rumore di processo
    Eigen::MatrixXd computeProcessNoise(double dt) const;
};

} // namespace fusion

#endif // FUSION_EKF_H
