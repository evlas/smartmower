#pragma once

#include "slam/types.h"
#include <Eigen/Dense>
#include <memory>

namespace slam {

/**
 * @brief Implementazione del filtro di Kalman esteso per la fusione di sensori
 */
class ExtendedKalmanFilter {
public:
    ExtendedKalmanFilter();
    ~ExtendedKalmanFilter() = default;

    /**
     * @brief Inizializza il filtro con uno stato iniziale
     */
    void initialize(const State& initial_state);

    /**
     * @brief Imposta direttamente lo stato corrente senza eseguire inizializzazione completa
     *        (nessun log, nessun reset di covarianze). Utile per applicare posizione/orientamento iniziali
     *        dopo la prima initialize.
     */
    void setState(const State& state) { state_ = state; }

    /**
     * @brief Predice lo stato successivo basandosi sul modello di moto e dati IMU
     */
    void predict(const IMUData& imu, double dt);

    /**
     * @brief Aggiorna la stima con dati GPS
     */
    void updateGPS(const GPSData& gps);

    /**
     * @brief Aggiorna la stima con odometria visiva
     */
    void updateVisualOdometry(const VisualOdometryData& vo);

    /**
     * @brief Aggiorna la stima con misura di heading (yaw) da magnetometro
     * @param yaw_heading heading misurato [rad], frame mondo (0=asse X, senso antiorario)
     * @param weight fattore [0..1] che controlla quanto fidarsi della misura (default 0.1)
     */
    void updateMagnetometer(double yaw_heading, double weight = 0.1);

    /**
     * @brief Aggiorna la stima con i dati di odometria da Pico
     * @param pico_odom Dati di odometria da Pico (posizione e orientamento)
     * @param dt Intervallo di tempo dall'ultimo aggiornamento
     */
    void updatePicoOdometry(const PicoOdometryData& pico_odom, double dt);

    /**
     * @brief Restituisce lo stato corrente stimato
     */
    const State& getState() const { return state_; }

    /**
     * @brief Restituisce la matrice di covarianza dello stato
     */
    const Eigen::MatrixXd& getCovariance() const { return P_; }

private:
    // Stato corrente e covarianza
    State state_;
    Eigen::MatrixXd P_;

    // Matrici del modello
    Eigen::MatrixXd F_;  // Matrice di transizione di stato
    Eigen::MatrixXd Q_;  // Covarianza del rumore di processo
    Eigen::MatrixXd R_gps_;  // Covarianza del rumore GPS
    Eigen::MatrixXd R_vo_;   // Covarianza del rumore odometria visiva
    double R_mag_ = 0.5;     // Varianza (semplificata) rumore heading magnetometro [rad^2]

    // Parametri
    static constexpr double g_ = 9.81;  // Accelerazione gravitazionale
};

} // namespace slam
