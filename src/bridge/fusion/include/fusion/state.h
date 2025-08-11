#ifndef FUSION_STATE_H
#define FUSION_STATE_H

#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <chrono>

namespace fusion {

// Struttura per lo stato del filtro EKF
struct State {
    // Timestamp dell'ultimo aggiornamento
    std::chrono::time_point<std::chrono::system_clock> timestamp;
    
    // Posizione (x, y, z) in metri nel sistema di riferimento globale
    Eigen::Vector3d position;
    
    // Velocità (vx, vy, vz) in m/s nel sistema di riferimento del corpo
    Eigen::Vector3d velocity_body;
    
    // Orientamento come quaternione (qw, qx, qy, qz)
    Eigen::Quaterniond orientation;
    
    // Bias dell'accelerometro (bx, by, bz) in m/s²
    Eigen::Vector3d accel_bias;
    
    // Bias del giroscopio (bx, by, bz) in rad/s
    Eigen::Vector3d gyro_bias;
    
    // Scala odometrica (sx, sy, sz)
    Eigen::Vector3d odom_scale;
    
    // Costruttore con valori di default
    State();
    
    // Resetta lo stato ai valori iniziali
    void reset();
};

// Struttura per le misure IMU
struct ImuData {
    // Accelerometro (m/s²)
    Eigen::Vector3d accel;
    
    // Giroscopio (rad/s)
    Eigen::Vector3d gyro;
    
    // Magnetometro (uT, opzionale)
    Eigen::Vector3d mag;
    
    // Timestamp
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

// Struttura per le misure GPS
struct GpsData {
    // Posizione (latitudine, longitudine, altitudine)
    double latitude;    // gradi
    double longitude;   // gradi
    double altitude;    // metri
    
    // Velocità (nord, est, giù) in m/s
    Eigen::Vector3d velocity;
    
    // Accuratezza orizzontale in metri
    double hdop;
    
    // Numero di satelliti
    int num_satellites;
    
    // Timestamp
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

// Struttura per le misure odometriche
struct OdomData {
    // Spostamento incrementale (dx, dy, dtheta)
    Eigen::Vector3d delta_pose;
    
    // Timestamp
    std::chrono::time_point<std::chrono::system_clock> timestamp;
};

} // namespace fusion

#endif // FUSION_STATE_H
