#pragma once

#include <Eigen/Dense>
#include <vector>
#include <cstdint>

namespace slam {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Matrix3d = Eigen::Matrix3d;
using Quaterniond = Eigen::Quaterniond;

// Stato del robot
struct State {
    Vector3d position;      // Posizione (x, y, z)
    Vector3d velocity;      // Velocità (vx, vy, vz)
    Quaterniond orientation;// Orientamento (quaternione)
    Vector3d bias_gyro;     // Bias giroscopio
    Vector3d bias_accel;    // Bias accelerometro
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

// Dati IMU
struct IMUData {
    Vector3d accel;         // Accelerometro (m/s²)
    Vector3d gyro;          // Giroscopio (rad/s)
    Vector3d mag;           // Magnetometro (uT) - opzionale
    bool has_mag = false;   // Presenza misura magnetometro
    uint64_t timestamp;     // Timestamp in µs
};

// Dati GPS
struct GPSData {
    Vector3d position;      // Lat, Lon, Alt
    Matrix3d covariance;    // Matrice di covarianza
    uint64_t timestamp;     // Timestamp in µs
};

// Dati odometria visiva
struct VisualOdometryData {
    Vector3d translation;   // Traslazione rispetto al frame precedente
    Quaterniond rotation;   // Rotazione rispetto al frame precedente
    double confidence;      // Affidabilità della stima (0-1)
    uint64_t timestamp;     // Timestamp in µs
};

// Dati odometria da Pico (encoder ruote)
struct PicoOdometryData {
    Vector3d position;      // Posizione stimata (x, y, theta)
    Vector3d velocity;      // Velocità lineare e angolare (vx, vy, omega)
    Matrix3d covariance;    // Matrice di covarianza della stima
    uint64_t timestamp;     // Timestamp in µs
    
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

} // namespace slam
