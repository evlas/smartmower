#pragma once

#include "slam/types.h"
#include "slam/ekf.h"
#include <memory>
#include <deque>
#include <mutex>

namespace slam {

/**
 * @brief Classe principale per la fusione di sensori SLAM
 */
class SensorFusion {
public:
    SensorFusion();
    ~SensorFusion() = default;

    /**
     * @brief Elabora un nuovo pacchetto IMU
     */
    void processIMU(const IMUData& imu);

    /**
     * @brief Elabora una misura di magnetometro
     * @param mag vettore campo magnetico (uT) nel frame del sensore
     * @param timestamp timestamp in µs
     */
    void processMagnetometer(const Vector3d& mag, uint64_t timestamp);

    /**
     * @brief Imposta il peso da usare nell'aggiornamento di yaw da magnetometro [0..1]
     */
    void setMagnetometerWeight(double w) { mag_weight_ = w; }

    /**
     * @brief Imposta la declinazione magnetica in gradi (positiva verso Est)
     */
    void setMagDeclinationDeg(double deg) {
        constexpr double kPi = 3.14159265358979323846;
        mag_declination_rad_ = deg * kPi / 180.0;
    }

    /**
     * @brief Imposta i parametri di calibrazione del magnetometro
     */
    void setMagCalibrationOffset(const Vector3d& off) { mag_offset_ = off; }
    void setMagCalibrationScale(const Vector3d& sca) { mag_scale_ = sca; }
    void setMagCalibrationAlignMatrix(const Eigen::Matrix3d& R) { mag_align_ = R; }

    /**
     * @brief Elabora un nuovo dato GPS
     */
    void processGPS(const GPSData& gps);

    /**
     * @brief Elabora un nuovo dato di odometria visiva
     */
    void processVisualOdometry(const VisualOdometryData& vo);

    /**
     * @brief Elabora un nuovo dato di odometria dalle ruote (Pico)
     */
    void processPicoOdometry(const PicoOdometryData& po);

    /**
     * @brief Restituisce lo stato attuale del robot
     */
    State getCurrentState() const;

    /**
     * @brief Restituisce la traiettoria stimata
     */
    std::vector<Vector3d> getTrajectory() const;
    
    /**
     * @brief Imposta la posizione iniziale
     */
    void setInitialPosition(const Vector3d& position);
    
    /**
     * @brief Imposta l'orientamento iniziale
     */
    void setInitialOrientation(const Vector3d& orientation);
    
    /**
     * @brief Imposta la risoluzione della mappa
     */
    void setMapResolution(double resolution);
    
    /**
     * @brief Imposta le dimensioni della mappa
     */
    void setMapSize(double width, double height);
    
    /**
     * @brief Imposta l'origine della mappa
     */
    void setMapOrigin(double x, double y);
    
    /**
     * @brief Imposta il numero di particelle per la localizzazione
     */
    void setParticleCount(int count);
    
    /**
     * @brief Imposta il rumore del modello di moto
     */
    void setMotionModelNoise(const std::vector<double>& noise);
    
    /**
     * @brief Imposta il rumore del modello del sensore
     */
    void setSensorModelNoise(double noise);

private:
    // Filtro di Kalman esteso
    std::unique_ptr<ExtendedKalmanFilter> ekf_;
    
    // Buffer per la sincronizzazione
    std::deque<IMUData> imu_buffer_;
    std::mutex buffer_mutex_;
    
    // Traiettoria stimata
    std::vector<Vector3d> trajectory_;
    
    // Timestamp dell'ultimo aggiornamento
    uint64_t last_update_time_ = 0;
    
    // Sincronizza i dati e aggiorna il filtro
    void synchronizeAndUpdate();
    
    // Trova i dati IMU più vicini a un timestamp
    bool findClosestIMUData(uint64_t timestamp, IMUData& result) const;

    // Peso fusione magnetometro
    double mag_weight_ = 0.1;
    double mag_declination_rad_ = 0.0;

    // Calibrazione magnetometro
    Vector3d mag_offset_ = Vector3d::Zero();
    Vector3d mag_scale_ = Vector3d::Ones();
    Eigen::Matrix3d mag_align_ = Eigen::Matrix3d::Identity();
};

} // namespace slam

