#include "slam/sensor_fusion.h"
#include <algorithm>
#include <chrono>
#include <cmath>

namespace slam {

SensorFusion::SensorFusion() 
    : ekf_(std::make_unique<ExtendedKalmanFilter>()),  // Inizializzazione nella lista di inizializzazione
      last_update_time_(0) {
    
    // Inizializza lo stato iniziale
    State init_state;
    init_state.position = Vector3d::Zero();
    init_state.velocity = Vector3d::Zero();
    init_state.orientation = Quaterniond::Identity();
    init_state.bias_gyro = Vector3d::Zero();
    init_state.bias_accel = Vector3d::Zero();
    
    // Inizializza il filtro con lo stato iniziale
    if (ekf_) {
        ekf_->initialize(init_state);
    } else {
        throw std::runtime_error("Impossibile inizializzare ExtendedKalmanFilter");
    }
}

void SensorFusion::processMagnetometer(const Vector3d& mag, uint64_t timestamp) {
    // Applica calibrazione (offset/scale/align) nel frame body
    Eigen::Vector3d m_cal = (mag - mag_offset_).cwiseProduct(mag_scale_);
    m_cal = mag_align_ * m_cal;

    // Ruota il vettore magnetico dal frame body al frame world usando l'orientamento corrente
    Eigen::Matrix3d Rbw = ekf_->getState().orientation.toRotationMatrix();
    Eigen::Vector3d m_world = Rbw * m_cal;

    // Proietta sul piano orizzontale (rimuovi componente Z)
    m_world.z() = 0.0;
    const double horiz_norm = m_world.head<2>().norm();
    if (horiz_norm < 1e-6) {
#ifdef DEBUG_SLAM_MAG
        std::cerr << "[MAG] vettore orizzontale troppo piccolo (|m_xy|=" << horiz_norm << ") scarto" << std::endl;
#endif
        return; // misura non affidabile
    }

    auto wrapPi = [](double a) {
        while (a > M_PI) a -= 2.0 * M_PI;
        while (a < -M_PI) a += 2.0 * M_PI;
        return a;
    };
    double heading = std::atan2(m_world.y(), m_world.x());
    heading = wrapPi(heading + mag_declination_rad_);

#ifdef DEBUG_SLAM_MAG
    std::cerr << "[MAG] |m_xy|=" << horiz_norm << " heading(rad)=" << heading
              << " weight=" << mag_weight_ << std::endl;
#endif
    ekf_->updateMagnetometer(heading, mag_weight_);

    if (last_update_time_ == 0) {
        last_update_time_ = timestamp;
    }
}

void SensorFusion::processIMU(const IMUData& imu) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // Aggiungi al buffer IMU
    imu_buffer_.push_back(imu);
    
    // Mantieni una dimensione massima del buffer
    constexpr size_t max_buffer_size = 1000;
    if (imu_buffer_.size() > max_buffer_size) {
        imu_buffer_.pop_front();
    }
    
    // Sincronizza e aggiorna se necessario
    synchronizeAndUpdate();
}

void SensorFusion::processGPS(const GPSData& gps) {
    if (last_update_time_ == 0) {
        // Prima misura GPS: inizializza la posizione
        State state = ekf_->getState();
        state.position = gps.position;
        ekf_->initialize(state);
        last_update_time_ = gps.timestamp;
        return;
    }
    
    // Trova i dati IMU più vicini a questo timestamp
    IMUData closest_imu;
    if (findClosestIMUData(gps.timestamp, closest_imu)) {
        double dt = (gps.timestamp - last_update_time_) * 1e-6;  // Converti in secondi
        if (dt > 0) {
            ekf_->predict(closest_imu, dt);
            ekf_->updateGPS(gps);
            last_update_time_ = gps.timestamp;
            
            // Aggiungi alla traiettoria
            trajectory_.push_back(ekf_->getState().position);
        }
    }
}

void SensorFusion::processVisualOdometry(const VisualOdometryData& vo) {
    if (last_update_time_ == 0) {
        last_update_time_ = vo.timestamp;
        return;
    }
    
    // Trova i dati IMU più vicini a questo timestamp
    IMUData closest_imu;
    if (findClosestIMUData(vo.timestamp, closest_imu)) {
        double dt = (vo.timestamp - last_update_time_) * 1e-6;  // Converti in secondi
        if (dt > 0) {
            ekf_->predict(closest_imu, dt);
            ekf_->updateVisualOdometry(vo);
            last_update_time_ = vo.timestamp;
            
            // Aggiungi alla traiettoria
            trajectory_.push_back(ekf_->getState().position);
        }
    }
}

void SensorFusion::processPicoOdometry(const PicoOdometryData& po) {
    if (last_update_time_ == 0) {
        last_update_time_ = po.timestamp;
        return;
    }

    // Trova i dati IMU più vicini a questo timestamp
    IMUData closest_imu;
    if (findClosestIMUData(po.timestamp, closest_imu)) {
        double dt = (po.timestamp - last_update_time_) * 1e-6;  // secondi
        if (dt > 0) {
            ekf_->predict(closest_imu, dt);
            ekf_->updatePicoOdometry(po, dt);
            last_update_time_ = po.timestamp;

            // Aggiorna traiettoria
            trajectory_.push_back(ekf_->getState().position);
        }
    }
}

State SensorFusion::getCurrentState() const {
    return ekf_ ? ekf_->getState() : State();
}

std::vector<Vector3d> SensorFusion::getTrajectory() const {
    return trajectory_;
}

void SensorFusion::setInitialPosition(const Vector3d& position) {
    State state = ekf_ ? ekf_->getState() : State();
    state.position = position;
    if (ekf_) {
        ekf_->setState(state);
    }
}

void SensorFusion::setInitialOrientation(const Vector3d& orientation) {
    State state = ekf_ ? ekf_->getState() : State();
    state.orientation = Eigen::AngleAxisd(orientation.x(), Vector3d::UnitX()) *
                       Eigen::AngleAxisd(orientation.y(), Vector3d::UnitY()) *
                       Eigen::AngleAxisd(orientation.z(), Vector3d::UnitZ());
    if (ekf_) {
        ekf_->setState(state);
    }
}

void SensorFusion::setMapResolution(double resolution) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    (void)resolution; // parametro non usato
    if (ekf_) {
        // Imposta la risoluzione della mappa nel filtro di Kalman
        // es. ekf_->setMapResolution(resolution);
    }
}

void SensorFusion::setMapSize(double width, double height) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    (void)width; (void)height; // parametri non usati
    if (ekf_) {
        // Imposta le dimensioni della mappa nel filtro di Kalman
        // es. ekf_->setMapSize(width, height);
    }
}

void SensorFusion::setMapOrigin(double x, double y) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    (void)x; (void)y; // parametri non usati
    if (ekf_) {
        // Imposta l'origine della mappa nel filtro di Kalman
        // es. ekf_->setMapOrigin(x, y);
    }
}

void SensorFusion::setParticleCount(int count) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    (void)count; // parametro non usato
    if (ekf_) {
        // Imposta il numero di particelle nel filtro di particelle
        // es. ekf_->setParticleCount(count);
    }
}

void SensorFusion::setMotionModelNoise(const std::vector<double>& noise) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    if (ekf_ && noise.size() >= 6) {
        // Imposta il rumore del modello di moto
        // es. ekf_->setProcessNoise(noise);
    }
    (void)noise; // parametro non usato se non impostato
}

void SensorFusion::setSensorModelNoise(double noise) {
    // Implementazione di esempio - potrebbe essere necessario adattare
    (void)noise; // parametro non usato
    if (ekf_) {
        // Imposta il rumore del modello del sensore
        // es. ekf_->setMeasurementNoise(noise);
    }
}

void SensorFusion::synchronizeAndUpdate() {
    // Qui potresti implementare la logica per sincronizzare i dati
    // da diversi sensori con timestamp diversi
    // (semplificato in questo esempio)
}

bool SensorFusion::findClosestIMUData(uint64_t timestamp, IMUData& result) const {
    if (imu_buffer_.empty()) {
        return false;
    }
    
    // Cerca il dato IMU con il timestamp più vicino
    auto it = std::min_element(imu_buffer_.begin(), imu_buffer_.end(),
        [timestamp](const IMUData& a, const IMUData& b) {
            return std::abs(static_cast<int64_t>(a.timestamp - timestamp)) < 
                   std::abs(static_cast<int64_t>(b.timestamp - timestamp));
        });
    
    if (it != imu_buffer_.end()) {
        result = *it;
        return true;
    }
    
    return false;
}

} // namespace slam
