#include "slam/obstacle_detection.h"
#include <cmath>
#include <algorithm>
#include <iostream>
#include <stdexcept>

namespace slam {

using namespace Eigen;

ObstacleDetector::ObstacleDetector(const std::shared_ptr<SlamConfig>& config)
    : config_(config) {
    if (!config_) {
        throw std::invalid_argument("Configurazione non valida");
    }
    
    // Inizializza i parametri dalla configurazione
    const auto& obstacle_config = config_->getObstacleConfig();
    window_size_ = obstacle_config.filter_window_size;
    min_distance_ = obstacle_config.min_distance;
    max_distance_ = obstacle_config.max_distance;
    obstacle_timeout_ = obstacle_config.obstacle_timeout;
    
    // Inizializza i filtri per i tre sensori
    for (auto& sensor : sensors_) {
        sensor.filtered_distance = max_distance_;
        sensor.variance = 0.0;
    }
}

ObstacleDetector::ObstacleDetector(size_t window_size, 
                                 double min_distance,
                                 double max_distance)
    : window_size_(window_size)
    , min_distance_(min_distance)
    , max_distance_(max_distance)
    , obstacle_timeout_(5.0) {  // Valore di default per i test
    // Inizializza i filtri per i tre sensori
    for (auto& sensor : sensors_) {
        sensor.filtered_distance = max_distance_;
        sensor.variance = 0.0;
    }
}

std::vector<DetectedObstacle> ObstacleDetector::processUltrasonicData(
    const pico::SensorData& sensor_data,
    const Vector3d& robot_pose) {
    
    std::vector<DetectedObstacle> obstacles;
    
    // Elabora i dati di ogni sensore
    for (int i = 0; i < 3; ++i) {
        double distance = sensor_data.us_distances[i];
        
        // Filtra il rumore e gestisci letture non valide
        if (distance < min_distance_ || distance > max_distance_) {
            distance = max_distance_;
        }
        
        // Applica il filtro di media mobile
        filterReading(sensors_[i], distance);
        
        // Se la distanza è inferiore alla massima, considera l'ostacolo
        if (sensors_[i].filtered_distance < max_distance_) {
            DetectedObstacle obstacle;
            double angle = SENSOR_ANGLES[i] + robot_pose.z(); // Aggiungi l'orientamento del robot
            
            // Calcola la posizione dell'ostacolo nel frame del robot
            obstacle.distance = sensors_[i].filtered_distance;
            obstacle.position.x() = obstacle.distance * std::cos(angle);
            obstacle.position.y() = obstacle.distance * std::sin(angle);
            
            // Calcola la confidenza in base alla varianza (maggiore varianza = minore confidenza)
            obstacle.confidence = 1.0 / (1.0 + sensors_[i].variance);
            obstacle.timestamp = sensor_data.timestamp * 1000; // Converti in microsecondi
            
            obstacles.push_back(obstacle);
        }
    }
    
    // Aggiungi gli ostacoli alla mappa
    updateObstacleMap(obstacles);
    
    return obstacles;
}

void ObstacleDetector::filterReading(UltrasonicReading& reading, double new_distance) {
    // Aggiungi la nuova lettura alla finestra
    reading.distances.push_back(new_distance);
    
    // Mantieni la dimensione massima della finestra
    if (reading.distances.size() > window_size_) {
        reading.distances.pop_front();
    }
    
    // Calcola la media e la varianza
    double sum = 0.0;
    double sum_sq = 0.0;
    
    for (double d : reading.distances) {
        sum += d;
        sum_sq += d * d;
    }
    
    size_t n = reading.distances.size();
    reading.filtered_distance = sum / n;
    reading.variance = (sum_sq - (sum * sum) / n) / n;
}

bool ObstacleDetector::isObstacleInDirection(double angle, double max_distance) const {
    // Normalizza l'angolo tra -π e π
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    angle -= M_PI;
    
    // Cerca l'ostacolo più vicino nella direzione specificata
    double min_dist = max_distance;
    
    for (const auto& obstacle : obstacle_map_) {
        // Calcola l'angolo dell'ostacolo rispetto al robot
        double obs_angle = std::atan2(obstacle.position.y(), obstacle.position.x());
        
        // Calcola la differenza angolare
        double angle_diff = std::abs(std::fmod(obs_angle - angle + 3*M_PI, 2*M_PI) - M_PI);
        
        // Se l'ostacolo è nella direzione specificata
        if (angle_diff < M_PI/4) {  // 45° di tolleranza
            min_dist = std::min(min_dist, obstacle.distance);
        }
    }
    
    return min_dist < max_distance;
}

double ObstacleDetector::getObstacleDistance(double angle, double max_distance) const {
    // Normalizza l'angolo tra -π e π
    angle = std::fmod(angle + M_PI, 2 * M_PI);
    if (angle < 0) angle += 2 * M_PI;
    angle -= M_PI;
    
    double min_dist = max_distance;
    
    for (const auto& obstacle : obstacle_map_) {
        // Calcola l'angolo dell'ostacolo rispetto al robot
        double obs_angle = std::atan2(obstacle.position.y(), obstacle.position.x());
        
        // Calcola la differenza angolare
        double angle_diff = std::abs(std::fmod(obs_angle - angle + 3*M_PI, 2*M_PI) - M_PI);
        
        // Se l'ostacolo è nella direzione specificata
        if (angle_diff < M_PI/6) {  // 30° di tolleranza
            min_dist = std::min(min_dist, obstacle.distance);
        }
    }
    
    return min_dist;
}

void ObstacleDetector::updateObstacleMap(const std::vector<DetectedObstacle>& obstacles) {
    // Aggiungi i nuovi ostacoli alla mappa
    obstacle_map_.insert(obstacle_map_.end(), obstacles.begin(), obstacles.end());
    
    // Rimuovi ostacoli troppo vecchi
    if (!obstacle_map_.empty()) {
        uint64_t current_time = obstacle_map_.back().timestamp;  // Usa l'ultimo timestamp disponibile
        uint64_t max_age = static_cast<uint64_t>(obstacle_timeout_ * 1e6);  // Converti in microsecondi
        
        obstacle_map_.erase(
            std::remove_if(obstacle_map_.begin(), obstacle_map_.end(),
                [current_time, max_age](const DetectedObstacle& obs) {
                    return (current_time - obs.timestamp) > max_age;
                }),
            obstacle_map_.end()
        );
    }
}

void ObstacleDetector::clearObstacleMap() {
    obstacle_map_.clear();
}

} // namespace slam
