#include "slam/config.h"
#include <iostream>

namespace slam {

bool SlamConfig::loadFromFile(const std::string& config_file) {
    // Crea un nuovo gestore di configurazione
    config_manager_ = createConfigManager();
    if (!config_manager_) {
        std::cerr << "Errore nella creazione del gestore di configurazione" << std::endl;
        return false;
    }

    // Carica il file di configurazione
    if (!config_manager_->loadFromFile(config_file)) {
        std::cerr << "Impossibile caricare il file di configurazione: " << config_file << std::endl;
        return false;
    }

    // Carica le varie sezioni della configurazione
    loadObstacleDetectionConfig();
    loadEkfConfig();

    return true;
}

void SlamConfig::loadObstacleDetectionConfig() {
    // Carica i valori di default
    obstacle_config_.min_distance = 0.3;  // 30 cm
    obstacle_config_.max_distance = 4.0;  // 4 metri
    obstacle_config_.filter_window_size = 5;
    obstacle_config_.obstacle_timeout = 5.0;  // 5 secondi

    // Se esiste la sezione "obstacle_detection" nel file di configurazione,
    // sovrascrivi i valori di default
    if (config_manager_->hasKey("obstacle_detection")) {
        auto obstacle_config = config_manager_->getObject("obstacle_detection");
        
        if (obstacle_config.contains("min_distance")) {
            obstacle_config_.min_distance = obstacle_config["min_distance"].get<double>();
        }
        
        if (obstacle_config.contains("max_distance")) {
            obstacle_config_.max_distance = obstacle_config["max_distance"].get<double>();
        }
        
        if (obstacle_config.contains("filter_window_size")) {
            obstacle_config_.filter_window_size = obstacle_config["filter_window_size"].get<size_t>();
        }
        
        if (obstacle_config.contains("obstacle_timeout")) {
            obstacle_config_.obstacle_timeout = obstacle_config["obstacle_timeout"].get<double>();
        }
    }
}

void SlamConfig::loadEkfConfig() {
    // Valori di default per il filtro di Kalman
    ekf_config_.process_noise_pos = 0.1;
    ekf_config_.process_noise_vel = 0.1;
    ekf_config_.process_noise_att = 0.01;
    ekf_config_.gps_noise = 0.5;
    ekf_config_.odom_noise = 0.05;

    // Carica la configurazione da file se presente
    if (config_manager_->hasKey("ekf")) {
        auto ekf_config = config_manager_->getObject("ekf");
        
        if (ekf_config.contains("process_noise_pos")) {
            ekf_config_.process_noise_pos = ekf_config["process_noise_pos"].get<double>();
        }
        
        if (ekf_config.contains("process_noise_vel")) {
            ekf_config_.process_noise_vel = ekf_config["process_noise_vel"].get<double>();
        }
        
        if (ekf_config.contains("process_noise_att")) {
            ekf_config_.process_noise_att = ekf_config["process_noise_att"].get<double>();
        }
        
        if (ekf_config.contains("gps_noise")) {
            ekf_config_.gps_noise = ekf_config["gps_noise"].get<double>();
        }
        
        if (ekf_config.contains("odom_noise")) {
            ekf_config_.odom_noise = ekf_config["odom_noise"].get<double>();
        }
    }
}

} // namespace slam
