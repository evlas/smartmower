#ifndef SLAM_CONFIG_H
#define SLAM_CONFIG_H

#include "config/config_manager.h"
#include <memory>

namespace slam {

/**
 * @brief Classe di configurazione per il modulo SLAM
 * 
 * Fornisce un'interfaccia tipizzata per accedere alle impostazioni di configurazione
 * del modulo SLAM, inclusa la gestione degli ostacoli e del filtro di Kalman.
 */
class SlamConfig {
public:
    // Struttura per la configurazione del rilevamento ostacoli
    struct ObstacleDetectionConfig {
        double min_distance;          // Distanza minima di rilevamento (m)
        double max_distance;          // Distanza massima di rilevamento (m)
        size_t filter_window_size;    // Dimensione della finestra per il filtraggio
        double obstacle_timeout;      // Timeout per la rimozione degli ostacoli (s)
    };

    // Struttura per la configurazione del filtro di Kalman
    struct EkfConfig {
        double process_noise_pos;     // Rumore di processo per la posizione
        double process_noise_vel;     // Rumore di processo per la velocità
        double process_noise_att;     // Rumore di processo per l'assetto
        double gps_noise;             // Rumore delle misure GPS
        double odom_noise;            // Rumore delle misure di odometria
    };

    /**
     * @brief Carica la configurazione da file
     * 
     * @param config_file Percorso del file di configurazione JSON
     * @return true Se il caricamento è avvenuto con successo
     * @return false In caso di errore
     */
    bool loadFromFile(const std::string& config_file);

    // Accesso alle configurazioni
    const ObstacleDetectionConfig& getObstacleConfig() const { return obstacle_config_; }
    const EkfConfig& getEkfConfig() const { return ekf_config_; }

    // Accesso diretto al gestore di configurazione
    const std::shared_ptr<ConfigManager>& getConfigManager() const { return config_manager_; }

private:
    // Carica la configurazione del rilevamento ostacoli
    void loadObstacleDetectionConfig();
    
    // Carica la configurazione del filtro di Kalman
    void loadEkfConfig();

    std::shared_ptr<ConfigManager> config_manager_;
    ObstacleDetectionConfig obstacle_config_;
    EkfConfig ekf_config_;
};

} // namespace slam

#endif // SLAM_CONFIG_H
