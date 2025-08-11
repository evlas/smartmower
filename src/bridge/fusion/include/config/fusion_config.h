#ifndef FUSION_CONFIG_H
#define FUSION_CONFIG_H

#include <string>
#include <memory>
#include "config/config_manager.h"

namespace fusion {
namespace config {

// Struttura per i parametri di configurazione della fusione
struct FusionConfig {
    // Parametri EKF
    struct {
        double initial_position_std;     // Deviazione standard posizione iniziale (m)
        double initial_velocity_std;     // Deviazione standard velocità iniziale (m/s)
        double initial_orientation_std;  // Deviazione standard orientamento iniziale (rad)
        double accel_noise_std;          // Deviazione standard rumore accelerometro (m/s²)
        double gyro_noise_std;           // Deviazione standard rumore giroscopio (rad/s)
        double accel_bias_std;           // Deviazione standard bias accelerometro (m/s²)
        double gyro_bias_std;            // Deviazione standard bias giroscopio (rad/s)
        double odom_scale_std;           // Deviazione standard scala odometria
    } ekf;
    
    // Parametri MQTT
    struct {
        std::string broker_address;      // Indirizzo del broker MQTT
        int broker_port;                 // Porta del broker MQTT
        std::string client_id;           // ID client MQTT
        std::string username;            // Username MQTT (opzionale)
        std::string password;            // Password MQTT (opzionale)
        std::string base_topic;          // Topic base per la pubblicazione
    } mqtt;
    
    // Parametri sensori
    struct {
        double imu_update_rate_hz;       // Frequenza di aggiornamento IMU (Hz)
        double gps_update_rate_hz;       // Frequenza di aggiornamento GPS (Hz)
        double odom_update_rate_hz;      // Frequenza di aggiornamento odometria (Hz)
    } sensors;
    
    // Carica la configurazione da un gestore di configurazione
    static std::shared_ptr<FusionConfig> fromConfigManager(
        const std::shared_ptr<fusion::config::ConfigManager>& config);
};

} // namespace config
} // namespace fusion

#endif // FUSION_CONFIG_H
