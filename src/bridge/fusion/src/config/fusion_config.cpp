#include "config/fusion_config.h"
#include <stdexcept>

namespace fusion {
namespace config {

std::shared_ptr<FusionConfig> FusionConfig::fromConfigManager(
    const std::shared_ptr<fusion::config::ConfigManager>& config) {
    
    auto fusion_config = std::make_shared<FusionConfig>();
    
    try {
        // Parametri EKF
        auto& ekf = fusion_config->ekf;
        ekf.initial_position_std = config->getDouble("fusion.ekf.initial_position_std", 0.1);
        ekf.initial_velocity_std = config->getDouble("fusion.ekf.initial_velocity_std", 0.1);
        ekf.initial_orientation_std = config->getDouble("fusion.ekf.initial_orientation_std", 0.1);
        ekf.accel_noise_std = config->getDouble("fusion.ekf.accel_noise_std", 0.1);
        ekf.gyro_noise_std = config->getDouble("fusion.ekf.gyro_noise_std", 0.01);
        ekf.accel_bias_std = config->getDouble("fusion.ekf.accel_bias_std", 0.001);
        ekf.gyro_bias_std = config->getDouble("fusion.ekf.gyro_bias_std", 0.0001);
        ekf.odom_scale_std = config->getDouble("fusion.ekf.odom_scale_std", 0.0001);
        
        // Parametri MQTT
        auto& mqtt = fusion_config->mqtt;
        mqtt.broker_address = config->getString("mqtt.broker", "localhost");
        mqtt.broker_port = config->getInt("mqtt.port", 1883);
        mqtt.client_id = config->getString("fusion.mqtt.client_id", "fusion_bridge");
        mqtt.username = config->getString("mqtt.username", "");
        mqtt.password = config->getString("mqtt.password", "");
        mqtt.base_topic = config->getString("fusion.mqtt.base_topic", "fusion");
        
        // Parametri sensori
        auto& sensors = fusion_config->sensors;
        sensors.imu_update_rate_hz = config->getDouble("fusion.sensors.imu_rate_hz", 100.0);
        sensors.gps_update_rate_hz = config->getDouble("fusion.sensors.gps_rate_hz", 10.0);
        sensors.odom_update_rate_hz = config->getDouble("fusion.sensors.odom_rate_hz", 50.0);
        
    } catch (const std::exception& e) {
        throw std::runtime_error("Errore nel caricamento della configurazione: " + std::string(e.what()));
    }
    
    return fusion_config;
}

} // namespace config
} // namespace fusion
