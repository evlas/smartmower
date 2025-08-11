#ifndef FUSION_SENSOR_FUSION_H
#define FUSION_SENSOR_FUSION_H

#include "fusion/ekf.h"
#include "fusion/state.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"
#include <memory>
#include <thread>
#include <atomic>
#include <string>

namespace fusion {

// Forward declarations
namespace config { class ConfigManager; }
namespace mqtt { 
    class MqttClient;
    class MqttTopics;
}

class SensorFusion {
public:
    SensorFusion(std::shared_ptr<fusion::config::ConfigManager> config,
                std::shared_ptr<fusion::mqtt::MqttClient> mqtt_client);
    
    ~SensorFusion();
    
    // Inizializza il sistema di fusione
    bool initialize();
    
    // Avvia il loop di fusione
    void start();
    
    // Ferma il loop di fusione
    void stop();
    
    // Restituisce lo stato corrente
    const State& getState() const { return ekf_.getState(); }
    
private:
    // Configurazione
    std::shared_ptr<fusion::config::ConfigManager> config_;
    
    // Client MQTT
    std::shared_ptr<fusion::mqtt::MqttClient> mqtt_client_;
    
    // Gestore dei topic MQTT
    std::unique_ptr<fusion::mqtt::MqttTopics> mqtt_topics_;
    
    // Filtro EKF
    ExtendedKalmanFilter ekf_;
    
    // Thread per l'elaborazione
    std::thread processing_thread_;
    std::atomic<bool> running_{false};
    
    // Frequenza di aggiornamento (Hz)
    double update_rate_hz_{100.0};
    
    // Inizializza i parametri dalla configurazione
    bool loadParameters();
    
    // Loop principale di elaborazione
    void processingLoop();
    
    // Gestisce i messaggi MQTT in arrivo
    void handleMqttMessage(const std::string& topic, const std::string& payload);
    
    // Pubblica lo stato corrente su MQTT
    void publishState();
};

} // namespace fusion

#endif // FUSION_SENSOR_FUSION_H
