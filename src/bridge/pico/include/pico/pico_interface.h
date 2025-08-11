#ifndef PICO_INTERFACE_H
#define PICO_INTERFACE_H

#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"

// Forward declaration per evitare dipendenze circolari
class ConfigManager;

namespace pico {

class PicoInterface {
public:
    PicoInterface(const std::shared_ptr<ConfigManager>& config);
    ~PicoInterface();

    bool initialize();
    void run();
    void shutdown();

private:
    std::shared_ptr<ConfigManager> config_;
    std::unique_ptr<mqtt::MqttClient> mqtt_client_;
    
    // Aggiungi qui i membri per la gestione della seriale e altri componenti
    
    bool initializeMqtt();
    void processMqttMessage(const std::string& topic, const std::string& payload);
    void processSerialData(const std::string& data);
};

} // namespace pico

#endif // PICO_INTERFACE_H
