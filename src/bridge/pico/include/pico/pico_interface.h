#ifndef PICO_INTERFACE_H
#define PICO_INTERFACE_H

#include <string>
#include <memory>
#include <thread>
#include <chrono>
#include <atomic>
#include <vector>
#include <mutex>
#include <condition_variable>
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "pico/pico_protocol.h"

// Forward declaration per evitare dipendenze circolari
class ConfigManager;

namespace pico {

class SerialInterface;

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
    std::unique_ptr<SerialInterface> serial_;
    
    // Buffer per i dati ricevuti
    std::vector<uint8_t> receive_buffer_;
    std::mutex buffer_mutex_;
    std::condition_variable cv_;
    
    // Thread per l'elaborazione dei dati
    std::atomic<bool> running_{false};
    std::thread process_thread_;
    
    // Contatori statistiche
    uint32_t messages_received_{0};
    uint32_t messages_published_{0};
    
    bool initializeMqtt();
    bool initializeSerial();
    
    // Gestione MQTT
    void processMqttMessage(const std::string& topic, const std::string& payload);
    void publishSensorData(const SensorData& data);
    void publishStatusReport(const StatusReport& report);
    
    // Gestione seriale
    void processReceivedData(const uint8_t* data, size_t length);
    void processThread();
    
    // Invio comandi
    bool sendMotorCommand(const MotorCommand& cmd);
    bool sendSystemCommand(const SystemCommand& cmd);
    
    // Utility
    std::string getCurrentTimeString() const;

private:
    // Callback per MQTT
    std::function<void(const std::string&, const std::string&)> mqtt_callback_;
    
    // Callback per la seriale
    std::function<void(const std::vector<uint8_t>&)> serial_callback_;
};

} // namespace pico

#endif // PICO_INTERFACE_H
