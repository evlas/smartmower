#include "pico/pico_interface.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <iostream>

using namespace pico;
#include <thread>
#include <chrono>

namespace pico {

PicoInterface::PicoInterface(const std::shared_ptr<ConfigManager>& config)
    : config_(config) {
    // Inizializza qui i membri
}

PicoInterface::~PicoInterface() {
    shutdown();
}

bool PicoInterface::initialize() {
    if (!initializeMqtt()) {
        std::cerr << "Errore nell'inizializzazione di MQTT" << std::endl;
        return false;
    }
    
    // Inizializza qui la connessione seriale
    
    return true;
}

void PicoInterface::run() {
    // Loop principale
    std::this_thread::sleep_for(std::chrono::seconds(1));
    // Aggiungi qui la logica principale
}

void PicoInterface::shutdown() {
    // Chiudi qui le connessioni e le risorse
    if (mqtt_client_) {
        mqtt_client_->disconnect();
    }
}

bool PicoInterface::initializeMqtt() {
    try {
        std::string broker = config_->getString("mqtt.broker", "localhost");
        int port = config_->getInt("mqtt.port", 1883);
        std::string client_id = "pico_bridge";
        
        mqtt_client_ = std::make_unique<mqtt::MqttClient>(
            broker,
            port,
            client_id,
            config_->getString("mqtt.username", ""),
            config_->getString("mqtt.password", "")
        );
        
        return mqtt_client_->connect();
    } catch (const std::exception& e) {
        std::cerr << "Errore MQTT: " << e.what() << std::endl;
        return false;
    }
}

void PicoInterface::processMqttMessage(const std::string& topic, const std::string& payload) {
    // Gestisci qui i messaggi MQTT in arrivo
    std::cout << "Messaggio ricevuto su " << topic << ": " << payload << std::endl;
}

void PicoInterface::processSerialData(const std::string& data) {
    // Elabora qui i dati ricevuti dalla seriale
    std::cout << "Dati dalla seriale: " << data << std::endl;
    
    // Pubblica i dati su MQTT se necessario
    if (mqtt_client_) {
        // Esempio: mqtt_client_->publish("pico/data", data);
    }
}

} // namespace pico
