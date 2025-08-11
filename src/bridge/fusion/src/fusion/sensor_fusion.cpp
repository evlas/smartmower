#include "fusion/sensor_fusion.h"
#include "config/config_manager.h"
#include <chrono>
#include <thread>
#include <iostream>
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;
using json = nlohmann::json;

namespace fusion {

SensorFusion::SensorFusion(std::shared_ptr<fusion::config::ConfigManager> config,
                         std::shared_ptr<fusion::mqtt::MqttClient> mqtt_client)
    : config_(config), mqtt_client_(mqtt_client) {
    // Inizializza MqttTopics con i valori di default
    std::string root_topic = config_->getString("mqtt.root_topic", "smartmower");
    std::string base_topic = config_->getString("fusion.mqtt.base_topic", "fusion");
    mqtt_topics_ = std::make_unique<fusion::mqtt::MqttTopics>(root_topic, base_topic);
    
    // Carica i sottotopic personalizzati dalla configurazione
    std::unordered_map<std::string, std::string> subtopics;
    
    // Usa direttamente i metodi getString per evitare l'uso di getJson()
    if (config_->hasKey("fusion.mqtt.subtopics.state")) {
        subtopics["state"] = config_->getString("fusion.mqtt.subtopics.state", "state");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.debug")) {
        subtopics["debug"] = config_->getString("fusion.mqtt.subtopics.debug", "debug");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.error")) {
        subtopics["error"] = config_->getString("fusion.mqtt.subtopics.error", "error");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.command")) {
        subtopics["command"] = config_->getString("fusion.mqtt.subtopics.command", "commands");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.imu")) {
        subtopics["imu"] = config_->getString("fusion.mqtt.subtopics.imu", "sensor/imu");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.gps")) {
        subtopics["gps"] = config_->getString("fusion.mqtt.subtopics.gps", "sensor/gps");
    }
    if (config_->hasKey("fusion.mqtt.subtopics.odom")) {
        subtopics["odom"] = config_->getString("fusion.mqtt.subtopics.odom", "sensor/odom");
    }
    
    if (!subtopics.empty()) {
        mqtt_topics_->setCustomSubtopics(subtopics);
    }
}

SensorFusion::~SensorFusion() {
    stop();
}

bool SensorFusion::initialize() {
    if (!loadParameters()) {
        std::cerr << "Errore nel caricamento dei parametri" << std::endl;
        return false;
    }
    
    // Inizializza lo stato iniziale
    State initial_state;
    initial_state.reset();
    ekf_.initialize(initial_state);
    
    // Configura le callback MQTT
    mqtt_client_->setMessageHandler(
        [this](const std::string& topic, const std::vector<uint8_t>& payload) {
            handleMqttMessage(topic, std::string(payload.begin(), payload.end()));
        }
    );
    
    if (!mqtt_client_->isConnected()) {
        std::cerr << "Errore: Connessione MQTT non stabilita" << std::endl;
        return false;
    }
    
    // Sottoscrizione ai topic MQTT utilizzando MqttTopics
    try {
        mqtt_client_->subscribe(mqtt_topics_->getImuTopic());
        mqtt_client_->subscribe(mqtt_topics_->getGpsTopic());
        mqtt_client_->subscribe(mqtt_topics_->getOdomTopic());
        
        std::cout << "Sottoscritto ai topic MQTT:" << std::endl;
        std::cout << "- IMU: " << mqtt_topics_->getImuTopic() << std::endl;
        std::cout << "- GPS: " << mqtt_topics_->getGpsTopic() << std::endl;
        std::cout << "- Odom: " << mqtt_topics_->getOdomTopic() << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Errore nella sottoscrizione ai topic MQTT: " << e.what() << std::endl;
        return false;
    }
    
    return true;
}

void SensorFusion::start() {
    if (running_) return;
    
    running_ = true;
    processing_thread_ = std::thread(&SensorFusion::processingLoop, this);
    
    std::cout << "Avvio del modulo di fusione sensori..." << std::endl;
}

void SensorFusion::stop() {
    if (!running_) return;
    
    running_ = false;
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    std::cout << "Arresto del modulo di fusione sensori..." << std::endl;
}

bool SensorFusion::loadParameters() {
    try {
        // Carica i parametri di configurazione direttamente
        update_rate_hz_ = config_->getDouble("fusion.sensors.imu_update_rate_hz", 100.0);
        
        std::cout << "Parametri di configurazione caricati:" << std::endl;
        std::cout << "- Frequenza di aggiornamento IMU: " << update_rate_hz_ << " Hz" << std::endl;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Errore nel caricamento dei parametri: " << e.what() << std::endl;
        return false;
    }
}

void SensorFusion::processingLoop() {
    auto last_update = std::chrono::steady_clock::now();
    
    while (running_) {
        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_update).count();
        
        if (dt >= 1.0 / update_rate_hz_) {
            // Esegui un passo di predizione con i dati IMU più recenti
            // (da implementare: gestione della coda dei dati IMU)
            
            // Pubblica lo stato attuale
            publishState();
            
            last_update = now;
        }
        
        // Attendi il prossimo ciclo
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
}

void SensorFusion::handleMqttMessage(const std::string& topic, const std::string& payload) {
    try {
        // TODO: Implementa il parsing dei messaggi MQTT e l'aggiornamento dell'EKF
        std::cout << "Messaggio ricevuto su " << topic << ": " << payload << std::endl;
    } catch (const std::exception& e) {
        std::cerr << "Errore nella gestione del messaggio MQTT: " << e.what() << std::endl;
    }
}

void SensorFusion::publishState() {
    try {
        // Prepara il messaggio JSON con lo stato attuale
        nlohmann::json msg;
        const auto& state = ekf_.getState();
        
        // Posizione
        msg["position"]["x"] = state.position.x();
        msg["position"]["y"] = state.position.y();
        msg["position"]["z"] = state.position.z();
        
        // Velocità
        msg["velocity"]["x"] = state.velocity_body.x();
        msg["velocity"]["y"] = state.velocity_body.y();
        msg["velocity"]["z"] = state.velocity_body.z();
        
        // Orientamento (quaternione)
        msg["orientation"]["w"] = state.orientation.w();
        msg["orientation"]["x"] = state.orientation.x();
        msg["orientation"]["y"] = state.orientation.y();
        msg["orientation"]["z"] = state.orientation.z();
        
        // Timestamp
        auto now = std::chrono::system_clock::now();
        auto duration = now.time_since_epoch();
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
        msg["timestamp"] = millis;
        
        // Pubblica il messaggio
        std::string topic = config_->getString("fusion.mqtt.topics.state", "fusion/state");
        mqtt_client_->publish(topic, msg.dump());
        
    } catch (const std::exception& e) {
        std::cerr << "Errore nella pubblicazione dello stato: " << e.what() << std::endl;
    }
}

} // namespace fusion
