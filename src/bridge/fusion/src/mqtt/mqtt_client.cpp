#include "mqtt/mqtt_client.h"
#include <mosquitto.h>
#include <cstring>
#include <iostream>

namespace fusion {
namespace mqtt {

// Implementazione dei metodi di supporto per i callback
void MqttClient::handleConnect(int rc) {
    setConnected(rc == 0);
    if (isConnected()) {
        std::cout << "Connesso al broker MQTT" << std::endl;
    } else {
        std::cerr << "Errore di connessione: " << mosquitto_connack_string(rc) << std::endl;
    }
}

void MqttClient::handleMessage(const struct mosquitto_message* msg) {
    if (msg->payload && msg->payloadlen > 0) {
        std::vector<uint8_t> payload(
            static_cast<const uint8_t*>(msg->payload),
            static_cast<const uint8_t*>(msg->payload) + msg->payloadlen
        );
        if (message_callback_) {
            message_callback_(msg->topic, payload);
        }
    }
}

// Callback per la connessione
void MqttClient::on_connect_wrapper(struct mosquitto* mosq, void* obj, int rc) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    client->handleConnect(rc);
}

// Callback per i messaggi in arrivo
void MqttClient::on_message_wrapper(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    client->handleMessage(msg);
}

MqttClient::MqttClient(const std::string& broker, int port,
                     const std::string& clientId,
                     const std::string& username,
                     const std::string& password)
    : mosq_(nullptr),
      broker_(broker),
      port_(port),
      client_id_(clientId),
      username_(username),
      password_(password),
      connected_(false),
      message_callback_(nullptr) {
    
    mosquitto_lib_init();
    mosq_ = mosquitto_new(client_id_.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("Errore nella creazione dell'istanza mosquitto");
    }
    
    // Configura i callback
    mosquitto_connect_callback_set(mosq_, &MqttClient::on_connect_wrapper);
    mosquitto_message_callback_set(mosq_, &MqttClient::on_message_wrapper);
    
    // Imposta username e password se forniti
    if (!username_.empty()) {
        mosquitto_username_pw_set(mosq_, username_.c_str(), 
                                password_.empty() ? nullptr : password_.c_str());
    }
}

MqttClient::~MqttClient() {
    if (connected_) {
        disconnect();
    }
    if (mosq_) {
        mosquitto_destroy(mosq_);
    }
    mosquitto_lib_cleanup();
}

bool MqttClient::connect() {
    if (!mosq_) return false;
    
    int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore di connessione: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    
    // Avvia il thread di loop MQTT
    mosquitto_loop_start(mosq_);
    connected_ = true;
    return true;
}

bool MqttClient::publish(const std::string& topic,
                        const std::vector<uint8_t>& payload,
                        int qos, bool retain) {
    if (!connected_ || !mosq_) return false;
    
    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), 
                              payload.size(), payload.data(),
                              qos, retain);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore nell'invio del messaggio: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    return true;
}

bool MqttClient::publish(const std::string& topic,
                        const std::string& payload,
                        int qos, bool retain) {
    std::vector<uint8_t> data(payload.begin(), payload.end());
    return publish(topic, data, qos, retain);
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
    if (!connected_ || !mosq_) return false;
    
    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore nella sottoscrizione: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    return true;
}

void MqttClient::disconnect() {
    if (connected_ && mosq_) {
        mosquitto_disconnect(mosq_);
        connected_ = false;
    }
}

void MqttClient::loop(int timeout) {
    if (mosq_) {
        mosquitto_loop(mosq_, 0, 1);
    }
}

} // namespace mqtt
} // namespace fusion
