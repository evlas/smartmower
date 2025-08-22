#include "mqtt/mqtt_client.h"
#include <mosquitto.h>
#include <cstring>
#include <iostream>

namespace mqtt {

// Implementazione dei metodi di supporto per i callback
void MqttClient::handleConnect(int rc) {
    setConnected(rc == 0);
    if (!isConnected()) {
        std::cerr << "Errore di connessione: " << mosquitto_connack_string(rc) << std::endl;
    }
}

void MqttClient::handleMessage(const struct mosquitto_message* msg) {
    if (msg->payload && msg->payloadlen > 0 && message_callback_) {
        std::vector<uint8_t> payload(
            static_cast<const uint8_t*>(msg->payload),
            static_cast<const uint8_t*>(msg->payload) + msg->payloadlen
        );
        message_callback_(std::string(msg->topic), payload);
    }
}

// Callback per la connessione
static void on_connect(struct mosquitto* mosq, void* obj, int rc) {
    (void)mosq; // parametro non usato
    MqttClient* client = static_cast<MqttClient*>(obj);
    client->handleConnect(rc);
}

// Callback per i messaggi in arrivo
static void on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg) {
    (void)mosq; // parametro non usato
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
      clientId_(clientId),
      username_(username),
      password_(password),
      connected_(false) {
    
    mosquitto_lib_init();
    mosq_ = mosquitto_new(clientId_.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("Errore nella creazione dell'istanza mosquitto");
    }
    
    // Configura i callback
    mosquitto_connect_callback_set(mosq_, on_connect);
    mosquitto_message_callback_set(mosq_, on_message);
    
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
    
    std::cout << "[MQTT] Connecting to " << broker_ << ":" << port_ 
              << " as '" << clientId_ << "'..." << std::endl;
    int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore di connessione: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    // Non avviare un thread interno: il loop viene gestito dal chiamante
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
    if (mosq_ && connected_) {
        mosquitto_disconnect(mosq_);
        connected_ = false;
    }
}

void MqttClient::loop(int timeout) {
    if (mosq_) {
        mosquitto_loop(mosq_, timeout, 1);
    }
}

} // namespace mqtt
