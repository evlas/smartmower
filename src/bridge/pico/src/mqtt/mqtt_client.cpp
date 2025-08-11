#include "mqtt/mqtt_client.h"
#include <mosquitto.h>
#include <cstring>
#include <stdexcept>
#include <iostream>
#include <vector>

using namespace std;

namespace mqtt {

// Implementazione dei metodi della classe MqttClient

MqttClient::MqttClient(const string& broker, int port,
                     const string& clientId,
                     const string& username,
                     const string& password)
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
        throw runtime_error("Errore nella creazione dell'istanza mosquitto");
    }
    
    // Configura i callback
    mosquitto_connect_callback_set(mosq_, on_connect);
    mosquitto_message_callback_set(mosq_, on_message);
    
    // Imposta username e password se forniti
    if (!username_.empty()) {
        int rc = mosquitto_username_pw_set(mosq_, username_.c_str(), 
                                         password_.empty() ? nullptr : password_.c_str());
        if (rc != MOSQ_ERR_SUCCESS) {
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
            throw runtime_error("Errore nell'impostazione delle credenziali MQTT");
        }
    }
}

MqttClient::~MqttClient() {
    disconnect();
    if (mosq_) {
        mosquitto_destroy(mosq_);
    }
    mosquitto_lib_cleanup();
}

bool MqttClient::connect() {
    if (connected_) {
        return true;
    }
    
    int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        cerr << "Errore di connessione MQTT: " << mosquitto_strerror(rc) << endl;
        return false;
    }
    
    // Avvia il loop in un thread separato
    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        mosquitto_disconnect(mosq_);
        cerr << "Errore nell'avvio del loop MQTT: " << mosquitto_strerror(rc) << endl;
        return false;
    }
    
    connected_ = true;
    return true;
}

void MqttClient::disconnect() {
    if (mosq_ && connected_) {
        mosquitto_disconnect(mosq_);
        mosquitto_loop_stop(mosq_, false);
        connected_ = false;
    }
}

bool MqttClient::publish(const string& topic, 
                        const vector<uint8_t>& payload,
                        int qos, bool retain) {
    if (!connected_ || !mosq_) {
        return false;
    }
    
    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), 
                              static_cast<int>(payload.size()),
                              payload.data(), qos, retain);
    
    if (rc != MOSQ_ERR_SUCCESS) {
        cerr << "Errore nella pubblicazione MQTT: " << mosquitto_strerror(rc) << endl;
        return false;
    }
    
    return true;
}

bool MqttClient::publish(const string& topic, 
                        const string& payload,
                        int qos, bool retain) {
    vector<uint8_t> data(payload.begin(), payload.end());
    return publish(topic, data, qos, retain);
}

bool MqttClient::subscribe(const string& topic, int qos) {
    if (!connected_ || !mosq_) {
        return false;
    }
    
    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        cerr << "Errore nella sottoscrizione MQTT: " << mosquitto_strerror(rc) << endl;
        return false;
    }
    
    return true;
}

void MqttClient::loop(int timeout) {
    if (mosq_) {
        mosquitto_loop(mosq_, 0, 1);
    }
}

void MqttClient::setMessageCallback(MessageCallback callback) {
    message_callback_ = move(callback);
}

// Implementazione dei callback statici
void MqttClient::on_connect(::mosquitto* mosq, void* obj, int rc) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    if (client) {
        client->handle_connect(rc);
    }
}

void MqttClient::on_message(::mosquitto* mosq, void* obj, 
                           const ::mosquitto_message* msg) {
    MqttClient* client = static_cast<MqttClient*>(obj);
    if (client && msg) {
        client->handle_message(msg);
    }
}

// Implementazione dei metodi virtuali per la gestione degli eventi
void MqttClient::handle_connect(int rc) {
    if (rc == 0) {
        connected_ = true;
        cout << "Connesso al broker MQTT su " << broker_ << ":" << port_ << endl;
    } else {
        connected_ = false;
        cerr << "Errore di connessione MQTT: " << mosquitto_connack_string(rc) << endl;
    }
}

void MqttClient::handle_message(const ::mosquitto_message* msg) {
    if (message_callback_ && msg->payload) {
        string topic = msg->topic;
        vector<uint8_t> payload(
            static_cast<const uint8_t*>(msg->payload),
            static_cast<const uint8_t*>(msg->payload) + msg->payloadlen
        );
        message_callback_(topic, payload);
    }
}

} // namespace mqtt

// Factory function per creare un'istanza del client MQTT
unique_ptr<mqtt::MqttClient> createMqttClient(
    const string& broker, 
    int port,
    const string& clientId,
    const string& username,
    const string& password) {
    
    try {
        return make_unique<mqtt::MqttClient>(broker, port, clientId, username, password);
    } catch (const exception& e) {
        cerr << "Errore nella creazione del client MQTT: " << e.what() << endl;
        return nullptr;
    }
}
