#include "mqtt/mqtt_client.h"
#include <iostream>

namespace mqtt {

MqttClient::MqttClient(const std::string& broker, int port,
                       const std::string& clientId,
                       const std::string& username,
                       const std::string& password)
    : mosq_(nullptr), broker_(broker), port_(port),
      clientId_(clientId), username_(username), password_(password) {
    mosquitto_lib_init();
    mosq_ = mosquitto_new(clientId_.c_str(), true, this);
    if (!mosq_) {
        throw std::runtime_error("Failed to create mosquitto instance");
    }
    mosquitto_connect_callback_set(mosq_, on_connect);
    mosquitto_message_callback_set(mosq_, on_message);
    if (!username_.empty()) {
        mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
    }
}

MqttClient::~MqttClient() {
    if (mosq_) {
        mosquitto_destroy(mosq_);
        mosquitto_lib_cleanup();
    }
}

bool MqttClient::connect() {
    int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "MQTT connect failed: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    rc = mosquitto_loop_start(mosq_);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "MQTT loop start failed: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    return true;
}

bool MqttClient::publish(const std::string& topic,
                         const std::vector<uint8_t>& payload,
                         int qos, bool retain) {
    int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), payload.size(), payload.data(), qos, retain);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "MQTT publish failed: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    return true;
}

bool MqttClient::publish(const std::string& topic,
                        const std::string& payload,
                        int qos, bool retain) {
    return publish(topic, 
                  std::vector<uint8_t>(payload.begin(), payload.end()),
                  qos, retain);
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
    int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "MQTT subscribe failed: " << mosquitto_strerror(rc) << std::endl;
        return false;
    }
    return true;
}

void MqttClient::disconnect() {
    if (mosq_) {
        mosquitto_disconnect(mosq_);
        mosquitto_loop_stop(mosq_, true);
    }
}

void MqttClient::loop(int timeout) {
    if (mosq_ && connected_) {
        mosquitto_loop(mosq_, timeout, 1);
    }
}

void MqttClient::on_connect(struct mosquitto* mosq, void* obj, int rc) {
    if (rc == 0) {
        std::cout << "Connected to MQTT broker\n";
    } else {
        std::cerr << "Unable to connect to MQTT broker, code: " << rc << std::endl;
    }
}

void MqttClient::on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg) {
    std::cout << "Received message on topic " << msg->topic << std::endl;
    // Payload handling to be implemented by user
}

} // namespace mqtt
