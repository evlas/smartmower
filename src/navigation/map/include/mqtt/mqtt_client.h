#pragma once

#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <atomic>
#include <mutex>
#include <mosquitto.h>

// Implementazione di un client MQTT minimale basato su libmosquitto.
// Interfaccia compatibile con l'uso nel Map Server.
namespace smartmower {
namespace mqtt {

class MqttClient {
public:
    using MessageCallback = std::function<void(const std::string& topic, const std::vector<uint8_t>& payload)>;

    MqttClient(const std::string& broker, int port,
               const std::string& clientId,
               const std::string& username = "",
               const std::string& password = "")
        : broker_(broker), port_(port), clientId_(clientId), username_(username), password_(password) {}

    bool connect() {
        ensureLibInit();
        std::lock_guard<std::mutex> lk(m_);
        if (mosq_) return connected_;
        mosq_ = mosquitto_new(clientId_.c_str(), true, this);
        if (!mosq_) {
            std::cerr << "[MQTT] mosquitto_new failed" << std::endl;
            return false;
        }
        if (!username_.empty()) {
            int rc = mosquitto_username_pw_set(mosq_, username_.c_str(), password_.empty() ? nullptr : password_.c_str());
            if (rc != MOSQ_ERR_SUCCESS) {
                std::cerr << "[MQTT] username_pw_set failed: " << mosquitto_strerror(rc) << std::endl;
                return false;
            }
        }
        mosquitto_connect_callback_set(mosq_, &MqttClient::on_connect_trampoline);
        mosquitto_message_callback_set(mosq_, &MqttClient::on_message_trampoline);
        int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] connect failed: " << mosquitto_strerror(rc) << " (" << broker_ << ":" << port_ << ")" << std::endl;
            mosquitto_destroy(mosq_); mosq_ = nullptr;
            connected_ = false;
            return false;
        }
        connected_ = true;
        return true;
    }
    void disconnect() {
        std::lock_guard<std::mutex> lk(m_);
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
        }
        connected_ = false;
    }

    bool publish(const std::string& topic, const std::string& payload, int qos = 1, bool retain = false) {
        std::lock_guard<std::mutex> lk(m_);
        if (!mosq_ || !connected_) return false;
        int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)payload.size(), payload.data(), qos, retain);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] publish failed: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        return true;
    }
    bool publish(const std::string& topic, const std::vector<uint8_t>& payload, int qos = 1, bool retain = false) {
        std::lock_guard<std::mutex> lk(m_);
        if (!mosq_ || !connected_) return false;
        int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)payload.size(), payload.data(), qos, retain);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] publish failed: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        return true;
    }

    bool subscribe(const std::string& topic, int qos = 1) {
        std::lock_guard<std::mutex> lk(m_);
        if (!mosq_ || !connected_) return false;
        int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] subscribe failed: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        return true;
    }

    void loop(int timeout_ms = 0) {
        std::lock_guard<std::mutex> lk(m_);
        if (!mosq_) return;
        int rc = mosquitto_loop(mosq_, timeout_ms, 1);
        if (rc == MOSQ_ERR_CONN_LOST) {
            connected_ = false;
        }
        if (!connected_) {
            int rrc = mosquitto_reconnect(mosq_);
            if (rrc == MOSQ_ERR_SUCCESS) {
                connected_ = true;
            }
        }
    }

    void setMessageCallback(MessageCallback cb) { msg_cb_ = std::move(cb); }

private:
    std::string broker_;
    int port_ = 1883;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_ = false;

    std::mutex m_;
    struct mosquitto* mosq_ = nullptr;

    MessageCallback msg_cb_ = nullptr;

    static void ensureLibInit() {
        static std::atomic<bool> inited{false};
        if (!inited.load()) {
            mosquitto_lib_init();
            inited.store(true);
        }
    }
    static void on_connect_trampoline(struct mosquitto* /*mosq*/, void* userdata, int rc) {
        auto* self = static_cast<MqttClient*>(userdata);
        if (!self) return;
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] on_connect error: " << mosquitto_strerror(rc) << std::endl;
        }
    }
    static void on_message_trampoline(struct mosquitto* /*mosq*/, void* userdata, const struct mosquitto_message* msg) {
        auto* self = static_cast<MqttClient*>(userdata);
        if (!self || !self->msg_cb_) return;
        std::vector<uint8_t> payload;
        if (msg->payloadlen > 0 && msg->payload) {
            const uint8_t* p = static_cast<const uint8_t*>(msg->payload);
            payload.assign(p, p + msg->payloadlen);
        }
        self->msg_cb_(msg->topic ? msg->topic : std::string(), payload);
    }
};

} // namespace mqtt
} // namespace smartmower
