#include "mqtt/mqtt_client.h"
#include <mosquitto.h>
#include <iostream>
#include <mutex>

namespace sm {

class MosquittoMqttClient : public MqttClient {
public:
    MosquittoMqttClient() {
        mosquitto_lib_init();
        mosq_ = mosquitto_new(nullptr, true, this);
        if (!mosq_) {
            std::cerr << "[MQTT] mosquitto_new failed" << std::endl;
        }
        mosquitto_connect_callback_set(mosq_, &MosquittoMqttClient::on_connect_static);
        mosquitto_message_callback_set(mosq_, &MosquittoMqttClient::on_message_static);
    }

    ~MosquittoMqttClient() override {
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_loop_stop(mosq_, true);
            mosquitto_destroy(mosq_);
        }
        mosquitto_lib_cleanup();
    }

    bool connect(const std::string& host, int port,
                 const std::string& username,
                 const std::string& password) override {
        if (!mosq_) return false;
        if (!username.empty()) {
            int rc = mosquitto_username_pw_set(mosq_, username.c_str(), password.empty() ? nullptr : password.c_str());
            if (rc != MOSQ_ERR_SUCCESS) {
                std::cerr << "[MQTT] username_pw_set error: " << rc << std::endl;
            }
        }
        int rc = mosquitto_connect(mosq_, host.c_str(), port, 60);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] connect failed: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        mosquitto_loop_start(mosq_);
        return true;
    }

    void disconnect() override {
        if (mosq_) {
            mosquitto_disconnect(mosq_);
            mosquitto_loop_stop(mosq_, true);
        }
    }

    bool publish(const std::string& topic, const std::string& payload, int qos, bool retain) override {
        if (!mosq_) return false;
        int rc = mosquitto_publish(mosq_, nullptr, topic.c_str(), static_cast<int>(payload.size()), payload.data(), qos, retain);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] publish error: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        return true;
    }

    bool subscribe(const std::string& topic, int qos) override {
        if (!mosq_) return false;
        int rc = mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[MQTT] subscribe error: " << mosquitto_strerror(rc) << std::endl;
            return false;
        }
        return true;
    }

    void setMessageHandler(MessageHandler handler) override {
        std::lock_guard<std::mutex> lk(cb_mtx_);
        handler_ = std::move(handler);
    }

private:
    static void on_connect_static(struct mosquitto* m, void* obj, int rc) {
        (void)m; (void)obj;
        if (rc == 0) {
            std::cout << "[MQTT] connected" << std::endl;
        } else {
            std::cerr << "[MQTT] connect rc=" << rc << std::endl;
        }
    }

    static void on_message_static(struct mosquitto* m, void* obj, const struct mosquitto_message* msg) {
        (void)m;
        auto* self = static_cast<MosquittoMqttClient*>(obj);
        if (!self) return;
        std::lock_guard<std::mutex> lk(self->cb_mtx_);
        if (self->handler_) {
            std::string topic = msg->topic ? msg->topic : "";
            std::string payload;
            if (msg->payload && msg->payloadlen > 0) payload.assign(static_cast<const char*>(msg->payload), msg->payloadlen);
            self->handler_(topic, payload);
        }
    }

    struct mosquitto* mosq_ {nullptr};
    MessageHandler handler_;
    std::mutex cb_mtx_;
};

MqttClient* createMqttClient() { return new MosquittoMqttClient(); }

} // namespace sm
