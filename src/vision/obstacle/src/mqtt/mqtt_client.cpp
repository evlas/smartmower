#include "mqtt/mqtt_client.h"
#include <cstring>

namespace mqtt {

MqttClient::MqttClient(const std::string& broker, int port,
                       const std::string& clientId,
                       const std::string& username,
                       const std::string& password)
: broker_(broker), port_(port), clientId_(clientId), username_(username), password_(password) {
    mosquitto_lib_init();
    mosq_ = mosquitto_new(clientId_.c_str(), true, this);
    if (!username_.empty()) {
        mosquitto_username_pw_set(mosq_, username_.c_str(), password_.c_str());
    }
    mosquitto_connect_callback_set(mosq_, &MqttClient::on_connect);
    mosquitto_message_callback_set(mosq_, &MqttClient::on_message);
}

MqttClient::~MqttClient() {
    if (mosq_) {
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    mosquitto_lib_cleanup();
}

bool MqttClient::connect() {
    int rc = mosquitto_connect(mosq_, broker_.c_str(), port_, 60);
    connected_ = (rc == MOSQ_ERR_SUCCESS);
    return connected_;
}

void MqttClient::disconnect() {
    if (mosq_) mosquitto_disconnect(mosq_);
    connected_ = false;
}

bool MqttClient::publish(const std::string& topic, const std::string& payload, int qos, bool retain) {
    return mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)payload.size(), payload.data(), qos, retain) == MOSQ_ERR_SUCCESS;
}

bool MqttClient::publish(const std::string& topic, const std::vector<uint8_t>& payload, int qos, bool retain) {
    return mosquitto_publish(mosq_, nullptr, topic.c_str(), (int)payload.size(), payload.data(), qos, retain) == MOSQ_ERR_SUCCESS;
}

bool MqttClient::subscribe(const std::string& topic, int qos) {
    return mosquitto_subscribe(mosq_, nullptr, topic.c_str(), qos) == MOSQ_ERR_SUCCESS;
}

void MqttClient::loop(int timeout_ms) {
    mosquitto_loop(mosq_, timeout_ms, 1);
}

void MqttClient::setMessageCallback(MessageCallback cb, void* user) {
    msg_cb_ = cb;
    msg_user_ = user;
}

void MqttClient::on_connect(struct mosquitto* mosq, void* obj, int rc) {
    (void)mosq; (void)rc;
    auto* self = static_cast<MqttClient*>(obj);
    self->connected_ = (rc == 0);
}

void MqttClient::on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg) {
    (void)mosq;
    auto* self = static_cast<MqttClient*>(obj);
    if (!self->msg_cb_) return;
    std::vector<uint8_t> payload;
    if (msg->payload && msg->payloadlen > 0) {
        payload.resize((size_t)msg->payloadlen);
        std::memcpy(payload.data(), msg->payload, (size_t)msg->payloadlen);
    }
    self->msg_cb_(msg->topic ? msg->topic : std::string(), payload, self->msg_user_);
}

} // namespace mqtt
