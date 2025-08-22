#pragma once

#include <string>
#include <vector>
#include <functional>
#include <iostream>
#include <cstdint>

namespace smartmower {
namespace mqtt {

class MqttClient {
public:
    using MessageCallback = std::function<void(const std::string& topic, const std::vector<uint8_t>& payload)>;

    MqttClient(const std::string& broker, int port,
               const std::string& clientId,
               const std::string& username = "",
               const std::string& password = "");
    ~MqttClient();

    bool connect();
    void disconnect();

    bool publish(const std::string& topic, const std::string& payload, int qos = 1, bool retain = false);
    bool publish(const std::string& topic, const std::vector<uint8_t>& payload, int qos = 1, bool retain = false);

    bool subscribe(const std::string& topic, int qos = 1);

    void loop(int timeout_ms = 0);

    void setMessageCallback(MessageCallback cb) { msg_cb_ = std::move(cb); }

private:
    std::string broker_;
    int port_ = 1883;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_ = false;

    MessageCallback msg_cb_ = nullptr;
    void* impl_ = nullptr; // puntatore opaco all'implementazione
};

} // namespace mqtt
} // namespace smartmower
