#pragma once

#include <string>
#include <functional>

namespace sm {

class MqttClient {
public:
    using MessageHandler = std::function<void(const std::string& topic, const std::string& payload)>;

    virtual ~MqttClient() = default;

    virtual bool connect(const std::string& host, int port,
                         const std::string& username = {},
                         const std::string& password = {}) = 0;

    virtual void disconnect() = 0;

    virtual bool publish(const std::string& topic, const std::string& payload, int qos = 0, bool retain = false) = 0;

    virtual bool subscribe(const std::string& topic, int qos = 0) = 0;

    virtual void setMessageHandler(MessageHandler handler) = 0;
};

MqttClient* createMqttClient();

} // namespace sm
