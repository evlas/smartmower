#ifndef VISION_OBSTACLE_MQTT_CLIENT_H
#define VISION_OBSTACLE_MQTT_CLIENT_H

#include <string>
#include <vector>
#include <mosquitto.h>

namespace mqtt {

class MqttClient {
public:
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

    using MessageCallback = void(*)(const std::string& topic, const std::vector<uint8_t>& payload, void* user);
    void setMessageCallback(MessageCallback cb, void* user);

private:
    static void on_connect(struct mosquitto* mosq, void* obj, int rc);
    static void on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);

    struct mosquitto* mosq_ = nullptr;
    std::string broker_;
    int port_ = 1883;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_ = false;

    MessageCallback msg_cb_ = nullptr;
    void* msg_user_ = nullptr;
};

} // namespace mqtt

#endif // VISION_OBSTACLE_MQTT_CLIENT_H
