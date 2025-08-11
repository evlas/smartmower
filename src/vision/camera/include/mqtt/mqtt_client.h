#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <mosquitto.h>
#include <vector>

namespace mqtt {

class MqttClient {
public:
    MqttClient(const std::string& broker, int port,
               const std::string& clientId,
               const std::string& username = "",
               const std::string& password = "");
    ~MqttClient();

    bool connect();
    bool publish(const std::string& topic,
                 const std::vector<uint8_t>& payload,
                 int qos = 1, bool retain = false);
    bool publish(const std::string& topic,
                 const std::string& payload,
                 int qos = 1, bool retain = false);
    bool subscribe(const std::string& topic, int qos = 1);
    void disconnect();
    
    // Aggiunto per gestire gli eventi MQTT
    void loop(int timeout = 0);

private:
    struct mosquitto* mosq_;
    std::string broker_;
    int port_;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_;
    static void on_connect(struct mosquitto* mosq, void* obj, int rc);
    static void on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);
};

} // namespace mqtt

#endif // MQTT_CLIENT_H
