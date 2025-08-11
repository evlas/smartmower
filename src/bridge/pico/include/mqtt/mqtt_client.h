#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <functional>
#include <memory>
#include <vector>
#include <mosquitto.h>

// Forward declarations per evitare conflitti di namespace
struct mosquitto;
struct mosquitto_message;

namespace mqtt {

class MqttClient {
public:
    using MessageCallback = std::function<void(const std::string& topic, const std::vector<uint8_t>& payload)>;

    MqttClient(const std::string& broker, int port,
               const std::string& clientId,
               const std::string& username = "",
               const std::string& password = "");
    virtual ~MqttClient();

    // Connessione al broker
    bool connect();
    
    // Pubblicazione messaggi
    bool publish(const std::string& topic,
                const std::vector<uint8_t>& payload,
                int qos = 1, bool retain = false);
                
    bool publish(const std::string& topic,
                const std::string& payload,
                int qos = 1, bool retain = false);
    
    // Sottoscrizione a topic
    bool subscribe(const std::string& topic, int qos = 1);
    
    // Disconnessione dal broker
    void disconnect();
    
    // Gestione messaggi in arrivo
    void loop(int timeout = 0);
    
    // Imposta il callback per i messaggi in arrivo
    void setMessageCallback(MessageCallback callback);

protected:
    struct mosquitto* mosq_;
    std::string broker_;
    int port_;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_;
    MessageCallback message_callback_;
    
    // Callback statici
    static void on_connect(struct mosquitto* mosq, void* obj, int rc);
    static void on_message(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);
    
    // Metodi virtuali per l'override nelle classi derivate
    virtual void handle_connect(int rc);
    virtual void handle_message(const struct ::mosquitto_message* msg);

};

} // namespace mqtt

// Factory function per creare un'istanza del client MQTT
std::unique_ptr<mqtt::MqttClient> createMqttClient(
    const std::string& broker, 
    int port,
    const std::string& clientId,
    const std::string& username = "",
    const std::string& password = "");

#endif // MQTT_CLIENT_H
