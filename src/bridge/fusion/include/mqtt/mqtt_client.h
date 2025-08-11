#ifndef FUSION_MQTT_CLIENT_H
#define FUSION_MQTT_CLIENT_H

#include <string>
#include <vector>
#include <functional>
#include <mosquitto.h>

// Forward declarations per evitare conflitti di namespace
struct mosquitto;
struct mosquitto_message;

namespace fusion {
namespace mqtt {

class MqttClient {
public:
    MqttClient(const std::string& broker, int port,
               const std::string& clientId,
               const std::string& username = "",
               const std::string& password = "");
    ~MqttClient();

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

    // Metodi e membri pubblici accessibili dai callback
    bool isConnected() const { return connected_; }
    void setConnected(bool connected) { connected_ = connected; }
    
    // Imposta il gestore dei messaggi in arrivo
    template<typename F>
    void setMessageHandler(F&& handler) {
        message_callback_ = [handler](const std::string& topic, const std::vector<uint8_t>& payload) {
            handler(topic, payload);
        };
    }
    
    // Metodi di supporto per i callback (pubblici per i callback C)
    void handleConnect(int rc);
    void handleMessage(const struct mosquitto_message* msg);

private:
    // Metodi di callback statici
    static void on_connect_wrapper(struct mosquitto* mosq, void* obj, int rc);
    static void on_message_wrapper(struct mosquitto* mosq, void* obj, const struct mosquitto_message* msg);
    
    // Istanza di mosquitto
    struct mosquitto* mosq_;
    
    // Dati di connessione
    std::string broker_;
    int port_;
    std::string client_id_;
    std::string username_;
    std::string password_;
    
    // Stato della connessione
    bool connected_;
    
    // Callback per i messaggi in arrivo
    std::function<void(const std::string&, const std::vector<uint8_t>&)> message_callback_;
};

} // namespace mqtt
} // namespace fusion

#endif // FUSION_MQTT_CLIENT_H
