#ifndef MQTT_CLIENT_H
#define MQTT_CLIENT_H

#include <string>
#include <vector>
#include <functional>
#include <mosquitto.h>

// Forward declarations per evitare conflitti di namespace
struct mosquitto;
struct mosquitto_message;

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
    
    // Gestione callback messaggi
    using MessageCallback = std::function<void(const std::string&, const std::vector<uint8_t>&)>;
    
    /**
     * @brief Imposta il callback per la ricezione dei messaggi
     * @param callback Funzione da chiamare quando arriva un messaggio
     */
    void setMessageCallback(MessageCallback callback) {
        message_callback_ = std::move(callback);
    }
    
    // Callback per i messaggi in arrivo
    MessageCallback message_callback_;
    
    // Metodi di supporto per i callback (pubblici per i callback C)
    void handleConnect(int rc);
    void handleMessage(const struct mosquitto_message* msg);

private:
    struct mosquitto* mosq_;
    std::string broker_;
    int port_;
    std::string clientId_;
    std::string username_;
    std::string password_;
    bool connected_;
};

} // namespace mqtt

#endif // MQTT_CLIENT_H
