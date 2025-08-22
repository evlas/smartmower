#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

#include <string>
#include <vector>
#include <map>

class MqttTopics {
public:
    // Inizializza i topic base dal file di configurazione
    static void initialize(const std::string& base_topic);
    
    // Restituisce il topic completo per un determinato tipo di messaggio
    static std::string getTopic(const std::string& topic_type);
    
    // Restituisce tutti i topic sottoscritti
    static std::vector<std::string> getSubscribedTopics();
    
    // Tipi di topic supportati
    static constexpr const char* TOPIC_DATA = "data";
    static constexpr const char* TOPIC_STATUS = "status";
    static constexpr const char* TOPIC_COMMANDS = "commands";
    
private:
    static std::string base_topic_;
    static std::map<std::string, std::string> topics_;
};

#endif // MQTT_TOPICS_H
