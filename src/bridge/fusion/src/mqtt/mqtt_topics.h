#ifndef FUSION_MQTT_TOPICS_H
#define FUSION_MQTT_TOPICS_H

#include <string>
#include <unordered_map>

namespace fusion {
namespace mqtt {

/**
 * @brief Gestisce i topic MQTT per il modulo di fusione
 */
class MqttTopics {
public:
    /**
     * @brief Costruttore
     * @param base_topic Topic base per tutti i messaggi (es. "smartmower/fusion")
     */
    explicit MqttTopics(const std::string& base_topic);
    
    /**
     * @brief Restituisce il topic per la pubblicazione dello stato
     */
    std::string getStateTopic() const;
    
    /**
     * @brief Restituisce il topic per la pubblicazione dei dati di debug
     */
    std::string getDebugTopic() const;
    
    /**
     * @brief Restituisce il topic per la pubblicazione degli errori
     */
    std::string getErrorTopic() const;
    
    /**
     * @brief Restituisce il topic per la sottoscrizione ai comandi
     */
    std::string getCommandTopic() const;
    
    /**
     * @brief Restituisce il topic per la sottoscrizione ai dati IMU
     */
    std::string getImuTopic() const;
    
    /**
     * @brief Restituisce il topic per la sottoscrizione ai dati GPS
     */
    std::string getGpsTopic() const;
    
    /**
     * @brief Restituisce il topic per la sottoscrizione ai dati odometrici
     */
    std::string getOdomTopic() const;
    
    /**
     * @brief Imposta i sottotopic personalizzati
     * @param subtopics Mappa di sottotopic personalizzati
     */
    void setCustomSubtopics(const std::unordered_map<std::string, std::string>& subtopics);
    
private:
    std::string base_topic_;
    
    // Sottotopic predefiniti
    struct {
        std::string state = "state";
        std::string debug = "debug";
        std::string error = "error";
        std::string command = "command";
        std::string imu = "sensor/imu";
        std::string gps = "sensor/gps";
        std::string odom = "sensor/odom";
    } subtopics_;
};

} // namespace mqtt
} // namespace fusion

#endif // FUSION_MQTT_TOPICS_H
