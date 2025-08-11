#ifndef FUSION_MQTT_TOPICS_H
#define FUSION_MQTT_TOPICS_H

#include <string>
#include <unordered_map>
#include <stdexcept>

namespace fusion {
namespace mqtt {

/**
 * @brief Gestisce i topic MQTT per il modulo di fusione
 * 
 * La classe gestisce la costruzione dei topic MQTT nel formato:
 *   <root_topic>/<base_topic>/<subtopic>
 * 
 * Dove:
 * - root_topic: prefisso globale dell'applicazione (es. "smartmower")
 * - base_topic: prefisso specifico del modulo (es. "fusion")
 * - subtopic: tipo specifico di messaggio (es. "state", "command", etc.)
 */
class MqttTopics {
public:
    /**
     * @brief Costruttore
     * @param root_topic Topic root globale (es. "smartmower")
     * @param base_topic Topic base del modulo (es. "fusion")
     */
    explicit MqttTopics(const std::string& root_topic, const std::string& base_topic);
    
    /**
     * @brief Restituisce il topic completo per il sottotopic specificato
     * @param subtopic Sottotopic da utilizzare
     * @return std::string Topic completo nel formato <root_topic>/<base_topic>/<subtopic>
     */
    std::string getFullTopic(const std::string& subtopic) const;
    
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
    
    /**
     * @brief Imposta il root topic
     * @param root_topic Nuovo root topic
     */
    void setRootTopic(const std::string& root_topic);
    
    /**
     * @brief Imposta il base topic
     * @param base_topic Nuovo base topic
     */
    void setBaseTopic(const std::string& base_topic);
    
private:
    std::string root_topic_;
    std::string base_topic_;
    
    // Sottotopic predefiniti
    struct {
        std::string state = "state";
        std::string debug = "debug";
        std::string error = "error";
        std::string command = "commands";
        std::string imu = "sensor/imu";
        std::string gps = "sensor/gps";
        std::string odom = "sensor/odom";
    } subtopics_;
    
    /**
     * @brief Pulisce un topic rimuovendo gli slash iniziali/finali
     * @param topic Topic da pulire
     * @return std::string Topic pulito
     */
    std::string cleanTopic(const std::string& topic) const;
};

} // namespace mqtt
} // namespace fusion

#endif // FUSION_MQTT_TOPICS_H
