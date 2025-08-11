#include "mqtt/mqtt_topics.h"
#include <stdexcept>

using namespace std;

// Inizializza le variabili statiche
string MqttTopics::base_topic_;
map<string, string> MqttTopics::topics_;

void MqttTopics::initialize(const string& base_topic) {
    base_topic_ = base_topic;
    
    // Rimuovi eventuali slash finali
    while (!base_topic_.empty() && base_topic_.back() == '/') {
        base_topic_.pop_back();
    }
    
    // Inizializza i topic standard
    topics_[TOPIC_DATA] = base_topic_ + "/data";
    topics_[TOPIC_STATUS] = base_topic_ + "/status";
    topics_[TOPIC_COMMANDS] = base_topic_ + "/commands";
}

string MqttTopics::getTopic(const string& topic_type) {
    auto it = topics_.find(topic_type);
    if (it != topics_.end()) {
        return it->second;
    }
    
    // Se il topic non esiste, lo crea dinamicamente
    string topic = base_topic_ + "/" + topic_type;
    topics_[topic_type] = topic;
    return topic;
}

vector<string> MqttTopics::getSubscribedTopics() {
    vector<string> result;
    
    // Aggiungi i topic di comando a cui vogliamo sottoscriverci
    result.push_back(getTopic(TOPIC_COMMANDS) + "/#"); // Sottoscrizione a tutti i sottotopic di commands
    
    return result;
}
