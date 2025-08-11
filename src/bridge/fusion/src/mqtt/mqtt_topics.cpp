#include "mqtt/mqtt_topics.h"
#include <sstream>
#include <stdexcept>
#include <algorithm>

using namespace std::string_literals;

namespace fusion {
namespace mqtt {

MqttTopics::MqttTopics(const std::string& root_topic, const std::string& base_topic)
    : root_topic_(root_topic), base_topic_(base_topic) {
    // Pulisci i topic all'inizializzazione
    root_topic_ = cleanTopic(root_topic_);
    base_topic_ = cleanTopic(base_topic_);
}

std::string MqttTopics::cleanTopic(const std::string& topic) const {
    if (topic.empty()) {
        return "";
    }
    
    // Rimuovi slash iniziali
    size_t start = topic.find_first_not_of("/");
    if (start == std::string::npos) {
        return "";
    }
    
    // Rimuovi slash finali
    size_t end = topic.find_last_not_of("/");
    
    return topic.substr(start, end - start + 1);
}

std::string MqttTopics::getFullTopic(const std::string& subtopic) const {
    std::string clean_subtopic = cleanTopic(subtopic);
    if (clean_subtopic.empty()) {
        throw std::invalid_argument("Sottotopic non può essere vuoto");
    }
    
    std::string result;
    
    // Aggiungi root_topic se non vuoto
    if (!root_topic_.empty()) {
        result = root_topic_ + "/";
    }
    
    // Aggiungi base_topic se non vuoto
    if (!base_topic_.empty()) {
        result += base_topic_ + "/";
    }
    
    // Aggiungi il sottotopic
    result += clean_subtopic;
    
    return result;
}

std::string MqttTopics::getStateTopic() const {
    return getFullTopic(subtopics_.state);
}

std::string MqttTopics::getDebugTopic() const {
    return getFullTopic(subtopics_.debug);
}

std::string MqttTopics::getErrorTopic() const {
    return getFullTopic(subtopics_.error);
}

std::string MqttTopics::getCommandTopic() const {
    return getFullTopic(subtopics_.command);
}

std::string MqttTopics::getImuTopic() const {
    return getFullTopic(subtopics_.imu);
}

std::string MqttTopics::getGpsTopic() const {
    return getFullTopic(subtopics_.gps);
}

std::string MqttTopics::getOdomTopic() const {
    return getFullTopic(subtopics_.odom);
}

void MqttTopics::setCustomSubtopics(const std::unordered_map<std::string, std::string>& subtopics) {
    for (const auto& [key, value] : subtopics) {
        if (key == "state") subtopics_.state = cleanTopic(value);
        else if (key == "debug") subtopics_.debug = cleanTopic(value);
        else if (key == "error") subtopics_.error = cleanTopic(value);
        else if (key == "command" || key == "commands") subtopics_.command = cleanTopic(value);
        else if (key == "imu") subtopics_.imu = cleanTopic(value);
        else if (key == "gps") subtopics_.gps = cleanTopic(value);
        else if (key == "odom") subtopics_.odom = cleanTopic(value);
    }
}

void MqttTopics::setRootTopic(const std::string& root_topic) {
    root_topic_ = cleanTopic(root_topic);
}

void MqttTopics::setBaseTopic(const std::string& base_topic) {
    base_topic_ = cleanTopic(base_topic);
}

} // namespace mqtt
} // namespace fusion
