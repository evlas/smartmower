#ifndef MQTT_TOPICS_H
#define MQTT_TOPICS_H

#include <string>
#include <unordered_map>
#include <stdexcept>

namespace mqtt {
namespace topics {

class TopicManager {
public:
    static TopicManager& getInstance() {
        static TopicManager instance;
        return instance;
    }

    void initialize(const std::string& rootTopic, 
                   const std::string& baseTopic,
                   const std::unordered_map<std::string, std::string>& subtopics) {
        root_ = ensureTrailingSlash(rootTopic);
        base_ = ensureNoTrailingSlash(baseTopic);
        
        for (const auto& [key, value] : subtopics) {
            topics_[key] = root_ + base_ + "/" + ensureNoLeadingSlash(value);
        }
    }

    const std::string& get(const std::string& subtopic) const {
        auto it = topics_.find(subtopic);
        if (it == topics_.end()) {
            throw std::runtime_error("Unknown topic: " + subtopic);
        }
        return it->second;
    }

    // Helper methods
    const std::string& data() const { return get("data"); }
    const std::string& status() const { return get("status"); }
    const std::string& commands() const { return get("commands"); }

private:
    TopicManager() = default;
    std::string root_;
    std::string base_;
    std::unordered_map<std::string, std::string> topics_;

    static std::string ensureTrailingSlash(const std::string& s) {
        if (s.empty() || s.back() != '/') return s + '/';
        return s;
    }

    static std::string ensureNoTrailingSlash(const std::string& s) {
        if (!s.empty() && s.back() == '/') return s.substr(0, s.length() - 1);
        return s;
    }

    static std::string ensureNoLeadingSlash(const std::string& s) {
        if (!s.empty() && s.front() == '/') return s.substr(1);
        return s;
    }
};

} // namespace topics
} // namespace mqtt

#endif // MQTT_TOPICS_H
