#ifndef VISION_OBSTACLE_MQTT_TOPICS_H
#define VISION_OBSTACLE_MQTT_TOPICS_H

#include <string>
#include <unordered_map>
#include <stdexcept>

namespace mqtt { namespace topics {

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
        topics_.clear();
        for (const auto& kv : subtopics) {
            topics_[kv.first] = root_ + base_ + "/" + ensureNoLeadingSlash(kv.second);
        }
    }

    const std::string& get(const std::string& sub) const {
        auto it = topics_.find(sub);
        if (it == topics_.end()) throw std::runtime_error("Unknown topic: " + sub);
        return it->second;
    }

    const std::string& data() const { return get("data"); }
    const std::string& status() const { return get("status"); }
    const std::string& commands() const { return get("commands"); }

private:
    TopicManager() = default;
    std::string root_;
    std::string base_;
    std::unordered_map<std::string, std::string> topics_;

    static std::string ensureTrailingSlash(const std::string& s) {
        return (s.empty() || s.back() == '/') ? s : s + "/";
    }
    static std::string ensureNoTrailingSlash(const std::string& s) {
        return (!s.empty() && s.back() == '/') ? s.substr(0, s.size()-1) : s;
    }
    static std::string ensureNoLeadingSlash(const std::string& s) {
        return (!s.empty() && s.front() == '/') ? s.substr(1) : s;
    }
};

}} // namespace mqtt::topics

#endif // VISION_OBSTACLE_MQTT_TOPICS_H
