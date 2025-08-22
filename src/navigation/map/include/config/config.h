#pragma once

#include <string>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <optional>
#include <nlohmann/json.hpp>

namespace smartmower {
namespace config {

class ConfigLoader {
public:
    explicit ConfigLoader(const std::string& path = "/opt/smartmower/etc/config/robot_config.json")
        : path_(path) {
        std::ifstream ifs(path_);
        if (!ifs.is_open()) {
            throw std::runtime_error("Impossibile aprire il file di configurazione: " + path_);
        }
        try {
            ifs >> cfg_;
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string("Errore parsing JSON config: ") + e.what());
        }
    }

    const nlohmann::json& json() const { return cfg_; }

    std::string rootTopic() const {
        return cfg_.value("mqtt", nlohmann::json{}).value("root_topic", std::string("smartmower"));
    }

    // Costruisce topic tipo root + "/" + base + "/" + sub
    std::optional<std::string> topicFrom(const std::string& key, const std::string& sub = "") const {
        if (!cfg_.contains("mqtt") || !cfg_["mqtt"].contains("topics") || !cfg_["mqtt"]["topics"].contains(key))
            return std::nullopt;
        const auto& t = cfg_["mqtt"]["topics"][key];
        std::string base = t.value("base", "");
        if (base.empty()) return std::nullopt;
        std::string full = rootTopic() + "/" + base;
        if (!sub.empty()) full += "/" + sub;
        return full;
    }

    // Topic noti
    std::string slamMapTopic() const {
        if (auto s = topicFrom("slam", "map")) return *s;
        return rootTopic() + std::string("/slam/map");
    }
    std::string slamPoseTopic() const {
        if (auto s = topicFrom("slam", "pose")) return *s;
        return rootTopic() + std::string("/slam/pose");
    }
    std::string visionObstacleTopic() const {
        if (auto s = topicFrom("obstacle", "data")) return *s;
        return rootTopic() + std::string("/vision/obstacle/data");
    }
    std::string picoDataTopic() const {
        if (auto s = topicFrom("pico", "data")) return *s;
        return rootTopic() + std::string("/bridge/pico/data");
    }
    std::string navCostmapStatusTopic() const {
        if (auto s = topicFrom("nav", "costmap/status")) return *s;
        return rootTopic() + std::string("/nav/costmap/status");
    }

    // Parametri mappa/navigation
    // Prima prova su navigation.map.*, poi fallback a navigation.costmap.* per retrocompatibilità
    double costmapResolutionM() const {
        const auto nav = cfg_.value("navigation", nlohmann::json{});
        if (nav.contains("map")) {
            const auto& m = nav["map"];
            if (m.contains("resolution_m")) return m.value("resolution_m", 0.1);
        }
        return nav.value("costmap", nlohmann::json{}).value("resolution_m", 0.1);
    }
    double inflationRadiusM() const {
        const auto nav = cfg_.value("navigation", nlohmann::json{});
        if (nav.contains("map")) {
            const auto& m = nav["map"];
            if (m.contains("inflation_radius_m")) return m.value("inflation_radius_m", 0.3);
        }
        return nav.value("costmap", nlohmann::json{}).value("inflation_radius_m", 0.3);
    }
    int publishRateHz() const {
        const auto nav = cfg_.value("navigation", nlohmann::json{});
        if (nav.contains("map")) {
            const auto& m = nav["map"];
            if (m.contains("publish_rate_hz")) return m.value("publish_rate_hz", 2);
        }
        return nav.value("costmap", nlohmann::json{}).value("publish_rate_hz", 2);
    }
    double obstacleTimeoutS() const {
        const auto nav = cfg_.value("navigation", nlohmann::json{});
        if (nav.contains("map")) {
            const auto& m = nav["map"];
            if (m.contains("obstacle_timeout_s")) return m.value("obstacle_timeout_s", 5.0);
        }
        return nav.value("costmap", nlohmann::json{}).value("obstacle_timeout_s", 5.0);
    }

private:
    std::string path_;
    nlohmann::json cfg_;
};

} // namespace config
} // namespace smartmower
