#ifndef OBSTACLE_CONFIG_MANAGER_H
#define OBSTACLE_CONFIG_MANAGER_H

#include <string>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace config {

class ConfigManager {
public:
    explicit ConfigManager(const std::string& path);
    bool load();

    std::string getString(const std::string& key, const std::string& def = "") const;
    int getInt(const std::string& key, int def = 0) const;
    double getDouble(const std::string& key, double def = 0.0) const;

    const nlohmann::json& root() const { return root_; }
    const nlohmann::json getObject(const std::string& dotted) const;
    const std::unordered_map<std::string, std::string> getMap(const std::string& dotted) const;

private:
    std::string path_;
    nlohmann::json root_;
};

} // namespace config

#endif // OBSTACLE_CONFIG_MANAGER_H
