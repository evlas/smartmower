#pragma once

#include <string>
#include <nlohmann/json.hpp>

namespace sm {

class ConfigManager {
public:
    virtual ~ConfigManager() = default;

    virtual bool load(const std::string& preferred_path = "/opt/smartmower/etc/config/robot_config.json",
                      const std::string& fallback_path = "src/config/robot_config.json") = 0;

    virtual bool hasKey(const std::string& key) const = 0;

    virtual std::string getString(const std::string& key, const std::string& def = "") const = 0;
    virtual int getInt(const std::string& key, int def = 0) const = 0;
    virtual double getDouble(const std::string& key, double def = 0.0) const = 0;
    virtual bool getBool(const std::string& key, bool def = false) const = 0;
    virtual nlohmann::json getObject(const std::string& key) const = 0;
    // Restituisce il valore JSON grezzo (object/array/number/string/bool/null). Se non trovato, null.
    virtual nlohmann::json get(const std::string& key) const = 0;
};

ConfigManager* createJsonConfigManager();

} // namespace sm
