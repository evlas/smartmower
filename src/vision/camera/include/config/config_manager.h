#ifndef CONFIG_MANAGER_H
#define CONFIG_MANAGER_H

#include <string>
#include <cjson/cJSON.h>
#include <unordered_map>
#include <nlohmann/json.hpp>

namespace config {

class ConfigManager {
public:
    explicit ConfigManager(const std::string& path);
    ~ConfigManager();

    bool load();
    std::string getString(const std::string& key, const std::string& def = "");
    int getInt(const std::string& key, int def = 0);
    bool getBool(const std::string& key, bool def = false);
    
    // Aggiunto per supportare oggetti annidati
    std::unordered_map<std::string, std::string> getObject(const std::string& key);
    
    // Get a JSON object directly
    nlohmann::json getJsonObject(const std::string& key);

private:
    std::string path_;
    cJSON* root_;
    
    cJSON* findNode(const std::string& key);
};

} // namespace config

#endif // CONFIG_MANAGER_H
