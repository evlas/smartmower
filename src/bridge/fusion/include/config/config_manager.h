#ifndef FUSION_CONFIG_MANAGER_H
#define FUSION_CONFIG_MANAGER_H

#include <string>
#include <memory>
#include <nlohmann/json.hpp>

namespace fusion {
namespace config {

class ConfigManager {
public:
    virtual ~ConfigManager() = default;

    // Carica la configurazione da file
    virtual bool loadFromFile(const std::string& filename) = 0;
    
    // Salva la configurazione su file
    virtual bool saveToFile(const std::string& filename) const = 0;
    
    // Accesso ai valori di configurazione
    virtual std::string getString(const std::string& key, const std::string& default_value = "") const = 0;
    virtual int getInt(const std::string& key, int default_value = 0) const = 0;
    virtual double getDouble(const std::string& key, double default_value = 0.0) const = 0;
    virtual bool getBool(const std::string& key, bool default_value = false) const = 0;
    
    // Accesso a oggetti e array annidati
    virtual nlohmann::json getObject(const std::string& key) const = 0;
    
    // Impostazione valori di configurazione
    virtual void setString(const std::string& key, const std::string& value) = 0;
    virtual void setInt(const std::string& key, int value) = 0;
    virtual void setDouble(const std::string& key, double value) = 0;
    virtual void setBool(const std::string& key, bool value) = 0;
    
    // Verifica esistenza di una chiave
    virtual bool hasKey(const std::string& key) const = 0;
};

// Factory function per creare un'istanza del gestore di configurazione
std::unique_ptr<ConfigManager> createConfigManager();

} // namespace config
} // namespace fusion

#endif // FUSION_CONFIG_MANAGER_H
