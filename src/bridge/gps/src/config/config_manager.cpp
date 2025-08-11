#include "config/config_manager.h"
#include <fstream>
#include <sstream>
#include <iostream>
#include <stdexcept>
#include <nlohmann/json.hpp>

using namespace std;
using json = nlohmann::json;

class JsonConfigManager : public ConfigManager {
    json config_;
    
    // Cerca una chiave nel formato "parent.child.grandchild"
    json* findKey(const string& key) {
        size_t pos = 0;
        json* current = &config_;
        string token;
        string remaining = key;
        
        while ((pos = remaining.find('.')) != string::npos) {
            token = remaining.substr(0, pos);
            remaining = remaining.substr(pos + 1);
            
            if (current->is_object() && current->contains(token)) {
                current = &((*current)[token]);
            } else {
                return nullptr;
            }
        }
        
        if (current->is_object() && current->contains(remaining)) {
            return &((*current)[remaining]);
        }
        
        return nullptr;
    }
    
    const json* findKey(const string& key) const {
        return const_cast<JsonConfigManager*>(this)->findKey(key);
    }
    
public:
    JsonConfigManager() = default;
    
    bool loadFromFile(const string& filename) override {
        ifstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        try {
            file >> config_;
            return true;
        } catch (const exception& e) {
            cerr << "Errore nel parsing del file di configurazione: " << e.what() << endl;
            return false;
        }
    }
    
    bool saveToFile(const string& filename) const override {
        ofstream file(filename);
        if (!file.is_open()) {
            return false;
        }
        
        try {
            file << config_.dump(4); // 4 spazi di indentazione
            return true;
        } catch (const exception& e) {
            cerr << "Errore nel salvataggio del file di configurazione: " << e.what() << endl;
            return false;
        }
    }
    
    string getString(const string& key, const string& default_value) const override {
        const json* value = findKey(key);
        return (value && value->is_string()) ? value->get<string>() : default_value;
    }
    
    int getInt(const string& key, int default_value) const override {
        const json* value = findKey(key);
        return (value && value->is_number_integer()) ? value->get<int>() : default_value;
    }
    
    double getDouble(const string& key, double default_value) const override {
        const json* value = findKey(key);
        return (value && value->is_number()) ? value->get<double>() : default_value;
    }
    
    bool getBool(const string& key, bool default_value) const override {
        const json* value = findKey(key);
        return (value && value->is_boolean()) ? value->get<bool>() : default_value;
    }
    
    json getObject(const string& key) const override {
        const json* value = findKey(key);
        return (value && value->is_object()) ? *value : json::object();
    }
    
    void setString(const string& key, const string& value) override {
        // Implementazione per la scrittura di stringhe
        // (non richiesta per la lettura della configurazione)
    }
    
    void setInt(const string& key, int value) override {
        // Implementazione per la scrittura di interi
        // (non richiesta per la lettura della configurazione)
    }
    
    void setDouble(const string& key, double value) override {
        // Implementazione per la scrittura di double
        // (non richiesta per la lettura della configurazione)
    }
    
    void setBool(const string& key, bool value) override {
        // Implementazione per la scrittura di booleani
        // (non richiesta per la lettura della configurazione)
    }
    
    bool hasKey(const string& key) const override {
        return findKey(key) != nullptr;
    }
};

// Factory function
unique_ptr<ConfigManager> createConfigManager() {
    return make_unique<JsonConfigManager>();
}
