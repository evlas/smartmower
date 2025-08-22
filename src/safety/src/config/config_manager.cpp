#include "config/config_manager.h"
#include <fstream>
#include <iostream>

namespace sm {

class JsonConfigManagerImpl : public ConfigManager {
public:
    bool load(const std::string& preferred_path, const std::string& fallback_path) override {
        using nlohmann::json;
        config_ = nlohmann::json::object();
        if (tryLoad(preferred_path)) return true;
        if (tryLoad(fallback_path)) return true;
        return false;
    }

    bool hasKey(const std::string& key) const override { return findKey(key) != nullptr; }

    std::string getString(const std::string& key, const std::string& def) const override {
        const auto* v = findKey(key);
        return (v && v->is_string()) ? v->get<std::string>() : def;
    }

    int getInt(const std::string& key, int def) const override {
        const auto* v = findKey(key);
        return (v && v->is_number_integer()) ? v->get<int>() : def;
    }

    double getDouble(const std::string& key, double def) const override {
        const auto* v = findKey(key);
        return (v && (v->is_number_float() || v->is_number_integer())) ? v->get<double>() : def;
    }

    bool getBool(const std::string& key, bool def) const override {
        const auto* v = findKey(key);
        return (v && v->is_boolean()) ? v->get<bool>() : def;
    }

    nlohmann::json getObject(const std::string& key) const override {
        const auto* v = findKey(key);
        return (v && v->is_object()) ? *v : nlohmann::json::object();
    }

    nlohmann::json get(const std::string& key) const override {
        const auto* v = findKey(key);
        if (v) return *v;
        return nullptr; // JSON null
    }

private:
    bool tryLoad(const std::string& path) {
        std::ifstream f(path);
        if (!f.is_open()) return false;
        try {
            f >> config_;
            return true;
        } catch (const std::exception& e) {
            std::cerr << "[safety][config] parse error: " << e.what() << " on file " << path << std::endl;
            return false;
        }
    }

    const nlohmann::json* findKey(const std::string& dotted) const {
        const nlohmann::json* cur = &config_;
        size_t start = 0, dot;
        while ((dot = dotted.find('.', start)) != std::string::npos) {
            auto key = dotted.substr(start, dot - start);
            if (!(cur->is_object() && cur->contains(key))) return nullptr;
            cur = &((*cur)[key]);
            start = dot + 1;
        }
        auto last = dotted.substr(start);
        if (cur->is_object() && cur->contains(last)) return &((*cur)[last]);
        return nullptr;
    }

    nlohmann::json config_;
};

ConfigManager* createJsonConfigManager() { return new JsonConfigManagerImpl(); }

} // namespace sm
