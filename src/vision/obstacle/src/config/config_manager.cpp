#include "config/config_manager.h"
#include <fstream>

using json = nlohmann::json;

namespace config {

ConfigManager::ConfigManager(const std::string& path) : path_(path) {}

bool ConfigManager::load() {
    try {
        std::ifstream f(path_);
        if (!f.is_open()) return false;
        f >> root_;
        return true;
    } catch (...) {
        return false;
    }
}

static const json* getDotted(const json& j, const std::string& dotted) {
    const json* cur = &j;
    size_t start = 0;
    while (start < dotted.size()) {
        size_t dot = dotted.find('.', start);
        std::string key = (dot == std::string::npos) ? dotted.substr(start) : dotted.substr(start, dot - start);
        if (!cur->contains(key)) return nullptr;
        cur = &cur->at(key);
        if (dot == std::string::npos) break;
        start = dot + 1;
    }
    return cur;
}

std::string ConfigManager::getString(const std::string& key, const std::string& def) const {
    const json* v = getDotted(root_, key);
    if (!v) return def;
    if (v->is_string()) return v->get<std::string>();
    return def;
}

int ConfigManager::getInt(const std::string& key, int def) const {
    const json* v = getDotted(root_, key);
    if (!v) return def;
    if (v->is_number_integer()) return v->get<int>();
    if (v->is_number()) return static_cast<int>(v->get<double>());
    return def;
}

double ConfigManager::getDouble(const std::string& key, double def) const {
    const json* v = getDotted(root_, key);
    if (!v) return def;
    if (v->is_number()) return v->get<double>();
    return def;
}

const json ConfigManager::getObject(const std::string& dotted) const {
    const json* v = getDotted(root_, dotted);
    return v ? *v : json::object();
}

const std::unordered_map<std::string, std::string> ConfigManager::getMap(const std::string& dotted) const {
    std::unordered_map<std::string, std::string> out;
    const json* v = getDotted(root_, dotted);
    if (!v || !v->is_object()) return out;
    for (auto it = v->begin(); it != v->end(); ++it) {
        if (it.value().is_string()) out[it.key()] = it.value().get<std::string>();
    }
    return out;
}

} // namespace config
