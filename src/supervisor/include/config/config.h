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
        reload();
    }

    void reload() {
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
    nlohmann::json& json() { return cfg_; }

    const std::string& path() const { return path_; }

    // Scrittura atomica del JSON su disco
    void saveAtomic() const {
        const std::string tmp = path_ + ".tmp";
        std::ofstream ofs(tmp, std::ios::trunc);
        if (!ofs.is_open()) throw std::runtime_error("Impossibile aprire temp file: " + tmp);
        ofs << cfg_.dump(4);
        ofs.flush();
        ofs.close();
        if (std::rename(tmp.c_str(), path_.c_str()) != 0) {
            throw std::runtime_error("Rename atomico fallito da " + tmp + " a " + path_);
        }
    }

    std::string rootTopic() const {
        return cfg_.value("mqtt", nlohmann::json{}).value("root_topic", std::string("smartmower"));
    }

    std::string servicesStatusBase() const {
        return rootTopic();
    }

private:
    std::string path_;
    nlohmann::json cfg_;
};

} // namespace config
} // namespace smartmower
