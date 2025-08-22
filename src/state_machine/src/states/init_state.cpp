#include "state_machine/states/init_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

InitState::InitState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

void InitState::publishPicoRelay(bool on) {
    if (!mqtt_) return;
    try {
        nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
        mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
    } catch (...) {
        // ignora
    }
}

InitState::~InitState() = default;

void InitState::onEnter() {
    // Carica topic da configurazione (nuovo schema mqtt.topics.* con fallback alle chiavi legacy)
    const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
    // Pico
    {
        std::string base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
        std::string sub  = cfg_.getString("mqtt.topics.pico.subtopics.status", "status");
        pico_topic_ = root + "/" + base + "/" + sub;
        // Fallback legacy se specificato
        std::string legacy = cfg_.getString("pico_logging.status_topic", "");
        if (!legacy.empty()) pico_topic_ = legacy;
        // Costruisci anche il topic comandi di sistema per controllare il relè
        pico_cmd_system_topic_ = root + "/" + base + "/commands/system";
    }
    // GPS
    {
        std::string base = cfg_.getString("mqtt.topics.gps.base", "bridge/gps");
        std::string sub  = cfg_.getString("mqtt.topics.gps.subtopics.status", "status");
        gps_topic_ = root + "/" + base + "/" + sub;
        std::string legacy = cfg_.getString("gps_logging.status_topic",  "");
        if (!legacy.empty()) gps_topic_ = legacy;
    }
    // Camera
    {
        std::string base = cfg_.getString("mqtt.topics.camera.base", "vision/camera");
        std::string sub  = cfg_.getString("mqtt.topics.camera.subtopics.status", "status");
        cam_topic_ = root + "/" + base + "/" + sub;
        std::string legacy = cfg_.getString("vision.status_topic",  "");
        if (!legacy.empty()) cam_topic_ = legacy;
    }

    // Registra handler per inoltrare i messaggi a InitState::onMessage
    if (sm_) {
        sm_->setExternalMqttHandler([this](const std::string& topic, const std::string& payload){
            this->onMessage(topic, payload);
        });
    }
    // Sottoscrizioni
    start();
}

void InitState::handleEvent(int /*eventTypeInt*/) {
    // Nessuna gestione specifica in questa fase
}

void InitState::tick() {
    // Se tutti i moduli sono pronti, genera INIT_COMPLETE
    if (ready() && sm_) {
        sm_->handleEvent(EventType::INIT_COMPLETE);
    }
}

void InitState::start() {
    if (!mqtt_) return;
    mqtt_->subscribe(pico_topic_, 0);
    mqtt_->subscribe(gps_topic_, 0);
    mqtt_->subscribe(cam_topic_, 0);
}

bool InitState::ready() const {
    return pico_ready_.load() && gps_ready_.load() && cam_ready_.load();
}

void InitState::onMessage(const std::string& topic, const std::string& payload) {
    if (topic == pico_topic_) {
        bool now_ready = payloadIndicatesPicoReady(payload) || payloadIndicatesReady(payload);
        if (now_ready) {
            bool was_ready = pico_ready_.load();
            if (!was_ready) {
                // Appena connesso: spegni il relè
                publishPicoRelay(false);
            }
            pico_ready_.store(true);
        }
    } else if (topic == gps_topic_) {
        if (payloadIndicatesGpsReady(payload) || payloadIndicatesReady(payload)) gps_ready_.store(true);
    } else if (topic == cam_topic_) {
        if (payloadIndicatesReady(payload)) cam_ready_.store(true);
    }
}

bool InitState::payloadIndicatesReady(const std::string& payload) {
    try {
        auto j = nlohmann::json::parse(payload);
        // accetta online:true come segnale di ready (es. camera)
        if (j.contains("online") && j["online"].is_boolean() && j["online"].get<bool>()) return true;
        if (j.contains("ready") && j["ready"].is_boolean() && j["ready"].get<bool>()) return true;
        if (j.contains("state") && j["state"].is_string()) {
            std::string s = j["state"].get<std::string>();
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
            if (s == "ready" || s == "online" || s == "ok") return true;
        }
    } catch (...) {
        // non-JSON, fallback
    }
    std::string s = payload;
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
    return (s.find("ready") != std::string::npos) || (s.find("online") != std::string::npos) || (s == "ok");
}

bool InitState::payloadIndicatesPicoReady(const std::string& payload) {
    try {
        auto j = nlohmann::json::parse(payload);
        if (j.contains("status") && j["status"].is_string()) {
            std::string s = j["status"].get<std::string>();
            std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return std::tolower(c); });
            if (s == "running" || s == "online") return true;
        }
    } catch (...) {}
    return false;
}

bool InitState::payloadIndicatesGpsReady(const std::string& payload) {
    try {
        auto j = nlohmann::json::parse(payload);
        if (j.contains("fix") && j["fix"].is_number()) {
            if (j["fix"].get<int>() >= 1) return true;
        }
        if (j.contains("satellites_used") && j["satellites_used"].is_number()) {
            if (j["satellites_used"].get<int>() > 0) return true;
        }
    } catch (...) {}
    return false;
}

} // namespace sm
