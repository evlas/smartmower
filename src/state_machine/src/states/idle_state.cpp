#include "state_machine/states/idle_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

IdleState::IdleState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

void IdleState::onEnter() {
    // Carica topic da configurazione con fallback sensati
    // Nota: il file robot_config.json non espone direttamente questi path: usiamo default "smartmower/..."
    mission_state_topic_ = cfg_.getString("mission.state_topic", "smartmower/mission/state");
    safety_state_topic_  = cfg_.getString("safety.state_topic",  "smartmower/safety/state");
    safety_estop_topic_  = cfg_.getString("safety.estop_topic",  "smartmower/safety/estop");
    // Costruisci topic comandi Pico per controllare il relè
    {
        const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
        const std::string pico_base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
        pico_cmd_system_topic_ = root + "/" + pico_base + "/commands/system";
    }

    // Registra handler per inoltrare i messaggi a IdleState::onMessage
    if (sm_) {
        sm_->setExternalMqttHandler([this](const std::string& topic, const std::string& payload){
            this->onMessage(topic, payload);
        });
    }
    // Sottoscrizioni ai topic rilevanti in IDLE
    if (mqtt_) {
        mqtt_->subscribe(mission_state_topic_, 0);
        mqtt_->subscribe(safety_state_topic_, 0);
        mqtt_->subscribe(safety_estop_topic_, 0);
        // All'ingresso in IDLE, assicurati che il relè sia OFF
        try {
            publishPicoRelay(false);
        } catch (...) { /* ignora */ }
    }
}

void IdleState::onExit() {
    // Rimuove l'handler esterno registrato da IDLE
    if (sm_) {
        sm_->setExternalMqttHandler({});
    }
}

void IdleState::handleEvent(int /*eventTypeInt*/) {
    // Gestione custom degli EventType se necessario (non usata per ora)
}

static std::string tolower_copy(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

void IdleState::onMessage(const std::string& topic, const std::string& payload) {
    // Demux per topic e genera eventi per la StateMachine
    if (!sm_) return;

    auto emit = [this](EventType e){ sm_->handleEvent(e); };

    // Prova JSON
    bool parsed = false;
    nlohmann::json j;
    try {
        j = nlohmann::json::parse(payload);
        parsed = true;
    } catch (...) {
        parsed = false;
    }

    if (topic == safety_estop_topic_) {
        if (parsed) {
            // accetta {"pressed": true} o {"estop": true} o {"state":"pressed"}
            if ((j.contains("pressed") && j["pressed"].is_boolean() && j["pressed"].get<bool>()) ||
                (j.contains("estop") && j["estop"].is_boolean() && j["estop"].get<bool>()) ||
                (j.contains("state") && j["state"].is_string() && tolower_copy(j["state"].get<std::string>()) == "pressed")) {
                emit(EventType::EMERGENCY_STOP);
                return;
            }
        }
        // Fallback testuale
        auto s = tolower_copy(payload);
        if (s == "1" || s == "true" || s == "pressed" || s == "estop") {
            emit(EventType::EMERGENCY_STOP);
            return;
        }
    }

    if (topic == safety_state_topic_) {
        if (parsed) {
            // Esempi accettati: {"battery_low":true} oppure {"battery":"low"}
            if ((j.contains("battery_low") && j["battery_low"].is_boolean() && j["battery_low"].get<bool>()) ||
                (j.contains("battery") && j["battery"].is_string() && tolower_copy(j["battery"].get<std::string>()) == "low")) {
                emit(EventType::BATTERY_LOW);
                return;
            }
        }
        auto s = tolower_copy(payload);
        if (s.find("battery_low") != std::string::npos || s == "low") {
            emit(EventType::BATTERY_LOW);
            return;
        }
    }

    if (topic == mission_state_topic_) {
        if (parsed) {
            // Esempi: {"state":"start"}|{"cmd":"start"}|{"manual":true}
            auto get_str = [&](const char* key)->std::string{
                if (j.contains(key) && j[key].is_string()) return tolower_copy(j[key].get<std::string>());
                return {};
            };
            auto state = get_str("state");
            auto cmd   = get_str("cmd");
            bool manual = (j.contains("manual") && j["manual"].is_boolean() && j["manual"].get<bool>());

            if (state == "start" || cmd == "start") { emit(EventType::START_MOWING); return; }
            if (state == "manual" || cmd == "manual" || manual) { emit(EventType::MANUAL_CONTROL); return; }
        }
        auto s = tolower_copy(payload);
        if (s == "start" || s == "start_mowing") { emit(EventType::START_MOWING); return; }
        if (s == "manual" || s == "manual_control") { emit(EventType::MANUAL_CONTROL); return; }
    }
}

void IdleState::tick() {
    // In questa prima migrazione non eseguiamo controllo periodico; tutto a eventi MQTT.
}

void IdleState::publishPicoRelay(bool on) {
    if (!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

} // namespace sm
