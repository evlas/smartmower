#include "state_machine/states/paused_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

PausedState::PausedState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

// Helper relay
void PausedState::publishPicoRelay(bool on){
    if(!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

static std::string tolower_copy_ps(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

void PausedState::onEnter() {
    mission_state_topic_ = cfg_.getString("mission.state_topic", "smartmower/mission/state");
    safety_state_topic_  = cfg_.getString("safety.state_topic",  "smartmower/safety/state");
    safety_estop_topic_  = cfg_.getString("safety.estop_topic",  "smartmower/safety/estop");
    // Costruisci topic comandi Pico
    {
        const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
        const std::string pico_base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
        pico_cmd_system_topic_ = root + "/" + pico_base + "/commands/system";
    }
    if (sm_) {
        sm_->setExternalMqttHandler([this](const std::string& topic, const std::string& payload){ this->onMessage(topic, payload); });
    }
    if (mqtt_) {
        mqtt_->subscribe(mission_state_topic_, 0);
        mqtt_->subscribe(safety_state_topic_, 0);
        mqtt_->subscribe(safety_estop_topic_, 0);
        // Policy: in PAUSED relè OFF
        try { publishPicoRelay(false); } catch(...) {}
    }
}

void PausedState::onExit() {
    if (sm_) sm_->setExternalMqttHandler({});
}

void PausedState::handleEvent(int) {}

void PausedState::tick() {}

void PausedState::onMessage(const std::string& topic, const std::string& payload) {
    if (!sm_) return;
    auto emit = [this](EventType e){ sm_->handleEvent(e); };

    nlohmann::json j; bool parsed=false;
    try { j = nlohmann::json::parse(payload); parsed=true; } catch(...) { parsed=false; }

    if (topic == safety_estop_topic_) {
        if (parsed) {
            if ((j.contains("pressed") && j["pressed"].is_boolean() && j["pressed"].get<bool>()) ||
                (j.contains("estop") && j["estop"].is_boolean() && j["estop"].get<bool>()) ||
                (j.contains("state") && j["state"].is_string() && tolower_copy_ps(j["state"].get<std::string>())=="pressed")) {
                emit(EventType::EMERGENCY_STOP);
                return;
            }
        }
        auto s = tolower_copy_ps(payload);
        if (s=="1"||s=="true"||s=="pressed"||s=="estop") { emit(EventType::EMERGENCY_STOP); return; }
    }

    if (topic == safety_state_topic_) {
        if (parsed) {
            if ((j.contains("battery_low") && j["battery_low"].is_boolean() && j["battery_low"].get<bool>()) ||
                (j.contains("battery") && j["battery"].is_string() && tolower_copy_ps(j["battery"].get<std::string>())=="low")) {
                emit(EventType::BATTERY_LOW); return;
            }
            if ((j.contains("error") && j["error"].is_boolean() && j["error"].get<bool>()) ||
                (j.contains("state") && j["state"].is_string() && tolower_copy_ps(j["state"].get<std::string>())=="error")) {
                emit(EventType::ERROR_OCCURRED); return;
            }
        }
        auto s = tolower_copy_ps(payload);
        if (s.find("battery_low")!=std::string::npos || s=="low") { emit(EventType::BATTERY_LOW); return; }
        if (s.find("error")!=std::string::npos || s=="error") { emit(EventType::ERROR_OCCURRED); return; }
    }

    if (topic == mission_state_topic_) {
        if (parsed) {
            auto get_str = [&](const char* key)->std::string{ if (j.contains(key) && j[key].is_string()) return tolower_copy_ps(j[key].get<std::string>()); return {}; };
            auto state = get_str("state");
            auto cmd   = get_str("cmd");
            if (state=="resume" || cmd=="resume" || state=="start" || cmd=="start") { emit(EventType::RESUME); return; }
        }
        auto s = tolower_copy_ps(payload);
        if (s=="resume" || s=="start") { emit(EventType::RESUME); return; }
    }
}

} // namespace sm
