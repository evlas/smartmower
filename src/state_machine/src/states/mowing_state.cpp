#include "state_machine/states/mowing_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

MowingState::MowingState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

static std::string tolower_copy_ms(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

void MowingState::publishPicoRelay(bool on){
    if(!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

void MowingState::publishPicoMotors(double linear_mps, double angular_rps){
    if(!mqtt_) return;
    double v_l = linear_mps - angular_rps * (wheel_base_m_/2.0);
    double v_r = linear_mps + angular_rps * (wheel_base_m_/2.0);
    nlohmann::json j; j["left"] = v_l; j["right"] = v_r; j["blade1"] = blade_on_?1.0:0.0; j["blade2"] = blade_on_?1.0:0.0;
    mqtt_->publish(pico_cmd_motors_topic_, j.dump(), 0, false);
}

void MowingState::onEnter() {
    // Carica topic
    mission_state_topic_ = cfg_.getString("mission.state_topic", "smartmower/mission/state");
    safety_state_topic_  = cfg_.getString("safety.state_topic",  "smartmower/safety/state");
    safety_estop_topic_  = cfg_.getString("safety.estop_topic",  "smartmower/safety/estop");
    const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
    const std::string pico_base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
    pico_cmd_system_topic_ = root + "/" + pico_base + "/commands/system";
    pico_cmd_motors_topic_ = root + "/" + pico_base + "/commands/motors";

    // Handler esterno
    if (sm_) {
        sm_->setExternalMqttHandler([this](const std::string& topic, const std::string& payload){ this->onMessage(topic, payload); });
    }
    // Sottoscrizioni
    if (mqtt_) {
        mqtt_->subscribe(mission_state_topic_, 0);
        mqtt_->subscribe(safety_state_topic_, 0);
        mqtt_->subscribe(safety_estop_topic_, 0);
    }
    // Abilita attuatori in MOWING
    try { publishPicoRelay(true); } catch(...) {}
}

void MowingState::onExit() {
    if (sm_) sm_->setExternalMqttHandler({});
    // Sicurezza: spegni relè in uscita
    try { publishPicoRelay(false); } catch(...) {}
}

void MowingState::handleEvent(int /*ev*/) {
}

void MowingState::tick() {
}

void MowingState::onMessage(const std::string& topic, const std::string& payload) {
    if (!sm_) return;
    auto emit = [this](EventType e){ sm_->handleEvent(e); };

    // Parse JSON se possibile
    nlohmann::json j; bool parsed=false;
    try { j = nlohmann::json::parse(payload); parsed=true; } catch(...) { parsed=false; }

    if (topic == safety_estop_topic_) {
        if (parsed) {
            if ((j.contains("pressed") && j["pressed"].is_boolean() && j["pressed"].get<bool>()) ||
                (j.contains("estop") && j["estop"].is_boolean() && j["estop"].get<bool>()) ||
                (j.contains("state") && j["state"].is_string() && tolower_copy_ms(j["state"].get<std::string>())=="pressed")) {
                emit(EventType::EMERGENCY_STOP);
                return;
            }
        }
        auto s = tolower_copy_ms(payload);
        if (s=="1"||s=="true"||s=="pressed"||s=="estop") { emit(EventType::EMERGENCY_STOP); return; }
    }

    if (topic == mission_state_topic_) {
        if (parsed) {
            auto get_str = [&](const char* key)->std::string{ if (j.contains(key) && j[key].is_string()) return tolower_copy_ms(j[key].get<std::string>()); return {}; };
            auto state = get_str("state");
            auto cmd   = get_str("cmd");
            if (state=="pause" || cmd=="pause") { emit(EventType::PAUSE); return; }
        }
        auto s = tolower_copy_ms(payload);
        if (s=="pause") { emit(EventType::PAUSE); return; }
    }
}

} // namespace sm
