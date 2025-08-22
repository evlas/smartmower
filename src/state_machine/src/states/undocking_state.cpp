#include "state_machine/states/undocking_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

UndockingState::UndockingState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

static void build_pico_system_topic(ConfigManager& cfg, std::string& out_topic){
    const std::string root = cfg.getString("mqtt.root_topic", "smartmower");
    const std::string pico_base = cfg.getString("mqtt.topics.pico.base", "bridge/pico");
    out_topic = root + "/" + pico_base + "/commands/system";
}

void UndockingState::publishPicoRelay(bool on){
    if(!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

void UndockingState::onEnter() {
    build_pico_system_topic(cfg_, pico_cmd_system_topic_);
    // Policy: in UNDOCKING relè ON
    try { publishPicoRelay(true); } catch(...) {}
}

void UndockingState::onExit() {}

void UndockingState::handleEvent(int) {}

void UndockingState::tick() {}

} // namespace sm
