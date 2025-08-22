#include "state_machine/states/emergency_stop_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>

namespace sm {

EmergencyStopState::EmergencyStopState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

void EmergencyStopState::onEnter() {
    // Costruisci topic
    safety_estop_topic_ = cfg_.getString("safety.estop_topic", "smartmower/safety/estop");
    const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
    const std::string pico_base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
    pico_cmd_system_topic_ = root + "/" + pico_base + "/commands/system";

    // Spegni sempre il relè all'ingresso in EMERGENCY_STOP
    publishPicoRelay(false);

    // Forza estop=true se non già true. Qui assumiamo retained dal broker: pubblichiamo true comunque.
    if (mqtt_) {
        nlohmann::json j; j["pressed"] = true; j["estop"] = true; j["state"] = "pressed";
        mqtt_->publish(safety_estop_topic_, j.dump(), 1, true);
    }
}

void EmergencyStopState::onExit() {
    // Nessuna azione specifica per ora
}

void EmergencyStopState::publishPicoRelay(bool on) {
    if (!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

} // namespace sm
