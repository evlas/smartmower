#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class EmergencyStopState : public IState {
public:
    EmergencyStopState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~EmergencyStopState() override = default;

    const char* name() const override { return "EMERGENCY_STOP"; }
    void onEnter() override;
    void onExit() override;
    void handleEvent(int) override {}
    void tick() override {}

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    // Topic
    std::string safety_estop_topic_;
    std::string pico_cmd_system_topic_;

    void publishPicoRelay(bool on);
};

} // namespace sm
