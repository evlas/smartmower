#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class ChargingState : public IState {
public:
    ChargingState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~ChargingState() override = default;

    const char* name() const override { return "CHARGING"; }
    void onEnter() override;
    void onExit() override;
    void handleEvent(int) override;
    void tick() override;

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    std::string pico_cmd_system_topic_;

    void publishPicoRelay(bool on);
};

} // namespace sm
