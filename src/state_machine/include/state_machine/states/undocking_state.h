#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class UndockingState : public IState {
public:
    UndockingState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~UndockingState() override = default;

    const char* name() const override { return "UNDOCKING"; }
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
