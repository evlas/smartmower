#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class PausedState : public IState {
public:
    PausedState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~PausedState() override = default;

    const char* name() const override { return "PAUSED"; }
    void onEnter() override;
    void onExit() override;
    void handleEvent(int) override;
    void tick() override;

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    // Topic base
    std::string mission_state_topic_;
    std::string safety_state_topic_;
    std::string safety_estop_topic_;

    // Topic verso Pico bridge
    std::string pico_cmd_system_topic_;

    void onMessage(const std::string& topic, const std::string& payload);

    // Helper
    void publishPicoRelay(bool on);
};

} // namespace sm
