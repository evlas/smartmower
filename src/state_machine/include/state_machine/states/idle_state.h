#pragma once

#include "state_machine/states/state.h"

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class IdleState : public IState {
public:
    IdleState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~IdleState() override = default;

    const char* name() const override { return "IDLE"; }
    void onEnter() override;
    void onExit() override;
    void handleEvent(int eventTypeInt) override;
    void tick() override;

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    // Topic osservati in IDLE
    std::string mission_state_topic_;
    std::string safety_state_topic_;
    std::string safety_estop_topic_;
    // Topic comandi sistema verso Pico (per relay)
    std::string pico_cmd_system_topic_;

    // Handler esterno per inoltro messaggi MQTT a IdleState
    void onMessage(const std::string& topic, const std::string& payload);
    void publishPicoRelay(bool on);
};

} // namespace sm
