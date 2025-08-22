#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class MowingState : public IState {
public:
    MowingState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~MowingState() override = default;

    const char* name() const override { return "MOWING"; }
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
    std::string pico_cmd_motors_topic_;

    // Stato locale
    bool blade_on_ {false};
    double wheel_base_m_ {0.35};

    // Handler messaggi MQTT esterno (registrato su StateMachine)
    void onMessage(const std::string& topic, const std::string& payload);

    // Helper
    void publishPicoRelay(bool on);
    void publishPicoMotors(double linear_mps, double angular_rps);
};

} // namespace sm
