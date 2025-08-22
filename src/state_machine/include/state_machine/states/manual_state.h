#pragma once

#include "state_machine/states/state.h"
#include <string>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class ManualState : public IState {
public:
    ManualState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~ManualState() override = default;

    const char* name() const override { return "MANUAL_CONTROL"; }
    void onEnter() override;
    void onExit() override;
    void tick() override;

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    // Topic manual in ingresso
    std::string manual_cmd_vel_topic_;
    std::string manual_blade_topic_;
    std::string manual_exit_topic_;

    // Topic di uscita verso controlli reali
    std::string out_cmd_vel_topic_;
    std::string out_blade_cmd_topic_;

    // Topic unico dei comandi della FSM (per comandi manuali in JSON)
    std::string fsm_cmd_topic_;

    // Topic verso Pico bridge
    std::string pico_cmd_system_topic_;
    std::string pico_cmd_motors_topic_;

    // Parametri cinematica
    double wheel_base_m_ {0.35};
    double max_linear_speed_mps_ {1.5};

    // Stato corrente
    bool blade_on_ {false};
    double last_lin_ {0.0};
    double last_ang_ {0.0};

    // Handler messaggi MQTT esterno (registrato su StateMachine)
    void onMessage(const std::string& topic, const std::string& payload);

    // Helper
    void publishPicoRelay(bool on);
    void publishPicoMotors(double linear_mps, double angular_rps);
};

} // namespace sm
