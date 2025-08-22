#pragma once

#include "state_machine/states/state.h"
#include <memory>
#include <string>
#include <atomic>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

class InitState : public IState {
public:
    InitState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm);
    ~InitState() override;

    const char* name() const override { return "INIT"; }
    void onEnter() override;
    void handleEvent(int eventTypeInt) override;
    void tick() override;

private:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;

    // Config dei topic readiness
    std::string pico_topic_;
    std::string gps_topic_;
    std::string cam_topic_;
    // Topic comandi sistema verso Pico (per relay)
    std::string pico_cmd_system_topic_;

    // Flag readiness
    std::atomic<bool> pico_ready_{false};
    std::atomic<bool> gps_ready_{false};
    std::atomic<bool> cam_ready_{false};

    // Helpers
    void start();
    void onMessage(const std::string& topic, const std::string& payload);
    bool ready() const;
    static bool payloadIndicatesReady(const std::string& payload);
    static bool payloadIndicatesPicoReady(const std::string& payload);
    static bool payloadIndicatesGpsReady(const std::string& payload);
    void publishPicoRelay(bool on);
};

} // namespace sm
