#pragma once

#include <string>
#include <memory>
#include <functional>
#include <atomic>
#include <chrono>
#include <unordered_map>
#include <unordered_set>

namespace sm {

class ConfigManager;
class MqttClient;
class InitState;
class IdleState;
class ManualState;
class EmergencyStopState;

enum class StateId {
    INIT,
    IDLE,
    MOWING,
    PAUSED,
    DOCKING,
    UNDOCKING,
    CHARGING,
    MANUAL_CONTROL,
    EMERGENCY_STOP,
    ERROR
};

// Eventi ispirati alla vecchia implementazione C
enum class EventType {
    NONE,
    INIT_COMPLETE,
    START_MOWING,
    BATTERY_LOW,
    BATTERY_FULL,
    EMERGENCY_STOP,
    MANUAL_CONTROL,
    ERROR_OCCURRED,
    PAUSE,
    RESUME,
    END_MANUAL_CONTROL,
    EMERGENCY_RECOVER,
    DOCKING_COMPLETE,
    UNDOCKING_COMPLETE,
    OBSTACLE_DETECTED,
    PERIMETER_CROSSED,
    AREA_COMPLETE,
    TIMEOUT,
    PICO_CONNECTED,
    GPS_CONNECTED,
    CAMERA_CONNECTED
};

class StateMachine {
public:
    StateMachine(ConfigManager& cfg, MqttClient* mqtt);
    ~StateMachine();

    bool start();
    void stop();

    // Esegue un ciclo (non bloccante a lungo)
    void tick();

    // Transizione esplicita (placeholder)
    void requestTransition(StateId next);

    // Gestione eventi
    void handleEvent(EventType ev);

    StateId currentState() const { return current_; }
    static const char* toString(StateId s);
    static const char* eventToString(EventType e);

private:
    void publishStatus();
    void publishEvent(EventType e);
    void transition(StateId next, const char* reason = nullptr);
    double timeInStateSeconds() const;
    // Parsing comandi esterni su MQTT
    bool parseAndHandleCommand(const std::string& payload);

    ConfigManager& cfg_;
    MqttClient* mqtt_;
    std::atomic<bool> running_{false};
    StateId current_ { StateId::INIT };
    StateId requested_ { StateId::INIT };
    StateId previous_ { StateId::INIT };
    unsigned long tick_count_ {0};
    EventType last_event_ { EventType::NONE };
    std::string last_reason_;
    std::chrono::steady_clock::time_point state_enter_tp_ { std::chrono::steady_clock::now() };

    // Topic comandi per controlli in IDLE (e generali)
    std::string cmd_topic_;

public:
    // Permette agli stati di registrare un handler MQTT specifico (es. InitState con InitChecker)
    void setExternalMqttHandler(std::function<void(const std::string&, const std::string&)> handler);

private:
    std::function<void(const std::string&, const std::string&)> external_mqtt_handler_;
    // Stato INIT dedicato (possiede InitChecker)
    std::unique_ptr<InitState> init_state_;
    // Stato IDLE dedicato (gestirà i controlli e sottoscrizioni)
    std::unique_ptr<IdleState> idle_state_;
    // Stato MANUAL dedicato (subscribe comandi manual, passthrough a controlli)
    std::unique_ptr<ManualState> manual_state_;
    // Stato EMERGENCY_STOP dedicato
    std::unique_ptr<EmergencyStopState> emergency_state_;

    // ---- Data-driven FSM ----
    struct StateSpec {
        std::unordered_set<EventType> allowed;
        std::unordered_map<EventType, StateId> transitions;
    };

    // Tabelle derivate dal JSON
    std::unordered_map<StateId, StateSpec> table_;
    std::unordered_map<std::string, EventType> event_by_name_;
    std::unordered_map<EventType, std::string> name_by_event_;
    std::unordered_map<std::string, StateId> state_by_name_;

    // Costruzione tabella da configurazione
    void buildDataDrivenTable_();
    // Mapping helper
    bool parseEventName_(const std::string& name, EventType& out) const;
    bool parseStateName_(const std::string& name, StateId& out) const;
};

} // namespace sm
