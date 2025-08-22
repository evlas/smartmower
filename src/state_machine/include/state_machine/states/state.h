#pragma once

#include <string>
#include <memory>

namespace sm {

class ConfigManager;
class MqttClient;
class StateMachine;

// Interfaccia comune per tutti gli stati della StateMachine
class IState {
public:
    virtual ~IState() = default;

    // Identificativo di stato (stringa per debug/log); lo StateId resta in StateMachine
    virtual const char* name() const = 0;

    // Hook chiamato quando si entra nello stato
    virtual void onEnter() {}

    // Hook chiamato quando si esce dallo stato
    virtual void onExit() {}

    // Gestione eventi specifici di stato
    virtual void handleEvent(int /*EventTypeInt*/) {}

    // Logica periodica dello stato
    virtual void tick() {}
};

// Factory base per creare stati; implementazioni specifiche per ciascuno stato
class StateFactory {
public:
    StateFactory(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
    : cfg_(cfg), mqtt_(mqtt), sm_(sm) {}
    virtual ~StateFactory() = default;

protected:
    ConfigManager& cfg_;
    MqttClient* mqtt_;
    StateMachine* sm_;
};

} // namespace sm
