#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include "state_machine/states/init_state.h"
#include "state_machine/states/idle_state.h"
#include "state_machine/states/manual_state.h"
#include "state_machine/states/emergency_stop_state.h"
#include <iostream>
#include <thread>
#include <chrono>
#include <ctime>
#include <nlohmann/json.hpp>
#include <unordered_map>
#include <unordered_set>

namespace sm {

static const char* kStateNames[] = {
    "INIT",
    "IDLE",
    "MOWING",
    "PAUSED",
    "DOCKING",
    "UNDOCKING",
    "CHARGING",
    "MANUAL_CONTROL",
    "EMERGENCY_STOP",
    "ERROR"
};

static const char* kEventNames[] = {
    "NONE",
    "INIT_COMPLETE",
    "START_MOWING",
    "BATTERY_LOW",
    "BATTERY_FULL",
    "EMERGENCY_STOP",
    "MANUAL_CONTROL",
    "ERROR_OCCURRED",
    "PAUSE",
    "RESUME",
    "END_MANUAL_CONTROL",
    "EMERGENCY_RECOVER",
    "DOCKING_COMPLETE",
    "UNDOCKING_COMPLETE",
    "OBSTACLE_DETECTED",
    "PERIMETER_CROSSED",
    "AREA_COMPLETE",
    "TIMEOUT",
    "PICO_CONNECTED",
    "GPS_CONNECTED",
    "CAMERA_CONNECTED"
};

const char* StateMachine::toString(StateId s) {
    return kStateNames[static_cast<int>(s)];
}

void StateMachine::setExternalMqttHandler(std::function<void(const std::string&, const std::string&)> handler) {
    external_mqtt_handler_ = std::move(handler);
}

bool StateMachine::parseAndHandleCommand(const std::string& payload) {
    using nlohmann::json;
    // Supporta sia stringa semplice (es. "manual") sia JSON {"cmd":"manual"}
    std::string cmd;
    bool recover_flag = false;
    try {
        // Prova JSON
        auto j = json::parse(payload);
        if (j.is_object()) {
            if (j.contains("cmd") && j["cmd"].is_string()) cmd = j["cmd"].get<std::string>();
            else if (j.contains("command") && j["command"].is_string()) cmd = j["command"].get<std::string>();
            // Comando diretto di recover via JSON: {"recover":true}
            if (j.contains("recover") && j["recover"].is_boolean() && j["recover"].get<bool>()) {
                recover_flag = true;
            }
        } else if (j.is_string()) {
            cmd = j.get<std::string>();
        }
    } catch (...) {
        // Non-JSON: usa payload grezzo
        cmd = payload;
    }
    // Normalizza
    auto tolower_str = [](std::string s){ for (auto& c: s) c = static_cast<char>(::tolower(static_cast<unsigned char>(c))); return s; };
    cmd = tolower_str(cmd);
    if (cmd.empty()) return false;

    // Mapping comandi -> eventi
    EventType ev = EventType::NONE;
    if (cmd == "manual" || cmd == "manual_control") ev = EventType::MANUAL_CONTROL;
    else if (cmd == "start" || cmd == "start_mowing") ev = EventType::START_MOWING;
    else if (cmd == "pause") ev = EventType::PAUSE;
    else if (cmd == "resume") {
        // In MANUAL_CONTROL "resume" deve significare uscire dal manuale
        if (current_ == StateId::MANUAL_CONTROL) ev = EventType::END_MANUAL_CONTROL;
        else ev = EventType::RESUME;
    }
    else if (cmd == "dock" || cmd == "docking") ev = EventType::BATTERY_LOW; // forza docking come evento
    else if (cmd == "undock" || cmd == "undocking") ev = EventType::UNDOCKING_COMPLETE; // kick per passare a MOWING dopo UNDOCKING
    else if (cmd == "estop" || cmd == "emergency_stop") ev = EventType::EMERGENCY_STOP;
    else if (cmd == "recover" || cmd == "clear_error" || recover_flag) ev = EventType::EMERGENCY_RECOVER;

    if (ev != EventType::NONE) {
        handleEvent(ev);
        return true;
    }
    return false;
}

const char* StateMachine::eventToString(EventType e) {
    return kEventNames[static_cast<int>(e)];
}

StateMachine::StateMachine(ConfigManager& cfg, MqttClient* mqtt)
: cfg_(cfg), mqtt_(mqtt) {}

StateMachine::~StateMachine() { stop(); }

bool StateMachine::start() {
    running_ = true;
    current_ = StateId::INIT;
    requested_ = current_;
    previous_ = current_;
    state_enter_tp_ = std::chrono::steady_clock::now();
    // Costruisci tabella data-driven da configurazione
    buildDataDrivenTable_();
    // Setup handler MQTT e InitState
    if (mqtt_) {
        // Topic comandi (costruito da configurazione mqtt.* con override opzionale state_machine.command_topic)
        std::string override_cmd = cfg_.getString("state_machine.command_topic", "");
        if (!override_cmd.empty()) {
            cmd_topic_ = override_cmd;
        } else {
            // Costruisci: mqtt.root_topic + "/" + mqtt.topics.state_machine.base + "/" + subtopics.commands
            const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
            const std::string base = cfg_.getString("mqtt.topics.state_machine.base", "state_machine");
            const std::string sub  = cfg_.getString("mqtt.topics.state_machine.subtopics.commands", "commands");
            cmd_topic_ = root + "/" + base + "/" + sub;
        }
        mqtt_->setMessageHandler([this](const std::string& topic, const std::string& payload){
            // 1) Comandi esterni
            if (!cmd_topic_.empty() && topic == cmd_topic_) {
                parseAndHandleCommand(payload);
            }
            // 2) Handler esterno registrato dagli stati (es. InitState)
            if (external_mqtt_handler_) external_mqtt_handler_(topic, payload);
        });
        // Sottoscrizioni
        mqtt_->subscribe(cmd_topic_, 0);
    }
    // Crea InitState per gestire readiness
    init_state_ = std::make_unique<InitState>(cfg_, mqtt_, this);
    init_state_->onEnter();
    publishStatus();
    return true;
}

void StateMachine::stop() {
    running_ = false;
}

void StateMachine::requestTransition(StateId next) {
    requested_ = next;
}

void StateMachine::tick() {
    if (!running_) return;

    // Transizione richiesta esplicitamente
    if (requested_ != current_) {
        transition(requested_, "requested");
    }

    // Tick counter e logiche reali per-stato
    ++tick_count_;

    // INIT: demandata a InitState
    if (current_ == StateId::INIT) {
        if (init_state_) init_state_->tick();
        // Timeout INIT da configurazione
        const int init_to = cfg_.getInt("state_machine.timeouts.init_timeout", 0);
        if (init_to > 0 && timeInStateSeconds() > static_cast<double>(init_to)) {
            handleEvent(EventType::TIMEOUT);
        }
        return; // niente altro da fare in INIT
    }

    // IDLE: demandata a IdleState (placeholder per ora)
    if (current_ == StateId::IDLE) {
        if (idle_state_) idle_state_->tick();
    }

    // Timeouts e guard basati su configurazione
    const int undocking_to = cfg_.getInt("state_machine.timeouts.undocking_sec", 60);
    const int docking_to   = cfg_.getInt("state_machine.timeouts.docking_sec", 120);
    const int max_mow      = cfg_.getInt("state_machine.guards.max_mowing_sec", 0); // 0 = disabilitato
    const double tins = timeInStateSeconds();

    switch (current_) {
        case StateId::UNDOCKING:
            if (undocking_to > 0 && tins > undocking_to) {
                handleEvent(EventType::TIMEOUT);
            }
            break;
        case StateId::DOCKING:
            if (docking_to > 0 && tins > docking_to) {
                handleEvent(EventType::TIMEOUT);
            }
            break;
        case StateId::MOWING:
            if (max_mow > 0 && tins > max_mow) {
                handleEvent(EventType::AREA_COMPLETE);
            }
            break;
        default:
            break;
    }
}

void StateMachine::publishStatus() {
    using nlohmann::json;
    std::string topic = "smartmower/status/state";
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    json j = {
        {"current", toString(current_)},
        {"previous", toString(previous_)},
        {"last_event", eventToString(last_event_)},
        {"last_reason", last_reason_},
        {"tick_count", tick_count_},
        {"time_in_state_sec", timeInStateSeconds()},
        {"timestamp", now}
    };
    std::string payload = j.dump();
    if (mqtt_) mqtt_->publish(topic, payload, 0, true);
    std::cout << "[STATE] " << payload << std::endl;
}

void StateMachine::publishEvent(EventType e) {
    using nlohmann::json;
    std::string topic = "smartmower/status/event";
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    json j = {
        {"event", eventToString(e)},
        {"state", toString(current_)},
        {"previous", toString(previous_)},
        {"time_in_state_sec", timeInStateSeconds()},
        {"timestamp", now}
    };
    std::string payload = j.dump();
    if (mqtt_) mqtt_->publish(topic, payload, 0, false);
    std::cout << "[EVENT] " << payload << std::endl;
}

void StateMachine::transition(StateId next, const char* reason) {
    if (next == current_) return;
    using nlohmann::json;
    std::string topic = "smartmower/status/transition";
    const auto now = std::chrono::system_clock::to_time_t(std::chrono::system_clock::now());
    json j = {
        {"from", toString(current_)},
        {"to", toString(next)},
        {"reason", reason ? reason : ""},
        {"time_in_state_sec", timeInStateSeconds()},
        {"timestamp", now}
    };
    std::string payload = j.dump();
    if (mqtt_) mqtt_->publish(topic, payload, 0, false);
    std::cout << "[TRANSITION] " << payload << std::endl;
    // onExit dello stato corrente se necessario
    if (current_ == StateId::INIT && init_state_) {
        init_state_->onExit();
        init_state_.reset();
        external_mqtt_handler_ = nullptr; // rimuovi handler esterno di INIT
    }
    if (current_ == StateId::IDLE && idle_state_) {
        idle_state_->onExit();
        idle_state_.reset();
        // Nota: l'handler esterno per IDLE verrà rimosso dallo stato stesso, se impostato
    }
    if (current_ == StateId::MANUAL_CONTROL && manual_state_) {
        manual_state_->onExit();
        manual_state_.reset();
    }
    if (current_ == StateId::EMERGENCY_STOP && emergency_state_) {
        emergency_state_->onExit();
        emergency_state_.reset();
    }
    previous_ = current_;
    current_ = next;
    last_reason_ = reason ? reason : "";
    state_enter_tp_ = std::chrono::steady_clock::now();
    // Consuma ogni richiesta pendente riallineando requested_ allo stato corrente
    requested_ = current_;
    publishStatus();

    // onEnter del nuovo stato
    if (current_ == StateId::INIT) {
        init_state_ = std::make_unique<InitState>(cfg_, mqtt_, this);
        init_state_->onEnter();
    }
    if (current_ == StateId::IDLE) {
        idle_state_ = std::make_unique<IdleState>(cfg_, mqtt_, this);
        idle_state_->onEnter();
    }
    if (current_ == StateId::MANUAL_CONTROL) {
        manual_state_ = std::make_unique<ManualState>(cfg_, mqtt_, this);
        manual_state_->onEnter();
    }
    if (current_ == StateId::EMERGENCY_STOP) {
        emergency_state_ = std::make_unique<EmergencyStopState>(cfg_, mqtt_, this);
        emergency_state_->onEnter();
    }
}

void StateMachine::handleEvent(EventType ev) {
    if (!running_) return;
    if (ev == EventType::NONE) return;
    last_event_ = ev;
    publishEvent(ev);

    // Gestione globale
    if (ev == EventType::EMERGENCY_STOP) {
        transition(StateId::EMERGENCY_STOP, "EMERGENCY_STOP");
        return;
    }
    if (ev == EventType::ERROR_OCCURRED) {
        transition(StateId::ERROR, "ERROR_OCCURRED");
        return;
    }
    // Data-driven: allowed_events + transitions da tabella
    auto it = table_.find(current_);
    if (it == table_.end()) {
        std::cerr << "[FSM] Stato corrente non presente in tabella: " << toString(current_) << std::endl;
        return;
    }
    const StateSpec& spec = it->second;
    // Guard
    if (!spec.allowed.empty() && spec.allowed.find(ev) == spec.allowed.end()) {
        std::cerr << "[FSM] Evento non ammesso in stato " << toString(current_)
                  << ": " << eventToString(ev) << std::endl;
        return;
    }
    // Transizione
    auto it2 = spec.transitions.find(ev);
    if (it2 != spec.transitions.end()) {
        transition(it2->second, eventToString(ev));
    } else {
        // Nessuna transizione definita: resta nello stato corrente
        std::cerr << "[FSM] Nessuna transizione per evento " << eventToString(ev)
                  << " in stato " << toString(current_) << std::endl;
    }
}

double StateMachine::timeInStateSeconds() const {
    using namespace std::chrono;
    return duration_cast<duration<double>>(steady_clock::now() - state_enter_tp_).count();
}

// ----------------- Data-driven helpers -----------------
bool StateMachine::parseEventName_(const std::string& name, EventType& out) const {
    // Accetta sia "EVENT_X" sia "X"
    std::string n = name;
    if (n.rfind("EVENT_", 0) == 0) n = n.substr(6);
    const int kEventCount = static_cast<int>(sizeof(kEventNames)/sizeof(kEventNames[0]));
    for (int i = 0; i < kEventCount; ++i) {
        if (n == kEventNames[i]) { out = static_cast<EventType>(i); return true; }
    }
    return false;
}

bool StateMachine::parseStateName_(const std::string& name, StateId& out) const {
    // Accetta sia "STATE_X" sia "X"
    std::string n = name;
    if (n.rfind("STATE_", 0) == 0) n = n.substr(6);
    for (int i = 0; i < static_cast<int>(StateId::ERROR) + 1; ++i) {
        if (n == kStateNames[i]) { out = static_cast<StateId>(i); return true; }
    }
    return false;
}

void StateMachine::buildDataDrivenTable_() {
    using nlohmann::json;
    // Pulisci
    table_.clear();
    event_by_name_.clear();
    name_by_event_.clear();
    state_by_name_.clear();

    // Mappa eventi da lista configurata (opzionale)
    try {
        auto evj = cfg_.getObject("state_machine.events");
        if (evj.is_array()) {
            for (const auto& e : evj) {
                if (!e.is_string()) continue;
                EventType et;
                if (parseEventName_(e.get<std::string>(), et)) {
                    event_by_name_[e.get<std::string>()] = et;
                    name_by_event_[et] = kEventNames[static_cast<int>(et)];
                }
            }
        }
    } catch (...) {}

    // Stati e transizioni
    try {
        auto states = cfg_.getObject("mqtt.topics.state_machine.states");
        if (!states.is_object()) return;
        for (auto it = states.begin(); it != states.end(); ++it) {
            const std::string state_name = it.key();
            StateId sid;
            if (!parseStateName_(state_name, sid)) continue;
            state_by_name_[state_name] = sid;
            StateSpec spec;
            const auto& s = it.value();
            // allowed_events
            if (s.contains("allowed_events") && s["allowed_events"].is_array()) {
                for (const auto& evn : s["allowed_events"]) {
                    if (!evn.is_string()) continue;
                    EventType et;
                    if (parseEventName_(evn.get<std::string>(), et)) spec.allowed.insert(et);
                }
            }
            // transitions
            if (s.contains("transitions") && s["transitions"].is_array()) {
                for (const auto& tr : s["transitions"]) {
                    if (!tr.is_object()) continue;
                    if (!tr.contains("on") || !tr.contains("to")) continue;
                    const auto onv = tr["on"], tov = tr["to"];
                    if (!onv.is_string() || !tov.is_string()) continue;
                    EventType et;
                    StateId tosid;
                    if (parseEventName_(onv.get<std::string>(), et) && parseStateName_(tov.get<std::string>(), tosid)) {
                        spec.transitions[et] = tosid;
                    }
                }
            }
            table_[sid] = std::move(spec);
        }
    } catch (...) {
        std::cerr << "[FSM] Errore durante buildDataDrivenTable_" << std::endl;
    }
}

} // namespace sm
