#include "state_machine/states/manual_state.h"
#include "state_machine/state_machine.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"
#include <nlohmann/json.hpp>
#include <algorithm>
#include <cctype>

namespace sm {

ManualState::ManualState(ConfigManager& cfg, MqttClient* mqtt, StateMachine* sm)
: cfg_(cfg), mqtt_(mqtt), sm_(sm) {}

// Helper: pubblica comando di sistema per accendere/spegnere relè
void ManualState::publishPicoRelay(bool on){
    if(!mqtt_) return;
    nlohmann::json j; j["action"] = "set_relay"; j["value"] = on ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_system_topic_, j.dump(), 1, false);
}

// Helper: mappa (lin,ang) a ruote/blade e pubblica su commands/motors
void ManualState::publishPicoMotors(double linear_mps, double angular_rps){
    if(!mqtt_) return;
    // Differential drive: v_l = v - w*L/2, v_r = v + w*L/2
    double v_l = linear_mps - angular_rps * (wheel_base_m_/2.0);
    double v_r = linear_mps + angular_rps * (wheel_base_m_/2.0);
    // Normalizza in frazione della velocità massima (se il bridge si attende m/s, inviamo valori raw)
    nlohmann::json j;
    j["left"] = v_l;
    j["right"] = v_r;
    j["blade1"] = blade_on_ ? 1.0 : 0.0;
    j["blade2"] = blade_on_ ? 1.0 : 0.0;
    mqtt_->publish(pico_cmd_motors_topic_, j.dump(), 0, false);
}

static std::string tolower_copy_ms(std::string s) {
    std::transform(s.begin(), s.end(), s.begin(), [](unsigned char c){ return static_cast<char>(std::tolower(c)); });
    return s;
}

void ManualState::onEnter() {
    // Carica topic da configurazione con default sensati
    manual_cmd_vel_topic_ = cfg_.getString("manual.cmd_vel_topic", "smartmower/manual/cmd_vel");
    manual_blade_topic_   = cfg_.getString("manual.blade_cmd_topic", "smartmower/manual/blade_cmd");
    manual_exit_topic_    = cfg_.getString("manual.exit_topic", "smartmower/manual/exit");

    out_cmd_vel_topic_    = cfg_.getString("control.cmd_vel_topic", "smartmower/control/cmd_vel");
    out_blade_cmd_topic_  = cfg_.getString("control.blade_cmd_topic", "smartmower/control/blade_cmd");

    // Topic verso Pico bridge
    const std::string root = cfg_.getString("mqtt.root_topic", "smartmower");
    const std::string pico_base = cfg_.getString("mqtt.topics.pico.base", "bridge/pico");
    pico_cmd_system_topic_ = root + "/" + pico_base + "/commands/system";
    pico_cmd_motors_topic_ = root + "/" + pico_base + "/commands/motors";

    // Topic unico comandi FSM (per comandi manuali JSON)
    {
        std::string override_cmd = cfg_.getString("state_machine.command_topic", "");
        if (!override_cmd.empty()) {
            fsm_cmd_topic_ = override_cmd;
        } else {
            const std::string base = cfg_.getString("mqtt.topics.state_machine.base", "state_machine");
            const std::string sub  = cfg_.getString("mqtt.topics.state_machine.subtopics.commands", "commands");
            fsm_cmd_topic_ = root + "/" + base + "/" + sub; // es. smartmower/state_machine/commands
        }
    }

    // Parametri cinematica
    wheel_base_m_ = cfg_.getDouble("hardware.dimensions.wheel_base", 0.35);
    max_linear_speed_mps_ = cfg_.getDouble("tuning.speeds.max_linear_speed", 1.5);

    // Registra handler per inoltrare i messaggi a ManualState::onMessage
    if (sm_) {
        sm_->setExternalMqttHandler([this](const std::string& topic, const std::string& payload){
            this->onMessage(topic, payload);
        });
    }
    // Sottoscrizioni ai topic manual (retrocompatibilità)
    if (mqtt_) {
        if (!manual_cmd_vel_topic_.empty()) mqtt_->subscribe(manual_cmd_vel_topic_, 0);
        if (!manual_blade_topic_.empty())   mqtt_->subscribe(manual_blade_topic_, 0);
        if (!manual_exit_topic_.empty())    mqtt_->subscribe(manual_exit_topic_, 0);
        // Nota: non è necessario sottoscrivere esplicitamente fsm_cmd_topic_ qui perché lo fa già la StateMachine.
    }

    // 1) All'ingresso: accendi il relè di potenza del Pico (abilita attuatori)
    publishPicoRelay(true);
}

void ManualState::onExit() {
    // Rimuove l'handler esterno registrato da MANUAL
    if (sm_) {
        sm_->setExternalMqttHandler({});
    }

    // 3) All'uscita: spegni il relè di potenza del Pico (disabilita attuatori)
    publishPicoRelay(false);
}

void ManualState::tick() {
    // Nessuna logica periodica obbligatoria per ora
}

void ManualState::onMessage(const std::string& topic, const std::string& payload) {
    if (!mqtt_ || !sm_) return;

    // 0) Nuovo schema: comandi manuali via topic comandi della FSM con JSON
    //    Esempi ammessi:
    //    {"manual":{"cmd_vel":{"linear":0.3,"angular":0.1}}}
    //    {"manual":{"blade":{"on":true}}}
    //    {"manual":{"exit":true}}  | {"manual":{"resume":true}}
    if (!fsm_cmd_topic_.empty() && topic == fsm_cmd_topic_) {
        try {
            auto j = nlohmann::json::parse(payload);
            if (j.is_object() && j.contains("manual")) {
                const auto& m = j["manual"];
                if (m.is_object()) {
                    // cmd_vel
                    if (m.contains("cmd_vel") && m["cmd_vel"].is_object()) {
                        double lin = 0.0, ang = 0.0;
                        const auto& cv = m["cmd_vel"];
                        if (cv.contains("linear")) lin = cv["linear"].get<double>();
                        if (cv.contains("angular")) ang = cv["angular"].get<double>();
                        last_lin_ = lin; last_ang_ = ang;
                        // forward opzionale ai topic di controllo
                        if (!out_cmd_vel_topic_.empty()) mqtt_->publish(out_cmd_vel_topic_, cv.dump(), 0, false);
                        publishPicoMotors(lin, ang);
                        return;
                    }
                    // blade
                    if (m.contains("blade") && m["blade"].is_object()) {
                        const auto& b = m["blade"];
                        if (b.contains("on")) {
                            blade_on_ = b["on"].get<bool>();
                            if (!out_blade_cmd_topic_.empty()) mqtt_->publish(out_blade_cmd_topic_, b.dump(), 0, false);
                            publishPicoMotors(last_lin_, last_ang_);
                            return;
                        }
                    }
                    // exit/resume
                    if ((m.contains("exit") && m["exit"].is_boolean() && m["exit"].get<bool>()) ||
                        (m.contains("resume") && m["resume"].is_boolean() && m["resume"].get<bool>())) {
                        sm_->handleEvent(EventType::END_MANUAL_CONTROL);
                        return;
                    }
                }
            }
        } catch (...) { /* ignora parse */ }
        // Se non è JSON manuale valido, continua ai branch legacy sotto
    }

    // Pass-through dei comandi manuali
    if (!manual_cmd_vel_topic_.empty() && topic == manual_cmd_vel_topic_) {
        if (!out_cmd_vel_topic_.empty()) mqtt_->publish(out_cmd_vel_topic_, payload, 0, false);
        // 2) Inoltra a Pico: traduci linear/angular in velocità ruote/blade mantenendo stato blade
        try{
            auto j = nlohmann::json::parse(payload);
            if(j.is_object()){
                double lin = 0.0, ang = 0.0;
                if(j.contains("linear")) lin = j["linear"].get<double>();
                if(j.contains("angular")) ang = j["angular"].get<double>();
                last_lin_ = lin; last_ang_ = ang;
                publishPicoMotors(lin, ang);
            }
        }catch(...){ /* ignora */ }
        return;
    }
    if (!manual_blade_topic_.empty() && topic == manual_blade_topic_) {
        if (!out_blade_cmd_topic_.empty()) mqtt_->publish(out_blade_cmd_topic_, payload, 0, false);
        // Aggiorna stato lama e inoltra a Pico come parte dei motori (blade1/2)
        try{
            auto j = nlohmann::json::parse(payload);
            if(j.is_object() && j.contains("on")){
                blade_on_ = j["on"].get<bool>();
                // reinvia motori con stesso lin/ang ma blade aggiornata
                publishPicoMotors(last_lin_, last_ang_);
            }
        }catch(...){ /* ignora */ }
        return;
    }
    if (!manual_exit_topic_.empty() && topic == manual_exit_topic_) {
        // Accetta diversi formati per "uscire" dal manuale: true, "exit", "resume"
        bool exit_manual = false;
        try {
            auto j = nlohmann::json::parse(payload);
            if (j.is_object()) {
                if (j.contains("exit") && j["exit"].is_boolean()) exit_manual = j["exit"].get<bool>();
                else if (j.contains("resume") && j["resume"].is_boolean()) exit_manual = j["resume"].get<bool>();
                else if (j.contains("cmd") && j["cmd"].is_string()) {
                    auto s = tolower_copy_ms(j["cmd"].get<std::string>());
                    exit_manual = (s == "exit" || s == "resume");
                }
            } else if (j.is_boolean()) {
                exit_manual = j.get<bool>();
            } else if (j.is_string()) {
                auto s = tolower_copy_ms(j.get<std::string>());
                exit_manual = (s == "exit" || s == "resume" || s == "true" || s == "1");
            }
        } catch (...) {
            auto s = tolower_copy_ms(payload);
            exit_manual = (s == "exit" || s == "resume" || s == "true" || s == "1");
        }
        if (exit_manual) {
            sm_->handleEvent(EventType::END_MANUAL_CONTROL);
        }
        return;
    }
}

} // namespace sm
