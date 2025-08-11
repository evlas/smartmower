#include "../../include/states/manual_control_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Istanza globale dello stato di controllo manuale
static ManualControlState manual_control_state_instance;

void manual_control_state_init(ManualControlState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_MANUAL_CONTROL, "MANUAL_CONTROL");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = manual_control_state_on_enter;
    state->base.base.on_exit = manual_control_state_on_exit;
    state->base.base.on_update = manual_control_state_on_update;
    state->base.base.on_event = manual_control_state_on_event;
    
    // Inizializza i campi specifici
    state->manual_start_time = 0;
    state->manual_active = false;
    strcpy(state->last_command, "");
    state->last_command_time = 0;
    state->previous_state_type = STATE_IDLE;
    
    // Timeout verrà impostato in on_enter dalla configurazione centralizzata
    state->base.timeout_enabled = true;
}

void manual_control_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Attivazione controllo manuale");
    
    ManualControlState* state = (ManualControlState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.manual_control_timeout;
    
    state->manual_start_time = time(NULL);
    state->manual_active = true;
    state->last_command_time = 0;
    
    // Accendi il relè del Pico per controllo manuale
    send_robot_command(machine, "relay_on", "manual_control");
    log_state_message(machine, "INFO", "Relè Pico acceso per controllo manuale");
    
    // Salva lo stato precedente per il ritorno
    if (machine->previous_state) {
        state->previous_state_type = machine->previous_state->type;
    } else {
        state->previous_state_type = STATE_IDLE;
    }
    
    // Ferma tutti i movimenti automatici
    send_robot_command(machine, "stop_all_motors", "manual_control");
    send_robot_command(machine, "disable_autonomous", "");
    
    // Abilita il controllo manuale
    send_robot_command(machine, "enable_manual_control", "");
    
    publish_state_change(machine, "Controllo manuale attivato");
    
    log_state_message(machine, "INFO", "Robot pronto per comandi manuali");
}

void manual_control_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    ManualControlState* state = (ManualControlState*)machine->current_state;
    
    // Controlla il timeout
    base_state_on_update(machine);
    
    // Controlla se non ci sono comandi da troppo tempo (timeout inattività)
    time_t current_time = time(NULL);
    if (state->last_command_time > 0 && 
        (current_time - state->last_command_time) > 120) { // 2 minuti senza comandi
        log_state_message(machine, "WARNING", "Nessun comando manuale da 2 minuti");
        // Invia heartbeat per mantenere la connessione
        send_robot_command(machine, "manual_heartbeat", "");
    }
    
    // Controlla condizioni di sicurezza anche in modalità manuale
    if (machine->robot_data->battery_level < 10.0) {
        log_state_message(machine, "WARNING", "Batteria bassa in modalità manuale");
        char battery_msg[64];
        snprintf(battery_msg, sizeof(battery_msg), "Batteria: %.1f%%", 
                machine->robot_data->battery_level);
        publish_state_change(machine, battery_msg);
    }
    
    // Controlla emergenze anche in modalità manuale
    if (machine->robot_data->perimeter_detected) {
        log_state_message(machine, "CRITICAL", "Perimetro attraversato in modalità manuale");
        state_machine_handle_event(machine, EVENT_EMERGENCY_STOP);
    }
}

void manual_control_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    ManualControlState* state = (ManualControlState*)machine->current_state;
    
    time_t manual_duration = time(NULL) - state->manual_start_time;
    
    // Spegni il relè del Pico al termine del controllo manuale
    send_robot_command(machine, "relay_off", "end_manual_control");
    log_state_message(machine, "INFO", "Relè Pico spento al termine controllo manuale");
    
    // Disabilita il controllo manuale
    send_robot_command(machine, "disable_manual_control", "");
    send_robot_command(machine, "enable_autonomous", "");
    
    char exit_msg[256];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Controllo manuale terminato dopo %ld secondi. Ultimo comando: %s",
            manual_duration, state->last_command[0] ? state->last_command : "nessuno");
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, "Controllo manuale disattivato");
}

void manual_control_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    ManualControlState* state = (ManualControlState*)machine->current_state;
    
    switch (event) {
        case EVENT_MANUAL_CONTROL:
            // Toggle: se già in modalità manuale, esci
            log_state_message(machine, "INFO", "Uscita dal controllo manuale");
            // Ritorna allo stato precedente o IDLE
            if (state->previous_state_type == STATE_MOWING) {
                state_machine_transition(machine, get_mowing_state());
            } else if (state->previous_state_type == STATE_CHARGING) {
                state_machine_transition(machine, get_charging_state());
            } else {
                state_machine_transition(machine, get_idle_state());
            }
            break;
            
        case EVENT_TIMEOUT:
            log_state_message(machine, "WARNING", "Timeout controllo manuale, ritorno automatico");
            state_machine_transition(machine, get_idle_state());
            break;
            
        case EVENT_EMERGENCY_STOP:
            log_state_message(machine, "CRITICAL", "Arresto di emergenza in modalità manuale");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_BATTERY_LOW:
            log_state_message(machine, "WARNING", "Batteria bassa in modalità manuale");
            // Suggerisci il docking ma non forzarlo
            publish_state_change(machine, "Batteria bassa - considerare il docking");
            break;
            
        case EVENT_PAUSE:
            log_state_message(machine, "INFO", "Pausa richiesta in modalità manuale");
            state_machine_transition(machine, get_paused_state());
            break;
            
        default:
            // In modalità manuale, la maggior parte degli eventi automatici vengono ignorati
            log_state_message(machine, "DEBUG", "Evento automatico ignorato in modalità manuale");
            break;
    }
    
    // Aggiorna il timestamp dell'ultimo comando
    state->last_command_time = time(NULL);
}

// Funzione per ottenere l'istanza dello stato
State* get_manual_control_state(void) {
    static bool initialized = false;
    if (!initialized) {
        manual_control_state_init(&manual_control_state_instance);
        initialized = true;
    }
    return (State*)&manual_control_state_instance;
}
