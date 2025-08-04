#include "../../include/states/paused_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Istanza globale dello stato di pausa
static PausedState paused_state_instance;

void paused_state_init(PausedState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_PAUSED, "PAUSED");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = paused_state_on_enter;
    state->base.base.on_exit = paused_state_on_exit;
    state->base.base.on_update = paused_state_on_update;
    state->base.base.on_event = paused_state_on_event;
    
    // Inizializza i campi specifici
    state->pause_start_time = 0;
    state->previous_state_type = STATE_IDLE;
    state->saved_position_x = 0.0;
    state->saved_position_y = 0.0;
    state->saved_orientation = 0.0;
    state->position_saved = false;
    strcpy(state->pause_reason, "Pausa generica");
    
    // Nessun timeout per lo stato di pausa (può durare indefinitamente)
    state->base.timeout_enabled = false;
}

void paused_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Sistema messo in pausa");
    
    PausedState* state = (PausedState*)machine->current_state;
    
    state->pause_start_time = time(NULL);
    state->position_saved = false;
    
    // Spegni il relè del Pico per risparmio energetico durante la pausa
    send_robot_command(machine, "relay_off", "pause_power_save");
    log_state_message(machine, "INFO", "Relè Pico spento per risparmio energetico in pausa");
    
    // Salva lo stato precedente
    if (machine->previous_state) {
        state->previous_state_type = machine->previous_state->type;
        
        // Determina il motivo della pausa basato sullo stato precedente
        switch (machine->previous_state->type) {
            case STATE_MOWING:
                strcpy(state->pause_reason, "Pausa durante taglio");
                break;
            case STATE_DOCKING:
                strcpy(state->pause_reason, "Pausa durante docking");
                break;
            case STATE_UNDOCKING:
                strcpy(state->pause_reason, "Pausa durante undocking");
                break;
            case STATE_CHARGING:
                strcpy(state->pause_reason, "Pausa durante ricarica");
                break;
            default:
                strcpy(state->pause_reason, "Pausa da stato sconosciuto");
                break;
        }
    } else {
        state->previous_state_type = STATE_IDLE;
        strcpy(state->pause_reason, "Pausa da stato iniziale");
    }
    
    // Salva la posizione corrente
    state->saved_position_x = machine->robot_data->x;
    state->saved_position_y = machine->robot_data->y;
    state->saved_orientation = machine->robot_data->orientation;
    state->position_saved = true;
    
    // Ferma tutti i movimenti ma mantieni i sistemi attivi
    send_robot_command(machine, "pause_all_motors", "paused");
    
    // Se era in taglio, ferma le lame
    if (state->previous_state_type == STATE_MOWING) {
        send_robot_command(machine, "stop_mowing", "paused");
    }
    
    // Se era in ricarica, mantieni la ricarica attiva
    if (state->previous_state_type == STATE_CHARGING) {
        send_robot_command(machine, "maintain_charging", "paused");
    }
    
    char pause_msg[256];
    snprintf(pause_msg, sizeof(pause_msg), 
            "%s - Posizione salvata: (%.2f, %.2f)",
            state->pause_reason, state->saved_position_x, state->saved_position_y);
    
    log_state_message(machine, "INFO", pause_msg);
    publish_state_change(machine, pause_msg);
}

void paused_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    PausedState* state = (PausedState*)machine->current_state;
    
    // Mantieni i sistemi vitali attivi
    static int heartbeat_counter = 0;
    heartbeat_counter++;
    
    if (heartbeat_counter % 300 == 0) { // Ogni 300 cicli (circa 30 secondi)
        send_robot_command(machine, "heartbeat", "paused");
        
        time_t pause_duration = time(NULL) - state->pause_start_time;
        char status_msg[128];
        snprintf(status_msg, sizeof(status_msg), 
                "Sistema in pausa da %ld secondi", pause_duration);
        log_state_message(machine, "INFO", status_msg);
    }
    
    // Controlla condizioni di sicurezza anche in pausa
    if (machine->robot_data->battery_level < 5.0 && 
        state->previous_state_type != STATE_CHARGING) {
        log_state_message(machine, "CRITICAL", "Batteria critica durante pausa");
        state_machine_handle_event(machine, EVENT_BATTERY_LOW);
    }
    
    // Controlla emergenze anche in pausa
    if (machine->robot_data->perimeter_detected) {
        log_state_message(machine, "CRITICAL", "Perimetro attraversato durante pausa");
        state_machine_handle_event(machine, EVENT_EMERGENCY_STOP);
    }
    
    // Se era in ricarica e la batteria è piena, segnala
    if (state->previous_state_type == STATE_CHARGING && 
        is_battery_full(machine)) {
        log_state_message(machine, "INFO", "Batteria piena durante pausa");
        publish_state_change(machine, "Batteria carica - pronto per ripresa");
    }
}

void paused_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    PausedState* state = (PausedState*)machine->current_state;
    
    time_t pause_duration = time(NULL) - state->pause_start_time;
    
    // Riaccendi il relè del Pico per riprendere le operazioni
    send_robot_command(machine, "relay_on", "resume_operations");
    log_state_message(machine, "INFO", "Relè Pico riacceso per ripresa operazioni");
    
    // Riattiva i sistemi se necessario
    send_robot_command(machine, "resume_systems", "unpaused");
    
    char exit_msg[256];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Ripresa dopo %ld secondi di pausa. Motivo: %s",
            pause_duration, state->pause_reason);
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, "Sistema ripreso dalla pausa");
}

void paused_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    PausedState* state = (PausedState*)machine->current_state;
    
    switch (event) {
        case EVENT_RESUME:
            log_state_message(machine, "INFO", "Ripresa richiesta");
            
            // Ritorna allo stato precedente
            switch (state->previous_state_type) {
                case STATE_MOWING:
                    log_state_message(machine, "INFO", "Ripresa taglio");
                    // Ripristina la posizione se necessario
                    if (state->position_saved) {
                        char pos_cmd[128];
                        snprintf(pos_cmd, sizeof(pos_cmd), "%.2f,%.2f,%.2f", 
                                state->saved_position_x, state->saved_position_y, 
                                state->saved_orientation);
                        send_robot_command(machine, "restore_position", pos_cmd);
                    }
                    state_machine_transition(machine, get_mowing_state());
                    break;
                    
                case STATE_CHARGING:
                    log_state_message(machine, "INFO", "Ripresa ricarica");
                    state_machine_transition(machine, get_charging_state());
                    break;
                    
                case STATE_DOCKING:
                    log_state_message(machine, "INFO", "Ripresa docking");
                    state_machine_transition(machine, get_docking_state());
                    break;
                    
                case STATE_UNDOCKING:
                    log_state_message(machine, "INFO", "Ripresa undocking");
                    state_machine_transition(machine, get_undocking_state());
                    break;
                    
                default:
                    log_state_message(machine, "INFO", "Ripresa verso IDLE");
                    state_machine_transition(machine, get_idle_state());
                    break;
            }
            break;
            
        case EVENT_EMERGENCY_STOP:
            log_state_message(machine, "CRITICAL", "Arresto di emergenza durante pausa");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_MANUAL_CONTROL:
            log_state_message(machine, "INFO", "Passaggio al controllo manuale da pausa");
            state_machine_transition(machine, get_manual_control_state());
            break;
            
        case EVENT_BATTERY_LOW:
            if (state->previous_state_type != STATE_CHARGING) {
                log_state_message(machine, "WARNING", "Batteria bassa durante pausa - avvio docking");
                state_machine_transition(machine, get_docking_state());
            }
            break;
            
        case EVENT_BATTERY_FULL:
            if (state->previous_state_type == STATE_CHARGING) {
                log_state_message(machine, "INFO", "Batteria piena - pausa mantenuta");
                publish_state_change(machine, "Ricarica completata - in attesa di ripresa");
            }
            break;
            
        default:
            // In pausa, la maggior parte degli eventi vengono ignorati
            log_state_message(machine, "DEBUG", "Evento ignorato durante pausa");
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_paused_state(void) {
    static bool initialized = false;
    if (!initialized) {
        paused_state_init(&paused_state_instance);
        initialized = true;
    }
    return (State*)&paused_state_instance;
}
