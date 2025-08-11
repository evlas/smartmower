#include "../../include/states/error_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Istanza globale dello stato di errore
static ErrorState error_state_instance;

void error_state_init(ErrorState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_ERROR, "ERROR");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = error_state_on_enter;
    state->base.base.on_exit = error_state_on_exit;
    state->base.base.on_update = error_state_on_update;
    state->base.base.on_event = error_state_on_event;
    
    // Inizializza i campi specifici
    state->error_start_time = 0;
    strcpy(state->error_message, "Errore generico");
    state->error_code = 0;
    state->recovery_attempted = false;
    state->recovery_attempts = 0;
    state->previous_state_type = STATE_IDLE;
    
    // Timeout verrà impostato in on_enter dalla configurazione centralizzata
    state->base.timeout_enabled = true;
}

void error_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    ErrorState* state = (ErrorState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.error_timeout;
    
    log_state_message(machine, "ERROR", "Sistema entrato in stato di errore");
    
    state->error_start_time = time(NULL);
    state->recovery_attempted = false;
    state->recovery_attempts = 0;
    
    // PRIMA AZIONE: Spegni il relè del Pico per sicurezza in caso di errore
    send_robot_command(machine, "relay_off", "error_safety");
    log_state_message(machine, "ERROR", "Relè Pico spento per sicurezza in stato errore");
    
    // Salva lo stato precedente
    if (machine->previous_state) {
        state->previous_state_type = machine->previous_state->type;
    } else {
        state->previous_state_type = STATE_IDLE;
    }
    
    // Ferma tutti i motori per sicurezza
    send_robot_command(machine, "stop_all_motors", "error_state");
    send_robot_command(machine, "disable_autonomous", "error");
    
    // Determina il tipo di errore e imposta il messaggio appropriato
    if (machine->robot_data->battery_level < 5.0) {
        strcpy(state->error_message, "Batteria critica");
        state->error_code = 1001;
    } else if (machine->robot_data->perimeter_detected) {
        strcpy(state->error_message, "Violazione perimetro");
        state->error_code = 2001;
    } else if (has_obstacle(machine)) {
        strcpy(state->error_message, "Ostacolo persistente");
        state->error_code = 3001;
    } else {
        strcpy(state->error_message, "Errore di sistema generico");
        state->error_code = 9999;
    }
    
    char error_details[512];
    snprintf(error_details, sizeof(error_details), 
            "ERRORE %d: %s - Tutti i sistemi fermati", 
            state->error_code, state->error_message);
    
    log_state_message(machine, "CRITICAL", error_details);
    publish_state_change(machine, error_details);
}

void error_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    ErrorState* state = (ErrorState*)machine->current_state;
    
    // Controlla il timeout
    base_state_on_update(machine);
    
    // Tentativo di recovery automatico dopo 30 secondi
    time_t current_time = time(NULL);
    if (!state->recovery_attempted && 
        (current_time - state->error_start_time) > 30) {
        
        state->recovery_attempted = true;
        state->recovery_attempts++;
        
        log_state_message(machine, "INFO", "Tentativo di recovery automatico");
        
        // Tentativo di recovery basato sul tipo di errore
        switch (state->error_code) {
            case 1001: // Batteria critica
                if (machine->robot_data->charging) {
                    log_state_message(machine, "INFO", "Recovery: batteria in ricarica");
                    if (machine->robot_data->battery_level > 10.0) {
                        log_state_message(machine, "INFO", "Recovery riuscito: batteria sufficiente");
                        state_machine_transition(machine, get_charging_state());
                        return;
                    }
                } else {
                    log_state_message(machine, "INFO", "Recovery: tentativo docking di emergenza");
                    send_robot_command(machine, "emergency_dock", "critical_battery");
                }
                break;
                
            case 2001: // Violazione perimetro
                if (!machine->robot_data->perimeter_detected) {
                    log_state_message(machine, "INFO", "Recovery riuscito: perimetro OK");
                    state_machine_transition(machine, get_idle_state());
                    return;
                }
                break;
                
            case 3001: // Ostacolo persistente
                if (!has_obstacle(machine)) {
                    log_state_message(machine, "INFO", "Recovery riuscito: ostacolo rimosso");
                    state_machine_transition(machine, get_idle_state());
                    return;
                } else {
                    send_robot_command(machine, "clear_obstacle_sensors", "");
                }
                break;
                
            default:
                log_state_message(machine, "INFO", "Recovery: reset sensori");
                send_robot_command(machine, "reset_sensors", "error_recovery");
                break;
        }
    }
    
    // Log periodico dello stato di errore
    static int log_counter = 0;
    log_counter++;
    if (log_counter % 600 == 0) { // Ogni 600 cicli (circa 1 minuto)
        char status_msg[512];
        snprintf(status_msg, sizeof(status_msg), 
                "Errore persistente: %s (Tentativi recovery: %d)",
                state->error_message, state->recovery_attempts);
        log_state_message(machine, "WARNING", status_msg);
    }
}

void error_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    ErrorState* state = (ErrorState*)machine->current_state;
    
    time_t error_duration = time(NULL) - state->error_start_time;
    
    // Riabilita i sistemi autonomi
    send_robot_command(machine, "enable_autonomous", "error_resolved");
    
    char exit_msg[512];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Uscita da stato di errore dopo %ld secondi. Errore: %s, Tentativi: %d",
            error_duration, state->error_message, state->recovery_attempts);
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, "Errore risolto - sistema ripristinato");
}

void error_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    ErrorState* state = (ErrorState*)machine->current_state;
    
    switch (event) {
        case EVENT_TIMEOUT:
            if (state->recovery_attempts < 3) {
                log_state_message(machine, "INFO", "Timeout - nuovo tentativo di recovery");
                state->recovery_attempted = false;
                state->error_start_time = time(NULL); // Reset timer
            } else {
                log_state_message(machine, "CRITICAL", "Recovery fallito - richiesto intervento manuale");
                state_machine_transition(machine, get_emergency_stop_state());
            }
            break;
            
        case EVENT_MANUAL_CONTROL:
            log_state_message(machine, "INFO", "Passaggio al controllo manuale per risoluzione errore");
            state_machine_transition(machine, get_manual_control_state());
            break;
            
        case EVENT_EMERGENCY_STOP:
            log_state_message(machine, "CRITICAL", "Arresto di emergenza da stato di errore");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_RESUME:
            log_state_message(machine, "INFO", "Ripresa forzata da stato di errore");
            // Ritorna allo stato precedente o IDLE
            if (state->previous_state_type == STATE_MOWING) {
                state_machine_transition(machine, get_mowing_state());
            } else if (state->previous_state_type == STATE_CHARGING) {
                state_machine_transition(machine, get_charging_state());
            } else {
                state_machine_transition(machine, get_idle_state());
            }
            break;
            
        default:
            // In stato di errore, ignora la maggior parte degli eventi
            log_state_message(machine, "DEBUG", "Evento ignorato in stato di errore");
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_error_state(void) {
    static bool initialized = false;
    if (!initialized) {
        error_state_init(&error_state_instance);
        initialized = true;
    }
    return (State*)&error_state_instance;
}
