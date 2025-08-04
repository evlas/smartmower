#include "../../include/states/docking_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

// Istanza globale dello stato di docking
static DockingState docking_state_instance;

void docking_state_init(DockingState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_DOCKING, "DOCKING");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = docking_state_on_enter;
    state->base.base.on_exit = docking_state_on_exit;
    state->base.base.on_update = docking_state_on_update;
    state->base.base.on_event = docking_state_on_event;
    
    // Inizializza i campi specifici
    state->docking_start_time = 0;
    state->dock_x = 0.0;
    state->dock_y = 0.0;
    state->path_calculated = false;
    state->approaching_dock = false;
    state->docking_complete = false;
    state->retry_count = 0;
    
    // Timeout verrà impostato in on_enter dalla configurazione centralizzata
    state->base.timeout_enabled = true;
}

void docking_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Avvio procedura di docking");
    
    DockingState* state = (DockingState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.docking_timeout;
    
    state->docking_start_time = time(NULL);
    state->path_calculated = false;
    state->approaching_dock = false;
    state->docking_complete = false;
    state->retry_count = 0;
    
    // Posizione del dock (configurabile dal config)
    state->dock_x = 0.0;
    state->dock_y = 0.0;
    
    // Verifica e accendi il relè del Pico se necessario (per navigazione verso dock)
    send_robot_command(machine, "relay_on", "docking_navigation");
    log_state_message(machine, "INFO", "Relè Pico acceso per navigazione verso dock");
    
    // Ferma il taglio se attivo
    send_robot_command(machine, "stop_mowing", "docking");
    
    // Salva la posizione corrente per eventuale ripresa
    machine->robot_data->last_mowing_x = machine->robot_data->x;
    machine->robot_data->last_mowing_y = machine->robot_data->y;
    
    publish_state_change(machine, "Procedura di docking avviata");
}

void docking_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    DockingState* state = (DockingState*)machine->current_state;
    
    // Controlla il timeout
    base_state_on_update(machine);
    
    // Calcola il percorso verso il dock se non ancora fatto
    if (!state->path_calculated) {
        char path_params[64];
        snprintf(path_params, sizeof(path_params), "%.2f,%.2f", state->dock_x, state->dock_y);
        send_robot_command(machine, "calculate_path_to_dock", path_params);
        state->path_calculated = true;
        log_state_message(machine, "INFO", "Percorso verso il dock calcolato");
    }
    
    // Calcola la distanza dal dock
    double distance = sqrt(pow(machine->robot_data->x - state->dock_x, 2) + 
                          pow(machine->robot_data->y - state->dock_y, 2));
    
    if (distance < 1.0 && !state->approaching_dock) {
        // Entro 1 metro dal dock, inizia l'approccio finale
        state->approaching_dock = true;
        send_robot_command(machine, "approach_dock", "precision");
        log_state_message(machine, "INFO", "Approccio finale al dock");
    }
    
    // Rileva docking completato tramite battery_state che passa a "charging"
    if (strcmp(machine->robot_data->battery_state, "charging") == 0 && 
        strcmp(machine->robot_data->previous_battery_state, "charging") != 0) {
        
        // Il robot si è collegato fisicamente al dock e inizia la carica
        state->docking_complete = true;
        log_state_message(machine, "INFO", "Docking completato - rilevata carica batteria");
        printf("DOCKING: battery_state transizione %s -> %s\n", 
               machine->robot_data->previous_battery_state, 
               machine->robot_data->battery_state);
        
        state_machine_handle_event(machine, EVENT_DOCKING_COMPLETE);
    }
    
    // Fallback: se molto vicino al dock e battery_state già "charging"
    else if (distance < 0.3 && strcmp(machine->robot_data->battery_state, "charging") == 0) {
        if (!state->docking_complete) {
            state->docking_complete = true;
            log_state_message(machine, "INFO", "Docking già completato (batteria in carica)");
            state_machine_handle_event(machine, EVENT_DOCKING_COMPLETE);
        }
    }
    
    // Controlla ostacoli durante il docking
    if (has_obstacle(machine) && !state->approaching_dock) {
        log_state_message(machine, "WARNING", "Ostacolo rilevato durante docking");
        send_robot_command(machine, "navigate_around_obstacle", "to_dock");
    }
    
    // Controlla se la batteria è troppo bassa per continuare
    if (machine->robot_data->battery_level < 5.0) {
        log_state_message(machine, "CRITICAL", "Batteria critica durante docking");
        send_robot_command(machine, "emergency_dock", "low_battery");
    }
}

void docking_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    DockingState* state = (DockingState*)machine->current_state;
    
    time_t docking_duration = time(NULL) - state->docking_start_time;
    
    char exit_msg[256];
    if (state->docking_complete) {
        snprintf(exit_msg, sizeof(exit_msg), 
                "Docking completato in %ld secondi. Tentativi: %d",
                docking_duration, state->retry_count + 1);
    } else {
        snprintf(exit_msg, sizeof(exit_msg), 
                "Uscita da docking dopo %ld secondi. Stato: incompleto",
                docking_duration);
    }
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, state->docking_complete ? 
                        "Docking completato con successo" : "Docking interrotto");
}

void docking_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    DockingState* state = (DockingState*)machine->current_state;
    
    switch (event) {
        case EVENT_DOCKING_COMPLETE:
            log_state_message(machine, "INFO", "Docking completato, transizione verso CHARGING");
            state_machine_transition(machine, get_charging_state());
            break;
            
        case EVENT_TIMEOUT:
            if (state->retry_count < 3) {
                state->retry_count++;
                state->path_calculated = false;
                state->approaching_dock = false;
                log_state_message(machine, "WARNING", "Timeout docking, nuovo tentativo");
                char retry_msg[64];
                snprintf(retry_msg, sizeof(retry_msg), "Tentativo %d/3", state->retry_count + 1);
                publish_state_change(machine, retry_msg);
            } else {
                log_state_message(machine, "ERROR", "Docking fallito dopo 3 tentativi");
                state_machine_transition(machine, get_error_state());
            }
            break;
            
        case EVENT_EMERGENCY_STOP:
            log_state_message(machine, "CRITICAL", "Arresto di emergenza durante docking");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_OBSTACLE_DETECTED:
            if (state->approaching_dock) {
                log_state_message(machine, "WARNING", "Ostacolo durante approccio finale");
                state->approaching_dock = false;
                send_robot_command(machine, "back_and_retry", "dock_approach");
            }
            break;
            
        default:
            // Delega agli eventi base
            base_state_on_event(machine, event);
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_docking_state(void) {
    static bool initialized = false;
    if (!initialized) {
        docking_state_init(&docking_state_instance);
        initialized = true;
    }
    return (State*)&docking_state_instance;
}
