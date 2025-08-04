#include "../../include/states/charging_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Istanza globale dello stato di ricarica
static ChargingState charging_state_instance;

void charging_state_init(ChargingState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_CHARGING, "CHARGING");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = charging_state_on_enter;
    state->base.base.on_exit = charging_state_on_exit;
    state->base.base.on_update = charging_state_on_update;
    state->base.base.on_event = charging_state_on_event;
    
    // Inizializza i campi specifici
    state->charging_start_time = 0;
    state->initial_battery_level = 0.0;
    state->mission_to_resume = false;
    state->resume_position_x = 0.0;
    state->resume_position_y = 0.0;
    
    // Timeout verrà impostato in on_enter dalla configurazione centralizzata
    state->base.timeout_enabled = true;
}

void charging_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Inizio ricarica della batteria");
    
    ChargingState* state = (ChargingState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.charging_timeout;
    
    // Inizializza il tempo di inizio ricarica
    state->charging_start_time = time(NULL);
    state->initial_battery_level = machine->robot_data->battery_level;
    
    // PRIMA AZIONE: Spegni il relè del Pico per risparmio energetico
    send_robot_command(machine, "relay_off", "power_saving");
    log_state_message(machine, "INFO", "Relè Pico spento per risparmio energetico");
    
    // Controlla se c'è una missione da riprendere
    if (!machine->robot_data->mission_complete && machine->robot_data->area_covered > 0) {
        state->mission_to_resume = true;
        state->resume_position_x = machine->robot_data->x;
        state->resume_position_y = machine->robot_data->y;
        log_state_message(machine, "INFO", "Missione da riprendere dopo la ricarica");
    } else {
        state->mission_to_resume = false;
        log_state_message(machine, "INFO", "Nessuna missione da riprendere");
    }
    
    // Avvia la ricarica
    send_robot_command(machine, "start_charging", "");
    machine->robot_data->charging = true;
    
    publish_state_change(machine, "Ricarica della batteria avviata");
}

void charging_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    ChargingState* state = (ChargingState*)machine->current_state;
    
    // Controlla il timeout
    base_state_on_update(machine);
    
    // Aggiorna lo stato di ricarica
    machine->robot_data->charging = true;
    
    // Rileva carica completa tramite battery_state che passa a "idle" + is_fully_charged
    if (strcmp(machine->robot_data->battery_state, "idle") == 0 && 
        strcmp(machine->robot_data->previous_battery_state, "charging") == 0 &&
        machine->robot_data->is_fully_charged) {
        
        log_state_message(machine, "INFO", "Batteria completamente carica - rilevata fine carica");
        printf("CHARGING: battery_state transizione %s -> %s (fully_charged=%s)\n", 
               machine->robot_data->previous_battery_state, 
               machine->robot_data->battery_state,
               machine->robot_data->is_fully_charged ? "true" : "false");
        
        state_machine_handle_event(machine, EVENT_BATTERY_FULL);
        return;
    }
    
    // Fallback: se battery_level >= 95% e non in carica
    else if (machine->robot_data->battery_level >= 95.0 && 
             strcmp(machine->robot_data->battery_state, "idle") == 0) {
        
        log_state_message(machine, "INFO", "Batteria carica (95%+) - fine carica");
        state_machine_handle_event(machine, EVENT_BATTERY_FULL);
        return;
    }
    
    // Log periodico dello stato di ricarica
    static int log_counter = 0;
    log_counter++;
    if (log_counter % 600 == 0) { // Ogni 600 cicli (circa 1 minuto se chiamato ogni 100ms)
        time_t charging_time = time(NULL) - state->charging_start_time;
        double battery_gain = machine->robot_data->battery_level - state->initial_battery_level;
        
        char progress_msg[128];
        snprintf(progress_msg, sizeof(progress_msg), 
                "Ricarica in corso: %.1f%% (+%.1f%% in %ld min)", 
                machine->robot_data->battery_level, battery_gain, charging_time / 60);
        log_state_message(machine, "INFO", progress_msg);
    }
}

void charging_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    ChargingState* state = (ChargingState*)machine->current_state;
    
    // Ferma la ricarica
    send_robot_command(machine, "stop_charging", "");
    machine->robot_data->charging = false;
    
    // Calcola il tempo totale di ricarica
    time_t total_charging_time = time(NULL) - state->charging_start_time;
    double battery_gain = machine->robot_data->battery_level - state->initial_battery_level;
    
    char exit_msg[256];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Ricarica completata. Batteria: %.1f%% (+%.1f%%), Tempo: %ld min",
            machine->robot_data->battery_level, battery_gain, total_charging_time / 60);
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, exit_msg);
}

void charging_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    ChargingState* state = (ChargingState*)machine->current_state;
    
    switch (event) {
        case EVENT_BATTERY_FULL:
            if (state->mission_to_resume) {
                log_state_message(machine, "INFO", "Batteria carica, ripresa della missione");
                // Torna allo stato di taglio per riprendere la missione
                state_machine_transition(machine, get_mowing_state());
            } else {
                log_state_message(machine, "INFO", "Batteria carica, transizione verso IDLE");
                state_machine_transition(machine, get_idle_state());
            }
            break;
            
        case EVENT_TIMEOUT:
            log_state_message(machine, "ERROR", "Timeout durante la ricarica");
            state_machine_transition(machine, get_error_state());
            break;
            
        case EVENT_PAUSE:
            log_state_message(machine, "INFO", "Ricarica messa in pausa");
            state_machine_transition(machine, get_paused_state());
            break;
            
        default:
            // Delega agli eventi base
            base_state_on_event(machine, event);
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_charging_state(void) {
    static bool initialized = false;
    if (!initialized) {
        charging_state_init(&charging_state_instance);
        initialized = true;
    }
    return (State*)&charging_state_instance;
}
