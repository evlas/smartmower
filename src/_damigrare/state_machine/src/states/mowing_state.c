#include "../../include/states/mowing_state.h"
#include "state_machine_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <time.h>

// Istanza globale dello stato di taglio
static MowingState mowing_state_instance;

void mowing_state_init(MowingState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_MOWING, "MOWING");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = mowing_state_on_enter;
    state->base.base.on_exit = mowing_state_on_exit;
    state->base.base.on_update = mowing_state_on_update;
    state->base.base.on_event = mowing_state_on_event;
    
    // Inizializza i campi specifici
    state->area_covered = 0.0;
    state->target_area = 6000.0; // 6000 mq come richiesto
    state->mowing_start_time = 0;
    state->pattern_initialized = false;
    state->current_pattern_step = 0;
    state->last_position_x = 0.0;
    state->last_position_y = 0.0;
    
    // Nessun timeout per lo stato di taglio (può durare ore)
    state->base.timeout_enabled = false;
}

void mowing_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Avvio del taglio dell'erba");
    
    MowingState* state = (MowingState*)machine->current_state;
    
    // Inizializza il tempo di inizio
    state->mowing_start_time = time(NULL);
    
    // Salva la posizione corrente
    state->last_position_x = machine->robot_data->x;
    state->last_position_y = machine->robot_data->y;
    
    // Verifica e accendi il relè del Pico se necessario (per motori e sistemi)
    send_robot_command(machine, "relay_on", "mowing_power");
    log_state_message(machine, "INFO", "Relè Pico acceso per alimentare sistemi di taglio");
    
    // Avvia i motori di taglio (ora che sono alimentati)
    send_robot_command(machine, "start_mowing", "");
    
    // Inizializza il pattern di taglio
    if (!state->pattern_initialized) {
        send_robot_command(machine, "init_pattern", "spiral");
        state->pattern_initialized = true;
        log_state_message(machine, "INFO", "Pattern di taglio a spirale inizializzato");
    }
    
    publish_state_change(machine, "Taglio dell'erba avviato");
}

void mowing_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    MowingState* state = (MowingState*)machine->current_state;
    
    // Controlla il livello della batteria
    if (is_battery_low(machine)) {
        log_state_message(machine, "WARNING", "Batteria bassa, ritorno alla base");
        state_machine_handle_event(machine, EVENT_BATTERY_LOW);
        return;
    }
    
    // Controlla ostacoli
    if (has_obstacle(machine)) {
        log_state_message(machine, "WARNING", "Ostacolo rilevato");
        state_machine_handle_event(machine, EVENT_OBSTACLE_DETECTED);
        return;
    }
    
    // Controlla il perimetro
    if (is_perimeter_detected(machine)) {
        log_state_message(machine, "WARNING", "Perimetro attraversato");
        state_machine_handle_event(machine, EVENT_PERIMETER_CROSSED);
        return;
    }
    
    // Calcola l'area coperta basandosi sul movimento
    double current_x = machine->robot_data->x;
    double current_y = machine->robot_data->y;
    
    double distance_moved = sqrt(pow(current_x - state->last_position_x, 2) + 
                                pow(current_y - state->last_position_y, 2));
    
    if (distance_moved > 0.1) { // Movimento minimo di 10cm
        // Stima dell'area coperta usando larghezza lama dalla configurazione
        double cutting_width = machine->robot_data->config ? 
                              machine->robot_data->config->robot.area.cutting_width_m : 0.30; // Default 30cm
        
        state->area_covered += distance_moved * cutting_width;
        state->last_position_x = current_x;
        state->last_position_y = current_y;
        
        // Aggiorna i dati del robot
        machine->robot_data->area_covered = state->area_covered;
    }
    
    // Controlla completamento basato sulla modalità configurata
    bool completion_reached = false;
    char completion_msg[128];
    
    // Modalità 1: Area specifica (se target_area è impostato)
    if (state->target_area > 0 && state->area_covered >= state->target_area) {
        completion_reached = true;
        snprintf(completion_msg, sizeof(completion_msg), 
                "Area target completata: %.1f/%.1f mq", 
                state->area_covered, state->target_area);
    }
    // Modalità 2: Prato completo (se target_area non è impostato o è 0)
    else if (state->target_area == 0 && machine->robot_data->config) {
        double total_lawn_area = machine->robot_data->config->robot.area.total_area_sqm;
        if (state->area_covered >= total_lawn_area) {
            completion_reached = true;
            snprintf(completion_msg, sizeof(completion_msg), 
                    "Prato completo tagliato: %.1f/%.1f mq", 
                    state->area_covered, total_lawn_area);
        }
    }
    
    if (completion_reached) {
        log_state_message(machine, "INFO", completion_msg);
        machine->robot_data->mission_complete = true;
        state_machine_handle_event(machine, EVENT_AREA_COMPLETE);
        return;
    }
    
    // Avanza nel pattern di taglio
    state->current_pattern_step++;
    if (state->current_pattern_step % 100 == 0) {
        char step_msg[64];
        snprintf(step_msg, sizeof(step_msg), "step_%d", state->current_pattern_step);
        send_robot_command(machine, "next_pattern_step", step_msg);
    }
    
    // Log periodico del progresso
    static int log_counter = 0;
    log_counter++;
    if (log_counter % 1000 == 0) { // Ogni 1000 cicli
        char progress_msg[128];
        snprintf(progress_msg, sizeof(progress_msg), 
                "Progresso: %.1f/%.1f mq (%.1f%%)", 
                state->area_covered, state->target_area,
                (state->area_covered / state->target_area) * 100.0);
        log_state_message(machine, "INFO", progress_msg);
    }
}

void mowing_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    MowingState* state = (MowingState*)machine->current_state;
    
    // Ferma i motori di taglio
    send_robot_command(machine, "stop_mowing", "");
    
    // Calcola il tempo totale di taglio
    time_t total_time = time(NULL) - state->mowing_start_time;
    
    char exit_msg[256];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Taglio terminato. Area coperta: %.1f mq, Tempo: %ld secondi",
            state->area_covered, total_time);
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, exit_msg);
}

void mowing_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    switch (event) {
        case EVENT_BATTERY_LOW:
            log_state_message(machine, "INFO", "Batteria bassa, transizione verso DOCKING");
            state_machine_transition(machine, get_docking_state());
            break;
            
        case EVENT_OBSTACLE_DETECTED:
            log_state_message(machine, "INFO", "Ostacolo rilevato, messa in pausa");
            state_machine_transition(machine, get_paused_state());
            break;
            
        case EVENT_PERIMETER_CROSSED:
            log_state_message(machine, "WARNING", "Perimetro attraversato, fermata di emergenza");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_AREA_COMPLETE:
            log_state_message(machine, "INFO", "Area completata, ritorno alla base");
            machine->robot_data->mission_complete = true;
            state_machine_transition(machine, get_docking_state());
            break;
            
        case EVENT_PAUSE:
            log_state_message(machine, "INFO", "Taglio messo in pausa");
            state_machine_transition(machine, get_paused_state());
            break;
            
        case EVENT_MANUAL_CONTROL:
            log_state_message(machine, "INFO", "Passaggio al controllo manuale");
            state_machine_transition(machine, get_manual_control_state());
            break;
            
        default:
            // Delega agli eventi base
            base_state_on_event(machine, event);
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_mowing_state(void) {
    static bool initialized = false;
    if (!initialized) {
        mowing_state_init(&mowing_state_instance);
        initialized = true;
    }
    return (State*)&mowing_state_instance;
}
