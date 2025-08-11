#include "../../include/states/undocking_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Istanza globale dello stato di undocking
static UndockingState undocking_state_instance;

void undocking_state_init(UndockingState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_UNDOCKING, "UNDOCKING");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = undocking_state_on_enter;
    state->base.base.on_exit = undocking_state_on_exit;
    state->base.base.on_update = undocking_state_on_update;
    state->base.base.on_event = undocking_state_on_event;
    
    // Inizializza i campi specifici
    state->undocking_start_time = 0;
    state->target_x = 0.0;
    state->target_y = 0.0;
    state->target_orientation = 0.0;
    state->path_calculated = false;
    state->motors_started = false;
    state->target_type = UNDOCKING_TARGET_DEFAULT;
    strcpy(state->target_description, "Non inizializzato");
    
    // Timeout verrà impostato in on_enter dalla configurazione centralizzata
    state->base.timeout_enabled = true;
}

// Funzione per determinare la destinazione intelligente dell'undocking
void determine_undocking_target(StateMachine* machine, UndockingState* state) {
    if (!machine || !state) return;
    
    RobotData* data = machine->robot_data;
    
    // Analizza il contesto per determinare la destinazione appropriata
    
    // CASO 0: Override manuale - destinazione specifica richiesta
    if (data->has_undocking_override) {
        state->target_x = data->override_target_x;
        state->target_y = data->override_target_y;
        state->target_orientation = data->override_target_orientation;
        state->target_type = UNDOCKING_TARGET_AREA;
        snprintf(state->target_description, sizeof(state->target_description),
                "Area specifica: %s", data->override_area_name);
        
        log_state_message(machine, "INFO", "Undocking: destinazione manuale specificata");
        
        // Reset dell'override dopo averlo usato
        data->has_undocking_override = false;
        memset(data->override_area_name, 0, sizeof(data->override_area_name));
    }
    // CASO 1: Ripresa dopo battery_low - torna alla posizione+direzione salvata
    else if (data->last_mowing_x != 0.0 || data->last_mowing_y != 0.0) {
        // Ha una posizione salvata da riprendere
        state->target_x = data->last_mowing_x;
        state->target_y = data->last_mowing_y;
        state->target_orientation = data->orientation;  // Ripristina anche la direzione
        state->target_type = UNDOCKING_TARGET_RESUME;
        snprintf(state->target_description, sizeof(state->target_description),
                "Ripresa lavoro a (%.2f, %.2f, %.1f°)", 
                state->target_x, state->target_y, state->target_orientation * 180.0 / M_PI);
        
        log_state_message(machine, "INFO", "Undocking: ripresa posizione+direzione salvata");
        
        // Reset della posizione salvata dopo averla usata
        data->last_mowing_x = 0.0;
        data->last_mowing_y = 0.0;
        // Nota: orientation non viene resettato perché è la direzione corrente
    }
    // CASO 2: Missione incompleta - continua nell'area corrente
    else if (!data->mission_complete && data->area_covered > 0) {
        // Missione in corso, va a una posizione strategica nell'area
        state->target_x = 5.0;  // Posizione nell'area di lavoro
        state->target_y = 3.0;
        state->target_orientation = 0.0;  // Orientamento standard per area
        state->target_type = UNDOCKING_TARGET_AREA;
        snprintf(state->target_description, sizeof(state->target_description),
                "Continua missione (%.1f%% completata)", 
                (data->area_covered / data->total_area) * 100);
        
        log_state_message(machine, "INFO", "Undocking: continua missione in corso");
    }
    // CASO 3: Nuova missione - posizione sicura standard
    else {
        // Posizione sicura standard a 2m dal dock
        state->target_x = 2.0;
        state->target_y = 0.0;
        state->target_orientation = 0.0;  // Orientamento standard
        state->target_type = UNDOCKING_TARGET_DEFAULT;
        snprintf(state->target_description, sizeof(state->target_description),
                "Posizione sicura standard");
        
        log_state_message(machine, "INFO", "Undocking: posizione sicura standard");
    }
    
    // Log della destinazione scelta
    char target_msg[128];
    snprintf(target_msg, sizeof(target_msg),
            "Destinazione undocking: %s -> (%.2f, %.2f, %.1f°)",
            state->target_description, state->target_x, state->target_y,
            state->target_orientation * 180.0 / M_PI);
    log_state_message(machine, "INFO", target_msg);
}

// Funzione helper per impostare override di destinazione
void set_undocking_area_override(StateMachine* machine, const char* area_name, 
                                double x, double y, double orientation) {
    if (!machine || !area_name) return;
    
    RobotData* data = machine->robot_data;
    
    // Imposta i parametri di override
    data->has_undocking_override = true;
    data->override_target_x = x;
    data->override_target_y = y;
    data->override_target_orientation = orientation;
    
    // Copia il nome dell'area (con sicurezza)
    strncpy(data->override_area_name, area_name, sizeof(data->override_area_name) - 1);
    data->override_area_name[sizeof(data->override_area_name) - 1] = '\0';
    
    // Log dell'impostazione
    char log_msg[128];
    snprintf(log_msg, sizeof(log_msg),
            "Override undocking impostato: %s -> (%.2f, %.2f, %.1f°)",
            area_name, x, y, orientation * 180.0 / M_PI);
    log_state_message(machine, "INFO", log_msg);
}

void undocking_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Inizio procedura di undocking...");
    
    UndockingState* state = (UndockingState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.undocking_timeout;
    
    state->undocking_start_time = time(NULL);
    state->path_calculated = false;
    state->motors_started = false;
    
    // PRIMA AZIONE: Attiva il relè del Pico per alimentare i sistemi di movimento
    send_robot_command(machine, "relay_on", "power_systems");
    log_state_message(machine, "INFO", "Relè Pico attivato - sistemi alimentati");
    
    // Determina la destinazione intelligente basata sul contesto
    determine_undocking_target(machine, state);
    
    // Ferma la ricarica se attiva
    if (machine->robot_data->charging) {
        send_robot_command(machine, "stop_charging", "");
        machine->robot_data->charging = false;
        log_state_message(machine, "INFO", "Ricarica interrotta per undocking");
    }
    
    // Avvia i motori di movimento (ora che sono alimentati)
    send_robot_command(machine, "start_motors", "undocking");
    state->motors_started = true;
    
    publish_state_change(machine, "Procedura di undocking avviata");
}

void undocking_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    UndockingState* state = (UndockingState*)machine->current_state;
    
    // Controlla il timeout
    base_state_on_update(machine);
    
    // Calcola il percorso se non ancora fatto
    if (!state->path_calculated) {
        char path_params[128];
        snprintf(path_params, sizeof(path_params), "%.2f,%.2f,%.3f", 
                state->target_x, state->target_y, state->target_orientation);
        send_robot_command(machine, "calculate_path", path_params);
        state->path_calculated = true;
        log_state_message(machine, "INFO", "Percorso di undocking calcolato");
    }
    
    // Controlla se ha raggiunto il punto di partenza
    double distance = sqrt(pow(machine->robot_data->x - state->target_x, 2) + 
                          pow(machine->robot_data->y - state->target_y, 2));
    
    if (distance < 0.3) { // Entro 30cm dal target
        log_state_message(machine, "INFO", "Punto di partenza raggiunto");
        state_machine_handle_event(machine, EVENT_UNDOCKING_COMPLETE);
    }
    
    // Controlla ostacoli durante l'undocking
    if (has_obstacle(machine)) {
        log_state_message(machine, "WARNING", "Ostacolo rilevato durante undocking");
        send_robot_command(machine, "stop_motors", "obstacle");
        // Riprova dopo una breve pausa
        send_robot_command(machine, "navigate_around_obstacle", "");
    }
}

void undocking_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    UndockingState* state = (UndockingState*)machine->current_state;
    
    time_t undocking_duration = time(NULL) - state->undocking_start_time;
    
    char exit_msg[256];
    snprintf(exit_msg, sizeof(exit_msg), 
            "Undocking completato in %ld secondi. Posizione: (%.2f, %.2f)",
            undocking_duration, machine->robot_data->x, machine->robot_data->y);
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, "Undocking completato con successo");
}

void undocking_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    switch (event) {
        case EVENT_UNDOCKING_COMPLETE:
            log_state_message(machine, "INFO", "Undocking completato, transizione verso MOWING");
            state_machine_transition(machine, get_mowing_state());
            break;
            
        case EVENT_TIMEOUT:
            log_state_message(machine, "ERROR", "Timeout durante undocking");
            state_machine_transition(machine, get_error_state());
            break;
            
        case EVENT_EMERGENCY_STOP:
            log_state_message(machine, "CRITICAL", "Arresto di emergenza durante undocking");
            state_machine_transition(machine, get_emergency_stop_state());
            break;
            
        case EVENT_OBSTACLE_DETECTED:
            log_state_message(machine, "WARNING", "Ostacolo persistente durante undocking");
            // Rimane nello stato e riprova
            break;
            
        default:
            // Delega agli eventi base
            base_state_on_event(machine, event);
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_undocking_state(void) {
    static bool initialized = false;
    if (!initialized) {
        undocking_state_init(&undocking_state_instance);
        initialized = true;
    }
    return (State*)&undocking_state_instance;
}
