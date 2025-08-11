#include "../include/state_machine.h"
#include "../include/states/base_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <unistd.h>

// Implementazione delle funzioni principali della macchina a stati

void state_machine_init(StateMachine* machine, State* initial_state, RobotData* robot_data) {
    if (!machine || !initial_state || !robot_data) {
        fprintf(stderr, "Errore: parametri NULL nella inizializzazione della macchina a stati\n");
        return;
    }
    
    machine->current_state = initial_state;
    machine->previous_state = NULL;
    machine->robot_data = robot_data;
    machine->running = true;
    machine->state_start_time = time(NULL);
    
    printf("Macchina a stati inizializzata con stato: %s\n", initial_state->name);
    
    // Notifica l'ingresso nello stato iniziale
    if (machine->current_state && machine->current_state->on_enter) {
        machine->current_state->on_enter(machine);
    }
}

void state_machine_transition(StateMachine* machine, State* new_state) {
    if (!machine || !new_state) {
        fprintf(stderr, "Errore: parametri NULL nella transizione di stato\n");
        return;
    }
    
    if (machine->current_state == new_state) {
        return; // Già nello stato richiesto
    }
    
    printf("Transizione di stato: %s -> %s\n", 
           machine->current_state ? machine->current_state->name : "NULL",
           new_state->name);
    
    // Pubblica la transizione di stato via MQTT PRIMA del cambio
    const char* from_state_name = machine->current_state ? machine->current_state->name : "NULL";
    const char* to_state_name = new_state->name;
    publish_state_transition(machine, from_state_name, to_state_name, "transition", "State machine transition");
    
    // Notifica l'uscita dallo stato corrente
    if (machine->current_state && machine->current_state->on_exit) {
        machine->current_state->on_exit(machine);
    }
    
    // Salva lo stato precedente
    machine->previous_state = machine->current_state;
    
    // Cambia stato
    machine->current_state = new_state;
    machine->state_start_time = time(NULL);
    
    // Notifica l'ingresso nel nuovo stato
    if (machine->current_state->on_enter) {
        machine->current_state->on_enter(machine);
    }
    
    // Pubblica il nuovo stato corrente via MQTT
    publish_state_change(machine, "State transition completed");
}

void state_machine_update(StateMachine* machine) {
    if (!machine || !machine->running) {
        return;
    }
    
    // Aggiorna i dati del robot dai sensori
    update_robot_data_from_mqtt(machine);
    
    // Aggiorna lo stato corrente
    if (machine->current_state && machine->current_state->on_update) {
        machine->current_state->on_update(machine);
    }
    
    // Pubblica periodicamente lo stato corrente (ogni 5 secondi)
    static time_t last_state_publish = 0;
    time_t current_time = time(NULL);
    if (current_time - last_state_publish >= 5) {
        publish_state_change(machine, "Periodic state update");
        last_state_publish = current_time;
    }
    
    // Aggiorna il timestamp
    machine->robot_data->last_update = current_time;
}

void state_machine_handle_event(StateMachine* machine, EventType event) {
    if (!machine || event == EVENT_NONE) {
        return;
    }
    
    printf("Gestione evento: %s nello stato %s\n", 
           event_type_to_string(event),
           machine->current_state ? machine->current_state->name : "NULL");
    
    // Pubblica l'evento via MQTT per eventi importanti
    const char* event_name = event_type_to_string(event);
    char event_data[256];
    snprintf(event_data, sizeof(event_data), "Event %s in state %s", 
             event_name, machine->current_state ? machine->current_state->name : "NULL");
    
    // Pubblica eventi critici
    switch (event) {
        case EVENT_EMERGENCY_STOP:
        case EVENT_ERROR_OCCURRED:
        case EVENT_BATTERY_LOW:
        case EVENT_BATTERY_FULL:
        case EVENT_TIMEOUT:
        case EVENT_OBSTACLE_DETECTED:
        case EVENT_PERIMETER_CROSSED:
            publish_state_event(machine, event_name, event_data);
            break;
        default:
            // Per altri eventi, pubblica solo se sono significativi
            if (event != EVENT_NONE) {
                publish_state_event(machine, event_name, event_data);
            }
            break;
    }
    
    // Gestione eventi globali (validi in tutti gli stati)
    switch (event) {
        case EVENT_EMERGENCY_STOP:
            state_machine_transition(machine, get_emergency_stop_state());
            return;
        case EVENT_ERROR_OCCURRED:
            state_machine_transition(machine, get_error_state());
            return;
        default:
            break;
    }
    
    // Delega la gestione dell'evento allo stato corrente
    if (machine->current_state && machine->current_state->on_event) {
        machine->current_state->on_event(machine, event);
    }
}

void state_machine_shutdown(StateMachine* machine) {
    if (!machine) {
        return;
    }
    
    printf("Spegnimento della macchina a stati\n");
    
    // Notifica l'uscita dallo stato corrente
    if (machine->current_state && machine->current_state->on_exit) {
        machine->current_state->on_exit(machine);
    }
    
    machine->running = false;
    machine->current_state = NULL;
    machine->previous_state = NULL;
}

// Funzioni di utilità

const char* state_type_to_string(StateType type) {
    switch (type) {
        case STATE_INIT: return "INIT";
        case STATE_IDLE: return "IDLE";
        case STATE_UNDOCKING: return "UNDOCKING";
        case STATE_MOWING: return "MOWING";
        case STATE_DOCKING: return "DOCKING";
        case STATE_CHARGING: return "CHARGING";
        case STATE_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case STATE_MANUAL_CONTROL: return "MANUAL_CONTROL";
        case STATE_ERROR: return "ERROR";
        case STATE_PAUSED: return "PAUSED";
        default: return "UNKNOWN";
    }
}

const char* event_type_to_string(EventType event) {
    switch (event) {
        case EVENT_NONE: return "NONE";
        case EVENT_INIT_COMPLETE: return "INIT_COMPLETE";
        case EVENT_START_MOWING: return "START_MOWING";
        case EVENT_BATTERY_LOW: return "BATTERY_LOW";
        case EVENT_BATTERY_FULL: return "BATTERY_FULL";
        case EVENT_EMERGENCY_STOP: return "EMERGENCY_STOP";
        case EVENT_MANUAL_CONTROL: return "MANUAL_CONTROL";
        case EVENT_ERROR_OCCURRED: return "ERROR_OCCURRED";
        case EVENT_PAUSE: return "PAUSE";
        case EVENT_RESUME: return "RESUME";
        case EVENT_EMERGENCY_RECOVER: return "EMERGENCY_RECOVER";
        case EVENT_DOCKING_COMPLETE: return "DOCKING_COMPLETE";
        case EVENT_UNDOCKING_COMPLETE: return "UNDOCKING_COMPLETE";
        case EVENT_OBSTACLE_DETECTED: return "OBSTACLE_DETECTED";
        case EVENT_PERIMETER_CROSSED: return "PERIMETER_CROSSED";
        case EVENT_AREA_COMPLETE: return "AREA_COMPLETE";
        case EVENT_TIMEOUT: return "TIMEOUT";
        case EVENT_PICO_CONNECTED: return "PICO_CONNECTED";
        case EVENT_GPS_CONNECTED: return "GPS_CONNECTED";
        case EVENT_SENSOR_FUSION_READY: return "SENSOR_FUSION_READY";
        default: return "UNKNOWN";
    }
}

double get_time_in_state(StateMachine* machine) {
    if (!machine) {
        return 0.0;
    }
    
    time_t current_time = time(NULL);
    return difftime(current_time, machine->state_start_time);
}
