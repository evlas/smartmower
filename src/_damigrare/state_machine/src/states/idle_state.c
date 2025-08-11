#include "../../include/states/idle_state.h"
#include "state_machine_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

// Istanza globale dello stato di attesa
static IdleState idle_state_instance;

void idle_state_init(IdleState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_IDLE, "IDLE");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = idle_state_on_enter;
    state->base.base.on_exit = idle_state_on_exit;
    state->base.base.on_update = idle_state_on_update;
    state->base.base.on_event = idle_state_on_event;
    
    // Inizializza i campi specifici
    state->waiting_for_command = true;
    state->last_heartbeat = 0;
    
    // Nessun timeout per lo stato idle
    state->base.timeout_enabled = false;
}

void idle_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Robot in attesa di comandi");
    
    IdleState* state = (IdleState*)machine->current_state;
    
    state->waiting_for_command = true;
    state->last_heartbeat = time(NULL);
    
    // Verifica e spegni il relè del Pico se necessario (stato di riposo)
    send_robot_command(machine, "relay_off", "idle_power_save");
    log_state_message(machine, "INFO", "Relè Pico spento per risparmio energetico in idle");
    
    // Ferma tutti i motori
    send_robot_command(machine, "stop_all_motors", "");
    
    publish_state_change(machine, "Robot in stato di attesa");
}

void idle_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    IdleState* state = (IdleState*)machine->current_state;
    
    // Invia heartbeat periodico
    time_t current_time = time(NULL);
    if (current_time - state->last_heartbeat >= 30) { // Ogni 30 secondi
        send_robot_command(machine, "heartbeat", "idle");
        state->last_heartbeat = current_time;
        log_state_message(machine, "DEBUG", "Heartbeat inviato");
    }
    
    // Controlla se la batteria è bassa (anche in idle)
    if (is_battery_low(machine) && !machine->robot_data->charging) {
        log_state_message(machine, "INFO", "Batteria bassa rilevata in idle, avvio docking");
        state_machine_handle_event(machine, EVENT_BATTERY_LOW);
    }
}

void idle_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Uscita dallo stato di attesa");
    publish_state_change(machine, "Robot attivato");
}

void idle_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    switch (event) {
        case EVENT_START_MOWING:
            log_state_message(machine, "INFO", "Comando di avvio taglio ricevuto");
            state_machine_transition(machine, get_undocking_state());
            break;
            
        case EVENT_BATTERY_LOW:
            log_state_message(machine, "INFO", "Batteria bassa, avvio docking per ricarica");
            state_machine_transition(machine, get_docking_state());
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
State* get_idle_state(void) {
    static bool initialized = false;
    if (!initialized) {
        idle_state_init(&idle_state_instance);
        initialized = true;
    }
    return (State*)&idle_state_instance;
}
