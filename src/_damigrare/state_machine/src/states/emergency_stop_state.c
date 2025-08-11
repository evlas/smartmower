#include "../../include/states/emergency_stop_state.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Istanza globale dello stato di emergenza
static EmergencyStopState emergency_stop_state_instance;

void emergency_stop_state_init(EmergencyStopState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_EMERGENCY_STOP, "EMERGENCY_STOP");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = emergency_stop_state_on_enter;
    state->base.base.on_exit = emergency_stop_state_on_exit;
    state->base.base.on_update = emergency_stop_state_on_update;
    state->base.base.on_event = emergency_stop_state_on_event;
    
    // Inizializza i campi specifici
    state->emergency_start_time = 0;
    strcpy(state->emergency_reason, "Motivo sconosciuto");
    
    // Nessun timeout per lo stato di emergenza (richiede intervento manuale)
    state->base.timeout_enabled = false;
}

void emergency_stop_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "CRITICAL", "ARRESTO DI EMERGENZA ATTIVATO!");
    
    EmergencyStopState* state = (EmergencyStopState*)machine->current_state;
    
    state->emergency_start_time = time(NULL);
    
    // PRIMA AZIONE: Spegni il relè del Pico per sicurezza
    send_robot_command(machine, "relay_off", "emergency_safety");
    log_state_message(machine, "CRITICAL", "Relè Pico spento per sicurezza");
    
    // Ferma immediatamente tutti i motori
    send_robot_command(machine, "emergency_stop", "immediate");
    
    // Determina il motivo dell'emergenza
    if (machine->robot_data->perimeter_detected) {
        strcpy(state->emergency_reason, "Perimetro attraversato");
    } else if (has_obstacle(machine)) {
        strcpy(state->emergency_reason, "Ostacolo critico rilevato");
    } else {
        strcpy(state->emergency_reason, "Arresto di emergenza manuale");
    }
    
    char emergency_msg[512];
    snprintf(emergency_msg, sizeof(emergency_msg), 
            "EMERGENZA: %s - Tutti i sistemi fermati", state->emergency_reason);
    
    log_state_message(machine, "CRITICAL", emergency_msg);
    publish_state_change(machine, emergency_msg);
}

void emergency_stop_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    // Mantieni tutti i motori fermi
    static int safety_counter = 0;
    safety_counter++;
    
    if (safety_counter % 100 == 0) { // Ogni 100 cicli
        send_robot_command(machine, "ensure_stopped", "safety_check");
        log_state_message(machine, "DEBUG", "Controllo di sicurezza: tutti i motori fermi");
    }
    
    // Log periodico dello stato di emergenza
    if (safety_counter % 1000 == 0) { // Ogni 1000 cicli
        log_state_message(machine, "WARNING", "Robot ancora in stato di emergenza - intervento richiesto");
    }
}

void emergency_stop_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    EmergencyStopState* state = (EmergencyStopState*)machine->current_state;
    
    time_t emergency_duration = time(NULL) - state->emergency_start_time;
    
    // Buffer sufficientemente grande per contenere il messaggio completo
    char exit_msg[512];
    size_t max_len = sizeof(exit_msg) - 1;  // Riserva spazio per il terminatore nullo
    
    // Formatta il messaggio
    int written = snprintf(exit_msg, max_len, 
            "Uscita dallo stato di emergenza dopo %ld secondi. Motivo: %s",
            emergency_duration, state->emergency_reason);
    
    // Gestisci eventuale troncamento
    if (written < 0) {
        // Errore nella formattazione
        strncpy(exit_msg, "Errore nel formato del messaggio di uscita", max_len);
    } else if ((size_t)written > max_len) {
        // Messaggio troncato, sostituisci con un messaggio più corto
        strncpy(exit_msg, "Uscita emergenza: motivo troncato per lunghezza eccessiva", max_len);
    }
    
    log_state_message(machine, "INFO", exit_msg);
    publish_state_change(machine, "Stato di emergenza risolto");
}

void emergency_stop_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    switch (event) {
        case EVENT_EMERGENCY_RECOVER:
            log_state_message(machine, "INFO", "Ripresa dallo stato di emergenza autorizzata");
            state_machine_transition(machine, get_idle_state());
            break;
            
        case EVENT_MANUAL_CONTROL:
            log_state_message(machine, "INFO", "Passaggio al controllo manuale per risoluzione emergenza");
            state_machine_transition(machine, get_manual_control_state());
            break;
            
        default:
            // In stato di emergenza, ignora la maggior parte degli eventi
            log_state_message(machine, "WARNING", "Evento ignorato durante emergenza");
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_emergency_stop_state(void) {
    static bool initialized = false;
    if (!initialized) {
        emergency_stop_state_init(&emergency_stop_state_instance);
        initialized = true;
    }
    return (State*)&emergency_stop_state_instance;
}
