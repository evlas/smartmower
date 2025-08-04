#include "../../include/states/init_state.h"
#include "state_machine_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>

// Istanza globale dello stato di inizializzazione
static InitState init_state_instance;

void init_state_init(InitState* state) {
    if (!state) return;
    
    base_state_init((BaseState*)state, STATE_INIT, "INIT");
    
    // Sovrascrivi le funzioni specifiche
    state->base.base.on_enter = init_state_on_enter;
    state->base.base.on_exit = init_state_on_exit;
    state->base.base.on_update = init_state_on_update;
    state->base.base.on_event = init_state_on_event;
    
    // Inizializza i flag di connessione
    state->pico_connected = false;
    state->gps_connected = false;
    state->sensor_fusion_ready = false;
    
    // Inizializza i timestamp
    state->last_pico_heartbeat = 0;
    state->last_gps_heartbeat = 0;
    state->last_fusion_status = 0;
    
    // Timeout verrà impostato in on_enter dove abbiamo accesso alla configurazione
    state->base.timeout_enabled = true;
}

void init_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Inizializzazione del sistema in corso...");
    
    InitState* state = (InitState*)machine->current_state;
    
    // Imposta timeout dall'configurazione centralizzata
    state->base.timeout_duration = machine->robot_data->config->robot.timeouts.init_timeout;
    
    // PRIMA AZIONE: Spegni il relè del Pico per sicurezza iniziale
    send_robot_command(machine, "relay_off", "init_safety");
    log_state_message(machine, "INFO", "Relè Pico spento per inizializzazione sicura");
    
    // Reset dei flag di connessione
    state->pico_connected = false;
    state->gps_connected = false;
    state->sensor_fusion_ready = false;
    
    // Reset dei timestamp
    state->last_pico_heartbeat = 0;
    state->last_gps_heartbeat = 0;
    state->last_fusion_status = 0;
    
    log_state_message(machine, "INFO", "In attesa di connessione con Pico...");
    log_state_message(machine, "INFO", "In attesa di connessione con GPS...");
    log_state_message(machine, "INFO", "In attesa di connessione con Sensor Fusion...");
    
    // Pubblica lo stato di inizializzazione
    publish_state_change(machine, "Sistema in fase di inizializzazione");
}

void init_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    InitState* state = (InitState*)machine->current_state;
    time_t now = time(NULL);
    
    // Controlla il timeout globale
    base_state_on_update(machine);
    
    // Verifica timeout per i singoli componenti (10 secondi senza heartbeat = disconnesso)
    const time_t HEARTBEAT_TIMEOUT = 10;
    bool component_status_changed = false;
    
    // Verifica connessione Pico
    if (state->pico_connected && state->last_pico_heartbeat > 0) {
        if (now - state->last_pico_heartbeat > HEARTBEAT_TIMEOUT) {
            log_state_message(machine, "WARNING", "Pico heartbeat timeout - disconnessione rilevata");
            publish_state_event(machine, "PICO_DISCONNECTED", "Pico heartbeat timeout during initialization");
            state->pico_connected = false;
            component_status_changed = true;
        }
    }
    
    // Verifica connessione GPS
    if (state->gps_connected && state->last_gps_heartbeat > 0) {
        if (now - state->last_gps_heartbeat > HEARTBEAT_TIMEOUT) {
            log_state_message(machine, "WARNING", "GPS heartbeat timeout - disconnessione rilevata");
            publish_state_event(machine, "GPS_DISCONNECTED", "GPS heartbeat timeout during initialization");
            state->gps_connected = false;
            component_status_changed = true;
        }
    }
    
    // Verifica connessione Fusion
    if (state->sensor_fusion_ready && state->last_fusion_status > 0) {
        if (now - state->last_fusion_status > HEARTBEAT_TIMEOUT) {
            log_state_message(machine, "WARNING", "Fusion status timeout - disconnessione rilevata");
            publish_state_event(machine, "FUSION_DISCONNECTED", "Sensor Fusion timeout during initialization");
            state->sensor_fusion_ready = false;
            component_status_changed = true;
        }
    }
    
    // Pubblica aggiornamento dello stato se ci sono stati cambiamenti
    if (component_status_changed) {
        char status_msg[256];
        snprintf(status_msg, sizeof(status_msg), 
                "Initialization progress: Pico=%s, GPS=%s, Fusion=%s",
                state->pico_connected ? "OK" : "WAITING",
                state->gps_connected ? "OK" : "WAITING", 
                state->sensor_fusion_ready ? "OK" : "WAITING");
        publish_state_change(machine, status_msg);
    }
    
    // Verifica se tutti i componenti sono connessi
    if (!state->pico_connected) {
        // Attesa connessione Pico tramite heartbeat su STATE_TOPIC_PICO_HEARTBEAT
        return;
    }
    
    if (!state->gps_connected) {
        // Attesa connessione GPS tramite heartbeat su STATE_TOPIC_GPS_HEARTBEAT
        return;
    }
    
    if (!state->sensor_fusion_ready) {
        // Attesa connessione Fusion tramite status su STATE_TOPIC_FUSION_STATUS
        return;
    }
    
    // Se tutti i componenti sono connessi, passa allo stato IDLE
    log_state_message(machine, "INFO", "Tutti i componenti sono connessi e attivi");
    state_machine_handle_event(machine, EVENT_INIT_COMPLETE);
}

void init_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Inizializzazione completata");
    publish_state_change(machine, "Inizializzazione completata con successo");
}

void init_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    switch (event) {
        case EVENT_INIT_COMPLETE:
            log_state_message(machine, "INFO", "Tutti i componenti sono pronti");
            log_state_message(machine, "INFO", "Transizione verso stato IDLE");
            state_machine_transition(machine, get_idle_state());
            break;
            
        case EVENT_TIMEOUT:
            log_state_message(machine, "ERROR", "Timeout durante l'inizializzazione");
            // Pubblica evento di timeout con dettagli sui componenti
            {
                InitState* state = (InitState*)machine->current_state;
                char timeout_msg[512];
                char missing_components[256] = "";
                
                // Costruisci lista componenti mancanti
                if (!state->pico_connected) {
                    strcat(missing_components, "Pico ");
                    log_state_message(machine, "ERROR", "- Pico non connesso");
                }
                if (!state->gps_connected) {
                    strcat(missing_components, "GPS ");
                    log_state_message(machine, "ERROR", "- GPS non connesso");
                }
                if (!state->sensor_fusion_ready) {
                    strcat(missing_components, "Fusion ");
                    log_state_message(machine, "ERROR", "- Sensor Fusion non pronto");
                }
                
                snprintf(timeout_msg, sizeof(timeout_msg), 
                        "Initialization timeout - Missing components: %s", 
                        strlen(missing_components) > 0 ? missing_components : "None");
                        
                publish_state_event(machine, "INIT_TIMEOUT", timeout_msg);
                
                // Pubblica stato finale prima dell'errore
                snprintf(timeout_msg, sizeof(timeout_msg), 
                        "Initialization failed: Pico=%s, GPS=%s, Fusion=%s",
                        state->pico_connected ? "OK" : "FAILED",
                        state->gps_connected ? "OK" : "FAILED", 
                        state->sensor_fusion_ready ? "OK" : "FAILED");
                publish_state_change(machine, timeout_msg);
            }
            state_machine_transition(machine, get_error_state());
            break;
            
        case EVENT_PICO_CONNECTED:
            log_state_message(machine, "INFO", "Pico connesso con successo");
            publish_state_event(machine, "PICO_CONNECTED", "Pico successfully connected during initialization");
            ((InitState*)machine->current_state)->pico_connected = true;
            ((InitState*)machine->current_state)->last_pico_heartbeat = time(NULL);
            // Pubblica progresso aggiornato
            {
                InitState* state = (InitState*)machine->current_state;
                char progress_msg[256];
                snprintf(progress_msg, sizeof(progress_msg), 
                        "Initialization progress: Pico=OK, GPS=%s, Fusion=%s",
                        state->gps_connected ? "OK" : "WAITING", 
                        state->sensor_fusion_ready ? "OK" : "WAITING");
                publish_state_change(machine, progress_msg);
            }
            break;
            
        case EVENT_GPS_CONNECTED:
            log_state_message(machine, "INFO", "GPS connesso con successo");
            publish_state_event(machine, "GPS_CONNECTED", "GPS successfully connected during initialization");
            ((InitState*)machine->current_state)->gps_connected = true;
            ((InitState*)machine->current_state)->last_gps_heartbeat = time(NULL);
            // Pubblica progresso aggiornato
            {
                InitState* state = (InitState*)machine->current_state;
                char progress_msg[256];
                snprintf(progress_msg, sizeof(progress_msg), 
                        "Initialization progress: Pico=%s, GPS=OK, Fusion=%s",
                        state->pico_connected ? "OK" : "WAITING", 
                        state->sensor_fusion_ready ? "OK" : "WAITING");
                publish_state_change(machine, progress_msg);
            }
            break;
            
        case EVENT_SENSOR_FUSION_READY:
            log_state_message(machine, "INFO", "Sensor Fusion pronto");
            publish_state_event(machine, "FUSION_READY", "Sensor Fusion ready during initialization");
            ((InitState*)machine->current_state)->sensor_fusion_ready = true;
            ((InitState*)machine->current_state)->last_fusion_status = time(NULL);
            // Pubblica progresso aggiornato
            {
                InitState* state = (InitState*)machine->current_state;
                char progress_msg[256];
                snprintf(progress_msg, sizeof(progress_msg), 
                        "Initialization progress: Pico=%s, GPS=%s, Fusion=OK",
                        state->pico_connected ? "OK" : "WAITING", 
                        state->gps_connected ? "OK" : "WAITING");
                publish_state_change(machine, progress_msg);
            }
            break;
            
        default:
            // Delega agli eventi base
            base_state_on_event(machine, event);
            break;
    }
}

// Funzione per ottenere l'istanza dello stato
State* get_init_state(void) {
    static bool initialized = false;
    if (!initialized) {
        init_state_init(&init_state_instance);
        initialized = true;
    }
    return (State*)&init_state_instance;
}
