#ifndef INIT_STATE_H
#define INIT_STATE_H

#include "base_state.h"

// Struttura per lo stato di inizializzazione
typedef struct {
    BaseState base;
    bool pico_connected;      // Flag per connessione Pico
    bool gps_connected;       // Flag per connessione GPS
    bool sensor_fusion_ready; // Flag per connessione Sensor Fusion
    time_t last_pico_heartbeat;    // Timestamp ultimo heartbeat Pico
    time_t last_gps_heartbeat;     // Timestamp ultimo heartbeat GPS
    time_t last_fusion_status;     // Timestamp ultimo status Fusion
} InitState;

// Funzioni specifiche per lo stato di inizializzazione
void init_state_init(InitState* state);
void init_state_on_enter(StateMachine* machine);
void init_state_on_update(StateMachine* machine);
void init_state_on_exit(StateMachine* machine);
void init_state_on_event(StateMachine* machine, EventType event);

#endif // INIT_STATE_H
