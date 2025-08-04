#ifndef DOCKING_STATE_H
#define DOCKING_STATE_H

#include "base_state.h"

// Struttura per lo stato di docking
typedef struct {
    BaseState base;
    time_t docking_start_time;
    double dock_x;
    double dock_y;
    bool path_calculated;
    bool approaching_dock;
    bool docking_complete;
    int retry_count;
} DockingState;

// Funzioni specifiche per lo stato di docking
void docking_state_init(DockingState* state);
void docking_state_on_enter(StateMachine* machine);
void docking_state_on_update(StateMachine* machine);
void docking_state_on_exit(StateMachine* machine);
void docking_state_on_event(StateMachine* machine, EventType event);

// Funzione per ottenere l'istanza dello stato
State* get_docking_state(void);

#endif // DOCKING_STATE_H
