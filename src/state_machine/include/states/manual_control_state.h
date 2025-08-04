#ifndef MANUAL_CONTROL_STATE_H
#define MANUAL_CONTROL_STATE_H

#include "base_state.h"

// Struttura per lo stato di controllo manuale
typedef struct {
    BaseState base;
    time_t manual_start_time;
    bool manual_active;
    char last_command[64];
    time_t last_command_time;
    StateType previous_state_type;
} ManualControlState;

// Funzioni specifiche per lo stato di controllo manuale
void manual_control_state_init(ManualControlState* state);
void manual_control_state_on_enter(StateMachine* machine);
void manual_control_state_on_update(StateMachine* machine);
void manual_control_state_on_exit(StateMachine* machine);
void manual_control_state_on_event(StateMachine* machine, EventType event);

// Funzione per ottenere l'istanza dello stato
State* get_manual_control_state(void);

#endif // MANUAL_CONTROL_STATE_H
