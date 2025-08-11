#ifndef PAUSED_STATE_H
#define PAUSED_STATE_H

#include "base_state.h"

// Struttura per lo stato di pausa
typedef struct {
    BaseState base;
    time_t pause_start_time;
    StateType previous_state_type;
    double saved_position_x;
    double saved_position_y;
    double saved_orientation;
    bool position_saved;
    char pause_reason[128];
} PausedState;

// Funzioni specifiche per lo stato di pausa
void paused_state_init(PausedState* state);
void paused_state_on_enter(StateMachine* machine);
void paused_state_on_update(StateMachine* machine);
void paused_state_on_exit(StateMachine* machine);
void paused_state_on_event(StateMachine* machine, EventType event);

// Funzione per ottenere l'istanza dello stato
State* get_paused_state(void);

#endif // PAUSED_STATE_H
