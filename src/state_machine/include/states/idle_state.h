#ifndef IDLE_STATE_H
#define IDLE_STATE_H

#include "base_state.h"

// Struttura per lo stato di attesa
typedef struct {
    BaseState base;
    bool waiting_for_command;
    time_t last_heartbeat;
} IdleState;

// Funzioni specifiche per lo stato di attesa
void idle_state_init(IdleState* state);
void idle_state_on_enter(StateMachine* machine);
void idle_state_on_update(StateMachine* machine);
void idle_state_on_exit(StateMachine* machine);
void idle_state_on_event(StateMachine* machine, EventType event);

#endif // IDLE_STATE_H
