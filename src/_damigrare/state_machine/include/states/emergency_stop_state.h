#ifndef EMERGENCY_STOP_STATE_H
#define EMERGENCY_STOP_STATE_H

#include "base_state.h"

// Struttura per lo stato di emergenza
typedef struct {
    BaseState base;
    time_t emergency_start_time;
    char emergency_reason[256];
} EmergencyStopState;

// Funzioni specifiche per lo stato di emergenza
void emergency_stop_state_init(EmergencyStopState* state);
void emergency_stop_state_on_enter(StateMachine* machine);
void emergency_stop_state_on_update(StateMachine* machine);
void emergency_stop_state_on_exit(StateMachine* machine);
void emergency_stop_state_on_event(StateMachine* machine, EventType event);

#endif // EMERGENCY_STOP_STATE_H
