#ifndef CHARGING_STATE_H
#define CHARGING_STATE_H

#include "base_state.h"

// Struttura per lo stato di ricarica
typedef struct {
    BaseState base;
    time_t charging_start_time;
    double initial_battery_level;
    bool mission_to_resume;
    double resume_position_x, resume_position_y;
} ChargingState;

// Funzioni specifiche per lo stato di ricarica
void charging_state_init(ChargingState* state);
void charging_state_on_enter(StateMachine* machine);
void charging_state_on_update(StateMachine* machine);
void charging_state_on_exit(StateMachine* machine);
void charging_state_on_event(StateMachine* machine, EventType event);

#endif // CHARGING_STATE_H
