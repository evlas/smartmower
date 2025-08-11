#ifndef MOWING_STATE_H
#define MOWING_STATE_H

#include "base_state.h"

// Struttura per lo stato di taglio
typedef struct {
    BaseState base;
    double area_covered;
    double target_area;
    time_t mowing_start_time;
    bool pattern_initialized;
    int current_pattern_step;
    double last_position_x, last_position_y;
} MowingState;

// Funzioni specifiche per lo stato di taglio
void mowing_state_init(MowingState* state);
void mowing_state_on_enter(StateMachine* machine);
void mowing_state_on_update(StateMachine* machine);
void mowing_state_on_exit(StateMachine* machine);
void mowing_state_on_event(StateMachine* machine, EventType event);

#endif // MOWING_STATE_H
