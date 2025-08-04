#ifndef ERROR_STATE_H
#define ERROR_STATE_H

#include "base_state.h"

// Struttura per lo stato di errore
typedef struct {
    BaseState base;
    time_t error_start_time;
    char error_message[256];
    int error_code;
    bool recovery_attempted;
    int recovery_attempts;
    StateType previous_state_type;
} ErrorState;

// Funzioni specifiche per lo stato di errore
void error_state_init(ErrorState* state);
void error_state_on_enter(StateMachine* machine);
void error_state_on_update(StateMachine* machine);
void error_state_on_exit(StateMachine* machine);
void error_state_on_event(StateMachine* machine, EventType event);

// Funzione per ottenere l'istanza dello stato
State* get_error_state(void);

#endif // ERROR_STATE_H
