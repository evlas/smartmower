#ifndef UNDOCKING_STATE_H
#define UNDOCKING_STATE_H

#include "base_state.h"

// Tipi di destinazione per undocking
typedef enum {
    UNDOCKING_TARGET_DEFAULT,     // Posizione sicura standard (2m dal dock)
    UNDOCKING_TARGET_RESUME,      // Riprende dalla posizione+direzione salvata
    UNDOCKING_TARGET_AREA         // Va a un'area specifica
} UndockingTargetType;

// Struttura per lo stato di undocking
typedef struct {
    BaseState base;
    time_t undocking_start_time;
    double target_x;
    double target_y;
    double target_orientation;        // Orientamento di destinazione (yaw)
    bool path_calculated;
    bool motors_started;
    UndockingTargetType target_type;  // Tipo di destinazione
    char target_description[64];      // Descrizione della destinazione
} UndockingState;

// Funzioni specifiche per lo stato di undocking
void undocking_state_init(UndockingState* state);
void undocking_state_on_enter(StateMachine* machine);
void undocking_state_on_update(StateMachine* machine);
void undocking_state_on_exit(StateMachine* machine);
void undocking_state_on_event(StateMachine* machine, EventType event);

// Funzioni helper e pubbliche
void determine_undocking_target(StateMachine* machine, UndockingState* state);
void set_undocking_area_override(StateMachine* machine, const char* area_name, 
                                double x, double y, double orientation);

#endif // UNDOCKING_STATE_H
