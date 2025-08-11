#ifndef BASE_STATE_H
#define BASE_STATE_H

#include "../state_machine.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <json-c/json.h>

// Struttura base per tutti gli stati
typedef struct {
    State base;
    time_t timeout_duration;  // Timeout per lo stato (0 = nessun timeout)
    bool timeout_enabled;
} BaseState;

// Funzioni di base comuni a tutti gli stati
void base_state_init(BaseState* state, StateType type, const char* name);
void base_state_on_enter(StateMachine* machine);
void base_state_on_exit(StateMachine* machine);
void base_state_on_update(StateMachine* machine);
void base_state_on_event(StateMachine* machine, EventType event);

// Funzioni di utilità per gli stati
void publish_state_change(StateMachine* machine, const char* details);
void publish_state_transition(StateMachine* machine, const char* from_state, const char* to_state, const char* event, const char* reason);
void publish_state_event(StateMachine* machine, const char* event_type, const char* event_data);
void log_state_message(StateMachine* machine, const char* level, const char* message);
bool check_timeout(StateMachine* machine, time_t timeout_seconds);
void send_robot_command(StateMachine* machine, const char* command, const char* params);

// Struttura per il risultato del rilevamento ostacoli stratificato
typedef struct {
    bool obstacle_detected;
    double recommended_speed_factor;  // 0.0 = stop, 1.0 = velocità massima
    char detection_source[32];        // Quale sensore ha rilevato l'ostacolo
    double closest_distance_m;        // Distanza ostacolo più vicino
} ObstacleDetectionResult;

// Funzioni per leggere i dati dei sensori
void update_robot_data_from_mqtt(StateMachine* machine);
bool is_battery_low(StateMachine* machine);
bool is_battery_full(StateMachine* machine);
bool has_obstacle(StateMachine* machine);
ObstacleDetectionResult detect_obstacles_stratified(StateMachine* machine);
double calculate_safe_speed(StateMachine* machine);
bool is_at_dock(StateMachine* machine);
bool is_perimeter_detected(StateMachine* machine);

#endif // BASE_STATE_H
