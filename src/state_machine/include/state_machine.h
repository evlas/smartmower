#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H

#include <stdint.h>
#include <stdbool.h>
#include <time.h>
#include <mosquitto.h>
#include "config.h"

// Forward declarations
typedef struct State State;
typedef struct StateMachine StateMachine;

// Definizione dei tipi di stato
typedef enum {
    STATE_INIT,
    STATE_IDLE,
    STATE_UNDOCKING,
    STATE_MOWING,
    STATE_DOCKING,
    STATE_CHARGING,
    STATE_EMERGENCY_STOP,
    STATE_MANUAL_CONTROL,
    STATE_ERROR,
    STATE_PAUSED,
    STATE_COUNT
} StateType;

// Eventi che possono scatenare transizioni
typedef enum {
    EVENT_NONE = 0,
    EVENT_INIT_COMPLETE,
    EVENT_START_MOWING,
    EVENT_BATTERY_LOW,
    EVENT_BATTERY_FULL,
    EVENT_EMERGENCY_STOP,
    EVENT_MANUAL_CONTROL,
    EVENT_ERROR_OCCURRED,
    EVENT_PAUSE,
    EVENT_RESUME,
    EVENT_EMERGENCY_RECOVER,
    EVENT_DOCKING_COMPLETE,
    EVENT_UNDOCKING_COMPLETE,
    EVENT_OBSTACLE_DETECTED,
    EVENT_PERIMETER_CROSSED,
    EVENT_AREA_COMPLETE,
    EVENT_TIMEOUT,
    EVENT_PICO_CONNECTED,
    EVENT_GPS_CONNECTED,
    EVENT_SENSOR_FUSION_READY,
    EVENT_COUNT
} EventType;

// Struttura per i dati del robot
typedef struct {
    // Posizione e orientamento (dal fusion sensor)
    double x, y, z;
    double roll, pitch, yaw;
    double vx, vy, vz;
    
    // Stato della batteria
    double battery_level;
    bool charging;
    char battery_state[16];          // "charging", "discharging", "idle"
    char previous_battery_state[16]; // Stato precedente per rilevare transizioni
    bool is_fully_charged;           // Flag carica completa
    double battery_voltage;          // Tensione batteria (V)
    double battery_current;          // Corrente batteria (A)
    
    // Sensori di rilevamento ostacoli
    double sonar_front_left;     // Distanza sonar anteriore sinistro (cm)
    double sonar_front_center;   // Distanza sonar anteriore centrale (cm)
    double sonar_front_right;    // Distanza sonar anteriore destro (cm)
    bool bumper_triggered;       // Bumper meccanico attivato
    bool camera_obstacle;        // Ostacolo rilevato dalla telecamera
    
    // Altri sensori
    bool perimeter_detected;
    bool grass_detected;
    
    // Stato del lavoro
    double area_covered;
    double total_area;
    bool mission_complete;
    double last_mowing_x;
    double last_mowing_y;
    double orientation;
    
    // Override per destinazione undocking specifica
    bool has_undocking_override;
    double override_target_x;
    double override_target_y;
    double override_target_orientation;
    char override_area_name[32];
    
    // Timestamp dell'ultimo aggiornamento
    time_t last_update;
    
    // Connessione MQTT
    struct mosquitto *mqtt_client;
    
    // Configurazione
    const AppConfig* config;  // Riferimento alla configurazione caricata
    bool debug_enabled;
    
    // File di log
    char log_file[256];
} RobotData;

// Struttura base per uno stato
struct State {
    StateType type;
    const char* name;
    void (*on_enter)(StateMachine* machine);
    void (*on_exit)(StateMachine* machine);
    void (*on_update)(StateMachine* machine);
    void (*on_event)(StateMachine* machine, EventType event);
};

// Struttura del contesto della macchina a stati
struct StateMachine {
    State* current_state;
    State* previous_state;
    RobotData* robot_data;
    bool running;
    time_t state_start_time;
};

// Funzioni della macchina a stati
void state_machine_init(StateMachine* machine, State* initial_state, RobotData* robot_data);
void state_machine_transition(StateMachine* machine, State* new_state);
void state_machine_update(StateMachine* machine);
void state_machine_handle_event(StateMachine* machine, EventType event);
void state_machine_shutdown(StateMachine* machine);

// Funzioni di inizializzazione
bool init_robot_data(RobotData* data, const AppConfig* config);
bool init_mqtt_connection(RobotData* data, const AppConfig* config);
void cleanup_resources(void);

// Funzioni di utilità
const char* state_type_to_string(StateType type);
const char* event_type_to_string(EventType event);
double get_time_in_state(StateMachine* machine);

// Funzioni per ottenere le istanze degli stati
State* get_init_state(void);
State* get_idle_state(void);
State* get_undocking_state(void);
State* get_mowing_state(void);
State* get_docking_state(void);
State* get_charging_state(void);
State* get_emergency_stop_state(void);
State* get_manual_control_state(void);
State* get_error_state(void);
State* get_paused_state(void);

#endif // STATE_MACHINE_H
