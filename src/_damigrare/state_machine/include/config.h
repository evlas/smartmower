#ifndef CONFIG_H
#define CONFIG_H

#include <stdbool.h>
#include <json-c/json.h>

// Struttura per la configurazione MQTT
typedef struct {
    char* broker;
    int port;
    char* username;
    char* password;
    char* client_id;
    int qos;
    bool retain;
    int keepalive;
    struct {
        struct {
            char* fusion_state;
            char* sensors;
            char* commands;
            char* vision_obstacle;
            char* vision_perimeter;
            char* vision_grass;
            char* battery;
            char* gps;
            char* imu;
        } subscribe;
        struct {
            char* state;
            char* commands;
            char* debug;
        } publish;
    } topics;
} MqttConfig;

// Struttura per la configurazione della batteria
typedef struct {
    double low_threshold;
    double full_threshold;
    double critical_threshold;
} BatteryConfig;

// Struttura per la configurazione dell'area
typedef struct {
    double total_area_sqm;
    double cutting_width_m;
    struct {
        double x;
        double y;
    } dock_position;
    double dock_tolerance_m;
} AreaConfig;

// Struttura per la configurazione dei timeout degli stati
typedef struct {
    int init_timeout;
    int undocking_timeout;
    int docking_timeout;
    int manual_control_timeout;
    int charging_timeout;
    int error_timeout;
    int component_heartbeat_timeout;
    int gps_timeout;
    int vision_timeout;
} StateTimeoutConfig;

// Struttura per i parametri della batteria
typedef struct {
    double discharge_current_threshold;
    double voltage_precision;
    double percentage_min;
    double percentage_max;
    int curve_points_max;
} BatteryParametersConfig;

// Struttura per i parametri della vision
typedef struct {
    double grass_coverage_threshold;
    int loop_frequency_hz;
    int loop_delay_ms;
    int detection_timeout_ms;
} VisionParametersConfig;

// Struttura per i parametri SLAM
typedef struct {
    int map_width;
    int map_height;
    double cell_size_m;
    int max_landmarks;
    double max_sonar_range_m;
    double gps_speed_threshold_ms;
} SlamParametersConfig;

// Struttura per i parametri di comunicazione
typedef struct {
    int mqtt_port;
    int mqtt_keepalive;
    int heartbeat_interval_sec;
    int max_topic_length;
    int max_payload_length;
    int buffer_size;
} CommunicationParametersConfig;

// Struttura per le frequenze dei loop
typedef struct {
    int state_machine_hz;
    int fusion_hz;
    int vision_hz;
    int path_planning_hz;
} LoopFrequenciesConfig;

// Struttura per i parametri hardware
typedef struct {
    char* default_uart_device;
    int gps_baudrate;
    int pico_baudrate;
    int uart_timeout_ms;
    int max_satellites;
} HardwareParametersConfig;

// Struttura per la configurazione della navigazione
typedef struct {
    char* pattern;
    double max_speed_mps;
    double min_speed_mps;
    double turn_radius_m;
    bool obstacle_avoidance;
    double perimeter_safety_distance_m;
} NavigationConfig;

// Struttura per la configurazione del rilevamento ostacoli
typedef struct {
    struct {
        double detection_range_m;
        double warning_distance_m;
        double deceleration_factor;
    } camera;
    struct {
        double detection_range_m;
        double warning_distance_m;
        double critical_distance_m;
        double deceleration_factor;
        double stop_distance_m;
    } sonar;
    struct {
        double contact_speed_mps;
        bool emergency_stop;
    } bumper;
    struct {
        double default_sonar_distance_m;
        double default_camera_distance_m;
        int sensor_timeout_ms;
        double safe_mode_speed_factor;
    } fallback;
} ObstacleDetectionConfig;

// Struttura principale della configurazione
typedef struct {
    MqttConfig mqtt;
    struct {
        BatteryConfig battery;
        AreaConfig area;
        NavigationConfig navigation;
        ObstacleDetectionConfig obstacle_detection;
        StateTimeoutConfig timeouts;
        BatteryParametersConfig battery_params;
        VisionParametersConfig vision_params;
        SlamParametersConfig slam_params;
        CommunicationParametersConfig comm_params;
        LoopFrequenciesConfig loop_frequencies;
        HardwareParametersConfig hardware_params;
    } robot;
} AppConfig;

/**
 * Carica la configurazione dal file specificato
 * 
 * @param config_path Percorso al file di configurazione JSON
 * @return Puntatore alla configurazione caricata, o NULL in caso di errore
 */
AppConfig* config_load(const char* config_path);

/**
 * Libera le risorse allocate per la configurazione
 * 
 * @param config Puntatore alla configurazione da liberare
 */
void config_free(AppConfig* config);

/**
 * Stampa la configurazione (per debug)
 * 
 * @param config Puntatore alla configurazione da stampare
 */
void config_print(const AppConfig* config);

#endif // CONFIG_H
