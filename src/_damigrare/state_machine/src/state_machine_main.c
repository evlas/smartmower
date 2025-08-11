#define _XOPEN_SOURCE 500
#include <unistd.h>
#include <getopt.h>
#include <errno.h>
#include <string.h>
#include <signal.h>
#include <mosquitto.h>

// mosquitto_socket is available in mosquitto 2.0.11
// and can be used to check connection status

#include "../include/state_machine.h"
#include "../include/state_machine_mqtt.h"
#include "../include/states/init_state.h"
#include "../include/states/undocking_state.h"
#include "../include/config.h"
#include <stdio.h>
#include <stdlib.h>
#include <json-c/json.h>
#include <pthread.h>

// Variabili globali
static StateMachine g_state_machine;
static RobotData g_robot_data;
static bool g_running = true;
static pthread_mutex_t g_data_mutex = PTHREAD_MUTEX_INITIALIZER;

// Callback MQTT
void on_connect(struct mosquitto *mosq, void *obj, int rc);
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);

// Gestione segnali
void signal_handler(int signum);

// Funzioni di inizializzazione
bool init_robot_data(RobotData* data, const AppConfig* config);
bool init_mqtt_connection(RobotData* data, const AppConfig* config);
void cleanup_resources(void);

// Include the new state headers
#include "states/undocking_state.h"
#include "states/docking_state.h"
#include "states/manual_control_state.h"
#include "states/error_state.h"
#include "states/paused_state.h"

void print_usage(const char* program_name) {
    printf("Utilizzo: %s [OPZIONI]\n", program_name);
    printf("\nOpzioni:\n");
    printf("  -c, --config FILE  Percorso al file di configurazione JSON\n");
    printf("  -h, --help         Mostra questo messaggio di aiuto\n");
}

int main(int argc, char* argv[]) {
    // Default config file is the centralized robot configuration
    const char* config_file = "/opt/smartmower/etc/config/robot_config.json";
    
    // Parsing degli argomenti della riga di comando
    static struct option long_options[] = {
        {"config", required_argument, 0, 'c'},
        {"help", no_argument, 0, 'h'},
        {0, 0, 0, 0}
    };
    
    int opt;
    while ((opt = getopt_long(argc, argv, "c:h", long_options, NULL)) != -1) {
        switch (opt) {
            case 'c':
                config_file = optarg;
                break;
            case 'h':
                print_usage(argv[0]);
                return 0;
            default:
                print_usage(argv[0]);
                return 1;
        }
    }
    
    printf("=== Smart Mower State Machine ===\n");
    printf("Caricamento configurazione da: %s\n", config_file);
    
    // Carica la configurazione
    AppConfig* config = config_load(config_file);
    if (!config) {
        fprintf(stderr, "Errore nel caricamento del file di configurazione\n");
        return 1;
    }
    
    // Stampa la configurazione per debug
    if (getenv("DEBUG_CONFIG")) {
        config_print(config);
    }
    
    // Gestione segnali
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Inizializza i dati del robot con la configurazione
    if (!init_robot_data(&g_robot_data, config)) {
        fprintf(stderr, "Errore nell'inizializzazione dei dati del robot\n");
        config_free(config);
        return 1;
    }
    
    // Inizializza la connessione MQTT con la configurazione
    if (!init_mqtt_connection(&g_robot_data, config)) {
        fprintf(stderr, "Errore nella connessione MQTT\n");
        cleanup_resources();
        config_free(config);
        return 1;
    }
    
    // Inizializza la macchina a stati
    state_machine_init(&g_state_machine, get_init_state(), &g_robot_data);
    
    printf("Sistema inizializzato. Avvio del loop principale...\n");
    
    // Loop principale
    while (g_running && g_state_machine.running) {
        // Processa i messaggi MQTT
        mosquitto_loop(g_robot_data.mqtt_client, 10, 1);
        
        // Aggiorna la macchina a stati
        state_machine_update(&g_state_machine);
        
        // Pausa per non sovraccaricare la CPU
        usleep(10000); // 10ms = 100Hz
    }
    
    printf("Spegnimento del sistema...\n");
    
    // Spegni la macchina a stati
    state_machine_shutdown(&g_state_machine);
    
    // Cleanup delle risorse
    cleanup_resources();
    
    printf("Sistema spento correttamente.\n");
    
    // Libera la configurazione
    config_free(config);
    
    return 0;
}

bool init_robot_data(RobotData* data, const AppConfig* config) {
    if (!data || !config) return false;
    
    // Initialize all fields to zero first
    memset(data, 0, sizeof(RobotData));
    
    // Save the config reference
    data->config = config;
    
    // Initialize position (dock)
    data->x = 0.0;
    data->y = 0.0;
    data->z = 0.0;
    
    // Initial orientation
    data->roll = 0.0;
    data->pitch = 0.0;
    data->yaw = 0.0;
    
    // Initial velocity
    data->vx = 0.0;
    data->vy = 0.0;
    data->vz = 0.0;
    
    // Battery state - inizializzato a valori neutri (dati reali via MQTT)
    data->battery_level = 0.0;
    data->charging = false;
    strcpy(data->battery_state, "unknown");
    strcpy(data->previous_battery_state, "unknown");
    data->is_fully_charged = false;
    data->battery_voltage = 0.0;
    data->battery_current = 0.0;
    
    // Sensori di rilevamento ostacoli
    data->sonar_front_left = 200.0;     // Inizializza a 200cm (nessun ostacolo)
    data->sonar_front_center = 200.0;
    data->sonar_front_right = 200.0;
    data->bumper_triggered = false;
    data->camera_obstacle = false;
    
    // Altri sensori
    data->perimeter_detected = false;
    data->grass_detected = true;
    
    // Work state
    data->area_covered = 0.0;
    data->total_area = 6000.0; // 6000 m²
    data->mission_complete = false;
    data->last_mowing_x = 0.0;
    data->last_mowing_y = 0.0;
    data->orientation = 0.0;
    
    // Override per destinazione undocking
    data->has_undocking_override = false;
    data->override_target_x = 0.0;
    data->override_target_y = 0.0;
    data->override_target_orientation = 0.0;
    memset(data->override_area_name, 0, sizeof(data->override_area_name));
    
    // Timestamp
    data->last_update = time(NULL);
    
    // Debug and log settings
    data->debug_enabled = true;  // Default to true for now
    snprintf(data->log_file, sizeof(data->log_file), "/var/log/smartmower/state_machine.log");
    
    printf("Dati del robot inizializzati\n");
    return true;
}

bool init_mqtt_connection(RobotData* data, const AppConfig* config) {
    if (!data) {
        fprintf(stderr, "Errore: puntatore a RobotData NULL\n");
        return false;
    }
    if (!config) {
        fprintf(stderr, "Errore: puntatore a AppConfig NULL\n");
        return false;
    }
    
    printf("Inizializzazione connessione MQTT...\n");
    
    // Verify MQTT config structure
    printf("  Verifica struttura MQTT...\n");
    if (!config->mqtt.broker) {
        fprintf(stderr, "  ERRORE: config->mqtt.broker è NULL\n");
    } else {
        printf("  Broker: %s\n", config->mqtt.broker);
    }
    
    // Use MQTT settings from loaded configuration file
    const char* mqtt_host = config->mqtt.broker ? config->mqtt.broker : STATE_MQTT_BROKER;
    int mqtt_port = config->mqtt.port > 0 ? config->mqtt.port : STATE_MQTT_PORT;
    int keepalive = config->mqtt.keepalive > 0 ? config->mqtt.keepalive : 60;
    
    printf("  Host: %s\n  Porta: %d\n  Keepalive: %d\n", mqtt_host, mqtt_port, keepalive);
    
    // Inizializza la libreria mosquitto
    printf("  Inizializzazione libreria mosquitto...\n");
    mosquitto_lib_init();
    
    // Crea il client MQTT con l'ID da state_machine_mqtt.h
    printf("  Creazione client MQTT...\n");
    char client_id[128];
    snprintf(client_id, sizeof(client_id), "%s_%d", STATE_MQTT_CLIENT_ID, getpid());
    printf("  Client ID: %s\n", client_id);
    
    printf("  Creazione istanza mosquitto...\n");
    data->mqtt_client = mosquitto_new(client_id, true, data);
    if (!data->mqtt_client) {
        fprintf(stderr, "Errore nella creazione del client MQTT: %s\n", strerror(errno));
        return false;
    }
    
    // Configura le credenziali MQTT dal file di configurazione
    const char* mqtt_username = config->mqtt.username ? config->mqtt.username : STATE_MQTT_USERNAME;
    const char* mqtt_password = config->mqtt.password ? config->mqtt.password : STATE_MQTT_PASSWORD;
    
    printf("  Configurazione credenziali MQTT: user=%s\n", mqtt_username);
    
    if (mosquitto_username_pw_set(data->mqtt_client, 
                                 mqtt_username, 
                                 mqtt_password) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Errore nell'impostazione delle credenziali MQTT\n");
        mosquitto_destroy(data->mqtt_client);
        data->mqtt_client = NULL;
        return false;
    }
#ifdef MOSQ_VER_MAJOR
#if MOSQ_VER_MAJOR >= 2
    // Imposta il timeout di connessione (solo per Mosquitto v2.x+)
    mosquitto_connect_bind_v5(data->mqtt_client, mqtt_host, mqtt_port, keepalive, NULL, NULL);
#else
    // Per versioni precedenti, usa connect_async
    mosquitto_connect_async(data->mqtt_client, mqtt_host, mqtt_port, keepalive);
#endif
#else
    // Se la macro non è definita, usa il metodo più vecchio
    mosquitto_connect_async(data->mqtt_client, mqtt_host, mqtt_port, keepalive);
#endif
    
    // Imposta i callback
    mosquitto_connect_callback_set(data->mqtt_client, on_connect);
    mosquitto_message_callback_set(data->mqtt_client, on_message);
    
    // Host and port are already set at the beginning of the function
    
    int max_retries = 3;
    int retry_delay = 2; // secondi
    int retry_count = 0;
    
    while (retry_count < max_retries) {
        printf("Tentativo di connessione MQTT %d/%d...\n", retry_count + 1, max_retries);
        
        // Usa connessione sincrona per maggiore affidabilità
        int rc = mosquitto_connect(data->mqtt_client, mqtt_host, mqtt_port, keepalive);
        
        if (rc == MOSQ_ERR_SUCCESS) {
            printf("✅ Connessione al broker MQTT stabilita con successo\n");
            
            // Avvia il loop di rete in un thread separato
            rc = mosquitto_loop_start(data->mqtt_client);
            if (rc != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "Errore nell'avvio del loop MQTT: %s\n", mosquitto_strerror(rc));
                return false;
            }
            
            printf("✅ Loop MQTT avviato con successo\n");
            return true;
        } else {
            fprintf(stderr, "❌ Errore nella connessione al broker MQTT: %s\n", 
                   mosquitto_strerror(rc));
        }
        
        retry_count++;
        if (retry_count < max_retries) {
            printf("Tentativo di riconnessione tra %d secondi...\n", retry_delay);
            sleep(retry_delay);
            // Aumenta il ritardo per il prossimo tentativo (backoff esponenziale)
            retry_delay *= 2;
        }
    }
    
    fprintf(stderr, "Errore nella connessione al broker MQTT dopo %d tentativi\n", max_retries);
    mosquitto_destroy(data->mqtt_client);
    data->mqtt_client = NULL;
    return false;
}

void on_connect(struct mosquitto *mosq, __attribute__((unused)) void *obj, int rc) {
    if (rc == 0) {
        printf("Connesso al broker MQTT\n");
            
        // Sottoscrivi ai topic usando le costanti di state_machine_mqtt.h
        int qos = 1; // Default QoS
        
        // Subscribe to all sensor and system topics
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_PICO_SENSORS, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_PICO_STATUS, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_PICO_HEARTBEAT, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_GPS_DATA, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_GPS_HEARTBEAT, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_FUSION_DATA, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_FUSION_STATUS, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_VISION_OBSTACLES, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_VISION_PERIMETER, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_VISION_GRASS, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_SLAM_POSE, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_SLAM_MAP, qos);
        mosquitto_subscribe(mosq, NULL, STATE_TOPIC_SLAM_STATUS, qos);
        
        // Subscribe to state machine commands
        char cmd_topic[256];
        snprintf(cmd_topic, sizeof(cmd_topic), "%s%s", STATE_MQTT_BASE_TOPIC, STATE_TOPIC_CMD);
        mosquitto_subscribe(mosq, NULL, cmd_topic, qos);
            
        printf("Sottoscrizione ai topic completata\n");
    } else {
        fprintf(stderr, "Connessione MQTT fallita: %s\n", mosquitto_connack_string(rc));
    }
}

void on_message(__attribute__((unused)) struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg) {
    RobotData* data = (RobotData*)obj;
    if (!data || !msg || !msg->payload || msg->payloadlen <= 0) return;
    
    // Protezione thread safety
    pthread_mutex_lock(&g_data_mutex);
    
    // Protezione contro messaggi troppo grandi
    if (msg->payloadlen > 4096) {
        fprintf(stderr, "MQTT message too large: %d bytes, skipping\n", msg->payloadlen);
        pthread_mutex_unlock(&g_data_mutex);
        return;
    }
    
    // Allocazione dinamica sicura per topic e payload
    char *topic = malloc(strlen(msg->topic) + 1);
    char *payload = malloc(msg->payloadlen + 1);
    
    if (!topic || !payload) {
        fprintf(stderr, "Memory allocation failed in MQTT callback\n");
        free(topic);
        free(payload);
        pthread_mutex_unlock(&g_data_mutex);
        return;
    }
    
    strcpy(topic, msg->topic);
    memcpy(payload, msg->payload, msg->payloadlen);
    payload[msg->payloadlen] = '\0';
    
    printf("Messaggio MQTT ricevuto: %s -> %s\n", topic, payload);
    
    // Parsing dei messaggi MQTT usando i topic corretti da state_machine_mqtt.h
    
    // Verifica connessione Pico tramite heartbeat
    if (strcmp(topic, STATE_TOPIC_PICO_HEARTBEAT) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root && !json_object_is_type(j_root, json_type_null)) {
            json_object *j_type, *j_status, *j_timestamp;
            
            if (json_object_object_get_ex(j_root, "type", &j_type) &&
                json_object_object_get_ex(j_root, "status", &j_status) &&
                json_object_object_get_ex(j_root, "timestamp", &j_timestamp)) {
                
                const char* msg_type = json_object_get_string(j_type);
                const char* status = json_object_get_string(j_status);
                
                if (strcmp(msg_type, "pico_heartbeat") == 0 && strcmp(status, "running") == 0) {
                    printf("Pico heartbeat ricevuto - sistema connesso\n");
                    // Aggiorna timestamp per il monitoring continuo
                    if (g_state_machine.current_state && g_state_machine.current_state->type == STATE_INIT) {
                        ((InitState*)g_state_machine.current_state)->last_pico_heartbeat = time(NULL);
                    }
                    state_machine_handle_event(&g_state_machine, EVENT_PICO_CONNECTED);
                }
            }
            json_object_put(j_root);
        }
    }
    // Verifica connessione GPS tramite heartbeat
    else if (strcmp(topic, STATE_TOPIC_GPS_HEARTBEAT) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_type, *j_status, *j_timestamp;
            
            if (json_object_object_get_ex(j_root, "type", &j_type) &&
                json_object_object_get_ex(j_root, "status", &j_status) &&
                json_object_object_get_ex(j_root, "timestamp", &j_timestamp)) {
                
                const char* msg_type = json_object_get_string(j_type);
                const char* status = json_object_get_string(j_status);
                
                if (strcmp(msg_type, "gps_heartbeat") == 0 && strcmp(status, "running") == 0) {
                    printf("GPS heartbeat ricevuto - sistema connesso\n");
                    // Aggiorna timestamp per il monitoring continuo
                    if (g_state_machine.current_state && g_state_machine.current_state->type == STATE_INIT) {
                        ((InitState*)g_state_machine.current_state)->last_gps_heartbeat = time(NULL);
                    }
                    state_machine_handle_event(&g_state_machine, EVENT_GPS_CONNECTED);
                }
            }
            json_object_put(j_root);
        }
    }
    // Verifica stato Fusion tramite status
    else if (strcmp(topic, STATE_TOPIC_FUSION_STATUS) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_type, *j_system;
            
            if (json_object_object_get_ex(j_root, "type", &j_type) &&
                json_object_object_get_ex(j_root, "system", &j_system)) {
                
                const char* msg_type = json_object_get_string(j_type);
                
                if (strcmp(msg_type, "fusion_status") == 0) {
                    json_object *j_running;
                    if (json_object_object_get_ex(j_system, "running", &j_running)) {
                        bool running = json_object_get_boolean(j_running);
                        if (running) {
                            printf("Fusion system pronto\n");
                            // Aggiorna timestamp per il monitoring continuo
                            if (g_state_machine.current_state && g_state_machine.current_state->type == STATE_INIT) {
                                ((InitState*)g_state_machine.current_state)->last_fusion_status = time(NULL);
                            }
                            state_machine_handle_event(&g_state_machine, EVENT_SENSOR_FUSION_READY);
                        }
                    }
                }
            }
            json_object_put(j_root);
        }
    }
    // Aggiorna i dati di posizione dal fusion sensor (nuovo formato espanso)
    else if (strcmp(topic, STATE_TOPIC_FUSION_DATA) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_type, *j_position, *j_velocity, *j_orientation;
            
            if (json_object_object_get_ex(j_root, "type", &j_type)) {
                const char* msg_type = json_object_get_string(j_type);
                
                if (strcmp(msg_type, "fusion_data") == 0) {
                    // Parse position data (new direct format)
                    if (json_object_object_get_ex(j_root, "position", &j_position)) {
                        json_object *j_x, *j_y, *j_z;
                        if (json_object_object_get_ex(j_position, "x", &j_x) &&
                            json_object_object_get_ex(j_position, "y", &j_y)) {
                            data->x = json_object_get_double(j_x);
                            data->y = json_object_get_double(j_y);
                            
                            // Optional: store z coordinate if needed
                            if (json_object_object_get_ex(j_position, "z", &j_z)) {
                                // data->z = json_object_get_double(j_z); // if z field exists
                            }
                        }
                    }
                    
                    // Parse velocity data (new format with vx, vy, vz, speed)
                    if (json_object_object_get_ex(j_root, "velocity", &j_velocity)) {
                        json_object *j_vx, *j_vy, *j_speed;
                        if (json_object_object_get_ex(j_velocity, "vx", &j_vx) &&
                            json_object_object_get_ex(j_velocity, "vy", &j_vy)) {
                            // Store velocity data if state machine needs it
                            // data->vx = json_object_get_double(j_vx);
                            // data->vy = json_object_get_double(j_vy);
                        }
                        
                        if (json_object_object_get_ex(j_velocity, "speed", &j_speed)) {
                            // Store speed magnitude if state machine needs it
                            // data->speed = json_object_get_double(j_speed);
                        }
                    }
                    
                    // Parse orientation data (new format with roll, pitch, yaw)
                    if (json_object_object_get_ex(j_root, "orientation", &j_orientation)) {
                        json_object *j_yaw;
                        if (json_object_object_get_ex(j_orientation, "yaw", &j_yaw)) {
                            // Store yaw orientation if state machine needs it
                            // data->yaw = json_object_get_double(j_yaw);
                        }
                    }
                    
                    printf("State machine updated position: x=%.3f, y=%.3f\n", data->x, data->y);
                }
            }
            json_object_put(j_root);
        }
    }
    // Aggiorna dati batteria completi dal Pico sensors (include battery_state)
    else if (strcmp(topic, STATE_TOPIC_PICO_SENSORS) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_type, *j_power;
            
            if (json_object_object_get_ex(j_root, "type", &j_type)) {
                // Get message type but don't store it since it's not used
                (void)json_object_get_string(j_type);
                
                // Handle both old and new power data formats
                if (json_object_object_get_ex(j_root, "power", &j_power)) {
                    // New format: power is an object with voltage, current, battery_percentage, etc.
                    if (json_object_is_type(j_power, json_type_object)) {
                        json_object *j_voltage, *j_current, *j_percentage, *j_state, *j_fully_charged;
                        
                        if (json_object_object_get_ex(j_power, "voltage", &j_voltage)) {
                            data->battery_voltage = json_object_get_double(j_voltage);
                        }
                        
                        if (json_object_object_get_ex(j_power, "current", &j_current)) {
                            data->battery_current = json_object_get_double(j_current);
                        }
                        
                        if (json_object_object_get_ex(j_power, "battery_percentage", &j_percentage)) {
                            double new_level = json_object_get_double(j_percentage);
                            if (data->battery_level != new_level) {
                                printf("DEBUG: Updating battery level: %.2f%% -> %.2f%%\n", 
                                       data->battery_level, new_level);
                                data->battery_level = new_level;
                            }
                        }
                        
                        if (json_object_object_get_ex(j_power, "battery_state", &j_state)) {
                            const char* new_state = json_object_get_string(j_state);
                            if (new_state) {
                                strncpy(data->battery_state, new_state, sizeof(data->battery_state) - 1);
                                data->battery_state[sizeof(data->battery_state) - 1] = '\0';
                                
                                // Aggiorna flag charging per compatibilità
                                data->charging = (strcmp(new_state, "charging") == 0);
                            }
                        }
                        
                        // Handle is_fully_charged if present
                        if (json_object_object_get_ex(j_power, "is_fully_charged", &j_fully_charged)) {
                            data->is_fully_charged = json_object_get_boolean(j_fully_charged);
                        }
                    }
                    // Old format: power is an array [voltage, current]
                    else if (json_object_is_type(j_power, json_type_array) && 
                             json_object_array_length(j_power) >= 2) {
                        // Get voltage and current from array
                        json_object *j_voltage = json_object_array_get_idx(j_power, 0);
                        json_object *j_current = json_object_array_get_idx(j_power, 1);
                        
                        if (j_voltage && j_current) {
                            data->battery_voltage = json_object_get_double(j_voltage);
                            data->battery_current = json_object_get_double(j_current);
                            
                            // Estimate battery level based on voltage (simplified)
                            double new_level = (data->battery_voltage - 20.0) * 100.0 / (25.2 - 20.0);
                            new_level = (new_level < 0) ? 0 : (new_level > 100) ? 100 : new_level;
                            
                            if (data->battery_level != new_level) {
                                printf("DEBUG: Estimated battery level: %.2f%% (from voltage: %.2fV)\n", 
                                       new_level, data->battery_voltage);
                                data->battery_level = new_level;
                            }
                            
                            // Set battery state based on current
                            if (data->battery_current < -0.1) {
                                strncpy(data->battery_state, "charging", sizeof(data->battery_state) - 1);
                                data->charging = true;
                            } else if (data->battery_current > 0.1) {
                                strncpy(data->battery_state, "discharging", sizeof(data->battery_state) - 1);
                                data->charging = false;
                            } else {
                                strncpy(data->battery_state, "idle", sizeof(data->battery_state) - 1);
                                data->charging = false;
                            }
                            data->battery_state[sizeof(data->battery_state) - 1] = '\0';
                        }
                    }
                    
                    // Update battery state transitions and handle low battery
                    if (strcmp(data->battery_state, data->previous_battery_state) != 0) {
                        printf("Transizione battery_state: %s -> %s\n", 
                               data->previous_battery_state, data->battery_state);
                        strcpy(data->previous_battery_state, data->battery_state);
                    }
                    
                    // Handle low battery condition
                    printf("DEBUG: Checking battery - Level: %.2f%%, State: %s\n", data->battery_level, data->battery_state);
                    if (data->battery_level < 20.0 && strcmp(data->battery_state, "discharging") == 0) {
                        if (g_state_machine.current_state && 
                            g_state_machine.current_state->type != STATE_DOCKING &&
                            g_state_machine.current_state->type != STATE_CHARGING) {
                            printf("Batteria scarica (%.1f%%) durante %s\n", 
                                   data->battery_level, data->battery_state);
                            state_machine_handle_event(&g_state_machine, EVENT_BATTERY_LOW);
                        }
                    }
                }
            }
            json_object_put(j_root);
        }
    }
    // Aggiorna dati batteria dal Pico status (legacy)
    else if (strcmp(topic, STATE_TOPIC_PICO_STATUS) == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_type, *j_battery;
            
            if (json_object_object_get_ex(j_root, "type", &j_type) &&
                json_object_object_get_ex(j_root, "battery", &j_battery)) {
                
                const char* msg_type = json_object_get_string(j_type);
                
                if (strcmp(msg_type, "status_report") == 0) {
                    json_object *j_level;
                    if (json_object_object_get_ex(j_battery, "level", &j_level)) {
                        data->battery_level = json_object_get_double(j_level);
                    }
                }
            }
            json_object_put(j_root);
        }
    }
    // Gestisci i comandi per la macchina a stati
    else if (strcmp(topic, STATE_TOPIC_CMD) == 0) {
        // Gestisci i comandi per la macchina a stati
        if (strcmp(payload, "start_mowing") == 0) {
            state_machine_handle_event(&g_state_machine, EVENT_START_MOWING);
        } else if (strcmp(payload, "emergency_stop") == 0) {
            state_machine_handle_event(&g_state_machine, EVENT_EMERGENCY_STOP);
        } else if (strcmp(payload, "pause") == 0) {
            state_machine_handle_event(&g_state_machine, EVENT_PAUSE);
        } else if (strcmp(payload, "resume") == 0) {
            // Use RESUME only for pause functionality
            if (g_state_machine.current_state && 
                g_state_machine.current_state->type == STATE_PAUSED) {
                state_machine_handle_event(&g_state_machine, EVENT_RESUME);
            } else if (g_state_machine.current_state && 
                      g_state_machine.current_state->type == STATE_EMERGENCY_STOP) {
                // Use EMERGENCY_RECOVER for emergency stop recovery
                state_machine_handle_event(&g_state_machine, EVENT_EMERGENCY_RECOVER);
            } else {
                // Default to RESUME if state is unknown
                state_machine_handle_event(&g_state_machine, EVENT_RESUME);
            }
        }
    }
    // Gestisci comando per impostare destinazione undocking specifica
    else if (strcmp(topic, "smartmower/commands/undocking_area") == 0) {
        json_object *j_root = json_tokener_parse(payload);
        if (j_root) {
            json_object *j_area_name, *j_x, *j_y, *j_orientation;
            
            if (json_object_object_get_ex(j_root, "area_name", &j_area_name) &&
                json_object_object_get_ex(j_root, "x", &j_x) &&
                json_object_object_get_ex(j_root, "y", &j_y) &&
                json_object_object_get_ex(j_root, "orientation", &j_orientation)) {
                
                const char* area_name = json_object_get_string(j_area_name);
                double x = json_object_get_double(j_x);
                double y = json_object_get_double(j_y);
                double orientation = json_object_get_double(j_orientation);
                
                // Imposta l'override usando la funzione helper
                set_undocking_area_override(&g_state_machine, area_name, x, y, orientation);
                
                printf("Override undocking impostato: %s -> (%.2f, %.2f, %.1f°)\n",
                       area_name, x, y, orientation * 180.0 / 3.14159265359);
            }
            json_object_put(j_root);
        }
    }
    
    // Libera la memoria allocata dinamicamente
    free(topic);
    free(payload);
    
    // Rilascia il mutex
    pthread_mutex_unlock(&g_data_mutex);
}

void signal_handler(int signum) {
    printf("\nSegnale %d ricevuto. Spegnimento in corso...\n", signum);
    g_running = false;
}

void cleanup_resources(void) {
    if (g_robot_data.mqtt_client) {
        mosquitto_disconnect(g_robot_data.mqtt_client);
        mosquitto_destroy(g_robot_data.mqtt_client);
        mosquitto_lib_cleanup();
        g_robot_data.mqtt_client = NULL;
    }
    
    mosquitto_lib_cleanup();
    printf("Risorse liberate\n");
}
