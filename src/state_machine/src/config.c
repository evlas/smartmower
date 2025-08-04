#define _GNU_SOURCE  // For strdup and strndup
#include "../include/config.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <json-c/json.h>
#include <unistd.h>

// Funzione di utilità per duplicare le stringhe dalla configurazione JSON
static char* json_strdup(json_object* obj, bool debug_enabled) {
    if (debug_enabled) {
        printf("  json_strdup: inizio con oggetto %p\n", (void*)obj);
    }
    
    if (!obj) {
        if (debug_enabled) {
            printf("  json_strdup: ERRORE - oggetto JSON NULL\n");
        }
        return NULL;
    }
    
    if (debug_enabled) {
        printf("  json_strdup: oggetto valido, tipo: %d\n", json_object_get_type(obj));
    }
    
    if (json_object_get_type(obj) != json_type_string) {
        if (debug_enabled) {
            printf("  json_strdup: ERRORE - oggetto non di tipo stringa (tipo: %d)\n", 
                   json_object_get_type(obj));
        }
        return NULL;
    }
    
    if (debug_enabled) {
        printf("  json_strdup: ottenimento stringa...\n");
    }
    
    const char* str = json_object_get_string(obj);
    if (!str) {
        if (debug_enabled) {
            printf("  json_strdup: ERRORE - impossibile ottenere la stringa dall'oggetto JSON\n");
        }
        return NULL;
    }
    
    if (debug_enabled) {
        printf("  json_strdup: stringa ottenuta: '%s'\n", str);
        printf("  json_strdup: allocazione memoria per la stringa...\n");
    }
    
    // Allocazione manuale della memoria per maggiore sicurezza
    size_t len = strlen(str);
    if (len > 1024) {
        if (debug_enabled) {
            printf("  json_strdup: ERRORE - stringa troppo lunga (%zu caratteri)\n", len);
        }
        return NULL;
    }
    
    char* result = malloc(len + 1);
    if (!result) {
        if (debug_enabled) {
            printf("  json_strdup: ERRORE CRITICO - allocazione memoria fallita per %zu bytes\n", len + 1);
        }
        return NULL;
    }
    
    strcpy(result, str);
    if (debug_enabled) {
        printf("  json_strdup: stringa duplicata con successo a %p (lunghezza: %zu)\n", (void*)result, len);
    }
    return result;
}

// Forward declaration of strdup for portability
#ifndef _POSIX_C_SOURCE
char *strdup(const char *s);
#endif

AppConfig* config_load(const char* config_path) {
    if (!config_path) {
        fprintf(stderr, "Errore: percorso del file di configurazione non specificato\n");
        return NULL;
    }

    // Verifica se il file esiste e può essere letto
    FILE* f = fopen(config_path, "r");
    if (!f) {
        fprintf(stderr, "Errore: impossibile aprire il file di configurazione %s: %s\n", 
                config_path, strerror(errno));
        return NULL;
    }
    fclose(f);

    // Carica il file JSON
    json_object* root = json_object_from_file(config_path);
    if (!root) {
        const char* error_ptr = json_util_get_last_err();
        fprintf(stderr, "Errore nel parsing del file di configurazione %s: %s\n", 
                config_path, error_ptr ? error_ptr : "errore sconosciuto");
        return NULL;
    }

    // Alloca la struttura di configurazione
    AppConfig* config = calloc(1, sizeof(AppConfig));
    if (!config) {
        json_object_put(root);
        return NULL;
    }

    // Carica prima la configurazione debug per determinare il livello di verbosità
    bool debug_enabled = false;
    json_object* debug_obj;
    if (json_object_object_get_ex(root, "debug", &debug_obj)) {
        json_object* enabled_obj = json_object_object_get(debug_obj, "enabled");
        if (enabled_obj && json_object_get_type(enabled_obj) == json_type_boolean) {
            debug_enabled = json_object_get_boolean(enabled_obj);
        }
    }

    // Sezione MQTT da system.communication
    printf("Caricamento configurazione MQTT...\n");
    json_object* system_obj;
    json_object* mqtt_obj = NULL;
    
    // Prima cerca la sezione system
    if (json_object_object_get_ex(root, "system", &system_obj)) {
        printf("  Trovata sezione system nel file di configurazione\n");
        // Poi cerca communication dentro system
        if (json_object_object_get_ex(system_obj, "communication", &mqtt_obj)) {
            printf("  Trovata sezione system.communication nel file di configurazione\n");
        } else {
            printf("  ATTENZIONE: sezione system.communication non trovata\n");
        }
    } else {
        printf("  ATTENZIONE: sezione system non trovata, provo con sezione mqtt legacy\n");
        // Fallback: prova con la vecchia sezione mqtt
        json_object_object_get_ex(root, "mqtt", &mqtt_obj);
    }
    
    if (mqtt_obj) {
        printf("  Trovata sezione MQTT nel file di configurazione\n");
        
        // Configurazione base MQTT
        printf("  Caricamento broker...\n");
        json_object* broker_obj = json_object_object_get(mqtt_obj, "mqtt_broker_host");
        printf("  Broker object ottenuto: %p\n", (void*)broker_obj);
        
        config->mqtt.broker = json_strdup(broker_obj, debug_enabled);
        printf("  Broker assignment completato\n");
        
        // Verifica integrità della struttura config
        printf("  Verifica integrità config: %p\n", (void*)config);
        printf("  Verifica integrità config->mqtt: %p\n", (void*)&config->mqtt);
        printf("  Verifica config->mqtt.broker pointer: %p\n", (void*)config->mqtt.broker);
        
        // Tentativo di accesso sicuro al broker
        if (config && config->mqtt.broker) {
            printf("  Broker caricato: %s\n", config->mqtt.broker);
        } else {
            printf("  ERRORE: config o broker NULL dopo assignment\n");
        }
        printf("  Continuazione con il campo porta...\n");
        
        // Debug output for port
        printf("  Caricamento porta...\n");
        json_object* port_obj = json_object_object_get(mqtt_obj, "mqtt_broker_port");
        if (!port_obj) {
            printf("    ERRORE: campo 'port' non trovato, usando default 1883\n");
            config->mqtt.port = 1883;
        } else if (json_object_get_type(port_obj) != json_type_int) {
            printf("    ERRORE: campo 'port' non è un numero intero, usando default 1883\n");
            config->mqtt.port = 1883;
        } else {
            config->mqtt.port = json_object_get_int(port_obj);
        }
        printf("  Porta: %d\n", config->mqtt.port);
        
        // Debug output for username
        printf("  Caricamento username...\n");
        json_object* username_obj = json_object_object_get(mqtt_obj, "mqtt_username");
        if (!username_obj) {
            printf("    ATTENZIONE: campo 'username' non trovato, impostato a NULL\n");
            config->mqtt.username = NULL;
        } else {
            config->mqtt.username = json_strdup(username_obj, debug_enabled);
            printf("    Username caricato: %s\n", config->mqtt.username ? config->mqtt.username : "NULL");
        }
        
        // Debug output for password
        printf("  Caricamento password...\n");
        json_object* password_obj = json_object_object_get(mqtt_obj, "mqtt_password");
        if (!password_obj) {
            printf("    ATTENZIONE: campo 'password' non trovato, impostato a NULL\n");
            config->mqtt.password = NULL;
        } else {
            config->mqtt.password = json_strdup(password_obj, debug_enabled);
            printf("    Password caricata: %s\n", config->mqtt.password ? "[set]" : "NULL");
        }
        
        // Debug output for client_id
        printf("  Caricamento client_id...\n");
        json_object* client_id_obj = json_object_object_get(mqtt_obj, "client_id");
        if (!client_id_obj) {
            printf("    ERRORE: campo obbligatorio 'client_id' non trovato, usando valore di default\n");
            const char* default_client_id = "state_machine_default";
            size_t len = strlen(default_client_id);
            config->mqtt.client_id = malloc(len + 1);
            if (!config->mqtt.client_id) {
                fprintf(stderr, "    ERRORE CRITICO: impossibile allocare memoria per il client_id di default\n");
                return NULL;
            }
            strcpy(config->mqtt.client_id, default_client_id);
            printf("    Client ID di default: %s\n", config->mqtt.client_id);
        } else {
            config->mqtt.client_id = json_strdup(client_id_obj, debug_enabled);
            if (!config->mqtt.client_id) {
                fprintf(stderr, "    ERRORE: impossibile duplicare il client_id\n");
                return NULL;
            }
            printf("    Client ID caricato: %s\n", config->mqtt.client_id);
        }
        
        // Debug output for QoS
        printf("  Caricamento QoS...\n");
        json_object* qos_obj = json_object_object_get(mqtt_obj, "qos");
        if (!qos_obj) {
            printf("    ATTENZIONE: campo 'qos' non trovato, usando default 1\n");
            config->mqtt.qos = 1;
        } else if (json_object_get_type(qos_obj) != json_type_int) {
            printf("    ATTENZIONE: campo 'qos' non è un numero intero, usando default 1\n");
            config->mqtt.qos = 1;
        } else {
            config->mqtt.qos = json_object_get_int(qos_obj);
            printf("    QoS impostato a: %d\n", config->mqtt.qos);
        }
        
        // Debug output for retain
        printf("  Caricamento flag retain...\n");
        json_object* retain_obj = json_object_object_get(mqtt_obj, "retain");
        if (!retain_obj) {
            printf("    ATTENZIONE: campo 'retain' non trovato, usando default false\n");
            config->mqtt.retain = false;
        } else if (json_object_get_type(retain_obj) != json_type_boolean) {
            printf("    ATTENZIONE: campo 'retain' non è un booleano, usando default false\n");
            config->mqtt.retain = false;
        } else {
            config->mqtt.retain = json_object_get_boolean(retain_obj);
            printf("    Retain impostato a: %s\n", config->mqtt.retain ? "true" : "false");
        }
        
        // Debug output for keepalive
        printf("  Caricamento keepalive...\n");
        json_object* keepalive_obj = json_object_object_get(mqtt_obj, "keepalive");
        if (!keepalive_obj) {
            printf("    ATTENZIONE: campo 'keepalive' non trovato, usando default 60 secondi\n");
            config->mqtt.keepalive = 60;
        } else if (json_object_get_type(keepalive_obj) != json_type_int) {
            printf("    ATTENZIONE: campo 'keepalive' non è un numero intero, usando default 60 secondi\n");
            config->mqtt.keepalive = 60;
        } else {
            config->mqtt.keepalive = json_object_get_int(keepalive_obj);
            printf("    Keepalive impostato a: %d secondi\n", config->mqtt.keepalive);
        }
        
        // Topic di sottoscrizione
        printf("  Caricamento topic MQTT...\n");
        json_object* topics_obj;
        if (json_object_object_get_ex(mqtt_obj, "topics", &topics_obj)) {
            printf("    Trovata sezione topics\n");
            
            // Subscribe topics
            json_object* subscribe_obj;
            if (json_object_object_get_ex(topics_obj, "subscribe", &subscribe_obj)) {
                printf("    Trovata sezione subscribe\n");
                
                json_object* topic_obj;
                
                topic_obj = json_object_object_get(subscribe_obj, "fusion_state");
                config->mqtt.topics.subscribe.fusion_state = json_strdup(topic_obj, debug_enabled);
                printf("      fusion_state: %s\n", config->mqtt.topics.subscribe.fusion_state ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "sensors");
                config->mqtt.topics.subscribe.sensors = json_strdup(topic_obj, debug_enabled);
                printf("      sensors: %s\n", config->mqtt.topics.subscribe.sensors ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "commands");
                config->mqtt.topics.subscribe.commands = json_strdup(topic_obj, debug_enabled);
                printf("      commands: %s\n", config->mqtt.topics.subscribe.commands ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "vision_obstacle");
                config->mqtt.topics.subscribe.vision_obstacle = json_strdup(topic_obj, debug_enabled);
                printf("      vision_obstacle: %s\n", config->mqtt.topics.subscribe.vision_obstacle ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "vision_perimeter");
                config->mqtt.topics.subscribe.vision_perimeter = json_strdup(topic_obj, debug_enabled);
                printf("      vision_perimeter: %s\n", config->mqtt.topics.subscribe.vision_perimeter ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "vision_grass");
                config->mqtt.topics.subscribe.vision_grass = json_strdup(topic_obj, debug_enabled);
                printf("      vision_grass: %s\n", config->mqtt.topics.subscribe.vision_grass ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "battery");
                config->mqtt.topics.subscribe.battery = json_strdup(topic_obj, debug_enabled);
                printf("      battery: %s\n", config->mqtt.topics.subscribe.battery ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "gps");
                config->mqtt.topics.subscribe.gps = json_strdup(topic_obj, debug_enabled);
                printf("      gps: %s\n", config->mqtt.topics.subscribe.gps ?: "NULL");
                
                topic_obj = json_object_object_get(subscribe_obj, "imu");
                config->mqtt.topics.subscribe.imu = json_strdup(topic_obj, debug_enabled);
                printf("      imu: %s\n", config->mqtt.topics.subscribe.imu ?: "NULL");
            }
            
            // Publish topics
            json_object* publish_obj;
            if (json_object_object_get_ex(topics_obj, "publish", &publish_obj)) {
                printf("    Trovata sezione publish\n");
                
                json_object* topic_obj;
                
                topic_obj = json_object_object_get(publish_obj, "state");
                config->mqtt.topics.publish.state = json_strdup(topic_obj, debug_enabled);
                printf("      state: %s\n", config->mqtt.topics.publish.state ?: "NULL");
                
                topic_obj = json_object_object_get(publish_obj, "commands");
                config->mqtt.topics.publish.commands = json_strdup(topic_obj, debug_enabled);
                printf("      commands: %s\n", config->mqtt.topics.publish.commands ?: "NULL");
                
                topic_obj = json_object_object_get(publish_obj, "debug");
                config->mqtt.topics.publish.debug = json_strdup(topic_obj, debug_enabled);
                printf("      debug: %s\n", config->mqtt.topics.publish.debug ?: "NULL");
            } else {
                printf("    ERRORE: Sezione publish non trovata\n");
            }
        }
    }
    
    // Sezione Robot
    json_object* robot_obj;
    if (json_object_object_get_ex(root, "robot", &robot_obj)) {
        // Configurazione batteria
        json_object* battery_obj;
        if (json_object_object_get_ex(robot_obj, "battery", &battery_obj)) {
            config->robot.battery.low_threshold = json_object_get_double(json_object_object_get(battery_obj, "low_threshold"));
            config->robot.battery.full_threshold = json_object_get_double(json_object_object_get(battery_obj, "full_threshold"));
            config->robot.battery.critical_threshold = json_object_get_double(json_object_object_get(battery_obj, "critical_threshold"));
        }
        
        // Configurazione area
        json_object* area_obj;
        if (json_object_object_get_ex(robot_obj, "area", &area_obj)) {
            config->robot.area.total_area_sqm = json_object_get_double(json_object_object_get(area_obj, "total_area_sqm"));
            config->robot.area.cutting_width_m = json_object_get_double(json_object_object_get(area_obj, "cutting_width_m"));
            
            json_object* dock_pos_obj;
            if (json_object_object_get_ex(area_obj, "dock_position", &dock_pos_obj)) {
                config->robot.area.dock_position.x = json_object_get_double(json_object_object_get(dock_pos_obj, "x"));
                config->robot.area.dock_position.y = json_object_get_double(json_object_object_get(dock_pos_obj, "y"));
            }
            
            config->robot.area.dock_tolerance_m = json_object_get_double(json_object_object_get(area_obj, "dock_tolerance_m"));
        }
        
        // Configurazione navigazione
        json_object* nav_obj;
        if (json_object_object_get_ex(robot_obj, "navigation", &nav_obj)) {
            config->robot.navigation.pattern = json_strdup(json_object_object_get(nav_obj, "pattern"), debug_enabled);
            config->robot.navigation.max_speed_mps = json_object_get_double(json_object_object_get(nav_obj, "max_speed_mps"));
            config->robot.navigation.turn_radius_m = json_object_get_double(json_object_object_get(nav_obj, "turn_radius_m"));
            config->robot.navigation.obstacle_avoidance = json_object_get_boolean(json_object_object_get(nav_obj, "obstacle_avoidance"));
            config->robot.navigation.perimeter_safety_distance_m = json_object_get_double(
                json_object_object_get(nav_obj, "perimeter_safety_distance_m"));
        }
    }
    
    // Sezione Tuning per i timeout degli stati
    json_object* tuning_obj;
    if (json_object_object_get_ex(root, "tuning", &tuning_obj)) {
        json_object* timeouts_obj;
        if (json_object_object_get_ex(tuning_obj, "state_timeouts", &timeouts_obj)) {
            if (debug_enabled) printf("Caricamento timeout degli stati...\n");
            
            config->robot.timeouts.init_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "init_timeout"));
            config->robot.timeouts.undocking_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "undocking_timeout"));
            config->robot.timeouts.docking_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "docking_timeout"));
            config->robot.timeouts.manual_control_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "manual_control_timeout"));
            config->robot.timeouts.charging_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "charging_timeout"));
            config->robot.timeouts.error_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "error_timeout"));
            config->robot.timeouts.component_heartbeat_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "component_heartbeat_timeout"));
            config->robot.timeouts.gps_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "gps_timeout"));
            config->robot.timeouts.vision_timeout = json_object_get_int(json_object_object_get(timeouts_obj, "vision_timeout"));
            
            if (debug_enabled) {
                printf("  init_timeout: %d\n", config->robot.timeouts.init_timeout);
                printf("  undocking_timeout: %d\n", config->robot.timeouts.undocking_timeout);
                printf("  docking_timeout: %d\n", config->robot.timeouts.docking_timeout);
                printf("  manual_control_timeout: %d\n", config->robot.timeouts.manual_control_timeout);
                printf("  charging_timeout: %d\n", config->robot.timeouts.charging_timeout);
                printf("  error_timeout: %d\n", config->robot.timeouts.error_timeout);
            }
        } else {
            // Valori di default se non specificati
            if (debug_enabled) printf("Timeout non trovati, usando valori di default\n");
            config->robot.timeouts.init_timeout = 30;
            config->robot.timeouts.undocking_timeout = 120;
            config->robot.timeouts.docking_timeout = 300;
            config->robot.timeouts.manual_control_timeout = 600;
            config->robot.timeouts.charging_timeout = 14400;
            config->robot.timeouts.error_timeout = 300;
            config->robot.timeouts.component_heartbeat_timeout = 10;
            config->robot.timeouts.gps_timeout = 5000;
            config->robot.timeouts.vision_timeout = 100;
        }
        
        // Caricamento parametri batteria
        json_object* battery_params_obj;
        if (json_object_object_get_ex(tuning_obj, "battery_parameters", &battery_params_obj)) {
            if (debug_enabled) printf("Caricamento parametri batteria...\n");
            config->robot.battery_params.discharge_current_threshold = json_object_get_double(json_object_object_get(battery_params_obj, "discharge_current_threshold"));
            config->robot.battery_params.voltage_precision = json_object_get_double(json_object_object_get(battery_params_obj, "voltage_precision"));
            config->robot.battery_params.percentage_min = json_object_get_double(json_object_object_get(battery_params_obj, "percentage_min"));
            config->robot.battery_params.percentage_max = json_object_get_double(json_object_object_get(battery_params_obj, "percentage_max"));
            config->robot.battery_params.curve_points_max = json_object_get_int(json_object_object_get(battery_params_obj, "curve_points_max"));
        } else {
            // Valori di default
            config->robot.battery_params.discharge_current_threshold = 0.1;
            config->robot.battery_params.voltage_precision = 0.001;
            config->robot.battery_params.percentage_min = 0.0;
            config->robot.battery_params.percentage_max = 100.0;
            config->robot.battery_params.curve_points_max = 12;
        }
        
        // Caricamento parametri vision
        json_object* vision_params_obj;
        if (json_object_object_get_ex(tuning_obj, "vision_parameters", &vision_params_obj)) {
            if (debug_enabled) printf("Caricamento parametri vision...\n");
            config->robot.vision_params.grass_coverage_threshold = json_object_get_double(json_object_object_get(vision_params_obj, "grass_coverage_threshold"));
            config->robot.vision_params.loop_frequency_hz = json_object_get_int(json_object_object_get(vision_params_obj, "loop_frequency_hz"));
            config->robot.vision_params.loop_delay_ms = json_object_get_int(json_object_object_get(vision_params_obj, "loop_delay_ms"));
            config->robot.vision_params.detection_timeout_ms = json_object_get_int(json_object_object_get(vision_params_obj, "detection_timeout_ms"));
        } else {
            // Valori di default
            config->robot.vision_params.grass_coverage_threshold = 10.0;
            config->robot.vision_params.loop_frequency_hz = 100;
            config->robot.vision_params.loop_delay_ms = 10;
            config->robot.vision_params.detection_timeout_ms = 100;
        }
        
        // Caricamento parametri SLAM
        json_object* slam_params_obj;
        if (json_object_object_get_ex(tuning_obj, "slam_parameters", &slam_params_obj)) {
            if (debug_enabled) printf("Caricamento parametri SLAM...\n");
            config->robot.slam_params.map_width = json_object_get_int(json_object_object_get(slam_params_obj, "map_width"));
            config->robot.slam_params.map_height = json_object_get_int(json_object_object_get(slam_params_obj, "map_height"));
            config->robot.slam_params.cell_size_m = json_object_get_double(json_object_object_get(slam_params_obj, "cell_size_m"));
            config->robot.slam_params.max_landmarks = json_object_get_int(json_object_object_get(slam_params_obj, "max_landmarks"));
            config->robot.slam_params.max_sonar_range_m = json_object_get_double(json_object_object_get(slam_params_obj, "max_sonar_range_m"));
            config->robot.slam_params.gps_speed_threshold_ms = json_object_get_double(json_object_object_get(slam_params_obj, "gps_speed_threshold_ms"));
        } else {
            // Valori di default
            config->robot.slam_params.map_width = 200;
            config->robot.slam_params.map_height = 200;
            config->robot.slam_params.cell_size_m = 0.1;
            config->robot.slam_params.max_landmarks = 100;
            config->robot.slam_params.max_sonar_range_m = 4.0;
            config->robot.slam_params.gps_speed_threshold_ms = 0.1;
        }
        
        // Caricamento parametri comunicazione
        json_object* comm_params_obj;
        if (json_object_object_get_ex(tuning_obj, "communication_parameters", &comm_params_obj)) {
            if (debug_enabled) printf("Caricamento parametri comunicazione...\n");
            config->robot.comm_params.mqtt_port = json_object_get_int(json_object_object_get(comm_params_obj, "mqtt_port"));
            config->robot.comm_params.mqtt_keepalive = json_object_get_int(json_object_object_get(comm_params_obj, "mqtt_keepalive"));
            config->robot.comm_params.heartbeat_interval_sec = json_object_get_int(json_object_object_get(comm_params_obj, "heartbeat_interval_sec"));
            config->robot.comm_params.max_topic_length = json_object_get_int(json_object_object_get(comm_params_obj, "max_topic_length"));
            config->robot.comm_params.max_payload_length = json_object_get_int(json_object_object_get(comm_params_obj, "max_payload_length"));
            config->robot.comm_params.buffer_size = json_object_get_int(json_object_object_get(comm_params_obj, "buffer_size"));
        } else {
            // Valori di default
            config->robot.comm_params.mqtt_port = 1883;
            config->robot.comm_params.mqtt_keepalive = 60;
            config->robot.comm_params.heartbeat_interval_sec = 5;
            config->robot.comm_params.max_topic_length = 512;
            config->robot.comm_params.max_payload_length = 1024;
            config->robot.comm_params.buffer_size = 2048;
        }
        
        // Caricamento frequenze loop
        json_object* loop_freq_obj;
        if (json_object_object_get_ex(tuning_obj, "loop_frequencies", &loop_freq_obj)) {
            if (debug_enabled) printf("Caricamento frequenze loop...\n");
            config->robot.loop_frequencies.state_machine_hz = json_object_get_int(json_object_object_get(loop_freq_obj, "state_machine_hz"));
            config->robot.loop_frequencies.fusion_hz = json_object_get_int(json_object_object_get(loop_freq_obj, "fusion_hz"));
            config->robot.loop_frequencies.vision_hz = json_object_get_int(json_object_object_get(loop_freq_obj, "vision_hz"));
            config->robot.loop_frequencies.path_planning_hz = json_object_get_int(json_object_object_get(loop_freq_obj, "path_planning_hz"));
        } else {
            // Valori di default
            config->robot.loop_frequencies.state_machine_hz = 100;
            config->robot.loop_frequencies.fusion_hz = 100;
            config->robot.loop_frequencies.vision_hz = 100;
            config->robot.loop_frequencies.path_planning_hz = 10;
        }
        
        // Caricamento parametri hardware
        json_object* hw_params_obj;
        if (json_object_object_get_ex(tuning_obj, "hardware_parameters", &hw_params_obj)) {
            if (debug_enabled) printf("Caricamento parametri hardware...\n");
            config->robot.hardware_params.default_uart_device = json_strdup(json_object_object_get(hw_params_obj, "default_uart_device"), debug_enabled);
            config->robot.hardware_params.gps_baudrate = json_object_get_int(json_object_object_get(hw_params_obj, "gps_baudrate"));
            config->robot.hardware_params.pico_baudrate = json_object_get_int(json_object_object_get(hw_params_obj, "pico_baudrate"));
            config->robot.hardware_params.uart_timeout_ms = json_object_get_int(json_object_object_get(hw_params_obj, "uart_timeout_ms"));
            config->robot.hardware_params.max_satellites = json_object_get_int(json_object_object_get(hw_params_obj, "max_satellites"));
        } else {
            // Valori di default
            config->robot.hardware_params.default_uart_device = strdup("/dev/ttyUSB0");
            config->robot.hardware_params.gps_baudrate = 9600;
            config->robot.hardware_params.pico_baudrate = 921600;
            config->robot.hardware_params.uart_timeout_ms = 1000;
            config->robot.hardware_params.max_satellites = 24;
        }
    }
    
    // Libera la memoria del JSON
    json_object_put(root);
    
    return config;
}

void config_free(AppConfig* config) {
    if (!config) return;
    
    // Libera la memoria della sezione MQTT
    free(config->mqtt.broker);
    free(config->mqtt.username);
    free(config->mqtt.password);
    free(config->mqtt.client_id);
    
    // Libera i topic di sottoscrizione
    free(config->mqtt.topics.subscribe.fusion_state);
    free(config->mqtt.topics.subscribe.sensors);
    free(config->mqtt.topics.subscribe.commands);
    free(config->mqtt.topics.subscribe.vision_obstacle);
    free(config->mqtt.topics.subscribe.vision_perimeter);
    free(config->mqtt.topics.subscribe.vision_grass);
    free(config->mqtt.topics.subscribe.battery);
    free(config->mqtt.topics.subscribe.gps);
    free(config->mqtt.topics.subscribe.imu);
    
    // Libera i topic di pubblicazione
    free(config->mqtt.topics.publish.state);
    free(config->mqtt.topics.publish.commands);
    free(config->mqtt.topics.publish.debug);
    
    // Libera la configurazione della navigazione
    free(config->robot.navigation.pattern);
    
    // Libera i parametri hardware
    free(config->robot.hardware_params.default_uart_device);
    
    // Libera la struttura principale
    free(config);
}

void config_print(const AppConfig* config) {
    if (!config) {
        printf("Configurazione non valida\n");
        return;
    }
    
    printf("=== Configurazione MQTT ===\n");
    printf("Broker: %s:%d\n", config->mqtt.broker, config->mqtt.port);
    printf("Client ID: %s\n", config->mqtt.client_id);
    printf("Keepalive: %d\n", config->mqtt.keepalive);
    printf("QoS: %d, Retain: %s\n", config->mqtt.qos, config->mqtt.retain ? "true" : "false");
    
    printf("\n=== Topic di Sottoscrizione ===\n");
    printf("Stato fusione: %s\n", config->mqtt.topics.subscribe.fusion_state);
    printf("Sensori: %s\n", config->mqtt.topics.subscribe.sensors);
    printf("Comandi: %s\n", config->mqtt.topics.subscribe.commands);
    
    printf("\n=== Configurazione Batteria ===\n");
    printf("Soglia bassa: %.1f%%\n", config->robot.battery.low_threshold);
    printf("Soglia piena: %.1f%%\n", config->robot.battery.full_threshold);
    printf("Soglia critica: %.1f%%\n", config->robot.battery.critical_threshold);
    
    printf("\n=== Configurazione Area ===\n");
    printf("Area totale: %.1f m²\n", config->robot.area.total_area_sqm);
    printf("Larghezza di taglio: %.2f m\n", config->robot.area.cutting_width_m);
    printf("Posizione base: (%.2f, %.2f)\n", 
           config->robot.area.dock_position.x, 
           config->robot.area.dock_position.y);
    printf("Tolleranza base: %.2f m\n", config->robot.area.dock_tolerance_m);
    
    printf("\n=== Configurazione Navigazione ===\n");
    printf("Pattern: %s\n", config->robot.navigation.pattern);
    printf("Velocità: %.2f m/s\n", config->robot.navigation.max_speed_mps);
    printf("Raggio di sterzata: %.2f m\n", config->robot.navigation.turn_radius_m);
    printf("Evitamento ostacoli: %s\n", config->robot.navigation.obstacle_avoidance ? "attivo" : "disattivo");
    printf("Distanza sicurezza perimetro: %.2f m\n", config->robot.navigation.perimeter_safety_distance_m);
    
    printf("\n=== Timeout degli Stati ===\n");
    printf("Init: %d s\n", config->robot.timeouts.init_timeout);
    printf("Undocking: %d s\n", config->robot.timeouts.undocking_timeout);
    printf("Docking: %d s\n", config->robot.timeouts.docking_timeout);
    printf("Controllo manuale: %d s\n", config->robot.timeouts.manual_control_timeout);
    printf("Ricarica: %d s\n", config->robot.timeouts.charging_timeout);
    printf("Errore: %d s\n", config->robot.timeouts.error_timeout);
    printf("Heartbeat componenti: %d s\n", config->robot.timeouts.component_heartbeat_timeout);
    printf("GPS: %d ms\n", config->robot.timeouts.gps_timeout);
    printf("Vision: %d ms\n", config->robot.timeouts.vision_timeout);
    
    printf("\n=== Parametri Batteria ===\n");
    printf("Soglia corrente scarica: %.3f A\n", config->robot.battery_params.discharge_current_threshold);
    printf("Precisione tensione: %.6f V\n", config->robot.battery_params.voltage_precision);
    printf("Percentuale min: %.1f%%\n", config->robot.battery_params.percentage_min);
    printf("Percentuale max: %.1f%%\n", config->robot.battery_params.percentage_max);
    printf("Punti curva max: %d\n", config->robot.battery_params.curve_points_max);
    
    printf("\n=== Parametri Vision ===\n");
    printf("Soglia copertura erba: %.1f%%\n", config->robot.vision_params.grass_coverage_threshold);
    printf("Frequenza loop: %d Hz\n", config->robot.vision_params.loop_frequency_hz);
    printf("Delay loop: %d ms\n", config->robot.vision_params.loop_delay_ms);
    printf("Timeout detection: %d ms\n", config->robot.vision_params.detection_timeout_ms);
    
    printf("\n=== Parametri SLAM ===\n");
    printf("Dimensioni mappa: %dx%d celle\n", config->robot.slam_params.map_width, config->robot.slam_params.map_height);
    printf("Dimensione cella: %.2f m\n", config->robot.slam_params.cell_size_m);
    printf("Landmarks max: %d\n", config->robot.slam_params.max_landmarks);
    printf("Range sonar max: %.1f m\n", config->robot.slam_params.max_sonar_range_m);
    printf("Soglia velocità GPS: %.2f m/s\n", config->robot.slam_params.gps_speed_threshold_ms);
    
    printf("\n=== Parametri Comunicazione ===\n");
    printf("Porta MQTT: %d\n", config->robot.comm_params.mqtt_port);
    printf("Keepalive MQTT: %d s\n", config->robot.comm_params.mqtt_keepalive);
    printf("Intervallo heartbeat: %d s\n", config->robot.comm_params.heartbeat_interval_sec);
    printf("Lunghezza topic max: %d\n", config->robot.comm_params.max_topic_length);
    printf("Lunghezza payload max: %d\n", config->robot.comm_params.max_payload_length);
    printf("Dimensione buffer: %d\n", config->robot.comm_params.buffer_size);
    
    printf("\n=== Frequenze Loop ===\n");
    printf("State machine: %d Hz\n", config->robot.loop_frequencies.state_machine_hz);
    printf("Fusion: %d Hz\n", config->robot.loop_frequencies.fusion_hz);
    printf("Vision: %d Hz\n", config->robot.loop_frequencies.vision_hz);
    printf("Path planning: %d Hz\n", config->robot.loop_frequencies.path_planning_hz);
    
    printf("\n=== Parametri Hardware ===\n");
    printf("Dispositivo UART: %s\n", config->robot.hardware_params.default_uart_device);
    printf("Baudrate GPS: %d\n", config->robot.hardware_params.gps_baudrate);
    printf("Baudrate Pico: %d\n", config->robot.hardware_params.pico_baudrate);
    printf("Timeout UART: %d ms\n", config->robot.hardware_params.uart_timeout_ms);
    printf("Satelliti max: %d\n", config->robot.hardware_params.max_satellites);
}
