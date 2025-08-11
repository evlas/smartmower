#include "../../include/states/base_state.h"
#include "state_machine_mqtt.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include <json-c/json.h>

// Implementazione delle funzioni base comuni a tutti gli stati

void base_state_init(BaseState* state, StateType type, const char* name) {
    if (!state || !name) {
        return;
    }
    
    state->base.type = type;
    state->base.name = name;
    state->base.on_enter = base_state_on_enter;
    state->base.on_exit = base_state_on_exit;
    state->base.on_update = base_state_on_update;
    state->base.on_event = base_state_on_event;
    
    state->timeout_duration = 0;
    state->timeout_enabled = false;
}

void base_state_on_enter(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Entrato nello stato base");
}

void base_state_on_exit(StateMachine* machine) {
    if (!machine) return;
    
    log_state_message(machine, "INFO", "Uscito dallo stato base");
}

void base_state_on_update(StateMachine* machine) {
    if (!machine) return;
    
    // Controllo timeout generico
    BaseState* base_state = (BaseState*)machine->current_state;
    if (base_state && base_state->timeout_enabled) {
        if (check_timeout(machine, base_state->timeout_duration)) {
            state_machine_handle_event(machine, EVENT_TIMEOUT);
        }
    }
}

void base_state_on_event(StateMachine* machine, EventType event) {
    if (!machine) return;
    
    // Gestione eventi base (può essere sovrascritta dagli stati specifici)
    switch (event) {
        case EVENT_TIMEOUT:
            log_state_message(machine, "WARNING", "Timeout nello stato");
            break;
        default:
            break;
    }
}

// Funzioni di utilità

void publish_state_change(StateMachine* machine, const char* details) {
    if (!machine || !machine->robot_data || !machine->robot_data->mqtt_client) {
        return;
    }
    
    json_object *j_state = json_object_new_object();
    json_object_object_add(j_state, "type", json_object_new_string(STATE_JSON_CURRENT));
    json_object_object_add(j_state, "state", json_object_new_string(machine->current_state->name));
    json_object_object_add(j_state, "timestamp", json_object_new_int64(time(NULL)));
    json_object_object_add(j_state, "details", json_object_new_string(details ? details : ""));
    json_object_object_add(j_state, "uptime", json_object_new_double(get_time_in_state(machine)));
    
    const char *state_json = json_object_to_json_string(j_state);
    
    char topic[256];
    snprintf(topic, sizeof(topic), "%s%s", STATE_MQTT_BASE_TOPIC, STATE_TOPIC_CURRENT);
    
    mosquitto_publish(machine->robot_data->mqtt_client, NULL, 
                     topic, 
                     strlen(state_json), state_json, 1, true);
    
    printf("Stato pubblicato su %s: %s\n", topic, state_json);
    json_object_put(j_state);
}

void publish_state_transition(StateMachine* machine, const char* from_state, const char* to_state, const char* event, const char* reason) {
    if (!machine || !machine->robot_data || !machine->robot_data->mqtt_client) {
        return;
    }
    
    json_object *j_transition = json_object_new_object();
    json_object_object_add(j_transition, "type", json_object_new_string(STATE_JSON_TRANSITION));
    json_object_object_add(j_transition, "timestamp", json_object_new_int64(time(NULL)));
    
    json_object *j_trans_data = json_object_new_object();
    json_object_object_add(j_trans_data, "from_state", json_object_new_string(from_state ? from_state : "unknown"));
    json_object_object_add(j_trans_data, "to_state", json_object_new_string(to_state ? to_state : "unknown"));
    json_object_object_add(j_trans_data, "trigger_event", json_object_new_string(event ? event : "unknown"));
    json_object_object_add(j_trans_data, "reason", json_object_new_string(reason ? reason : ""));
    
    json_object_object_add(j_transition, "transition", j_trans_data);
    
    const char *transition_json = json_object_to_json_string(j_transition);
    
    char topic[256];
    snprintf(topic, sizeof(topic), "%s%s", STATE_MQTT_BASE_TOPIC, STATE_TOPIC_TRANSITIONS);
    
    mosquitto_publish(machine->robot_data->mqtt_client, NULL, 
                     topic, 
                     strlen(transition_json), transition_json, 1, false);
    
    printf("Transizione pubblicata su %s: %s\n", topic, transition_json);
    json_object_put(j_transition);
}

void publish_state_event(StateMachine* machine, const char* event_type, const char* event_data) {
    if (!machine || !machine->robot_data || !machine->robot_data->mqtt_client) {
        return;
    }
    
    json_object *j_event = json_object_new_object();
    json_object_object_add(j_event, "type", json_object_new_string(STATE_JSON_STATUS));
    json_object_object_add(j_event, "timestamp", json_object_new_int64(time(NULL)));
    json_object_object_add(j_event, "current_state", json_object_new_string(machine->current_state ? machine->current_state->name : "unknown"));
    json_object_object_add(j_event, "event_type", json_object_new_string(event_type ? event_type : "unknown"));
    json_object_object_add(j_event, "event_data", json_object_new_string(event_data ? event_data : ""));
    
    const char *event_json = json_object_to_json_string(j_event);
    
    char topic[256];
    snprintf(topic, sizeof(topic), "%s%s", STATE_MQTT_BASE_TOPIC, STATE_TOPIC_STATUS);
    
    mosquitto_publish(machine->robot_data->mqtt_client, NULL, 
                     topic, 
                     strlen(event_json), event_json, 1, false);
    
    printf("Evento pubblicato su %s: %s\n", topic, event_json);
    json_object_put(j_event);
}

void log_state_message(StateMachine* machine, const char* level, const char* message) {
    if (!machine || !level || !message) {
        return;
    }
    
    time_t now = time(NULL);
    struct tm *tm_info = localtime(&now);
    char timestamp[64];
    strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", tm_info);
    
    printf("[%s] [%s] [%s] %s\n", 
           timestamp, level, 
           machine->current_state ? machine->current_state->name : "NULL", 
           message);
    
    // Se il debug è abilitato, scrivi anche su file
    if (machine->robot_data && machine->robot_data->debug_enabled) {
        FILE *log_file = fopen(machine->robot_data->log_file, "a");
        if (log_file) {
            fprintf(log_file, "[%s] [%s] [%s] %s\n", 
                   timestamp, level, 
                   machine->current_state ? machine->current_state->name : "NULL", 
                   message);
            fclose(log_file);
        }
    }
}

bool check_timeout(StateMachine* machine, time_t timeout_seconds) {
    if (!machine || timeout_seconds <= 0) {
        return false;
    }
    
    double time_in_state = get_time_in_state(machine);
    return time_in_state >= timeout_seconds;
}

void send_robot_command(StateMachine* machine, const char* command, const char* params) {
    if (!machine || !command || !machine->robot_data || !machine->robot_data->mqtt_client) {
        return;
    }
    
    // Crea un messaggio JSON conforme a state_machine_mqtt.h per comandi
    json_object *j_cmd = json_object_new_object();
    json_object_object_add(j_cmd, "type", json_object_new_string(STATE_JSON_COMMAND));
    json_object_object_add(j_cmd, "command", json_object_new_string(command));
    json_object_object_add(j_cmd, "parameters", json_object_new_string(params ? params : ""));
    json_object_object_add(j_cmd, "timestamp", json_object_new_int64(time(NULL)));
    json_object_object_add(j_cmd, "source", json_object_new_string("state_machine"));
    
    const char *cmd_json = json_object_to_json_string(j_cmd);
    
    // Usa il topic corretto da state_machine_mqtt.h
    char topic[256];
    snprintf(topic, sizeof(topic), "%s%s", STATE_MQTT_BASE_TOPIC, STATE_TOPIC_COMMANDS);
    
    mosquitto_publish(machine->robot_data->mqtt_client, NULL, 
                     topic, 
                     strlen(cmd_json), cmd_json, 1, false);
    
    printf("Comando inviato su %s: %s\n", topic, cmd_json);
    json_object_put(j_cmd);
    
    log_state_message(machine, "INFO", "Comando inviato al robot");
}

// Funzioni per leggere i dati dei sensori

void update_robot_data_from_mqtt(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return;
    }
    
    // Questa funzione dovrebbe essere chiamata dal callback MQTT
    // Per ora è un placeholder - l'implementazione reale dipende
    // dai messaggi MQTT ricevuti dal fusion sensor e altri componenti
}

bool is_battery_low(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return false;
    }
    
    return machine->robot_data->battery_level < 20.0; // 20% soglia batteria bassa
}

bool is_battery_full(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return false;
    }
    
    return machine->robot_data->battery_level >= 95.0; // 95% soglia batteria piena
}

// Funzione per rilevamento ostacoli stratificato con decelerazione progressiva
ObstacleDetectionResult detect_obstacles_stratified(StateMachine* machine) {
    ObstacleDetectionResult result = {false, 1.0, "none", 999.0};
    
    if (!machine || !machine->robot_data) {
        strcpy(result.detection_source, "error");
        result.recommended_speed_factor = 0.0;
        return result;
    }
    
    RobotData* data = machine->robot_data;
    const AppConfig* config = data->config;
    
    // Fallback sicuro se configurazione non disponibile
    double sonar_critical = config ? config->robot.obstacle_detection.sonar.critical_distance_m : 0.3;
    double sonar_warning = config ? config->robot.obstacle_detection.sonar.warning_distance_m : 0.5;
    double sonar_stop = config ? config->robot.obstacle_detection.sonar.stop_distance_m : 0.15;
    double camera_warning = config ? config->robot.obstacle_detection.camera.warning_distance_m : 2.0;
    
    // PRIORITÀ 1: BUMPER (Contatto diretto - STOP IMMEDIATO)
    if (data->bumper_triggered) {
        result.obstacle_detected = true;
        result.recommended_speed_factor = 0.0;  // STOP
        strcpy(result.detection_source, "bumper_contact");
        result.closest_distance_m = 0.0;
        return result;
    }
    
    // PRIORITÀ 2: SONAR (Distanza ravvicinata - Decelerazione forte)
    double min_sonar_distance = 999.0;
    bool sonar_detected = false;
    
    // Controlla tutti e tre i sonar (converti da cm a metri)
    double sonar_distances[3] = {
        data->sonar_front_left / 100.0,
        data->sonar_front_center / 100.0,
        data->sonar_front_right / 100.0
    };
    
    for (int i = 0; i < 3; i++) {
        if (sonar_distances[i] < min_sonar_distance) {
            min_sonar_distance = sonar_distances[i];
        }
        
        if (sonar_distances[i] <= sonar_stop) {
            result.obstacle_detected = true;
            result.recommended_speed_factor = 0.0;  // STOP
            snprintf(result.detection_source, sizeof(result.detection_source), "sonar_%d_stop", i);
            result.closest_distance_m = sonar_distances[i];
            return result;
        }
        else if (sonar_distances[i] <= sonar_critical) {
            sonar_detected = true;
            result.obstacle_detected = true;
            result.recommended_speed_factor = 0.1;  // Velocità molto bassa
            snprintf(result.detection_source, sizeof(result.detection_source), "sonar_%d_critical", i);
            result.closest_distance_m = sonar_distances[i];
        }
        else if (sonar_distances[i] <= sonar_warning && !sonar_detected) {
            result.obstacle_detected = true;
            result.recommended_speed_factor = config ? config->robot.obstacle_detection.sonar.deceleration_factor : 0.3;
            snprintf(result.detection_source, sizeof(result.detection_source), "sonar_%d_warning", i);
            result.closest_distance_m = sonar_distances[i];
        }
    }
    
    // PRIORITÀ 3: TELECAMERA (Distanza lunga - Decelerazione graduale)
    if (data->camera_obstacle && !result.obstacle_detected) {
        result.obstacle_detected = true;
        result.recommended_speed_factor = config ? config->robot.obstacle_detection.camera.deceleration_factor : 0.7;
        strcpy(result.detection_source, "camera_long_range");
        result.closest_distance_m = camera_warning;  // Stima distanza telecamera
    }
    
    return result;
}

// Funzione per calcolare la velocità sicura basata sui sensori
double calculate_safe_speed(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return 0.0;  // Fallback sicuro
    }
    
    ObstacleDetectionResult obstacle_result = detect_obstacles_stratified(machine);
    const AppConfig* config = machine->robot_data->config;
    
    // Velocità massima dalla configurazione (fallback: 1.11 m/s = 4 km/h)
    double max_speed = config ? config->robot.navigation.max_speed_mps : 1.11;
    double min_speed = config ? config->robot.navigation.min_speed_mps : 0.1;
    
    // Se c'è un ostacolo, usa il fattore di decelerazione raccomandato
    if (obstacle_result.obstacle_detected) {
        double recommended_speed = max_speed * obstacle_result.recommended_speed_factor;
        
        // Assicurati che la velocità non sia mai sotto il minimo (tranne per stop = 0.0)
        if (obstacle_result.recommended_speed_factor > 0.0 && recommended_speed < min_speed) {
            recommended_speed = min_speed;
        }
        
        // Log della decelerazione per debugging
        char speed_log[128];
        snprintf(speed_log, sizeof(speed_log), 
                "Decelerazione: %s -> %.2f m/s (fattore: %.2f, distanza: %.2fm)",
                obstacle_result.detection_source, recommended_speed, 
                obstacle_result.recommended_speed_factor, obstacle_result.closest_distance_m);
        log_state_message(machine, "DEBUG", speed_log);
        
        return recommended_speed;
    }
    
    // Nessun ostacolo: velocità massima
    return max_speed;
}

// Funzione di compatibilità per il codice esistente
bool has_obstacle(StateMachine* machine) {
    ObstacleDetectionResult result = detect_obstacles_stratified(machine);
    return result.obstacle_detected;
}

bool is_at_dock(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return false;
    }
    
    // Controlla se il robot è vicino alla posizione di dock
    // Per ora usiamo una posizione fissa (0, 0)
    double dock_x = 0.0, dock_y = 0.0;
    double distance = sqrt(pow(machine->robot_data->x - dock_x, 2) + 
                          pow(machine->robot_data->y - dock_y, 2));
    
    return distance < 0.5; // Entro 50cm dal dock
}

bool is_perimeter_detected(StateMachine* machine) {
    if (!machine || !machine->robot_data) {
        return false;
    }
    
    return machine->robot_data->perimeter_detected;
}
