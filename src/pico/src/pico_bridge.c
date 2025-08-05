/**
 * Pico Bridge - MQTT Bridge for Smart Mower Pico Firmware (REFACTORED)
 * 
 * Connects Raspberry Pi Pico firmware to MQTT broker
 * - Receives sensor data from Pico via UART
 * - Publishes data to MQTT topics
 * - Receives MQTT commands and sends to Pico
 * 
 * This version uses the pico_mqtt.h header for all MQTT definitions
 */

#define _GNU_SOURCE
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <signal.h>
#include <errno.h>
#include <jansson.h>
#include <mosquitto.h>
#include <pthread.h>
#include <sys/time.h>
#include <sys/ioctl.h>
#include <time.h>
#include <math.h>
#include "pico_mqtt.h"  // Include our MQTT definitions

#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif

// Default configuration values
#define DEFAULT_CONFIG_FILE "/opt/smartmower/etc/config/robot_config.json"
#define DEFAULT_BAUDRATE B115200
#define DEFAULT_BUFFER_SIZE 2048
#define MAX_TOPIC_LEN 512
#define MAX_PAYLOAD_LEN 1024

// Battery voltage curve point
typedef struct {
    int percentage;
    float voltage_per_cell;
} voltage_curve_point_t;

// Battery state enumeration
typedef enum {
    BATTERY_DISCHARGING,    // current > 0 (battery powers robot)
    BATTERY_CHARGING,       // current < 0 (battery being charged)
    BATTERY_IDLE           // current ≈ 0 (no significant current flow)
} battery_state_t;

// Battery charging configuration
typedef struct {
    float full_voltage_per_cell;    // Voltage per cell for full charge (V)
    float trickle_current_ma;       // Trickle current threshold (mA)
    int stable_time_seconds;        // Time to maintain conditions (s)
    float current_threshold_ma;     // Current threshold for state detection (mA)
} charging_config_t;

// Battery configuration structure
typedef struct {
    char type[64];                      // Battery type (lithium_ion, etc.)
    int cells;                          // Number of cells in series
    float nominal_voltage_per_cell;     // Nominal voltage per cell (V)
    float max_voltage_per_cell;         // Maximum voltage per cell (V)
    float min_voltage_per_cell;         // Minimum voltage per cell (V)
    float capacity_ah;                  // Battery capacity in Ah
    charging_config_t charging;         // Charging configuration
    voltage_curve_point_t curve[12];    // Voltage curve points
    int curve_points;                   // Number of curve points
} battery_config_t;

// MQTT topic configuration
typedef struct {
    char base[256];
    struct {
        char sensors[64];
        char odometry[64];
        char status[64];
        char heartbeat[64];
        struct {
            char motors[64];
            char system[64];
        } commands;
    } subtopics;
    int qos;
    bool retain;
} mqtt_config_t;

// Configuration structure
typedef struct {
    // Pico UART configuration
    char uart_device[256];
    int baudrate;
    int uart_timeout;
    
    // Battery profile reference
    char battery_profile[64];           // Name of battery profile to use
    battery_config_t battery;           // Loaded battery configuration
    
    // MQTT configuration
    char mqtt_host[256];
    int mqtt_port;
    char mqtt_username[256];
    char mqtt_password[256];
    char client_id[64];
    int keepalive;
    mqtt_config_t mqtt;                 // MQTT topic configuration
    
    // Logging configuration
    int log_level;
    char log_file[256];
} config_t;

// Global variables
static int uart_fd = -1;
static struct mosquitto *mosq = NULL;
static config_t config;
static volatile int running = 1;
static pthread_t uart_thread;

// Data storage for periodic publishing
static pico_sensor_data_t latest_sensor_data;
static pico_status_report_t latest_status_report;
static bool has_sensor_data = false;
static bool has_status_data = false;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

/**
 * Calculate battery percentage based on voltage and current using INA226 data
 * Uses voltage curve interpolation for accurate SoC estimation
 */
float calculate_battery_percentage(float voltage, float current) {
    if (config.battery.cells == 0 || config.battery.curve_points == 0) {
        return -1.0f; // Invalid configuration
    }
    
    // Calculate voltage per cell
    float voltage_per_cell = voltage / config.battery.cells;
    
    // Clamp voltage to valid range
    if (voltage_per_cell > config.battery.max_voltage_per_cell) {
        voltage_per_cell = config.battery.max_voltage_per_cell;
    }
    if (voltage_per_cell < config.battery.min_voltage_per_cell) {
        voltage_per_cell = config.battery.min_voltage_per_cell;
    }
    
    // Apply current compensation (voltage drop under load)
    // For Li-Ion: ~0.1V drop per 1A load (simplified model)
    float compensated_voltage = voltage_per_cell;
    if (current > 0.1f) { // Only compensate for significant discharge current
        compensated_voltage += (current * 0.05f); // Conservative compensation
    }
    
    // Find percentage using voltage curve interpolation
    for (int i = 0; i < config.battery.curve_points - 1; i++) {
        float v1 = config.battery.curve[i].voltage_per_cell;
        float v2 = config.battery.curve[i + 1].voltage_per_cell;
        int p1 = config.battery.curve[i].percentage;
        int p2 = config.battery.curve[i + 1].percentage;
        
        // Check if voltage is between these two points
        if ((compensated_voltage >= v2 && compensated_voltage <= v1) ||
            (compensated_voltage >= v1 && compensated_voltage <= v2)) {
            
            // Linear interpolation
            float voltage_range = v1 - v2;
            if (fabs(voltage_range) < 0.001f) {
                return (float)p1; // Avoid division by zero
            }
            
            float voltage_offset = compensated_voltage - v2;
            float percentage_range = (float)(p1 - p2);
            float percentage = (float)p2 + (voltage_offset / voltage_range) * percentage_range;
            
            // Clamp to valid range
            if (percentage > 100.0f) percentage = 100.0f;
            if (percentage < 0.0f) percentage = 0.0f;
            
            return percentage;
        }
    }
    
    // If voltage is outside curve range, return boundary values
    if (compensated_voltage >= config.battery.curve[0].voltage_per_cell) {
        return (float)config.battery.curve[0].percentage;
    } else {
        return (float)config.battery.curve[config.battery.curve_points - 1].percentage;
    }
}

/**
 * Get battery state based on current direction
 */
battery_state_t get_battery_state(float current) {
    float threshold = config.battery.charging.current_threshold_ma;
    
    if (current > (threshold / 1000.0f)) {
        return BATTERY_DISCHARGING;
    } else if (current < -(threshold / 1000.0f)) {
        return BATTERY_CHARGING;
    } else {
        return BATTERY_IDLE;
    }
}

/**
 * Get battery state as string
 */
const char* get_battery_state_string(battery_state_t state) {
    switch (state) {
        case BATTERY_DISCHARGING: return "discharging";
        case BATTERY_CHARGING: return "charging";
        case BATTERY_IDLE: return "idle";
        default: return "unknown";
    }
}

/**
 * Check if battery is fully charged based on voltage, current and time criteria
 */
bool is_battery_fully_charged(float voltage, float current) {
    static time_t stable_start = 0;
    static bool was_stable = false;
    
    float voltage_per_cell = voltage / config.battery.cells;
    float current_ma = fabs(current * 1000.0f);
    
    // Conditions for full charge:
    bool voltage_ok = (voltage_per_cell >= config.battery.charging.full_voltage_per_cell);
    bool current_ok = (current_ma <= config.battery.charging.trickle_current_ma);
    bool is_charging = (current < 0); // Must be in charging state
    
    bool conditions_met = voltage_ok && current_ok && is_charging;
    
    if (conditions_met) {
        if (!was_stable) {
            stable_start = time(NULL);
            was_stable = true;
        }
        
        // Check stability over time
        if ((time(NULL) - stable_start) >= config.battery.charging.stable_time_seconds) {
            return true; // BATTERY FULLY CHARGED
        }
    } else {
        was_stable = false;
    }
    
    return false;
}

/**
 * Get remaining time for charge completion (in seconds)
 */
int get_charge_complete_time_remaining(float voltage, float current) {
    static time_t stable_start = 0;
    static bool was_stable = false;
    
    float voltage_per_cell = voltage / config.battery.cells;
    float current_ma = fabs(current * 1000.0f);
    
    bool voltage_ok = (voltage_per_cell >= config.battery.charging.full_voltage_per_cell);
    bool current_ok = (current_ma <= config.battery.charging.trickle_current_ma);
    bool is_charging = (current < 0);
    
    bool conditions_met = voltage_ok && current_ok && is_charging;
    
    if (conditions_met) {
        if (!was_stable) {
            stable_start = time(NULL);
            was_stable = true;
        }
        
        int elapsed = (int)(time(NULL) - stable_start);
        int remaining = config.battery.charging.stable_time_seconds - elapsed;
        return (remaining > 0) ? remaining : 0;
    } else {
        was_stable = false;
        return config.battery.charging.stable_time_seconds;
    }
}

// Function prototypes
int init_uart(const char *device, int baudrate);
int init_mqtt(config_t *config);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);
void process_pico_data(const char *json_data);
void send_command_to_pico(const char *command);
void uart_reader_thread(void *arg);
void publish_periodic_status(void);
void publish_bridge_heartbeat(void);
int load_config(const char *filename, config_t *config);
speed_t get_baudrate_constant(int baudrate);
void process_binary_data(const uint8_t *data, size_t length);
void sensor_data_to_json(const pico_sensor_data_t *data, char *json_buffer, size_t buffer_size);
void status_report_to_json(const pico_status_report_t *data, char *json_buffer, size_t buffer_size);
// Helper function to publish MQTT messages with proper configuration
static int publish_mqtt(const char *subtopic, const char *payload, bool retain) {
    if (!mosq || !subtopic || !payload) {
        return -1;
    }
    
    // Build full topic
    char topic[512];
    snprintf(topic, sizeof(topic), "%s/%s", config.mqtt.base, subtopic);
    
    // Publish with configured QoS and retain flag
    int rc = mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 
                              config.mqtt.qos, retain ? config.mqtt.retain : false);
    
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Error publishing to %s: %s\n", topic, mosquitto_strerror(rc));
        return -1;
    }
    
    if (config.log_level > 1) {
        printf("Published to %s (QoS: %d, Retain: %d): %s\n", 
               topic, config.mqtt.qos, retain ? config.mqtt.retain : 0, payload);
    }
    
    return 0;
}

void odometry_data_to_json(const pico_status_report_t *data, char *json_buffer, size_t buffer_size);
void send_motor_command(float left, float right, float blade1, float blade2);
void send_system_command(uint8_t cmd_id, float value);
void signal_handler(int sig);

int main(int argc, char *argv[]) {
    const char *config_file = "config.json";
    
    // Parse command line arguments
    if (argc > 1) {
        config_file = argv[1];
    }
    
    // Load configuration from JSON file
    if (load_config(config_file, &config) != 0) {
        fprintf(stderr, "Failed to load configuration from %s\n", config_file);
        return 1;
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("Starting Pico Bridge with config: %s\n", config_file);
    printf("UART: %s @ %d baud\n", config.uart_device, config.baudrate);
    printf("MQTT: %s:%d, client_id: %s\n", config.mqtt_host, config.mqtt_port, config.client_id);
    printf("MQTT base topic: %s\n", config.mqtt.base);
    printf("MQTT credentials: user='%s'\n", config.mqtt_username);
    printf("MQTT QoS: %d, Retain: %s\n", config.mqtt.qos, config.mqtt.retain ? "true" : "false");
    printf("MQTT topics: sensors=%s, odometry=%s, status=%s, heartbeat=%s\n",
           config.mqtt.subtopics.sensors,
           config.mqtt.subtopics.odometry,
           config.mqtt.subtopics.status,
           config.mqtt.subtopics.heartbeat);
    printf("MQTT command topics: motors=%s, system=%s\n",
           config.mqtt.subtopics.commands.motors,
           config.mqtt.subtopics.commands.system);
    
    // Initialize UART
    uart_fd = init_uart(config.uart_device, config.baudrate);
    if (uart_fd < 0) {
        fprintf(stderr, "Failed to initialize UART\n");
        return 1;
    }
    
    // Initialize MQTT
    if (init_mqtt(&config) != 0) {
        fprintf(stderr, "Failed to initialize MQTT\n");
        close(uart_fd);
        return 1;
    }
    
    // Start UART reader thread
    if (pthread_create(&uart_thread, NULL, (void*)uart_reader_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create UART reader thread\n");
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        close(uart_fd);
        return 1;
    }
    
    // Main loop
    time_t last_heartbeat = 0;
    time_t last_status = 0;
    
    while (running) {
        mosquitto_loop(mosq, 1000, 1);
        
        time_t now = time(NULL);
        
        // Publish heartbeat every 5 seconds
        if (now - last_heartbeat >= PICO_HEARTBEAT_INTERVAL_SEC) {
            publish_bridge_heartbeat();
            last_heartbeat = now;
        }
        
        // Publish bridge status every second
        // Sensor data is published immediately when received from Pico (100Hz)
        if (now - last_status >= 1) {
            publish_periodic_status();
            last_status = now;
        }
        
        sleep(1);
    }
    
    // Cleanup
    running = 0;
    pthread_join(uart_thread, NULL);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    close(uart_fd);
    
    printf("Pico Bridge stopped\n");
    return 0;
}

void signal_handler(int sig) {
    (void)sig; // Suppress unused parameter warning
    printf("\nReceived signal, shutting down...\n");
    running = 0;
}

int init_uart(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening UART device");
        return -1;
    }
    
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting UART attributes");
        close(fd);
        return -1;
    }
    
    speed_t baud = get_baudrate_constant(baudrate);
    cfsetospeed(&tty, baud);
    cfsetispeed(&tty, baud);
    
    tty.c_cflag = (tty.c_cflag & ~CSIZE) | CS8;
    tty.c_iflag &= ~IGNBRK;
    tty.c_lflag = 0;
    tty.c_oflag = 0;
    tty.c_cc[VMIN] = 0;
    tty.c_cc[VTIME] = 5;
    
    tty.c_iflag &= ~(IXON | IXOFF | IXANY);
    tty.c_cflag |= (CLOCAL | CREAD);
    tty.c_cflag &= ~(PARENB | PARODD);
    tty.c_cflag &= ~CSTOPB;
    tty.c_cflag &= ~CRTSCTS;
    
    if (tcsetattr(fd, TCSANOW, &tty) != 0) {
        perror("Error setting UART attributes");
        close(fd);
        return -1;
    }
    
    return fd;
}

// Helper function to build topic strings
static void build_topic(char *dest, size_t dest_size, const char *base, const char *subtopic) {
    snprintf(dest, dest_size, "%s/%s", base, subtopic);
}

// Helper function to subscribe to a topic with error checking
static int mqtt_subscribe_topic(const char *topic, int qos) {
    if (!mosq || !topic) return -1;
    
    int rc = mosquitto_subscribe(mosq, NULL, topic, qos);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to subscribe to topic %s: %s\n", 
                topic, mosquitto_strerror(rc));
        return -1;
    }
    printf("Subscribed to topic: %s (QoS: %d)\n", topic, qos);
    return 0;
}

int init_mqtt(config_t *config) {
    mosquitto_lib_init();
    
    // Create MQTT client with configured client ID
    mosq = mosquitto_new(config->client_id, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Failed to create mosquitto client\n");
        return -1;
    }
    
    // Set username and password if provided
    if (strlen(config->mqtt_username) > 0) {
        printf("Setting MQTT credentials: user='%s'\n", config->mqtt_username);
        mosquitto_username_pw_set(mosq, config->mqtt_username, config->mqtt_password);
    } else {
        printf("No MQTT credentials provided, connecting without authentication\n");
    }
    
    // Set message callback
    mosquitto_message_callback_set(mosq, mqtt_message_callback);
    
    // Connect to broker
    printf("Attempting to connect to MQTT broker %s:%d with client ID '%s' (keepalive: %d)\n", 
           config->mqtt_host, config->mqtt_port, config->client_id, config->keepalive);
    
    int rc = mosquitto_connect(mosq, config->mqtt_host, config->mqtt_port, config->keepalive);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to connect to MQTT broker: %s\n", mosquitto_strerror(rc));
        return -1;
    }
    
    printf("Connected to MQTT broker\n");
    
    // Subscribe to command topics
    char topic_buffer[512];
    
    // Subscribe to motor commands
    build_topic(topic_buffer, sizeof(topic_buffer), 
               config->mqtt.base, 
               config->mqtt.subtopics.commands.motors);
    mqtt_subscribe_topic(topic_buffer, config->mqtt.qos);
    
    // Subscribe to system commands
    build_topic(topic_buffer, sizeof(topic_buffer),
               config->mqtt.base,
               config->mqtt.subtopics.commands.system);
    mqtt_subscribe_topic(topic_buffer, config->mqtt.qos);
    
    printf("MQTT initialization complete\n");
    return 0;
}

void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    (void)mosq;
    (void)userdata;
    
    if (message->payloadlen == 0) return;
    
    char *topic = message->topic;
    char *payload = (char*)message->payload;
    
    printf("Received MQTT message on topic: %s\n", topic);
    
    // Build expected command topics for comparison
    char motor_cmd_topic[512];
    char system_cmd_topic[512];
    snprintf(motor_cmd_topic, sizeof(motor_cmd_topic), "%s/%s", 
             config.mqtt.base, config.mqtt.subtopics.commands.motors);
    snprintf(system_cmd_topic, sizeof(system_cmd_topic), "%s/%s",
             config.mqtt.base, config.mqtt.subtopics.commands.system);
    
    if (payload) {
        json_error_t error;
        json_t *root = json_loads(payload, 0, &error);
        if (!root) {
            fprintf(stderr, "JSON parse error in MQTT message: %s\n", error.text);
            return;
        }
        
        // Handle motor commands using configured topics
        if (strcmp(topic, motor_cmd_topic) == 0) {
            json_t *left = json_object_get(root, "left_speed");
            json_t *right = json_object_get(root, "right_speed");
            json_t *blade1 = json_object_get(root, "blade1_speed");
            json_t *blade2 = json_object_get(root, "blade2_speed");
            
            if (json_is_number(left) && json_is_number(right) && 
                json_is_number(blade1) && json_is_number(blade2)) {
                float left_val = json_number_value(left);
                float right_val = json_number_value(right);
                float blade1_val = json_number_value(blade1);
                float blade2_val = json_number_value(blade2);
                
                if (config.log_level > 0) {
                    printf("Received motor command: L=%.1f R=%.1f B1=%.1f B2=%.1f\n", 
                           left_val, right_val, blade1_val, blade2_val);
                }
                
                send_motor_command(left_val, right_val, blade1_val, blade2_val);
            }
        }
        // Handle system commands using configured topics
        else if (strcmp(topic, system_cmd_topic) == 0) {
            json_t *command = json_object_get(root, "command");
            json_t *value = json_object_get(root, "value");
            
            if (json_is_string(command)) {
                const char *cmd_str = json_string_value(command);
                float cmd_value = json_is_number(value) ? json_number_value(value) : 0.0;
                
                if (strcmp(cmd_str, "emergency_stop") == 0) {
                    send_system_command(PICO_SYS_CMD_EMERGENCY_STOP, cmd_value);
                } else if (strcmp(cmd_str, "reset") == 0) {
                    send_system_command(PICO_SYS_CMD_RESET, cmd_value);
                } else if (strcmp(cmd_str, "calibrate") == 0) {
                    send_system_command(PICO_SYS_CMD_CALIBRATE, cmd_value);
                }
            }
        }
        
        json_decref(root);
    }
}

void process_binary_data(const uint8_t *data, size_t length) {
    if (length < 1) return;
    
    uint8_t msg_type = data[0];
    printf("Received message type: 0x%02X, length: %zu\n", msg_type, length);
    
    switch (msg_type) {
        case PICO_MSG_SENSOR_DATA:
            printf("Processing PICO_MSG_SENSOR_DATA\n");
            if (length >= sizeof(pico_sensor_data_t)) {
                const pico_sensor_data_t *sensor_data = (const pico_sensor_data_t*)data;
                
                // Store latest data for potential future use
                pthread_mutex_lock(&data_mutex);
                memcpy(&latest_sensor_data, data, sizeof(pico_sensor_data_t));
                has_sensor_data = true;
                pthread_mutex_unlock(&data_mutex);
                
                // Publish sensor data (100Hz from Pico)
                char json_buffer[MAX_PAYLOAD_LEN];
                sensor_data_to_json(sensor_data, json_buffer, sizeof(json_buffer));
                
                // Publish with configured QoS and retain flag
                if (publish_mqtt(config.mqtt.subtopics.sensors, json_buffer, false) == 0) {
                    if (config.log_level > 1) {
                        printf("Sensor data published successfully\n");
                    }
                }
            } else {
                printf("Sensor data length too small: %zu < %zu\n", length, sizeof(pico_sensor_data_t));
            }
            break;
            
        case PICO_MSG_STATUS_REPORT:
            printf("Processing PICO_MSG_STATUS_REPORT\n");
            if (length >= sizeof(pico_status_report_t)) {
                const pico_status_report_t *status_data = (const pico_status_report_t*)data;
                
                // Store latest data for potential future use
                pthread_mutex_lock(&data_mutex);
                memcpy(&latest_status_report, data, sizeof(pico_status_report_t));
                has_status_data = true;
                pthread_mutex_unlock(&data_mutex);
                
                // Publish odometry data (motors, encoders)
                char json_buffer[MAX_PAYLOAD_LEN];
                odometry_data_to_json(status_data, json_buffer, sizeof(json_buffer));
                
                // Publish with configured QoS and retain flag
                if (publish_mqtt(config.mqtt.subtopics.odometry, json_buffer, false) == 0) {
                    if (config.log_level > 1) {
                        printf("Odometry data published successfully\n");
                    }
                }
            } else {
                printf("Status data length too small: %zu < %zu\n", length, sizeof(pico_status_report_t));
            }
            break;
            
        default:
            printf("Unknown message type: 0x%02X\n", msg_type);
            break;
    }
}

void sensor_data_to_json(const pico_sensor_data_t *data, char *json_buffer, size_t buffer_size) {
    // Calculate battery percentage from INA226 data (voltage and current)
    float battery_percentage = calculate_battery_percentage(data->power[0], data->power[1]);
    
    // Get battery state and charging status
    battery_state_t battery_state = get_battery_state(data->power[1]);
    const char* battery_state_str = get_battery_state_string(battery_state);
    bool is_fully_charged = is_battery_fully_charged(data->power[0], data->power[1]);
    int charge_time_remaining = get_charge_complete_time_remaining(data->power[0], data->power[1]);
    
    // Check individual charge completion conditions
    float voltage_per_cell = data->power[0] / config.battery.cells;
    float current_ma = fabs(data->power[1] * 1000.0f);
    bool voltage_ok = (voltage_per_cell >= config.battery.charging.full_voltage_per_cell);
    bool current_ok = (current_ma <= config.battery.charging.trickle_current_ma);
    
    snprintf(json_buffer, buffer_size,
        "{"
        "\"type\":\"%s\","
        "\"timestamp\":%u,"
        "\"imu\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"magnetometer\":[%.3f,%.3f,%.3f],"
        "\"ultrasonic\":[%.3f,%.3f,%.3f],"
        "\"power\":{"
            "\"voltage\":%.2f,"
            "\"current\":%.2f,"
            "\"battery_percentage\":%.1f,"
            "\"battery_state\":\"%s\","
            "\"is_fully_charged\":%s,"
            "\"charge_complete_conditions\":{"
                "\"voltage_ok\":%s,"
                "\"current_ok\":%s,"
                "\"stable_time_remaining\":%d"
            "}"
        "},"
        "\"safety\":{"
            "\"emergency_stop\":%s,"
            "\"rain_sensor\":%s,"
            "\"bumper\":%s,"
            "\"lift_sensor\":%s"
        "}"
        "}",
        PICO_JSON_SENSOR_DATA,
        data->timestamp,
        data->imu[0], data->imu[1], data->imu[2], data->imu[3], data->imu[4], data->imu[5],
        data->magnetometer[0], data->magnetometer[1], data->magnetometer[2],
        data->ultrasonic[0], data->ultrasonic[1], data->ultrasonic[2],
        data->power[0], data->power[1], battery_percentage,
        battery_state_str,
        (is_fully_charged ? "true" : "false"),
        (voltage_ok ? "true" : "false"),
        (current_ok ? "true" : "false"),
        charge_time_remaining,
        (data->flags[0] ? "true" : "false"),
        (data->flags[1] ? "true" : "false"),
        (data->flags[2] ? "true" : "false"),
        (data->flags[3] ? "true" : "false")
    );
}

void status_report_to_json(const pico_status_report_t *data, char *json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
        "{"
        "\"type\":\"%s\","
        "\"timestamp\":%u,"
        "\"motors\":{"
            "\"left_speed\":%.1f,"
            "\"right_speed\":%.1f,"
            "\"blade1_speed\":%.1f,"
            "\"blade2_speed\":%.1f,"
            "\"left_rpm\":%.1f,"
            "\"right_rpm\":%.1f,"
            "\"blade1_rpm\":%.1f,"
            "\"blade2_rpm\":%.1f"
        "},"
        "\"encoders\":[%u,%u,%u,%u],"
        "\"system\":{"
            "\"running\":true,"
            "\"uart_connected\":%s,"
            "\"communication_ok\":%s,"
            "\"relay_state\":%s"
        "}"
        "}",
        PICO_JSON_STATUS_REPORT,  // Use constant from header
        data->timestamp,
        data->motor_speeds[0], data->motor_speeds[1], data->motor_speeds[2], data->motor_speeds[3],
        data->motor_rpm[0], data->motor_rpm[1], data->motor_rpm[2], data->motor_rpm[3],
        data->encoder_counts[0], data->encoder_counts[1], data->encoder_counts[2], data->encoder_counts[3],
        (uart_fd >= 0 ? "true" : "false"),
        (uart_fd >= 0 ? "true" : "false"),
        (data->relay_state ? "true" : "false")
    );
}

void odometry_data_to_json(const pico_status_report_t *data, char *json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
        "{"
        "\"type\":\"%s\","
        "\"timestamp\":%u,"
        "\"motors\":{"
            "\"left_speed\":%.1f,"
            "\"right_speed\":%.1f,"
            "\"blade1_speed\":%.1f,"
            "\"blade2_speed\":%.1f,"
            "\"left_rpm\":%.1f,"
            "\"right_rpm\":%.1f,"
            "\"blade1_rpm\":%.1f,"
            "\"blade2_rpm\":%.1f"
        "},"
        "\"encoders\":[%u,%u,%u,%u],"
        "\"system\":{"
            "\"running\":true,"
            "\"relay_state\":%s"
        "}"
        "}",
        PICO_JSON_ODOMETRY_DATA,  // Use odometry constant from header
        data->timestamp,
        data->motor_speeds[0], data->motor_speeds[1], data->motor_speeds[2], data->motor_speeds[3],
        data->motor_rpm[0], data->motor_rpm[1], data->motor_rpm[2], data->motor_rpm[3],
        data->encoder_counts[0], data->encoder_counts[1], data->encoder_counts[2], data->encoder_counts[3],
        (data->relay_state ? "true" : "false")
    );
}

void send_motor_command(float left, float right, float blade1, float blade2) {
    pico_motor_command_t cmd = {
        .type = PICO_MSG_MOTOR_CMD,
        .left_speed = left,
        .right_speed = right,
        .blade1_speed = blade1,
        .blade2_speed = blade2
    };
    
    if (uart_fd >= 0) {
        write(uart_fd, &cmd, sizeof(cmd));
        printf("Sent motor command: L=%.1f R=%.1f B1=%.1f B2=%.1f\n", left, right, blade1, blade2);
    }
}

void send_system_command(uint8_t cmd_id, float value) {
    pico_system_command_t cmd = {
        .type = PICO_MSG_SYSTEM_CMD,
        .command_id = cmd_id,
        .value = value
    };
    
    if (uart_fd >= 0) {
        write(uart_fd, &cmd, sizeof(cmd));
        printf("Sent system command: ID=%d Value=%.2f\n", cmd_id, value);
    }
}

void publish_bridge_heartbeat(void) {
    static time_t start_time = 0;
    time_t now = time(NULL);
    
    if (start_time == 0) {
        start_time = now;
    }
    
    char json_buffer[MAX_PAYLOAD_LEN];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\":\"%s\","
        "\"timestamp\":%ld,"
        "\"status\":\"running\","
        "\"uptime\":%ld"
        "}",
        PICO_JSON_BRIDGE_HEARTBEAT,  // Use constant from header
        now,
        now - start_time
    );
    
    // Publish with configured QoS and retain flag
    publish_mqtt(config.mqtt.subtopics.heartbeat, json_buffer, true);
}

// Publish latest sensor data to MQTT
void publish_sensor_data(void) {
    pthread_mutex_lock(&data_mutex);
    
    if (has_sensor_data) {
        char json_buffer[MAX_PAYLOAD_LEN];
        sensor_data_to_json(&latest_sensor_data, json_buffer, sizeof(json_buffer));
        
        // Publish with configured QoS and retain flag
        publish_mqtt(config.mqtt.subtopics.sensors, json_buffer, true);
    }
    
    pthread_mutex_unlock(&data_mutex);
}

// Publish bridge status to MQTT
void publish_periodic_status(void) {
    time_t now = time(NULL);
    
    // Check UART connection status
    bool uart_connected = (uart_fd >= 0);
    
    // Check if we have recent data from Pico
    pthread_mutex_lock(&data_mutex);
    bool communication_ok = has_sensor_data || has_status_data;
    pthread_mutex_unlock(&data_mutex);
    
    char json_buffer[MAX_PAYLOAD_LEN];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\":\"pico_status\","
        "\"timestamp\":%ld,"
        "\"system\":{"
            "\"running\":true,"
            "\"uart_connected\":%s,"
            "\"communication_ok\":%s"
        "},"
        "\"performance\":{"
            "\"update_rate\":1,"
            "\"data_age\":%ld"
        "}"
        "}",
        now,
        uart_connected ? "true" : "false",
        communication_ok ? "true" : "false",
        now
    );
    
    // Publish with configured QoS and retain flag
    publish_mqtt(config.mqtt.subtopics.status, json_buffer, true);
}

void uart_reader_thread(void *arg) {
    (void)arg;
    uint8_t buffer[DEFAULT_BUFFER_SIZE];
    
    while (running) {
        ssize_t bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            process_binary_data(buffer, bytes_read);
        } else if (bytes_read < 0 && errno != EAGAIN) {
            perror("UART read error");
            break;
        }
        usleep(10000); // 10ms delay
    }
}

// Load battery profile from centralized config
int load_battery_profile(json_t *root, const char *profile_name, battery_config_t *battery) {
    json_t *profiles = json_object_get(root, "battery_profiles");
    if (!profiles) {
        fprintf(stderr, "No battery_profiles section found\n");
        return -1;
    }
    
    json_t *profile = json_object_get(profiles, profile_name);
    if (!profile) {
        fprintf(stderr, "Battery profile '%s' not found\n", profile_name);
        return -1;
    }
    
    // Load profile parameters
    json_t *type = json_object_get(profile, "type");
    if (json_is_string(type)) {
        strcpy(battery->type, json_string_value(type));
    }
    
    json_t *cells = json_object_get(profile, "cells");
    if (json_is_integer(cells)) {
        battery->cells = json_integer_value(cells);
    }
    
    json_t *nominal_v = json_object_get(profile, "nominal_voltage_per_cell");
    if (json_is_number(nominal_v)) {
        battery->nominal_voltage_per_cell = json_number_value(nominal_v);
    }
    
    json_t *max_v = json_object_get(profile, "max_voltage_per_cell");
    if (json_is_number(max_v)) {
        battery->max_voltage_per_cell = json_number_value(max_v);
    }
    
    json_t *min_v = json_object_get(profile, "min_voltage_per_cell");
    if (json_is_number(min_v)) {
        battery->min_voltage_per_cell = json_number_value(min_v);
    }
    
    json_t *capacity = json_object_get(profile, "capacity_ah");
    if (json_is_number(capacity)) {
        battery->capacity_ah = json_number_value(capacity);
    }
    
    // Load charging parameters
    json_t *charging = json_object_get(profile, "charging");
    if (charging) {
        json_t *full_v = json_object_get(charging, "full_voltage_per_cell");
        if (json_is_number(full_v)) {
            battery->charging.full_voltage_per_cell = json_number_value(full_v);
        }
        
        json_t *trickle = json_object_get(charging, "trickle_current_ma");
        if (json_is_number(trickle)) {
            battery->charging.trickle_current_ma = json_number_value(trickle);
        }
        
        json_t *stable_time = json_object_get(charging, "stable_time_seconds");
        if (json_is_integer(stable_time)) {
            battery->charging.stable_time_seconds = json_integer_value(stable_time);
        }
        
        json_t *current_thresh = json_object_get(charging, "current_threshold_ma");
        if (json_is_number(current_thresh)) {
            battery->charging.current_threshold_ma = json_number_value(current_thresh);
        }
    }
    
    // Load voltage curve
    json_t *curve = json_object_get(profile, "voltage_curve");
    if (json_is_object(curve)) {
        battery->curve_points = 0;
        const char *key;
        json_t *value;
        json_object_foreach(curve, key, value) {
            if (battery->curve_points < 12 && json_is_number(value)) {
                int percentage = atoi(key);
                float voltage = json_number_value(value);
                battery->curve[battery->curve_points].percentage = percentage;
                battery->curve[battery->curve_points].voltage_per_cell = voltage;
                battery->curve_points++;
            }
        }
    }
    
    return 0;
}

// Configuration loading from centralized robot_config.json
int load_config(const char *filename, config_t *config) {
    // Set defaults first
    strncpy(config->uart_device, "/dev/ttyAMA1", sizeof(config->uart_device) - 1);
    config->baudrate = 115200;
    config->uart_timeout = 1000;
    strncpy(config->battery_profile, "battery_24v_lipo", sizeof(config->battery_profile) - 1);
    config->log_level = 1;  // Default to INFO level
    strncpy(config->log_file, "/var/log/smartmower/pico_bridge.log", sizeof(config->log_file) - 1);
    
    // MQTT defaults from config file
    strncpy(config->mqtt_host, "localhost", sizeof(config->mqtt_host) - 1);
    config->mqtt_port = 1883;
    config->keepalive = 60;
    strncpy(config->client_id, "pico_bridge", sizeof(config->client_id) - 1);
    strncpy(config->mqtt_username, "mower", sizeof(config->mqtt_username) - 1);
    strncpy(config->mqtt_password, "smart", sizeof(config->mqtt_password) - 1);
    
    // Set MQTT topics from config
    strncpy(config->mqtt.base, "pico", sizeof(config->mqtt.base) - 1);
    strncpy(config->mqtt.subtopics.sensors, "sensors", sizeof(config->mqtt.subtopics.sensors) - 1);
    strncpy(config->mqtt.subtopics.odometry, "odometry", sizeof(config->mqtt.subtopics.odometry) - 1);
    strncpy(config->mqtt.subtopics.status, "status", sizeof(config->mqtt.subtopics.status) - 1);
    strncpy(config->mqtt.subtopics.heartbeat, "heartbeat", sizeof(config->mqtt.subtopics.heartbeat) - 1);
    strncpy(config->mqtt.subtopics.commands.motors, "cmd/motors", sizeof(config->mqtt.subtopics.commands.motors) - 1);
    strncpy(config->mqtt.subtopics.commands.system, "cmd/system", sizeof(config->mqtt.subtopics.commands.system) - 1);
    config->mqtt.qos = 1;
    config->mqtt.retain = false;
    
    // Set battery defaults (will be overridden by profile if it exists)
    strncpy(config->battery.type, "lithium_ion", sizeof(config->battery.type) - 1);
    config->battery.cells = 6;
    config->battery.nominal_voltage_per_cell = 3.7f;
    config->battery.max_voltage_per_cell = 4.2f;
    config->battery.min_voltage_per_cell = 3.0f;
    config->battery.capacity_ah = 5.0f;
    config->battery.curve_points = 0;
    
    FILE *file = fopen(filename, "r");
    if (!file) {
        printf("Config file not found, using defaults\n");
        return 0;
    }
    
    json_error_t error;
    json_t *root = json_load_file(filename, 0, &error);
    if (!root) {
        fprintf(stderr, "JSON error in config file: %s\n", error.text);
        fclose(file);
        return -1;
    }
    
    // Parse pico configuration from centralized configuration
    json_t *pico_config = json_object_get(root, "pico_config");
    if (pico_config) {
        // UART Configuration
        json_t *uart_device = json_object_get(pico_config, "uart_device");
        if (json_is_string(uart_device)) {
            strncpy(config->uart_device, json_string_value(uart_device), 
                   sizeof(config->uart_device) - 1);
        }
        
        json_t *baudrate = json_object_get(pico_config, "baudrate");
        if (json_is_integer(baudrate)) {
            config->baudrate = json_integer_value(baudrate);
        }
        
        json_t *timeout = json_object_get(pico_config, "uart_timeout_ms");
        if (json_is_integer(timeout)) {
            config->uart_timeout = json_integer_value(timeout);
        }
        
        // Battery Configuration
        json_t *battery_profile = json_object_get(pico_config, "battery_profile");
        if (json_is_string(battery_profile)) {
            strncpy(config->battery_profile, json_string_value(battery_profile), 
                   sizeof(config->battery_profile) - 1);
        }
        
        // Logging Configuration
        json_t *logging = json_object_get(root, "pico_logging");
        if (logging) {
            json_t *log_level = json_object_get(logging, "level");
            if (json_is_string(log_level)) {
                const char *level = json_string_value(log_level);
                if (strcasecmp(level, "debug") == 0) config->log_level = 0;
                else if (strcasecmp(level, "info") == 0) config->log_level = 1;
                else if (strcasecmp(level, "warning") == 0) config->log_level = 2;
                else if (strcasecmp(level, "error") == 0) config->log_level = 3;
            }
            
            json_t *log_file = json_object_get(logging, "file");
            if (json_is_string(log_file)) {
                strncpy(config->log_file, json_string_value(log_file), 
                       sizeof(config->log_file) - 1);
            }
        }
    }
    
    // Parse configuration from tuning section
    json_t *tuning = json_object_get(root, "tuning");
    if (tuning) {
        
        // Parse communication parameters for MQTT
        json_t *comm = json_object_get(tuning, "communication_parameters");
        if (comm) {
            json_t *port = json_object_get(comm, "mqtt_port");
            if (json_is_integer(port)) {
                config->mqtt_port = json_integer_value(port);
            }
            
            json_t *keepalive = json_object_get(comm, "mqtt_keepalive");
            if (json_is_integer(keepalive)) {
                config->keepalive = json_integer_value(keepalive);
            }
        }
    }
    
    // Load battery profile from centralized configuration
    if (load_battery_profile(root, config->battery_profile, &config->battery) != 0) {
        fprintf(stderr, "Failed to load battery profile '%s', using defaults\n", config->battery_profile);
    } else {
        printf("Loaded battery profile '%s': %s %dS %.1fAh\n", 
               config->battery_profile, config->battery.type, 
               config->battery.cells, config->battery.capacity_ah);
        
        // Sort curve points by percentage (descending for easier lookup)
        for (int i = 0; i < config->battery.curve_points - 1; i++) {
            for (int j = i + 1; j < config->battery.curve_points; j++) {
                if (config->battery.curve[i].percentage < config->battery.curve[j].percentage) {
                    voltage_curve_point_t temp = config->battery.curve[i];
                    config->battery.curve[i] = config->battery.curve[j];
                    config->battery.curve[j] = temp;
                }
            }
        }
        
        printf("Loaded %d battery voltage curve points\n", config->battery.curve_points);
    }
    
    // Parse MQTT configuration
    json_t *mqtt = json_object_get(root, "mqtt");
    if (mqtt) {
        // Get MQTT broker settings
        json_t *broker = json_object_get(mqtt, "broker");
        if (json_is_string(broker)) {
            strncpy(config->mqtt_host, json_string_value(broker), 
                   sizeof(config->mqtt_host) - 1);
        }
        
        json_t *port = json_object_get(mqtt, "port");
        if (json_is_integer(port)) {
            config->mqtt_port = json_integer_value(port);
        }
        
        json_t *username = json_object_get(mqtt, "username");
        if (json_is_string(username)) {
            strncpy(config->mqtt_username, json_string_value(username), 
                   sizeof(config->mqtt_username) - 1);
        }
        
        json_t *password = json_object_get(mqtt, "password");
        if (json_is_string(password)) {
            strncpy(config->mqtt_password, json_string_value(password), 
                   sizeof(config->mqtt_password) - 1);
        }
        
        // Parse MQTT topics
        json_t *topics = json_object_get(mqtt, "topics");
        if (topics) {
            json_t *pico = json_object_get(topics, "pico");
            if (pico) {
                // Get base topic
                json_t *base = json_object_get(pico, "base");
                if (json_is_string(base)) {
                    strncpy(config->mqtt.base, json_string_value(base), 
                           sizeof(config->mqtt.base) - 1);
                }
                
                // Get keepalive from pico section if available, otherwise from root mqtt
                json_t *keepalive = json_object_get(pico, "keepalive");
                if (!keepalive) {
                    keepalive = json_object_get(mqtt, "keepalive");
                }
                if (json_is_integer(keepalive)) {
                    config->keepalive = json_integer_value(keepalive);
                }
                
                // Get client ID from pico section if available
                json_t *client_id = json_object_get(pico, "client_id");
                if (json_is_string(client_id)) {
                    strncpy(config->client_id, json_string_value(client_id),
                           sizeof(config->client_id) - 1);
                }
                
                // Get QoS and retain settings
                json_t *qos = json_object_get(pico, "qos");
                if (json_is_integer(qos)) {
                    int qos_val = json_integer_value(qos);
                    if (qos_val >= 0 && qos_val <= 2) {
                        config->mqtt.qos = qos_val;
                    }
                }
                
                json_t *retain = json_object_get(pico, "retain");
                if (json_is_boolean(retain)) {
                    config->mqtt.retain = json_boolean_value(retain);
                }
                
                // Get subtopics
                json_t *subtopics = json_object_get(pico, "subtopics");
                if (subtopics) {
                    json_t *sensors = json_object_get(subtopics, "sensors");
                    if (json_is_string(sensors)) {
                        strncpy(config->mqtt.subtopics.sensors, json_string_value(sensors),
                               sizeof(config->mqtt.subtopics.sensors) - 1);
                    }
                    
                    json_t *odometry = json_object_get(subtopics, "odometry");
                    if (json_is_string(odometry)) {
                        strncpy(config->mqtt.subtopics.odometry, json_string_value(odometry),
                               sizeof(config->mqtt.subtopics.odometry) - 1);
                    }
                    
                    json_t *status = json_object_get(subtopics, "status");
                    if (json_is_string(status)) {
                        strncpy(config->mqtt.subtopics.status, json_string_value(status),
                               sizeof(config->mqtt.subtopics.status) - 1);
                    }
                    
                    json_t *heartbeat = json_object_get(subtopics, "heartbeat");
                    if (json_is_string(heartbeat)) {
                        strncpy(config->mqtt.subtopics.heartbeat, json_string_value(heartbeat),
                               sizeof(config->mqtt.subtopics.heartbeat) - 1);
                    }
                    
                    // Get command subtopics
                    json_t *commands = json_object_get(subtopics, "commands");
                    if (commands) {
                        json_t *motors = json_object_get(commands, "motors");
                        if (json_is_string(motors)) {
                            strncpy(config->mqtt.subtopics.commands.motors, json_string_value(motors),
                                   sizeof(config->mqtt.subtopics.commands.motors) - 1);
                        }
                        
                        json_t *system = json_object_get(commands, "system");
                        if (json_is_string(system)) {
                            strncpy(config->mqtt.subtopics.commands.system, json_string_value(system),
                                   sizeof(config->mqtt.subtopics.commands.system) - 1);
                        }
                    }
                }
            }
        }
    }
            
    // Load MQTT password from system.communication section if available
    json_t *system = json_object_get(root, "system");
    if (system) {
        json_t *communication = json_object_get(system, "communication");
        if (communication) {
            json_t *mqtt_password = json_object_get(communication, "mqtt_password");
            if (mqtt_password && json_is_string(mqtt_password)) {
                strncpy(config->mqtt_password, json_string_value(mqtt_password), 
                       sizeof(config->mqtt_password) - 1);
                printf("DEBUG: Loaded MQTT password from JSON\n");
            } else {
                printf("DEBUG: No MQTT password found in JSON, using default\n");
            }
        }
    }
    
    printf("Pico Config loaded: %s @ %d baud, timeout %dms\n", 
           config->uart_device, config->baudrate, config->uart_timeout);
    printf("MQTT: %s:%d, username: %s\n", 
           config->mqtt_host, config->mqtt_port, 
           strlen(config->mqtt_username) > 0 ? config->mqtt_username : "(none)");
    
    if (root) json_decref(root);
    if (file) fclose(file);
    return 0;
}

speed_t get_baudrate_constant(int baudrate) {
    switch (baudrate) {
        case 9600: return B9600;
        case 19200: return B19200;
        case 38400: return B38400;
        case 57600: return B57600;
        case 115200: return B115200;
        case 230400: return B230400;
        case 460800: return B460800;
        case 921600: return B921600;
        default: return B115200;
    }
}
