/**
 * PICO and GPS Data Simulator
 * 
 * Simulates a robot moving along an 80m x 100m rectangular path,
 * publishing PICO sensor data and GPS coordinates via MQTT.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <signal.h>
#include <sys/time.h>  // For gettimeofday and struct timeval
#include <stdint.h>    // For uint32_t
#include <mosquitto.h>
#include <jansson.h>
#include <pthread.h>

// Constants
#define PI 3.14159265358979323846
#define WHEEL_BASE 0.5  // Distance between wheels in meters

// MQTT Topics
#define TOPIC_PICO_SENSORS "/pico/sensors"
#define TOPIC_PICO_STATUS "/pico/status"
#define TOPIC_GPS "/gps/data"
#define TOPIC_GPS_STATUS "/gps/status"
#define TOPIC_PICO_HEARTBEAT "smartmower/pico/heartbeat"
#define TOPIC_GPS_HEARTBEAT "smartmower/gps/heartbeat"

// Forward declarations
uint32_t get_timestamp_ms(void);
void update_robot_position(void);
void publish_pico_sensor_data(void);
void publish_gps_data(void);
void publish_odometry_data(void);
void publish_mqtt(const char *topic, const char *payload);
void publish_heartbeat(struct mosquitto *mosq);

// Global variables
volatile sig_atomic_t running = 1;
time_t start_time;  // System start time for uptime calculation

// Robot state structure
typedef struct robot_state {
    // Position and orientation
    double x;           // meters, 0-100
    double y;           // meters, 0-80
    double heading;     // radians, 0-2π
    
    // Motor control
    float left_speed;   // -100 to +100
    float right_speed;  // -100 to +100
    
    // System state
    int relay_state;    // 0 = off, 1 = on
    int emergency_stop; // 0 = normal, 1 = emergency stop
    
    // Timers
    time_t last_update;
    uint32_t timestamp_ms;
    
    // MQTT
    struct mosquitto *mosq;
    char mqtt_host[256];
    int mqtt_port;
    char mqtt_username[64];
    char mqtt_password[64];
    char base_topic[64];
    
    // Mutex for thread safety
    pthread_mutex_t mutex;
} robot_state_t;

// Global instance of robot state
static robot_state_t robot;

// Constants
#define PI 3.14159265358979323846
#define WHEEL_BASE 0.5      // meters between wheels
#define WHEEL_RADIUS 0.1    // meters
#define TICKS_PER_REV 360   // encoder ticks per wheel revolution

// MQTT Topics (will be prefixed with base_topic)
#define TOPIC_PICO_SENSORS "/pico/sensors"
#define TOPIC_PICO_ODOMETRY "/pico/odometry"
#define TOPIC_PICO_CMD "/pico/cmd"

// Function prototypes
void signal_handler(int sig);
int load_config(const char *filename);
int init_mqtt(void);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, 
                         const struct mosquitto_message *message);
void update_robot_position(void);
void publish_pico_sensor_data(void);
void publish_gps_data(void);
void publish_odometry_data(void);
void *simulation_loop(void *arg);

int main(int argc, char *argv[]) {
    // Initialize robot state with default values
    memset(&robot, 0, sizeof(robot));
    robot.x = 0.0;  // Start at origin
    robot.y = 0.0;
    robot.heading = 0.0;  // Facing east
    robot.relay_state = 0;  // Relay off by default
    robot.emergency_stop = 0;  // No emergency stop
    robot.left_speed = 0;
    robot.right_speed = 0;
    robot.timestamp_ms = get_timestamp_ms();
    pthread_mutex_init(&robot.mutex, NULL);
    
    const char *config_file = "/opt/smartmower/etc/config/robot_config.json";
    
    // Parse command line arguments
    if (argc > 1) {
        config_file = argv[1];
    }
    
    // Load configuration
    printf("Loading configuration from %s...\n", config_file);
    if (load_config(config_file) != 0) {
        fprintf(stderr, "Failed to load configuration from %s\n", config_file);
        return 1;
    }
    
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Initialize MQTT
    printf("Initializing MQTT connection to %s:%d...\n", robot.mqtt_host, robot.mqtt_port);
    if (init_mqtt() != 0) {
        fprintf(stderr, "Failed to initialize MQTT\n");
        return 1;
    }
    
    // Start simulation thread
    printf("Starting simulation thread...\n");
    pthread_t sim_thread;
    if (pthread_create(&sim_thread, NULL, simulation_loop, NULL) != 0) {
        fprintf(stderr, "Failed to create simulation thread\n");
        return 1;
    }
    
    printf("\n=======================================\n");
    printf("  PICO and GPS Simulator Started\n");
    printf("  MQTT Broker: %s:%d\n", robot.mqtt_host, robot.mqtt_port);
    printf("  Base Topic: %s\n", robot.base_topic);
    printf("  Simulating 80m x 100m area\n");
    printf("  Press Ctrl+C to exit\n");
    printf("=======================================\n\n");
    
    // Main thread handles MQTT network loop
    while (running) {
        // Process MQTT network events
        int rc = mosquitto_loop(robot.mosq, 100, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "MQTT network loop error: %s\n", mosquitto_strerror(rc));
            // Attempt to reconnect
            if (mosquitto_reconnect(robot.mosq) != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "Failed to reconnect to MQTT broker, retrying in 5s...\n");
                sleep(5);
            } else {
                printf("Successfully reconnected to MQTT broker\n");
                // Resubscribe to topics after reconnection
                char topic[512];
                snprintf(topic, sizeof(topic), "%s%s", robot.base_topic, TOPIC_PICO_CMD);
                mosquitto_subscribe(robot.mosq, NULL, topic, 0);
            }
        }
        
        // Small delay to prevent high CPU usage (1ms)
        const struct timespec sleep_time = {0, 1000000};  // 1ms = 1,000,000ns
        nanosleep(&sleep_time, NULL);
    }
    
    // Cleanup
    printf("Shutting down simulator...\n");
    
    // Wait for simulation thread to finish
    pthread_join(sim_thread, NULL);
    
    // Cleanup MQTT
    if (robot.mosq) {
        mosquitto_disconnect(robot.mosq);
        mosquitto_destroy(robot.mosq);
    }
    mosquitto_lib_cleanup();
    
    // Cleanup mutex
    pthread_mutex_destroy(&robot.mutex);
    
    printf("Simulator stopped.\n");
    return 0;
}

// Signal handler for clean shutdown
void signal_handler(int sig) {
    printf("Received signal %d, shutting down...\n", sig);
    running = 0;
}

// Load configuration from JSON file
int load_config(const char *filename) {
    json_error_t error;
    json_t *root = json_load_file(filename, 0, &error);
    
    if (!root) {
        fprintf(stderr, "Error parsing config file: %s (line %d, column %d)\n", 
                error.text, error.line, error.column);
        return -1;
    }
    
    // Set default values
    strncpy(robot.mqtt_host, "localhost", sizeof(robot.mqtt_host));
    robot.mqtt_port = 1883;
    strncpy(robot.mqtt_username, "", sizeof(robot.mqtt_username));
    strncpy(robot.mqtt_password, "", sizeof(robot.mqtt_password));
    strncpy(robot.base_topic, "smartmower", sizeof(robot.base_topic));
    
    // Get MQTT configuration
    json_t *system = json_object_get(root, "system");
    if (system) {
        json_t *comm = json_object_get(system, "communication");
        if (comm) {
            const char *host = json_string_value(json_object_get(comm, "mqtt_broker_host"));
            if (host) strncpy(robot.mqtt_host, host, sizeof(robot.mqtt_host) - 1);
            
            json_t *port = json_object_get(comm, "mqtt_broker_port");
            if (port) robot.mqtt_port = json_integer_value(port);
            
            const char *user = json_string_value(json_object_get(comm, "mqtt_username"));
            if (user) strncpy(robot.mqtt_username, user, sizeof(robot.mqtt_username) - 1);
            
            const char *pass = json_string_value(json_object_get(comm, "mqtt_password"));
            if (pass) strncpy(robot.mqtt_password, pass, sizeof(robot.mqtt_password) - 1);
            
            const char *base = json_string_value(json_object_get(comm, "mqtt_base_topic"));
            if (base) strncpy(robot.base_topic, base, sizeof(robot.base_topic) - 1);
        }
    }
    
    json_decref(root);
    printf("Configuration loaded from %s\n", filename);
    return 0;
}

// Initialize MQTT connection
int init_mqtt(void) {
    mosquitto_lib_init();
    
    // Create a unique client ID
    char client_id[64];
    snprintf(client_id, sizeof(client_id), "pico_simulator_%d", getpid());
    
    robot.mosq = mosquitto_new(client_id, true, NULL);
    if (!robot.mosq) {
        fprintf(stderr, "Error: Out of memory\n");
        return -1;
    }
    
    // Set username and password if provided
    if (strlen(robot.mqtt_username) > 0) {
        mosquitto_username_pw_set(robot.mosq, robot.mqtt_username, robot.mqtt_password);
    }
    
    // Set message callback
    mosquitto_message_callback_set(robot.mosq, mqtt_message_callback);
    
    // Connect to broker
    if (mosquitto_connect(robot.mosq, robot.mqtt_host, robot.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Unable to connect to MQTT broker at %s:%d\n", 
                robot.mqtt_host, robot.mqtt_port);
        return -1;
    }
    
    // Subscribe to command topics
    char topic[512];
    snprintf(topic, sizeof(topic), "%s%s", robot.base_topic, TOPIC_PICO_CMD);
    mosquitto_subscribe(robot.mosq, NULL, topic, 0);
    
    printf("Connected to MQTT broker at %s:%d\n", robot.mqtt_host, robot.mqtt_port);
    return 0;
}

// MQTT message callback
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, 
                          const struct mosquitto_message *message) {
    (void)mosq;
    (void)userdata;
    
    if (!message->payload) return;
    
    printf("Received message on topic %s: %.*s\n", 
           message->topic, message->payloadlen, (char*)message->payload);
    
    // Parse JSON message
    json_error_t error;
    json_t *root = json_loadb(message->payload, message->payloadlen, 0, &error);
    if (!root) {
        fprintf(stderr, "Error parsing JSON: %s\n", error.text);
        return;
    }
    
    // Check message type
    const char *type = json_string_value(json_object_get(root, "type"));
    if (!type) {
        json_decref(root);
        return;
    }
    
    pthread_mutex_lock(&robot.mutex);
    
    if (strcmp(type, "motor_command") == 0) {
        // Handle motor command
        json_t *left = json_object_get(root, "left_speed");
        json_t *right = json_object_get(root, "right_speed");
        
        if (json_is_number(left) && json_is_number(right)) {
            robot.left_speed = json_number_value(left);
            robot.right_speed = json_number_value(right);
            printf("Set motor speeds: left=%.1f, right=%.1f\n", 
                   robot.left_speed, robot.right_speed);
        }
    } else if (strcmp(type, "system_command") == 0) {
        // Handle system command
        const char *cmd = json_string_value(json_object_get(root, "command"));
        if (cmd) {
            if (strcmp(cmd, "emergency_stop") == 0) {
                robot.emergency_stop = 1;
                printf("Emergency stop activated\n");
            } else if (strcmp(cmd, "resume") == 0) {
                robot.emergency_stop = 0;
                printf("Resumed normal operation\n");
            } else if (strcmp(cmd, "relay_on") == 0) {
                robot.relay_state = 1;
                printf("Relay turned ON\n");
            } else if (strcmp(cmd, "relay_off") == 0) {
                robot.relay_state = 0;
                robot.left_speed = 0;
                robot.right_speed = 0;
                printf("Relay turned OFF, motors stopped\n");
            }
        }
    }
    
    // Release the mutex
    pthread_mutex_unlock(&robot.mutex);
    
    // Clean up JSON
    json_decref(root);
}

// Publish message to MQTT
void publish_mqtt(const char *topic_suffix, const char *payload) {
    if (!robot.mosq) return;
    
    char full_topic[512];
    snprintf(full_topic, sizeof(full_topic), "%s%s", robot.base_topic, topic_suffix);
    
    int rc = mosquitto_publish(robot.mosq, NULL, full_topic, strlen(payload), 
                              payload, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "MQTT publish error: %s\n", mosquitto_strerror(rc));
    }
}

// Publish heartbeat
void publish_heartbeat(struct mosquitto *mosq) {
    if (!mosq) {
        fprintf(stderr, "Error: MQTT client not initialized in publish_heartbeat\n");
        return;
    }
    
    // Get current timestamp
    time_t now = time(NULL);
    uint32_t timestamp = (uint32_t)now;
    static time_t start_time = 0;
    
    if (start_time == 0) {
        start_time = now;
    }
    
    // Publish PICO heartbeat with all required fields
    char pico_heartbeat[512];
    snprintf(pico_heartbeat, sizeof(pico_heartbeat),
        "{\"type\":\"pico_heartbeat\","
        "\"status\":\"running\","
        "\"timestamp\":%u"
        "}",
        timestamp
    );
    
    // Publish GPS heartbeat with all required fields
    char gps_heartbeat[512];
    snprintf(gps_heartbeat, sizeof(gps_heartbeat),
        "{\"type\":\"gps_heartbeat\","
        "\"status\":\"running\","
        "\"timestamp\":%u"
        "}",
        timestamp
    );
    
    // Publish using direct MQTT publish with QoS 1 (at least once delivery)
    int rc1 = mosquitto_publish(mosq, NULL, TOPIC_PICO_HEARTBEAT, 
                              strlen(pico_heartbeat), pico_heartbeat, 1, true);
    int rc2 = mosquitto_publish(mosq, NULL, TOPIC_GPS_HEARTBEAT, 
                              strlen(gps_heartbeat), gps_heartbeat, 1, true);
    
    if (rc1 == MOSQ_ERR_SUCCESS && rc2 == MOSQ_ERR_SUCCESS) {
        printf("Published heartbeat messages (PICO & GPS)\n");
    } else {
        fprintf(stderr, "Error publishing heartbeats: PICO=%s, GPS=%s\n",
                mosquitto_strerror(rc1), mosquitto_strerror(rc2));
    }
}

// Get current timestamp in milliseconds
uint32_t get_timestamp_ms(void) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    return (uint32_t)((tv.tv_sec * 1000) + (tv.tv_usec / 1000));
}

// Update robot position based on motor speeds and time elapsed
void update_robot_position(void) {
    static uint32_t last_update = 0;
    uint32_t now = get_timestamp_ms();
    
    if (last_update == 0) {
        last_update = now;
        return;
    }
    
    float dt = (now - last_update) / 1000.0f; // Convert to seconds
    if (dt <= 0) return;
    
    pthread_mutex_lock(&robot.mutex);
    
    // Apply emergency stop if active
    if (robot.emergency_stop || !robot.relay_state) {
        robot.left_speed = 0;
        robot.right_speed = 0;
    }
    
    // Calculate linear and angular velocity
    float v_left = robot.left_speed * 0.01f; // Convert to m/s
    float v_right = robot.right_speed * 0.01f;
    
    float v = (v_right + v_left) / 2.0f; // Linear velocity (m/s)
    float w = (v_right - v_left) / WHEEL_BASE; // Angular velocity (rad/s)
    
    // Update position and heading
    if (fabs(w) < 0.001f) {
        // Moving straight
        robot.x += v * dt * cos(robot.heading);
        robot.y += v * dt * sin(robot.heading);
    } else {
        // Moving in an arc
        float r = v / w; // Radius of the arc
        robot.x += -r * sin(robot.heading) + r * sin(robot.heading + w * dt);
        robot.y += r * cos(robot.heading) - r * cos(robot.heading + w * dt);
        robot.heading += w * dt;
    }
    
    // Normalize heading to 0-2π
    while (robot.heading < 0) robot.heading += 2 * PI;
    while (robot.heading >= 2 * PI) robot.heading -= 2 * PI;
    
    // Keep robot within bounds (0-100m x 0-80m)
    if (robot.x < 0) robot.x = 0;
    if (robot.x > 100) robot.x = 100;
    if (robot.y < 0) robot.y = 0;
    if (robot.y > 80) robot.y = 80;
    
    last_update = now;
    robot.timestamp_ms = now;
    
    pthread_mutex_unlock(&robot.mutex);
}

// Generate and publish PICO status
void publish_pico_status(void)
{
    // Create JSON objects
    json_t *root = json_object();
    json_t *system_obj = json_object();
    json_t *perf_obj = json_object();
    
    // Get current timestamp in seconds
    time_t now = time(NULL);
    
    // Add status fields
    json_object_set_new(root, "type", json_string("pico_status"));
    json_object_set_new(root, "timestamp", json_integer(now));
    
    // System status
    json_object_set_new(system_obj, "running", json_boolean(1));
    json_object_set_new(system_obj, "battery_voltage", json_real(24.5 + ((double)rand() / RAND_MAX * 0.5 - 0.25))); // 24.25-24.75V
    json_object_set_new(system_obj, "battery_percent", json_integer(80 + (rand() % 10))); // 80-90%
    json_object_set_new(system_obj, "cpu_temp", json_real(45.0 + ((double)rand() / RAND_MAX * 5.0))); // 45-50°C
    json_object_set_new(system_obj, "uptime_seconds", json_integer(now - start_time));
    json_object_set_new(system_obj, "communication_ok", json_boolean(1));
    json_object_set_new(system_obj, "relay_state", json_boolean(1));
    
    // Performance metrics
    json_object_set_new(perf_obj, "update_rate", json_real(10.0)); // 10Hz update rate
    json_object_set_new(perf_obj, "data_age", json_real(0.1 + ((double)rand() / RAND_MAX * 0.1))); // 0.1-0.2s
    
    // Add objects to root
    json_object_set_new(root, "system", system_obj);
    json_object_set_new(root, "performance", perf_obj);
    
    // Convert to JSON string
    char *json_str = json_dumps(root, JSON_INDENT(2));
    publish_mqtt(TOPIC_PICO_STATUS, json_str);
    
    // Cleanup
    free(json_str);
    json_decref(root);
}

// Generate and publish GPS status
void publish_gps_status(void)
{
    // Create JSON objects
    json_t *root = json_object();
    json_t *system_obj = json_object();
    json_t *perf_obj = json_object();
    
    // Get current timestamp in seconds
    time_t now = time(NULL);
    static time_t last_fix_time = 0;
    static double update_rate = 10.0;
    
    // Simulate GPS fix (95% chance)
    int has_fix = (rand() % 20) != 0;
    if (has_fix) last_fix_time = now;
    
    // Basic status
    json_object_set_new(root, "type", json_string("gps_status"));
    json_object_set_new(root, "timestamp", json_integer(now));
    
    // System status
    json_object_set_new(system_obj, "running", json_boolean(1));
    json_object_set_new(system_obj, "uart_connected", json_boolean(1));
    json_object_set_new(system_obj, "communication_ok", json_boolean(1));
    json_object_set_new(system_obj, "last_fix_time", json_integer(last_fix_time));
    json_object_set_new(system_obj, "module_type", json_string("u-blox NEO-M8N"));
    
    // Performance metrics
    update_rate = 9.5 + ((double)rand() / RAND_MAX * 1.0);
    double data_age = has_fix ? (0.1 + ((double)rand() / RAND_MAX * 0.2)) : 5.0;
    json_object_set_new(perf_obj, "update_rate", json_real(update_rate));
    json_object_set_new(perf_obj, "data_age", json_real(data_age));
    
    // Satellite info
    int sats_in_view = has_fix ? (8 + rand() % 6) : (3 + rand() % 4);
    int sats_in_use = has_fix ? (sats_in_view - (rand() % 3)) : 0;
    
    json_object_set_new(root, "satellites_in_view", json_integer(sats_in_view));
    json_object_set_new(root, "satellites_used", json_integer(sats_in_use));
    json_object_set_new(root, "hdop", json_real(has_fix ? (0.8 + (double)rand() / RAND_MAX * 0.5) : (5.0 + (double)rand() / RAND_MAX * 10.0)));
    
    // Add objects to root
    json_object_set_new(root, "system", system_obj);
    json_object_set_new(root, "performance", perf_obj);
    
    // Publish
    char *json_str = json_dumps(root, JSON_INDENT(2));
    publish_mqtt(TOPIC_GPS_STATUS, json_str);
    
    // Cleanup
    free(json_str);
    json_decref(root);
}

// Generate and publish PICO sensor data
void publish_pico_sensor_data(void)
{
    static uint32_t last_publish = 0;
    uint32_t now = get_timestamp_ms();
    
    // Publish at ~100Hz
    if (now - last_publish < 10) return;
    last_publish = now;
    
    // Get a consistent snapshot of the robot state
    pthread_mutex_lock(&robot.mutex);
    double x = robot.x;
    double y = robot.y;
    int emergency_stop = robot.emergency_stop;
    uint32_t timestamp = robot.timestamp_ms;
    pthread_mutex_unlock(&robot.mutex);
    
    // Generate simulated sensor data
    float imu[6] = {
        0.0f, 0.0f, 9.8f,  // Accelerometer (m/s²) - gravity on Z axis
        0.0f, 0.0f, 0.0f   // Gyroscope (rad/s) - no rotation by default
    };
    
    // Add some noise to IMU
    for (int i = 0; i < 6; i++) {
        imu[i] += ((float)rand() / RAND_MAX - 0.5f) * 0.1f;
    }
    
    // Generate ultrasonic sensor data (distance to obstacles in meters)
    float ultrasonic[3] = {
        (float)(80.0 - y) + ((float)rand() / RAND_MAX - 0.5f) * 0.1f,  // Left
        (float)(100.0 - x) + ((float)rand() / RAND_MAX - 0.5f) * 0.1f, // Front
        y + ((float)rand() / RAND_MAX - 0.5f) * 0.1f                   // Right
    };
    
    // Generate power data with realistic values
    float voltage = 25.0f + ((float)rand() / RAND_MAX - 0.5f) * 0.2f;  // Voltage (V)
    float current = 2.0f + ((float)rand() / RAND_MAX - 0.5f) * 0.1f;   // Current (A)
    
    // Calculate battery percentage based on voltage (simplified)
    float min_voltage = 20.0f;  // 5V per cell * 4 cells (20V total)
    float max_voltage = 25.2f;  // 6.3V per cell * 4 cells (25.2V total)
    float battery_percentage = ((voltage - min_voltage) / (max_voltage - min_voltage)) * 100.0f;
    battery_percentage = (battery_percentage < 0) ? 0 : (battery_percentage > 100) ? 100 : battery_percentage;
    
    // Determine battery state based on current
    const char* battery_state = "idle";
    if (current < -0.1f) {
        battery_state = "charging";
    } else if (current > 0.1f) {
        battery_state = "discharging";
    }
    
    // Check if battery is fully charged
    bool is_fully_charged = (battery_percentage > 98.0f && current < 0.1f);
    
    // Create JSON payload with complete battery information
    char payload[2048];
    snprintf(payload, sizeof(payload),
        "{"
        "\"type\":\"sensor_data\","
        "\"timestamp\":%u,"
        "\"imu\":[%.4f,%.4f,%.4f,%.4f,%.4f,%.4f],"
        "\"magnetometer\":[%.4f,%.4f,%.4f],"
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
            "\"rain_sensor\":false,"
            "\"bumper\":false,"
            "\"lift_sensor\":false"
        "}"
        "}",
        timestamp,
        imu[0], imu[1], imu[2], imu[3], imu[4], imu[5],  // IMU data
        0.0f, 0.0f, 0.0f,  // Magnetometer (not simulated)
        ultrasonic[0], ultrasonic[1], ultrasonic[2],  // Ultrasonic
        voltage, current, battery_percentage, battery_state,
        is_fully_charged ? "true" : "false",
        (voltage >= 25.0f) ? "true" : "false",  // voltage_ok
        (fabs(current) <= 0.1f) ? "true" : "false",  // current_ok
        0,  // stable_time_remaining (not implemented)
        emergency_stop ? "true" : "false"
    );
    
    // Publish sensor data
    publish_mqtt(TOPIC_PICO_SENSORS, payload);
}

// Generate and publish GPS data
void publish_gps_data(void) {
    static uint32_t last_publish = 0;
    uint32_t now = get_timestamp_ms();
    
    // Publish at ~1Hz
    if (now - last_publish < 1000) return;
    last_publish = now;
    
    // Get a consistent snapshot of the robot state
    pthread_mutex_lock(&robot.mutex);
    double x = robot.x;
    double y = robot.y;
    (void)x;  // Mark as used to suppress unused variable warning
    (void)y;  // Mark as used to suppress unused variable warning
    uint32_t timestamp = robot.timestamp_ms;
    pthread_mutex_unlock(&robot.mutex);
    
    // Convert robot position to GPS coordinates (simplified)
    // Base coordinates (example: somewhere in Europe)
    double base_lat = 45.4642;  // Base latitude
    double base_lon = 9.1900;   // Base longitude
    
    // Convert meters to degrees (approximate)
    // 1 degree of latitude ~= 111,111 meters
    // 1 degree of longitude ~= 111,111 * cos(latitude) meters
    double lat = base_lat + (y / 111111.0);
    double lon = base_lon + (x / (111111.0 * cos(base_lat * PI / 180.0)));
    
    // Add some realistic GPS noise (in degrees)
    lat += ((double)rand() / RAND_MAX - 0.5) * 0.00001;  // ~1m accuracy
    lon += ((double)rand() / RAND_MAX - 0.5) * 0.00001;
    
    // Create GPS fix message
    char payload[1024];
    snprintf(payload, sizeof(payload),
        "{"
        "\"type\":\"gps_fix\","
        "\"timestamp\":%u,"
        "\"fix\":{"
            "\"latitude\":%.8f,"
            "\"longitude\":%.8f,"
            "\"altitude\":120.5,"
            "\"speed\":0.0,"
            "\"heading\":0.0,"
            "\"satellites\":8,"
            "\"hdop\":1.2,"
            "\"fix_quality\":1,"
            "\"fix_type\":3"
        "},"
        "\"status\":{"
            "\"valid\":true,"
            "\"status\":\"A\","
            "\"mode\":\"A\""
        "}"
        "}",
        timestamp,
        lat,
        lon
    );
    
    // Publish GPS data
    publish_mqtt(TOPIC_GPS, payload);
}

// Publish odometry data (encoder ticks and motor RPM)
void publish_odometry_data(void) {
    char payload[512];
    
    // Simulate encoder ticks based on robot movement
    static uint32_t left_encoder = 0;
    static uint32_t right_encoder = 0;
    
    // Calculate encoder increments based on motor speeds and time
    // Assuming 1000 ticks per revolution and wheel radius from config
    double dt = 0.1; // 100ms update rate
    double wheel_circumference = 2 * PI * 0.1; // 0.1m wheel radius
    double ticks_per_meter = 1000.0 / wheel_circumference;
    
    // Calculate distance traveled by each wheel
    double left_speed_ms = robot.left_speed * 0.01; // Convert to m/s
    double right_speed_ms = robot.right_speed * 0.01;
    
    double left_distance = left_speed_ms * dt;
    double right_distance = right_speed_ms * dt;
    
    // Update encoder counts
    left_encoder += (uint32_t)(left_distance * ticks_per_meter);
    right_encoder += (uint32_t)(right_distance * ticks_per_meter);
    
    // Calculate RPM from motor speeds
    double left_rpm = robot.left_speed * 0.1; // Convert to RPM
    double right_rpm = robot.right_speed * 0.1;
    
    // Create odometry JSON payload
    snprintf(payload, sizeof(payload),
        "{"
        "\"type\":\"odometry_data\","
        "\"timestamp\":%u,"
        "\"encoders\":[%u,%u,0,0],"
        "\"motors\":{"
            "\"left_rpm\":%.2f,"
            "\"right_rpm\":%.2f"
        "}"
        "}",
        get_timestamp_ms(),
        left_encoder,
        right_encoder,
        left_rpm,
        right_rpm
    );
    
    // Publish odometry data
    publish_mqtt(TOPIC_PICO_ODOMETRY, payload);
}

// Main simulation loop
void* simulation_loop(void* arg) {
    (void)arg;
    
    printf("Starting simulation loop...\n");
    
    // Initialize random number generator and start time
    srand(time(NULL));
    start_time = time(NULL);
    
    // Timers for different publishing rates
    uint32_t last_pico_status = 0;
    uint32_t last_gps_status = 0;
    uint32_t last_heartbeat = 0;
    
    while (running) {
        uint32_t now = get_timestamp_ms();
        
        // Update robot position based on current motor speeds
        update_robot_position();
        
        // Publish sensor data (100Hz)
        publish_pico_sensor_data();
        
        // Publish GPS data (1Hz, handled internally)
        publish_gps_data();
        
        // Publish odometry data (100Hz)
        publish_odometry_data();
        
        // Publish PICO status every 5 seconds
        if (now - last_pico_status >= 5000) {
            publish_pico_status();
            last_pico_status = now;
        }
        
        // Publish GPS status every 10 seconds
        if (now - last_gps_status >= 10000) {
            publish_gps_status();
            last_gps_status = now;
        }
        
        // Publish heartbeat messages every 5 seconds
        if (now - last_heartbeat >= 5000) {
            publish_heartbeat(robot.mosq);
            last_heartbeat = now;
        }
        
        // Process MQTT events
        int rc = mosquitto_loop(robot.mosq, 0, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "MQTT loop error: %s\n", mosquitto_strerror(rc));
            // Attempt to reconnect
            if (mosquitto_reconnect(robot.mosq) != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "Failed to reconnect to MQTT broker\n");
                sleep(5); // Wait before retrying
            }
        }
        
        // Small delay to prevent 100% CPU usage (1ms)
        const struct timespec sleep_time = {0, 1000000};  // 1ms = 1,000,000ns
        nanosleep(&sleep_time, NULL);
    }
    
    return NULL;
}
