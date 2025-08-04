/**
 * Pico Bridge - MQTT Bridge for Smart Mower Pico Firmware
 * 
 * Connects Raspberry Pi Pico firmware to MQTT broker
 * - Receives sensor data from Pico via UART
 * - Publishes data to MQTT topics
 * - Receives MQTT commands and sends to Pico
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
#include "pico_mqtt.h"  // Include our MQTT definitions

#ifndef CRTSCTS
#define CRTSCTS 020000000000
#endif

#define DEFAULT_UART_DEVICE "/dev/ttyUSB0"
#define DEFAULT_BAUDRATE B921600
#define BUFFER_SIZE 2048
#define MAX_TOPIC_LEN 512
#define MAX_PAYLOAD_LEN 1024

#pragma pack(push, 1)  // Allineamento a 1 byte per compatibilità

// Struttura dati sensori (dal Pico al bridge)
typedef struct {
    uint8_t type;           // MSG_SENSOR_DATA
    uint32_t timestamp;     // timestamp in ms
    float imu[6];          // accelerometro + giroscopio [ax,ay,az,gx,gy,gz]
    float magnetometer[3]; // magnetometro [mx,my,mz]
    float ultrasonic[3];   // 3 sensori ultrasuoni [left,center,right]
    float power[2];        // voltaggio, corrente [V,A]
    uint8_t flags[4];      // emergency_stop, rain, bumper, lift
} sensor_data_t;

// Struttura report di stato (dal Pico al bridge)
typedef struct {
    uint8_t type;          // MSG_STATUS_REPORT
    uint32_t timestamp;    // timestamp in ms
    float motor_speeds[4]; // velocità attuali [left,right,blade1,blade2]
    float motor_rpm[4];    // RPM attuali [left,right,blade1,blade2]
    uint32_t encoder_counts[4]; // contatori encoder
    uint8_t system_flags;  // flags di sistema
    uint8_t relay_state;   // stato relay
} status_report_t;

// Comando motori (dal bridge al Pico)
typedef struct {
    uint8_t type;          // MSG_MOTOR_CMD
    float left_speed;      // velocità motore sinistro (-100 a +100)
    float right_speed;     // velocità motore destro (-100 a +100)
    float blade1_speed;    // velocità lama 1 (0 a 100)
    float blade2_speed;    // velocità lama 2 (0 a 100)
} motor_command_t;

// Comando sistema (dal bridge al Pico)
typedef struct {
    uint8_t type;          // MSG_SYSTEM_CMD
    uint8_t command_id;    // SYS_CMD_*
    float value;           // valore opzionale
} system_command_t;

#pragma pack(pop)

// Configuration structure
typedef struct {
    // Pico UART configuration
    char device[256];
    int baudrate;
    int timeout_ms;
    
    // MQTT configuration
    char mqtt_broker[256];
    int mqtt_port;
    char mqtt_username[256];
    char mqtt_password[256];
    char base_topic[256];
    char client_id[256];
    
    // Logging configuration
    char log_level[32];
    char log_file[256];
} config_t;

// Global variables
static volatile int keep_running = 1;
static struct mosquitto *mosq = NULL;
static int uart_fd = -1;
static config_t config;
static time_t last_status_time = 0;

// Function prototypes
void signal_handler(int signal);
int load_config(const char *filename, config_t *config);
speed_t get_baudrate_constant(int baudrate);
int init_uart(const char *device, int baudrate);
int init_mqtt(config_t *config);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);
void process_binary_data(const uint8_t *data, size_t length);
void sensor_data_to_json(const sensor_data_t *data, char *json_buffer, size_t buffer_size);
void status_report_to_json(const status_report_t *data, char *json_buffer, size_t buffer_size);
void send_motor_command(float left, float right, float blade1, float blade2);
void send_system_command(uint8_t cmd_id, float value);
void *uart_reader_thread(void *arg);
void publish_periodic_status(void);
void publish_bridge_heartbeat(void);
void publish_default_status(void);

int main(int argc, char *argv[]) {
    const char *config_file = "pico_bridge_config.json";
    
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

    printf("Smart Mower Pico Bridge starting...\n");

    // Initialize UART
    uart_fd = init_uart(config.device, config.baudrate);
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
    pthread_t uart_thread;
    if (pthread_create(&uart_thread, NULL, uart_reader_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create UART thread\n");
        return 1;
    }

    printf("Pico Bridge running. Press Ctrl+C to stop.\n");

    // Main loop
    while (keep_running) {
        mosquitto_loop(mosq, 100, 1);
        publish_periodic_status();
    }

    // Cleanup
    pthread_join(uart_thread, NULL);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    close(uart_fd);
    
    printf("Pico Bridge stopped.\n");
    return 0;
}

void signal_handler(int sig) {
    (void)sig; // Suppress unused parameter warning
    keep_running = 0;
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

    speed_t baud_const = get_baudrate_constant(baudrate);
    cfsetospeed(&tty, baud_const);
    cfsetispeed(&tty, baud_const);

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

int init_mqtt(config_t *config) {
    mosquitto_lib_init();
    
    mosq = mosquitto_new(config->client_id, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error creating MQTT client\n");
        return -1;
    }

    mosquitto_message_callback_set(mosq, mqtt_message_callback);
    
    // Set username and password if provided
    if (strlen(config->mqtt_username) > 0) {
        mosquitto_username_pw_set(mosq, config->mqtt_username, config->mqtt_password);
    }

    if (mosquitto_connect(mosq, config->mqtt_broker, config->mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Unable to connect to MQTT broker\n");
        return -1;
    }

    // Subscribe to command topics
    char topic[MAX_TOPIC_LEN];
    snprintf(topic, sizeof(topic), "%s/cmd/+", config->base_topic);
    mosquitto_subscribe(mosq, NULL, topic, 0);

    return 0;
}

void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    (void)mosq; // Suppress unused parameter warning
    (void)userdata; // Suppress unused parameter warning
    if (message->payloadlen) {
        char *topic = message->topic;
        char *payload = (char*)message->payload;
        
        // Parse JSON payload
        json_error_t error;
        json_t *root = json_loads(payload, 0, &error);
        if (!root) {
            fprintf(stderr, "JSON parse error in MQTT message: %s\n", error.text);
            return;
        }
        
        // Handle motor commands
        if (strstr(topic, PICO_TOPIC_CMD_MOTORS)) {
            json_t *left = json_object_get(root, "left_speed");
            json_t *right = json_object_get(root, "right_speed");
            json_t *blade1 = json_object_get(root, "blade1_speed");
            json_t *blade2 = json_object_get(root, "blade2_speed");
            
            if (json_is_number(left) && json_is_number(right) && 
                json_is_number(blade1) && json_is_number(blade2)) {
                send_motor_command(
                    json_number_value(left),
                    json_number_value(right),
                    json_number_value(blade1),
                    json_number_value(blade2)
                );
            }
        }
        // Handle system commands
        else if (strstr(topic, PICO_TOPIC_CMD_SYSTEM)) {
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

void process_pico_data(const char *json_data) {
    json_error_t error;
    json_t *root = json_loads(json_data, 0, &error);
    
    if (!root) {
        fprintf(stderr, "JSON parse error: %s\n", error.text);
        return;
    }

    json_t *type = json_object_get(root, "type");
    if (!json_is_string(type)) {
        json_decref(root);
        return;
    }

    const char *msg_type = json_string_value(type);
    char topic[MAX_TOPIC_LEN];
    
    // Publish based on message type
    if (strcmp(msg_type, "sensor_data") == 0) {
        snprintf(topic, sizeof(topic), "%s/sensors", config.base_topic);
        mosquitto_publish(mosq, NULL, topic, strlen(json_data), json_data, 0, false);
    }
    else if (strcmp(msg_type, "status_report") == 0) {
        snprintf(topic, sizeof(topic), "%s/status", config.base_topic);
        mosquitto_publish(mosq, NULL, topic, strlen(json_data), json_data, 0, false);
    }

    json_decref(root);
}

void send_command_to_pico(const char *command) {
    if (uart_fd >= 0) {
        write(uart_fd, command, strlen(command));
        write(uart_fd, "\n", 1);
    }
}

void *uart_reader_thread(void *arg) {
    (void)arg; // Suppress unused parameter warning
    uint8_t buffer[BUFFER_SIZE];
    
    while (keep_running) {
        int bytes_read = read(uart_fd, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            process_binary_data(buffer, bytes_read);
        }
        usleep(1000); // 1ms delay
    }
    
    return NULL;
}

void publish_periodic_status(void) {
    time_t current_time = time(NULL);
    
    // Publish bridge heartbeat every 5 seconds
    if (current_time - last_status_time >= 5) {
        publish_bridge_heartbeat();
        last_status_time = current_time;
    }
}

void publish_bridge_heartbeat(void) {
    char topic[MAX_TOPIC_LEN];
    char payload[MAX_PAYLOAD_LEN];
    
    // Publish only bridge system status (heartbeat)
    snprintf(payload, sizeof(payload), 
        "{"
        "\"type\":\"bridge_heartbeat\","
        "\"timestamp\":%ld,"
        "\"system\":{"
            "\"running\":true,"
            "\"communication_ok\":true,"
            "\"relay_state\":false"
        "}"
        "}",
        (long)time(NULL)
    );
    
    snprintf(topic, sizeof(topic), "%s/bridge/status", config.base_topic);
    mosquitto_publish(mosq, NULL, topic, strlen(payload), payload, 0, false);
}

int load_config(const char *filename, config_t *config) {
    FILE *file = fopen(filename, "r");
    if (!file) {
        fprintf(stderr, "Cannot open config file: %s\n", filename);
        return -1;
    }
    
    // Read file content
    fseek(file, 0, SEEK_END);
    long length = ftell(file);
    fseek(file, 0, SEEK_SET);
    
    char *content = malloc(length + 1);
    if (!content) {
        fclose(file);
        return -1;
    }
    
    fread(content, 1, length, file);
    content[length] = '\0';
    fclose(file);
    
    // Parse JSON
    json_error_t error;
    json_t *root = json_loads(content, 0, &error);
    free(content);
    
    if (!root) {
        fprintf(stderr, "JSON parse error: %s\n", error.text);
        return -1;
    }
    
    // Parse pico section
    json_t *pico = json_object_get(root, "pico");
    if (pico) {
        json_t *device = json_object_get(pico, "device");
        if (json_is_string(device)) {
            strncpy(config->device, json_string_value(device), sizeof(config->device) - 1);
        }
        
        json_t *baudrate = json_object_get(pico, "baudrate");
        if (json_is_integer(baudrate)) {
            config->baudrate = json_integer_value(baudrate);
        }
        
        json_t *timeout = json_object_get(pico, "timeout_ms");
        if (json_is_integer(timeout)) {
            config->timeout_ms = json_integer_value(timeout);
        }
    }
    
    // Parse mqtt section
    json_t *mqtt = json_object_get(root, "mqtt");
    if (mqtt) {
        json_t *broker = json_object_get(mqtt, "broker");
        if (json_is_string(broker)) {
            strncpy(config->mqtt_broker, json_string_value(broker), sizeof(config->mqtt_broker) - 1);
        }
        
        json_t *port = json_object_get(mqtt, "port");
        if (json_is_integer(port)) {
            config->mqtt_port = json_integer_value(port);
        }
        
        json_t *username = json_object_get(mqtt, "username");
        if (json_is_string(username)) {
            strncpy(config->mqtt_username, json_string_value(username), sizeof(config->mqtt_username) - 1);
        }
        
        json_t *password = json_object_get(mqtt, "password");
        if (json_is_string(password)) {
            strncpy(config->mqtt_password, json_string_value(password), sizeof(config->mqtt_password) - 1);
        }
        
        json_t *base_topic = json_object_get(mqtt, "base_topic");
        if (json_is_string(base_topic)) {
            strncpy(config->base_topic, json_string_value(base_topic), sizeof(config->base_topic) - 1);
        }
        
        json_t *client_id = json_object_get(mqtt, "client_id");
        if (json_is_string(client_id)) {
            strncpy(config->client_id, json_string_value(client_id), sizeof(config->client_id) - 1);
        }
    }
    
    // Parse logging section
    json_t *logging = json_object_get(root, "logging");
    if (logging) {
        json_t *level = json_object_get(logging, "level");
        if (json_is_string(level)) {
            strncpy(config->log_level, json_string_value(level), sizeof(config->log_level) - 1);
        }
        
        json_t *log_file = json_object_get(logging, "file");
        if (json_is_string(log_file)) {
            strncpy(config->log_file, json_string_value(log_file), sizeof(config->log_file) - 1);
        }
    }
    
    json_decref(root);
    
    printf("Configuration loaded from %s\n", filename);
    printf("UART: %s @ %d baud\n", config->device, config->baudrate);
    printf("MQTT: %s:%d, topic: %s\n", config->mqtt_broker, config->mqtt_port, config->base_topic);
    
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
        case 500000: return B500000;
        case 576000: return B576000;
        case 921600: return B921600;
        case 1000000: return B1000000;
        case 1152000: return B1152000;
        case 1500000: return B1500000;
        case 2000000: return B2000000;
        case 2500000: return B2500000;
        case 3000000: return B3000000;
        case 3500000: return B3500000;
        case 4000000: return B4000000;
        default:
            fprintf(stderr, "Unsupported baudrate: %d, using 115200\n", baudrate);
            return B115200;
    }
}

void process_binary_data(const uint8_t *data, size_t length) {
    if (length < 1) return;
    
    uint8_t msg_type = data[0];
    
    switch (msg_type) {
        case PICO_MSG_SENSOR_DATA:
            if (length >= sizeof(sensor_data_t)) {
                const sensor_data_t *sensor_data = (const sensor_data_t*)data;
                char json_buffer[MAX_PAYLOAD_LEN];
                sensor_data_to_json(sensor_data, json_buffer, sizeof(json_buffer));
                
                char topic[MAX_TOPIC_LEN];
                snprintf(topic, sizeof(topic), "%s%s", config.base_topic, PICO_TOPIC_SENSORS);
                mosquitto_publish(mosq, NULL, topic, strlen(json_buffer), json_buffer, 0, false);
            }
            break;
            
        case PICO_MSG_STATUS_REPORT:
            if (length >= sizeof(status_report_t)) {
                const status_report_t *status_data = (const status_report_t*)data;
                char json_buffer[MAX_PAYLOAD_LEN];
                status_report_to_json(status_data, json_buffer, sizeof(json_buffer));
                
                char topic[MAX_TOPIC_LEN];
                snprintf(topic, sizeof(topic), "%s%s", config.base_topic, PICO_TOPIC_STATUS);
                mosquitto_publish(mosq, NULL, topic, strlen(json_buffer), json_buffer, 0, false);
            }
            break;
            
        default:
            printf("Unknown message type: 0x%02X\n", msg_type);
            break;
    }
}

void sensor_data_to_json(const sensor_data_t *data, char *json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
        "{"
        "\"type\":\"sensor_data\","
        "\"timestamp\":%u,"
        "\"imu\":[%.3f,%.3f,%.3f,%.3f,%.3f,%.3f],"
        "\"magnetometer\":[%.3f,%.3f,%.3f],"
        "\"ultrasonic\":[%.3f,%.3f,%.3f],"
        "\"power\":[%.3f,%.3f],"
        "\"safety\":{"
            "\"emergency_stop\":%s,"
            "\"rain_sensor\":%s,"
            "\"bumper\":%s,"
            "\"lift_sensor\":%s"
        "}"
        "}",
        data->timestamp,
        data->imu[0], data->imu[1], data->imu[2], data->imu[3], data->imu[4], data->imu[5],
        data->magnetometer[0], data->magnetometer[1], data->magnetometer[2],
        data->ultrasonic[0], data->ultrasonic[1], data->ultrasonic[2],
        data->power[0], data->power[1],
        (data->flags[0] ? "true" : "false"),
        (data->flags[1] ? "true" : "false"),
        (data->flags[2] ? "true" : "false"),
        (data->flags[3] ? "true" : "false")
    );
}

void status_report_to_json(const status_report_t *data, char *json_buffer, size_t buffer_size) {
    snprintf(json_buffer, buffer_size,
        "{"
        "\"type\":\"status_report\","
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
        data->timestamp,
        data->motor_speeds[0], data->motor_speeds[1], data->motor_speeds[2], data->motor_speeds[3],
        data->motor_rpm[0], data->motor_rpm[1], data->motor_rpm[2], data->motor_rpm[3],
        data->encoder_counts[0], data->encoder_counts[1], data->encoder_counts[2], data->encoder_counts[3],
        (data->relay_state ? "true" : "false")
    );
}

void send_motor_command(float left, float right, float blade1, float blade2) {
    motor_command_t cmd = {
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
    system_command_t cmd = {
        .type = PICO_MSG_SYSTEM_CMD,
        .command_id = cmd_id,
        .value = value
    };
    
    if (uart_fd >= 0) {
        write(uart_fd, &cmd, sizeof(cmd));
        printf("Sent system command: ID=%d Value=%.1f\n", cmd_id, value);
    }
}
