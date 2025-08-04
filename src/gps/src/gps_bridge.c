/**
 * GPS Bridge - Connects a physical GPS device to MQTT (REFACTORED)
 * 
 * This program reads NMEA data from a serial GPS device and publishes
 * the parsed data to an MQTT broker.
 * 
 * This version uses the gps_mqtt.h header for all MQTT definitions
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <errno.h>
#include <jansson.h>
#include <mosquitto.h>
#include <sys/stat.h>
#include <pthread.h>
#include <sys/time.h>
#include <math.h>
#include "gps_mqtt.h"  // Include our MQTT definitions

// Default configuration values
#define DEFAULT_CONFIG_FILE "/opt/smartmower/etc/config/robot_config.json"
#define DEFAULT_BAUDRATE B9600
#define DEFAULT_BUFFER_SIZE 1024

// Configuration structure
typedef struct {
    // GPS device configuration
    char gps_device[256];
    int baudrate;
    int timeout;
    
    // MQTT configuration
    char mqtt_host[256];
    int mqtt_port;
    char mqtt_username[256];
    char mqtt_password[256];
    char base_topic[256];
    int keepalive;
    
    // Logging configuration
    int log_level;
    char log_file[256];
} config_t;

// Structure for satellite information
typedef struct {
    int prn;              // Satellite PRN number
    int elevation;        // Elevation in degrees (0-90)
    int azimuth;          // Azimuth in degrees (0-359)
    int snr;              // Signal to noise ratio (0-99 dB)
    int used;             // 1 if used in position fix
} satellite_info_t;

// Structure for constellation data
typedef struct {
    char type[16];        // Constellation type ("GPS", "GLONASS", etc.)
    int num_satellites;   // Number of satellites in view
    int num_used;         // Number of satellites used in fix
    satellite_info_t sats[24];  // Satellite details (max 24 per constellation)
} constellation_data_t;

// GPS data structure
typedef struct {
    // Position and movement
    double latitude;      // Decimal degrees (positive = N, negative = S)
    double longitude;     // Decimal degrees (positive = E, negative = W)
    double altitude;      // Altitude in meters above sea level
    double speed;         // Speed in km/h
    double course;        // Course over ground in degrees (0-359)
    
    // Fix information
    int fix_quality;      // GPS fix quality (0=invalid, 1=GPS, 2=DGPS, etc.)
    int fix_type;         // Fix type (1=no fix, 2=2D, 3=3D)
    char fix_mode;        // Fix mode ('A'=automatic, 'M'=manual)
    int satellites_used;  // Number of satellites used in fix
    int satellites_view;  // Number of satellites in view
    
    // Precision indicators
    double pdop;          // Position dilution of precision
    double hdop;          // Horizontal dilution of precision
    double vdop;          // Vertical dilution of precision
    
    // Time information
    time_t timestamp;     // Unix timestamp
    char time_str[16];    // Time string (HHMMSS.sss)
    char date_str[16];    // Date string (DDMMYY)
    
    // Status flags
    int is_valid;         // 1 if GPS data is valid
    int has_rtk;          // 1 if RTK correction available
    int rtk_quality;      // RTK quality indicator
    
    // SBAS information
    char sbas_type[16];   // SBAS type
    int sbas_prn;         // SBAS PRN
    
    // Constellation data
    constellation_data_t gps;
    constellation_data_t glonass;
    constellation_data_t galileo;
    constellation_data_t beidou;
    constellation_data_t qzss;
    constellation_data_t sbas;
} gps_data_t;

// Global variables
static int gps_fd = -1;
static struct mosquitto *mosq = NULL;
static config_t config;
static volatile int running = 1;
static gps_data_t current_gps_data;
static pthread_mutex_t data_mutex = PTHREAD_MUTEX_INITIALIZER;

// Function prototypes
int load_config(const char *filename, config_t *config);
int init_serial(const char *device, int baudrate);
int init_mqtt(config_t *config);
void signal_handler(int signal);
void publish_gps_data(void);
void publish_gps_status(void);
void publish_gps_heartbeat(void);
void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message);
int parse_nmea(const char *sentence);
void *gps_reader_thread(void *arg);
void init_gps_data(void);

int main(int argc, char *argv[]) {
    const char *config_file = DEFAULT_CONFIG_FILE;
    
    // Parse command line arguments
    if (argc > 1) {
        config_file = argv[1];
    }
    
    // Initialize GPS data structure
    init_gps_data();
    
    // Load configuration from JSON file
    if (load_config(config_file, &config) != 0) {
        fprintf(stderr, "Failed to load configuration from %s\n", config_file);
        return 1;
    }
    
    // Setup signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("Starting GPS Bridge with config: %s\n", config_file);
    printf("GPS: %s @ %d baud\n", config.gps_device, config.baudrate);
    printf("MQTT: %s:%d, topic: %s\n", config.mqtt_host, config.mqtt_port, config.base_topic);
    
    // Initialize GPS serial port
    gps_fd = init_serial(config.gps_device, config.baudrate);
    if (gps_fd < 0) {
        fprintf(stderr, "Failed to initialize GPS serial port\n");
        return 1;
    }
    
    // Initialize MQTT
    if (init_mqtt(&config) != 0) {
        fprintf(stderr, "Failed to initialize MQTT\n");
        close(gps_fd);
        return 1;
    }
    
    // Start GPS reader thread
    pthread_t gps_thread;
    if (pthread_create(&gps_thread, NULL, gps_reader_thread, NULL) != 0) {
        fprintf(stderr, "Failed to create GPS reader thread\n");
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        close(gps_fd);
        return 1;
    }
    
    // Main loop
    time_t last_heartbeat = 0;
    time_t last_data_publish = 0;
    
    while (running) {
        mosquitto_loop(mosq, 1000, 1);
        
        time_t now = time(NULL);
        
        // Publish heartbeat using header constant
        if (now - last_heartbeat >= GPS_HEARTBEAT_INTERVAL_SEC) {
            publish_gps_heartbeat();
            last_heartbeat = now;
        }
        
        // Publish GPS data periodically
        if (now - last_data_publish >= 1) {  // Every second
            publish_gps_data();
            publish_gps_status();
            last_data_publish = now;
        }
        
        sleep(1);
    }
    
    // Cleanup
    running = 0;
    pthread_join(gps_thread, NULL);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    close(gps_fd);
    
    printf("GPS Bridge stopped\n");
    return 0;
}

void init_gps_data(void) {
    memset(&current_gps_data, 0, sizeof(current_gps_data));
    current_gps_data.fix_quality = 0;
    current_gps_data.fix_type = 1;  // No fix
    current_gps_data.fix_mode = 'A';
    current_gps_data.pdop = 99.99;
    current_gps_data.hdop = 99.99;
    current_gps_data.vdop = 99.99;
    strcpy(current_gps_data.sbas_type, "NONE");
}

// Configuration loading from centralized robot_config.json
int load_config(const char *filename, config_t *config) {
    // Set defaults
    strcpy(config->base_topic, GPS_MQTT_BASE_TOPIC);
    strcpy(config->gps_device, "/dev/ttyAMA2");
    config->baudrate = 115200;
    config->timeout = 1000;
    strcpy(config->mqtt_host, "localhost");
    config->mqtt_port = 1883;
    config->keepalive = 60;
    config->log_level = 1;
    strcpy(config->log_file, "gps_bridge.log");
    // Initialize MQTT credentials to empty
    config->mqtt_username[0] = '\0';
    config->mqtt_password[0] = '\0';
    
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
    
    // Parse GPS configuration from centralized configuration (root level)
    json_t *gps_config = json_object_get(root, "gps_config");
    if (gps_config) {
        json_t *device = json_object_get(gps_config, "uart_device");
        if (device && json_is_string(device)) {
            strcpy(config->gps_device, json_string_value(device));
        }
        
        json_t *baudrate = json_object_get(gps_config, "baudrate");
        if (baudrate && json_is_integer(baudrate)) {
            config->baudrate = json_integer_value(baudrate);
        }
        
        json_t *timeout = json_object_get(gps_config, "uart_timeout_ms");
        if (timeout && json_is_integer(timeout)) {
            config->timeout = json_integer_value(timeout);
        }
        
        json_t *protocol = json_object_get(gps_config, "protocol");
        if (protocol && json_is_string(protocol)) {
            printf("GPS Protocol: %s\n", json_string_value(protocol));
        }
        
        json_t *max_satellites = json_object_get(gps_config, "max_satellites");
        if (max_satellites && json_is_integer(max_satellites)) {
            printf("GPS Max Satellites: %lld\n", (long long)json_integer_value(max_satellites));
        }
    }
    
    // Parse GPS logging configuration from root level
    json_t *gps_logging = json_object_get(root, "gps_logging");
    if (gps_logging) {
        json_t *level = json_object_get(gps_logging, "level");
        if (level && json_is_string(level)) {
            if (strcmp(json_string_value(level), "debug") == 0) {
                config->log_level = 2;
            } else if (strcmp(json_string_value(level), "info") == 0) {
                config->log_level = 1;
            } else {
                config->log_level = 0;
            }
        }
        
        json_t *file = json_object_get(gps_logging, "file");
        if (file && json_is_string(file)) {
            strcpy(config->log_file, json_string_value(file));
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
    
    // Parse MQTT credentials from system.communication section
    json_t *system = json_object_get(root, "system");
    if (system) {
        json_t *communication = json_object_get(system, "communication");
        if (communication) {
            json_t *mqtt_host = json_object_get(communication, "mqtt_broker_host");
            if (mqtt_host && json_is_string(mqtt_host)) {
                strcpy(config->mqtt_host, json_string_value(mqtt_host));
            }
            
            json_t *mqtt_port = json_object_get(communication, "mqtt_broker_port");
            if (mqtt_port && json_is_integer(mqtt_port)) {
                config->mqtt_port = json_integer_value(mqtt_port);
            }
            
            json_t *mqtt_username = json_object_get(communication, "mqtt_username");
            if (mqtt_username && json_is_string(mqtt_username)) {
                strcpy(config->mqtt_username, json_string_value(mqtt_username));
            }
            
            json_t *mqtt_password = json_object_get(communication, "mqtt_password");
            if (mqtt_password && json_is_string(mqtt_password)) {
                strcpy(config->mqtt_password, json_string_value(mqtt_password));
            }
        }
    }
    
    printf("GPS Config loaded: %s @ %d baud, timeout %dms\n", 
           config->gps_device, config->baudrate, config->timeout);
    printf("GPS Logging: level %d, file %s\n", 
           config->log_level, config->log_file);
    printf("MQTT: %s:%d, username: %s\n", 
           config->mqtt_host, config->mqtt_port, 
           strlen(config->mqtt_username) > 0 ? config->mqtt_username : "(none)");
    
    json_decref(root);
    fclose(file);
    return 0;
}

int init_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_SYNC);
    if (fd < 0) {
        perror("Error opening GPS device");
        return -1;
    }
    
    struct termios tty;
    if (tcgetattr(fd, &tty) != 0) {
        perror("Error getting GPS attributes");
        close(fd);
        return -1;
    }
    
    // Configure serial port for GPS (8N1, no flow control)
    cfsetospeed(&tty, baudrate);
    cfsetispeed(&tty, baudrate);
    
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
        perror("Error setting GPS attributes");
        close(fd);
        return -1;
    }
    
    return fd;
}

int init_mqtt(config_t *config) {
    mosquitto_lib_init();
    
    mosq = mosquitto_new(GPS_MQTT_CLIENT_ID, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Failed to create mosquitto client\n");
        return -1;
    }
    
    // Set username and password if provided
    if (strlen(config->mqtt_username) > 0) {
        mosquitto_username_pw_set(mosq, config->mqtt_username, config->mqtt_password);
    }
    
    mosquitto_message_callback_set(mosq, mqtt_message_callback);
    
    if (mosquitto_connect(mosq, config->mqtt_host, config->mqtt_port, config->keepalive) != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Failed to connect to MQTT broker\n");
        return -1;
    }
    
    // Subscribe to command topics using header definitions
    char topic[512];
    snprintf(topic, sizeof(topic), "%s%s", config->base_topic, GPS_TOPIC_CMD);
    mosquitto_subscribe(mosq, NULL, topic, 0);
    
    printf("MQTT connected and subscribed to command topics\n");
    return 0;
}

void mqtt_message_callback(struct mosquitto *mosq, void *userdata, const struct mosquitto_message *message) {
    (void)mosq;
    (void)userdata;
    
    if (message->payloadlen == 0) return;
    
    char *topic = message->topic;
    char *payload = (char*)message->payload;
    
    printf("Received MQTT message on topic: %s\n", topic);
    
    if (payload && strstr(topic, GPS_TOPIC_CMD)) {
        json_error_t error;
        json_t *root = json_loads(payload, 0, &error);
        if (!root) {
            fprintf(stderr, "JSON parse error in MQTT message: %s\n", error.text);
            return;
        }
        
        json_t *command = json_object_get(root, "command");
        if (json_is_string(command)) {
            const char *cmd_str = json_string_value(command);
            printf("GPS command received: %s\n", cmd_str);
            
            // Handle GPS commands
            if (strcmp(cmd_str, "reset") == 0) {
                printf("GPS reset command received\n");
                // Implement GPS reset logic
            } else if (strcmp(cmd_str, "cold_start") == 0) {
                printf("GPS cold start command received\n");
                // Implement GPS cold start logic
            } else if (strcmp(cmd_str, "warm_start") == 0) {
                printf("GPS warm start command received\n");
                // Implement GPS warm start logic
            }
        }
        
        json_decref(root);
    }
}

void publish_gps_data(void) {
    pthread_mutex_lock(&data_mutex);
    
    json_t *root = json_object();
    if (!root) {
        pthread_mutex_unlock(&data_mutex);
        return;
    }
    
    // Use JSON message type from header
    json_object_set_new(root, "type", json_string(GPS_JSON_DATA));
    json_object_set_new(root, "timestamp", json_integer(current_gps_data.timestamp));
    json_object_set_new(root, "valid", json_boolean(current_gps_data.is_valid));
    
    // Position data
    json_t *position = json_object();
    json_object_set_new(position, "latitude", json_real(current_gps_data.latitude));
    json_object_set_new(position, "longitude", json_real(current_gps_data.longitude));
    json_object_set_new(position, "altitude", json_real(current_gps_data.altitude));
    json_object_set_new(position, "speed", json_real(current_gps_data.speed));
    json_object_set_new(position, "course", json_real(current_gps_data.course));
    json_object_set_new(root, "position", position);
    
    // Fix information
    json_t *fix = json_object();
    json_object_set_new(fix, "quality", json_integer(current_gps_data.fix_quality));
    json_object_set_new(fix, "type", json_integer(current_gps_data.fix_type));
    char mode_str[2] = {current_gps_data.fix_mode, '\0'};
    json_object_set_new(fix, "mode", json_string(mode_str));
    json_object_set_new(fix, "satellites_used", json_integer(current_gps_data.satellites_used));
    json_object_set_new(fix, "satellites_view", json_integer(current_gps_data.satellites_view));
    json_object_set_new(root, "fix", fix);
    
    // Precision data
    json_t *precision = json_object();
    json_object_set_new(precision, "pdop", json_real(current_gps_data.pdop));
    json_object_set_new(precision, "hdop", json_real(current_gps_data.hdop));
    json_object_set_new(precision, "vdop", json_real(current_gps_data.vdop));
    json_object_set_new(root, "precision", precision);
    
    // Time information
    json_t *time_info = json_object();
    json_object_set_new(time_info, "time", json_string(current_gps_data.time_str));
    json_object_set_new(time_info, "date", json_string(current_gps_data.date_str));
    json_object_set_new(root, "time", time_info);
    
    pthread_mutex_unlock(&data_mutex);
    
    // Publish using header topic constant
    char *json_str = json_dumps(root, JSON_COMPACT);
    if (json_str) {
        char topic[512];
        snprintf(topic, sizeof(topic), "%s%s", config.base_topic, GPS_TOPIC_DATA);
        
        int rc = mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Error publishing GPS data: %s\n", mosquitto_strerror(rc));
        }
        
        free(json_str);
    }
    
    json_decref(root);
}

void publish_gps_status(void) {
    json_t *root = json_object();
    if (!root) return;
    
    // Use JSON message type from header
    json_object_set_new(root, "type", json_string(GPS_JSON_STATUS));
    json_object_set_new(root, "timestamp", json_integer(time(NULL)));
    
    // System status
    json_t *system = json_object();
    json_object_set_new(system, "running", json_boolean(true));
    json_object_set_new(system, "uart_connected", json_boolean(gps_fd >= 0));
    json_object_set_new(system, "communication_ok", json_boolean(gps_fd >= 0));
    json_object_set_new(system, "last_fix_time", json_integer(current_gps_data.timestamp));
    json_object_set_new(system, "module_type", json_string("Generic NMEA"));
    json_object_set_new(root, "system", system);
    
    // Performance data
    json_t *performance = json_object();
    json_object_set_new(performance, "update_rate", json_real(1.0));  // 1 Hz
    json_object_set_new(performance, "data_age", json_real(time(NULL) - current_gps_data.timestamp));
    json_object_set_new(root, "performance", performance);
    
    // Publish using header topic constant
    char *json_str = json_dumps(root, JSON_COMPACT);
    if (json_str) {
        char topic[512];
        snprintf(topic, sizeof(topic), "%s%s", config.base_topic, GPS_TOPIC_STATUS);
        
        int rc = mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Error publishing GPS status: %s\n", mosquitto_strerror(rc));
        }
        
        free(json_str);
    }
    
    json_decref(root);
}

void publish_gps_heartbeat(void) {
    static time_t start_time = 0;
    json_t *root = json_object();
    if (!root) return;
    
    time_t now = time(NULL);
    if (start_time == 0) {
        start_time = now;
    }
    
    json_object_set_new(root, "type", json_string("gps_heartbeat"));
    json_object_set_new(root, "timestamp", json_integer(now));
    json_object_set_new(root, "status", json_string("running"));
    json_object_set_new(root, "uptime", json_integer(now - start_time));
    
    // Publish using header topic constant
    char *json_str = json_dumps(root, JSON_COMPACT);
    if (json_str) {
        char topic[512];
        snprintf(topic, sizeof(topic), "%s%s", config.base_topic, GPS_TOPIC_BRIDGE_HEARTBEAT);
        
        int rc = mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Error publishing GPS heartbeat: %s\n", mosquitto_strerror(rc));
        }
        
        free(json_str);
    }
    
    json_decref(root);
}

void signal_handler(int signal) {
    (void)signal;
    printf("\nReceived signal, shutting down...\n");
    running = 0;
}

void *gps_reader_thread(void *arg) {
    (void)arg;
    char buffer[DEFAULT_BUFFER_SIZE];
    char line[DEFAULT_BUFFER_SIZE];
    int line_pos = 0;
    
    while (running) {
        ssize_t bytes_read = read(gps_fd, buffer, sizeof(buffer) - 1);
        if (bytes_read > 0) {
            buffer[bytes_read] = '\0';
            
            // Process each character
            for (int i = 0; i < bytes_read; i++) {
                char c = buffer[i];
                
                if (c == '\n' || c == '\r') {
                    if (line_pos > 0) {
                        line[line_pos] = '\0';
                        
                        // Parse NMEA sentence
                        if (line[0] == '$') {
                            parse_nmea(line);
                        }
                        
                        line_pos = 0;
                    }
                } else if ((size_t)line_pos < sizeof(line) - 1) {
                    line[line_pos++] = c;
                }
            }
        } else if (bytes_read < 0 && errno != EAGAIN) {
            perror("GPS read error");
            break;
        }
        
        usleep(10000); // 10ms delay
    }
    
    return NULL;
}

int parse_nmea(const char *sentence) {
    // Simplified NMEA parsing - implement full parsing as needed
    pthread_mutex_lock(&data_mutex);
    
    if (strncmp(sentence, "$GPGGA", 6) == 0 || strncmp(sentence, "$GNGGA", 6) == 0) {
        // Parse GGA sentence for position and fix quality
        // This is a simplified implementation
        current_gps_data.timestamp = time(NULL);
        current_gps_data.is_valid = 1;
    }
    
    pthread_mutex_unlock(&data_mutex);
    return 0;
}
