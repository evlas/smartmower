/**
 * GPS Bridge - Connects a physical GPS device to MQTT
 * 
 * This program reads NMEA data from a serial GPS device and publishes
 * the parsed data to an MQTT broker.
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

// Default configuration values
#define DEFAULT_CONFIG_FILE "/opt/smartmower/etc/config/robot_config.json"
#define DEFAULT_BAUDRATE B9600
#define DEFAULT_BUFFER_SIZE 1024

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

// Main GPS data structure
typedef struct gps_data {
    // Position and movement
    double latitude;      // Decimal degrees (positive = N, negative = S)
    double longitude;     // Decimal degrees (positive = E, negative = W)
    double altitude;      // Meters above sea level
    double speed;         // Speed in km/h
    double course;        // Course in degrees (0-359)
    
    // Fix information
    int fix_quality;      // 0=invalid, 1=GPS fix, 2=DGPS fix, etc.
    int fix_type;         // 1=no fix, 2=2D, 3=3D
    char fix_mode;        // A=autonomous, D=DGPS, E=DR, M=manual, S=simulated
    
    // Precision data
    double pdop;          // Positional dilution of precision
    double hdop;          // Horizontal dilution of precision
    double vdop;          // Vertical dilution of precision
    
    // Time data
    time_t timestamp;     // Unix timestamp
    char time_str[32];    // Formatted UTC time (HHMMSS.SSS)
    char date_str[32];    // Formatted date (DDMMYY)
    
    // SBAS data (EGNOS, WAAS, etc.)
    char sbas_type[16];   // SBAS type ("EGNOS", "WAAS", "MSAS", etc.)
    int sbas_prn;         // PRN of SBAS satellite used
    int sbas_corrections; // Number of SBAS corrections applied
    
    // Constellation data
    constellation_data_t gps;
    constellation_data_t glonass;
    constellation_data_t galileo;
    constellation_data_t beidou;
    constellation_data_t qzss;
    constellation_data_t sbas;
    
    // Overall status
    int is_valid;         // Flag indicating if data is valid
    int num_constellations; // Number of active constellations
    int total_satellites; // Total satellites in view
    int total_used;       // Total satellites used in fix
} gps_data_t;

// Global variables
static volatile int keep_running = 1;
static struct mosquitto *mosq = NULL;
gps_data_t current_gps_data;

// Inizializza la struttura dei dati GPS a zero
void init_gps_data() {
    memset(&current_gps_data, 0, sizeof(current_gps_data));
    
    // Inizializza le stringhe di tempo
    strncpy(current_gps_data.time_str, "000000", sizeof(current_gps_data.time_str)-1);
    strncpy(current_gps_data.date_str, "010100", sizeof(current_gps_data.date_str)-1); // 01-01-2000
    
    // Inizializza i tipi di costellazione
    strncpy(current_gps_data.gps.type, "GPS", sizeof(current_gps_data.gps.type)-1);
    strncpy(current_gps_data.glonass.type, "GLONASS", sizeof(current_gps_data.glonass.type)-1);
    strncpy(current_gps_data.galileo.type, "GALILEO", sizeof(current_gps_data.galileo.type)-1);
    strncpy(current_gps_data.beidou.type, "BEIDOU", sizeof(current_gps_data.beidou.type)-1);
    strncpy(current_gps_data.qzss.type, "QZSS", sizeof(current_gps_data.qzss.type)-1);
    strncpy(current_gps_data.sbas.type, "SBAS", sizeof(current_gps_data.sbas.type)-1);
    
    // Inizializza i contatori
    current_gps_data.fix_quality = 0;
    current_gps_data.fix_type = 1; // 1 = nessun fix
    current_gps_data.fix_mode = 'N'; // N = nessun dato
    current_gps_data.is_valid = 0;
    current_gps_data.num_constellations = 0;
    current_gps_data.total_satellites = 0;
    current_gps_data.total_used = 0;
    
    // Inizializza i valori DOP (Dilution of Precision)
    current_gps_data.pdop = 99.99;
    current_gps_data.hdop = 99.99;
    current_gps_data.vdop = 99.99;
    
    // Inizializza i dati SBAS
    strncpy(current_gps_data.sbas_type, "NONE", sizeof(current_gps_data.sbas_type)-1);
    current_gps_data.sbas_prn = 0;
    current_gps_data.sbas_corrections = 0;
}

// Forward declaration of gps_data_t
struct gps_data_t {
    // Posizione e movimento
    double latitude;        // Gradi decimali (positivo = N, negativo = S)
    double longitude;       // Gradi decimali (positivo = E, negativo = W)
    double altitude;        // Metri sopra il livello del mare
    double speed;           // Velocità in km/h
    double course;          // Rotta in gradi (0-359)
    
    // Informazioni sul fix
    int fix_quality;        // Qualità del fix: 0=invalid, 1=GPS fix, 2=DGPS fix, etc.
    int fix_type;           // 1=nessun fix, 2=2D, 3=3D
    char fix_mode;          // A=autonomo, D=DGPS, E=DR, M=manuale, S=simulato
    
    // Dati di precisione
    double pdop;            // Precisione posizionale
    double hdop;            // Precisione orizzontale
    double vdop;            // Precisione verticale
    
    // Dati temporali
    time_t timestamp;       // Timestamp Unix
    char time_str[32];      // Ora UTC formattata (HHMMSS.SSS)
    char date_str[32];      // Data formattata (DDMMYY)
    
    // Dati SBAS (EGNOS, WAAS, etc.)
    char sbas_type[16];     // Tipo di SBAS ("EGNOS", "WAAS", "MSAS", etc.)
    int sbas_prn;           // PRN del satellite SBAS usato
    int sbas_corrections;   // Numero di correzioni SBAS applicate
    
    // Dettagli costellazioni
    constellation_data_t gps;
    constellation_data_t glonass;
    constellation_data_t galileo;
    constellation_data_t beidou;
    constellation_data_t qzss;
    constellation_data_t sbas;
    
    // Stato complessivo
    int is_valid;           // Flag che indica se i dati sono validi
    int num_constellations; // Numero di costellazioni attive
    int total_satellites;   // Totale satelliti in vista
    int total_used;         // Totale satelliti usati per il fix
};

// Configuration structure
typedef struct {
    // GPS Configuration
    char device[256];
    int baudrate;
    char protocol[16];
    int timeout_ms;
    
    // MQTT Configuration
    char mqtt_broker[256];
    int mqtt_port;
    char mqtt_username[64];
    char mqtt_password[64];
    char mqtt_base_topic[256];
    char mqtt_client_id[64];
    
    // Logging
    char log_level[16];
    char log_file[256];
} config_t;

// gps_data_t is now defined at the top of the file

// Function prototypes
void signal_handler(int signal);
int load_config(const char *filename, config_t *config);
int init_serial(const char *device, int baudrate);
int init_mqtt(config_t *config);
void cleanup_mqtt(void);
void publish_gps_data(int fd, config_t *config);
void publish_parsed_data(const config_t *config);
void handle_signal(int signal);
void init_gps_data();
int parse_nmea(const char *sentence, char *topic, size_t topic_len, char *payload, size_t payload_len);

int main(int argc, char *argv[]) {
    fprintf(stderr, "=== Starting GPS Bridge ===\n");
    fprintf(stderr, "Compiled on %s at %s\n", __DATE__, __TIME__);
    
    // Inizializza i dati GPS a zero
    init_gps_data();
    
    int fd;
    config_t config;
    const char *config_file = DEFAULT_CONFIG_FILE;
    
    // Parse command line arguments
    if (argc > 1) {
        config_file = argv[1];
    }
    fprintf(stderr, "Using configuration file: %s\n", config_file);
    
    // Load configuration
    fprintf(stderr, "Loading configuration from %s...\n", config_file);
    
    if (load_config(config_file, &config) != 0) {
        fprintf(stderr, "Failed to load configuration from %s\n", config_file);
        return EXIT_FAILURE;
    }
    
    fprintf(stderr, "Configuration loaded successfully. Device: %s, Baudrate: %d\n", 
           config.device, config.baudrate);
    
    // Initialize serial port
    fprintf(stderr, "Initializing serial port %s at %d baud...\n", config.device, config.baudrate);
    
    fd = init_serial(config.device, config.baudrate);
    if (fd < 0) {
        fprintf(stderr, "Failed to initialize serial port %s\n", config.device);
        return EXIT_FAILURE;
    }
    
    fprintf(stderr, "Serial port initialized successfully.\n");
    
    // Initialize MQTT
    fprintf(stderr, "Initializing MQTT connection...\n");
    
    if (init_mqtt(&config) != 0) {
        fprintf(stderr, "Failed to initialize MQTT\n");
        close(fd);
        return EXIT_FAILURE;
    }
    
    fprintf(stderr, "MQTT initialized successfully. Starting main loop...\n");
    
    // Set up signal handler for clean shutdown
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    printf("GPS Bridge started. Press Ctrl+C to exit.\n");
    
    // Main loop
    while (keep_running) {
        publish_gps_data(fd, &config);
        
        // Pubblica i dati GPS in formato JSON
        publish_parsed_data(&config);
        
        // Small delay to prevent high CPU usage
        struct timespec ts = {1, 0}; // 1 secondo
        nanosleep(&ts, NULL);
    }
    
    // Cleanup
    printf("Shutting down...\n");
    close(fd);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    return EXIT_SUCCESS;
}

// Load configuration from JSON file
int load_config(const char *filename, config_t *config) {
    json_t *root, *gps, *mqtt, *logging;
    json_error_t error;
    
    root = json_load_file(filename, 0, &error);
    if (!root) {
        fprintf(stderr, "Error parsing JSON: %s (line %d, column %d)\n", 
                error.text, error.line, error.column);
        return -1;
    }
    
    // Parse GPS configuration
    gps = json_object_get(root, "gps");
    if (gps) {
        const char *device = json_string_value(json_object_get(gps, "device"));
        if (device) strncpy(config->device, device, sizeof(config->device)-1);
        
        config->baudrate = json_integer_value(json_object_get(gps, "baudrate"));
        
        const char *protocol = json_string_value(json_object_get(gps, "protocol"));
        if (protocol) strncpy(config->protocol, protocol, sizeof(config->protocol)-1);
        
        config->timeout_ms = json_integer_value(json_object_get(gps, "timeout_ms"));
    }
    
    // Parse MQTT configuration
    mqtt = json_object_get(root, "mqtt");
    if (mqtt) {
        const char *broker = json_string_value(json_object_get(mqtt, "broker"));
        if (broker) strncpy(config->mqtt_broker, broker, sizeof(config->mqtt_broker)-1);
        
        config->mqtt_port = json_integer_value(json_object_get(mqtt, "port"));
        
        const char *username = json_string_value(json_object_get(mqtt, "username"));
        if (username) strncpy(config->mqtt_username, username, sizeof(config->mqtt_username)-1);
        
        const char *password = json_string_value(json_object_get(mqtt, "password"));
        if (password) strncpy(config->mqtt_password, password, sizeof(config->mqtt_password)-1);
        
        const char *base_topic = json_string_value(json_object_get(mqtt, "base_topic"));
        if (base_topic) strncpy(config->mqtt_base_topic, base_topic, sizeof(config->mqtt_base_topic)-1);
        
        const char *client_id = json_string_value(json_object_get(mqtt, "client_id"));
        if (client_id) {
            strncpy(config->mqtt_client_id, client_id, sizeof(config->mqtt_client_id)-1);
        } else {
            snprintf(config->mqtt_client_id, sizeof(config->mqtt_client_id), "gps_bridge_%d", getpid());
        }
    }
    
    // Parse logging configuration
    logging = json_object_get(root, "logging");
    if (logging) {
        const char *log_level = json_string_value(json_object_get(logging, "level"));
        if (log_level) strncpy(config->log_level, log_level, sizeof(config->log_level)-1);
        
        const char *log_file = json_string_value(json_object_get(logging, "file"));
        if (log_file) strncpy(config->log_file, log_file, sizeof(config->log_file)-1);
    }
    
    json_decref(root);
    return 0;
}

// Initialize serial port
int init_serial(const char *device, int baudrate) {
    int fd = open(device, O_RDWR | O_NOCTTY | O_NDELAY);
    if (fd == -1) {
        perror("Unable to open serial port");
        return -1;
    }
    
    struct termios options;
    tcgetattr(fd, &options);
    
    // Set baud rate
    cfsetispeed(&options, baudrate);
    cfsetospeed(&options, baudrate);
    
    // 8N1
    options.c_cflag &= ~PARENB;
    options.c_cflag &= ~CSTOPB;
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;
    
    // Enable receiver, ignore modem control lines
    options.c_cflag |= (CLOCAL | CREAD);
    
    // Disable flow control
    options.c_cflag &= ~CRTSCTS;
    
    // Raw input
    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    
    // Disable software flow control
    options.c_iflag &= ~(IXON | IXOFF | IXANY);
    
    // Raw output
    options.c_oflag &= ~OPOST;
    
    // Apply the new settings
    tcsetattr(fd, TCSANOW, &options);
    
    return fd;
}

// Initialize MQTT connection
int init_mqtt(config_t *config) {
    printf("Initializing MQTT connection to %s:%d...\n", config->mqtt_broker, config->mqtt_port);
    
    // Initialize Mosquitto library
    mosquitto_lib_init();
    
    // Create a new Mosquitto client instance
    mosq = mosquitto_new(config->mqtt_client_id, true, NULL);
    if (!mosq) {
        fprintf(stderr, "Error: Out of memory\n");
        return -1;
    }
    
    // Set username and password if provided
    if (strlen(config->mqtt_username) > 0) {
        printf("Setting MQTT username: %s\n", config->mqtt_username);
        mosquitto_username_pw_set(mosq, config->mqtt_username, config->mqtt_password);
    }
    
    // Connect to MQTT broker
    printf("Connecting to MQTT broker at %s:%d...\n", config->mqtt_broker, config->mqtt_port);
    int rc = mosquitto_connect(mosq, config->mqtt_broker, config->mqtt_port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        fprintf(stderr, "Unable to connect to MQTT broker: %s\n", mosquitto_strerror(rc));
        mosquitto_destroy(mosq);
        mosq = NULL;
        return -1;
    }
    
    printf("Successfully connected to MQTT broker\n");
    
    return 0;
}

// Signal handler for clean shutdown
void signal_handler(int signal) {
    (void)signal; // Unused parameter
    keep_running = 0;
}

// Pubblica i dati GNSS in formato JSON
void publish_parsed_data(const config_t *config) {
    if (!mosq) {
        fprintf(stderr, "Errore: MQTT non inizializzato\n");
        return;
    }
    
    json_t *root = json_object();
    if (!root) {
        fprintf(stderr, "Errore creazione JSON\n");
        return;
    }
    
    // Informazioni base
    json_object_set_new(root, "valid", json_boolean(current_gps_data.is_valid));
    
    // Posizione
    json_t *pos = json_object();
    json_object_set_new(pos, "lat", json_real(current_gps_data.latitude));
    json_object_set_new(pos, "lon", json_real(current_gps_data.longitude));
    json_object_set_new(pos, "alt", json_real(current_gps_data.altitude));
    json_object_set_new(pos, "speed", json_real(current_gps_data.speed));
    json_object_set_new(pos, "course", json_real(current_gps_data.course));
    json_object_set_new(root, "position", pos);
    
    // Fix info
    json_t *fix = json_object();
    json_object_set_new(fix, "quality", json_integer(current_gps_data.fix_quality));
    json_object_set_new(fix, "type", json_integer(current_gps_data.fix_type));
    char mode_str[2] = {current_gps_data.fix_mode, '\0'};
    json_object_set_new(fix, "mode", json_string(mode_str));
    json_object_set_new(root, "fix", fix);
    
    // Precisione
    json_t *prec = json_object();
    json_object_set_new(prec, "pdop", json_real(current_gps_data.pdop));
    json_object_set_new(prec, "hdop", json_real(current_gps_data.hdop));
    json_object_set_new(prec, "vdop", json_real(current_gps_data.vdop));
    json_object_set_new(root, "precision", prec);
    
    // Tempo
    json_t *time = json_object();
    json_object_set_new(time, "timestamp", json_integer(current_gps_data.timestamp));
    json_object_set_new(time, "time", json_string(current_gps_data.time_str));
    json_object_set_new(time, "date", json_string(current_gps_data.date_str));
    json_object_set_new(root, "time", time);
    
    // SBAS
    if (current_gps_data.sbas_type[0] != '\0' && 
        strcmp(current_gps_data.sbas_type, "NONE") != 0) {
        json_t *sbas = json_object();
        json_object_set_new(sbas, "type", json_string(current_gps_data.sbas_type));
        json_object_set_new(sbas, "prn", json_integer(current_gps_data.sbas_prn));
        json_object_set_new(root, "sbas", sbas);
    }
    
    // Costellazioni
    const constellation_data_t *constellations[] = {
        &current_gps_data.gps,
        &current_gps_data.glonass,
        &current_gps_data.galileo,
        &current_gps_data.beidou,
        &current_gps_data.qzss,
        &current_gps_data.sbas
    };
    
    json_t *consts = json_object();
    const char *const_names[] = {"gps", "glonass", "galileo", "beidou", "qzss", "sbas"};
    
    for (int i = 0; i < 6; i++) {
        if (constellations[i]->num_satellites > 0) {
            json_t *c = json_object();
            json_object_set_new(c, "sats_in_view", json_integer(constellations[i]->num_satellites));
            json_object_set_new(c, "sats_used", json_integer(constellations[i]->num_used));
            json_object_set_new(consts, const_names[i], c);
        }
    }
    json_object_set_new(root, "constellations", consts);
    
    // Converti e pubblica
    char *json_str = json_dumps(root, JSON_INDENT(2));
    if (json_str) {
        char topic[512];
        snprintf(topic, sizeof(topic), "%s/status", config->mqtt_base_topic);
        
        int rc = mosquitto_publish(mosq, NULL, topic, strlen(json_str), json_str, 0, false);
        if (rc != MOSQ_ERR_SUCCESS) {
            fprintf(stderr, "Errore pubblicazione JSON: %s\n", mosquitto_strerror(rc));
        }
        
        free(json_str);
    }
    
    json_decref(root);
}

// Publish GPS data to MQTT
void publish_gps_data(int fd, config_t *config) {
    static char buffer[1024];
    static size_t pos = 0;
    char topic[512];
    char payload[1024];
    ssize_t n;
    
    // Read data from serial port
    n = read(fd, buffer + pos, sizeof(buffer) - pos - 1);
    if (n <= 0) {
        if (errno != EAGAIN) {
            // Silent error - no perror
        }
        return;
    }
    
    pos += n;
    buffer[pos] = '\0';
    
    // Process complete NMEA sentences
    char *start = buffer;
    char *end;
    
    while ((end = strchr(start, '\n')) != NULL) {
        *end = '\0';
        
        // Parse NMEA sentence
        if (parse_nmea(start, topic, sizeof(topic), payload, sizeof(payload)) == 0) {
            // Construct full MQTT topic
            char full_topic[1024];
            snprintf(full_topic, sizeof(full_topic), "%s/%s", config->mqtt_base_topic, topic);
            
            // Publish to MQTT with debug info
            fprintf(stderr, "Publishing to MQTT topic: %s\n", full_topic);
            fprintf(stderr, "Payload: %s\n", payload);
            
            int rc = mosquitto_publish(mosq, NULL, full_topic, strlen(payload), payload, 0, false);
            if (rc != MOSQ_ERR_SUCCESS) {
                fprintf(stderr, "Error publishing to MQTT: %s\n", mosquitto_strerror(rc));
            }
        }
        
        start = end + 1;
    }
    
    // Move remaining data to the beginning of the buffer
    if (start != buffer) {
        size_t remaining = buffer + pos - start;
        if (remaining > 0) {
            memmove(buffer, start, remaining);
        }
        pos = remaining;
    } else if (pos >= sizeof(buffer) - 1) {
        // Buffer overflow, discard data
        pos = 0;
    }
}

// Parse NMEA sentence and extract data
int parse_nmea(const char *sentence, char *topic, size_t topic_len, char *payload, size_t payload_len) {
    // Skip empty or invalid sentences
    if (sentence == NULL || strlen(sentence) < 7) {
        return -1;
    }

    // Check for supported NMEA sentence types (both GPS and GLONASS)
    const char *sentence_type = sentence + 2; // Skip the $GP or $GN prefix
    
    // Handle GPS (GP) and GLONASS (GN) prefixes
    if ((sentence[1] != 'G' || (sentence[2] != 'P' && sentence[2] != 'N')) && 
        (sentence[1] != 'B' && sentence[1] != 'I' && sentence[1] != 'Q')) {
        return -1; // Not a supported NMEA sentence
    }

    // Common NMEA sentence types (both GPS and GLONASS)
    if (strncmp(sentence_type, "GGA", 3) == 0) {
        // GPS Fix Data
        snprintf(topic, topic_len, "fix/gga");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    } else if (strncmp(sentence_type, "RMC", 3) == 0) {
        // Recommended Minimum Navigation Information
        snprintf(topic, topic_len, "nav/rmc");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    } else if (strncmp(sentence_type, "VTG", 3) == 0) {
        // Track Made Good and Ground Speed
        snprintf(topic, topic_len, "nav/vtg");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    } else if (strncmp(sentence_type, "GSA", 3) == 0) {
        // GPS DOP and active satellites
        snprintf(topic, topic_len, "status/gsa");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    } else if (strncmp(sentence_type, "GSV", 3) == 0) {
        // Satellites in view
        snprintf(topic, topic_len, "status/gsv");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    } else if (strncmp(sentence_type, "GLL", 3) == 0) {
        // Geographic Position - Latitude/Longitude
        snprintf(topic, topic_len, "pos/gll");
        snprintf(payload, payload_len, "%s", sentence);
        return 0;
    }
    
    return -1; // Unsupported sentence type
}
