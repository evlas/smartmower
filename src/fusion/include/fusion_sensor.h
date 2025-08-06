#ifndef FUSION_SENSOR_H
#define FUSION_SENSOR_H

#include <string>
#include <vector>
#include <atomic>
#include <mutex>
#include <Eigen/Dense>

using namespace Eigen;

// Forward declarations
struct mosquitto;
struct mosquitto_message;

// Struttura per la configurazione del sensore di fusione
struct FusionConfig {
    // Configurazione IMU
    struct {
        int update_rate_hz;
        int accel_range_g;
        int gyro_range_dps;
        int mag_range_ut;
        struct {
            double accel_offset[3];
            double gyro_offset[3];
            double mag_offset[3];
            double mag_scale[3];
        } calibration;
    } imu;

    // Configurazione GPS
    struct {
        int update_rate_hz;
        int timeout_ms;
        double hdop_threshold;
        int min_satellites;
    } gps;

    // Parametri di fusione
    struct {
        int update_rate_hz;
        int publish_rate_hz;
        struct {
            double position;
            double velocity;
            double orientation;
        } process_noise;
        struct {
            double gps_position;
            double imu_accel;
            double imu_gyro;
            double odometry;
        } measurement_noise;
        struct {
            double velocity;
            double position;
            double orientation;
        } filter_alpha;
    } fusion_parameters;

    // Configurazione logging
    struct {
        bool enabled;
        std::string level;
        std::string file;
        bool save_raw_data;
        std::string data_dir;
    } fusion_logging;

    // Configurazione MQTT
    struct {
        std::string broker_host;
        int broker_port;
        std::string username;
        std::string password;
        std::string client_id;
        std::string base_topic;
        int qos;
        bool retain;
        int keepalive;
    } mqtt_settings;

    // Configurazione odometria
    struct {
        double wheel_radius;           // Raggio della ruota in metri
        int ticks_per_revolution;      // Impulsi encoder per giro ruota
        double gear_ratio;             // Rapporto di riduzione del motore
        double wheel_base;             // Distanza tra le ruote (carreggiata) in metri
        double wheel_track;            // Distanza tra gli assi delle ruote (passo) in metri
        double ticks_per_meter;        // Calcolato in base ai parametri sopra
    } odometry;

    // Configurazione di sicurezza
    struct {
        bool emergency_stop_enabled;   // Abilita/disabilita l'arresto di emergenza
        bool lift_sensor_enabled;      // Abilita/disabilita il sensore di sollevamento
        double max_linear_velocity;    // Velocità lineare massima (m/s)
        double max_angular_velocity;   // Velocità angolare massima (rad/s)
        double max_tilt_angle;         // Angolo di inclinazione massimo (gradi)
        double max_voltage;            // Tensione massima batteria (V)
        double min_voltage;            // Tensione minima batteria (V)
        double max_motor_temp;         // Temperatura massima motori (°C)
    } safety;
};

// Dichiarazione della variabile globale di configurazione
extern FusionConfig g_config;

// Dichiarazione delle funzioni
extern void calculate_ticks_per_meter();
extern bool load_config(const std::string& filename);
extern void init_default_config();

// Dichiarazione delle funzioni di callback MQTT
extern void on_connect(struct mosquitto *mosq, void *obj, int rc);
extern void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);

// Dichiarazione del gestore dei segnali
extern void signal_handler(int signum);

// Variabile globale per il controllo dell'esecuzione
extern std::atomic<bool> g_running;

// Mutex per la sincronizzazione
extern std::mutex g_mutex;

#endif // FUSION_SENSOR_H
