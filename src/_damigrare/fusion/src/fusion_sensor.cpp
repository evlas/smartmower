#include "fusion_sensor.h"
#include <iostream>
#include <fstream>
#include <string>
#include <cstring>
#include <chrono>
#include <thread>
#include <atomic>
#include <queue>
#include <cmath>
#include <signal.h>
#include <mosquitto.h>
#include <json-c/json.h>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Fusion MQTT definitions
#include "fusion_mqtt.h"
#include <Eigen/Dense>

using namespace std::chrono;
using namespace Eigen;

// Forward declarations
void on_connect(struct mosquitto *mosq, void *obj, int rc);
void on_message(struct mosquitto *mosq, void *obj, const struct mosquitto_message *msg);
void signal_handler(int signum);
bool load_config(const std::string& filename);
void calculate_ticks_per_meter();

// Global variables for signal handling
std::atomic<bool> g_running{true};
std::mutex g_mutex;
FusionConfig g_config;  // Global configuration instance

// La struttura FusionConfig è completamente definita in fusion_sensor.h

// Funzione per calcolare i tick per metro
void calculate_ticks_per_meter() {
    if (g_config.odometry.wheel_radius > 0 && 
        g_config.odometry.ticks_per_revolution > 0 && 
        g_config.odometry.gear_ratio > 0) {
        
        double wheel_circumference = 2.0 * M_PI * g_config.odometry.wheel_radius;
        double ticks_per_wheel_rev = g_config.odometry.ticks_per_revolution * 
                                   g_config.odometry.gear_ratio;
        g_config.odometry.ticks_per_meter = ticks_per_wheel_rev / wheel_circumference;
        
        // Log dei parametri calcolati
        std::cout << "Odometry parameters:" << std::endl;
        std::cout << "  Wheel radius: " << g_config.odometry.wheel_radius << " m" << std::endl;
        std::cout << "  Ticks per revolution: " << g_config.odometry.ticks_per_revolution << std::endl;
        std::cout << "  Gear ratio: " << g_config.odometry.gear_ratio << std::endl;
        std::cout << "  Ticks per meter: " << g_config.odometry.ticks_per_meter << std::endl;
    } else {
        std::cerr << "Errore: parametri odometria non validi per il calcolo dei tick per metro" << std::endl;
    }
}

// Funzione per caricare la configurazione
bool load_config(const std::string& filename) {
    // Inizializzazione con valori di default
    g_config = {
        // IMU defaults
        .imu = {
            .update_rate_hz = 100,
            .accel_range_g = 4,
            .gyro_range_dps = 500,
            .mag_range_ut = 1300,
            .calibration = {
                .accel_offset = {0.0, 0.0, 0.0},
                .gyro_offset = {0.0, 0.0, 0.0},
                .mag_offset = {0.0, 0.0, 0.0},
                .mag_scale = {1.0, 1.0, 1.0}
            }
        },
        // GPS defaults
        .gps = {
            .update_rate_hz = 10,
            .timeout_ms = 2000,
            .hdop_threshold = 2.0,
            .min_satellites = 5
        },
        // Fusion parameters defaults
        .fusion_parameters = {
            .update_rate_hz = 100,
            .publish_rate_hz = 50,
            .process_noise = {
                .position = 0.1,
                .velocity = 0.1,
                .orientation = 0.01
            },
            .measurement_noise = {
                .gps_position = 1.0,
                .imu_accel = 0.1,
                .imu_gyro = 0.01,
                .odometry = 0.05
            },
            .filter_alpha = {
                .velocity = 0.2,
                .position = 0.1,
                .orientation = 0.3
            }
        },
        // Logging defaults
        .fusion_logging = {
            .enabled = true,
            .level = "info",
            .file = "/var/log/smartmower/fusion_sensor.log",
            .save_raw_data = false,
            .data_dir = "/var/log/smartmower/data"
        },
        // MQTT defaults
        .mqtt_settings = {
            .broker_host = "localhost",
            .broker_port = 1883,
            .username = "",
            .password = "",
            .client_id = "fusion_sensor",
            .base_topic = "smartmower/fusion",
            .qos = 1,
            .retain = false,
            .keepalive = 60
        },
        // Odometry defaults
        .odometry = {
            .wheel_radius = 0.1,           // Valore di esempio in metri
            .ticks_per_revolution = 1000,  // Valore di esempio
            .gear_ratio = 30.0,            // Valore di esempio
            .wheel_base = 0.5,             // Valore di esempio in metri
            .wheel_track = 0.3,            // Valore di esempio in metri
            .ticks_per_meter = 0.0         // Sarà calcolato successivamente
        },
        // Safety defaults
        .safety = {
            .emergency_stop_enabled = true,
            .lift_sensor_enabled = true,
            .max_linear_velocity = 0.5,     // 0.5 m/s
            .max_angular_velocity = 1.0,    // 1.0 rad/s
            .max_tilt_angle = 15.0,         // 15 gradi
            .max_voltage = 29.4,            // 29.4V (esempio per batteria 24V)
            .min_voltage = 21.0,            // 21.0V (soglia di scarica)
            .max_motor_temp = 80.0          // 80°C massimi
        }
    };
    // Inizializzazione degli array di calibrazione
    for (int i = 0; i < 3; i++) {
        g_config.imu.calibration.accel_offset[i] = 0.0;
        g_config.imu.calibration.gyro_offset[i] = 0.0;
        g_config.imu.calibration.mag_offset[i] = 0.0;
        g_config.imu.calibration.mag_scale[i] = 1.0;
    }
    
    // GPS defaults
    g_config.gps.update_rate_hz = 5;
    g_config.gps.timeout_ms = 2000;
    g_config.gps.hdop_threshold = 3.0;
    g_config.gps.min_satellites = 4;
    
    // Odometry defaults
    g_config.odometry.wheel_base = 0.5;
    g_config.odometry.wheel_radius = 0.1;
    g_config.odometry.ticks_per_revolution = 1000;
    
    // Fusion parameters defaults
    g_config.fusion_parameters.update_rate_hz = 100;
    g_config.fusion_parameters.publish_rate_hz = 20;
    g_config.fusion_parameters.process_noise.position = 0.1;
    g_config.fusion_parameters.process_noise.velocity = 0.1;
    g_config.fusion_parameters.process_noise.orientation = 0.01;
    g_config.fusion_parameters.measurement_noise.gps_position = 1.0;
    g_config.fusion_parameters.measurement_noise.imu_accel = 0.1;
    g_config.fusion_parameters.measurement_noise.imu_gyro = 0.01;
    g_config.fusion_parameters.measurement_noise.odometry = 0.05;
    g_config.fusion_parameters.filter_alpha.velocity = 0.2;
    g_config.fusion_parameters.filter_alpha.position = 0.1;
    g_config.fusion_parameters.filter_alpha.orientation = 0.3;
    
    // MQTT defaults (from root level in robot_config.json)
    g_config.mqtt_settings.qos = 1;
    g_config.mqtt_settings.retain = false;
    g_config.mqtt_settings.keepalive = 60;
    g_config.mqtt_settings.broker_host = "localhost";
    g_config.mqtt_settings.broker_port = 1883;
    g_config.mqtt_settings.username = "mower";
    g_config.mqtt_settings.password = "smart";
    
    // Logging defaults
    g_config.fusion_logging.enabled = true;
    g_config.fusion_logging.level = "info";
    g_config.fusion_logging.file = "/var/log/smartmower/fusion.log";
    g_config.fusion_logging.save_raw_data = false;
    g_config.fusion_logging.data_dir = "/var/log/smartmower";
    
    // Load from centralized config file
    json_object* root = json_object_from_file(filename.c_str());
    if (!root) {
        std::cout << "Config file not found, using defaults" << std::endl;
        return true;
    }
    
    // Get fusion_config directly from root
    json_object* fusion_config;
    if (json_object_object_get_ex(root, "fusion_config", &fusion_config)) {
            // Parse IMU config
            json_object* imu_obj;
            if (json_object_object_get_ex(fusion_config, "imu", &imu_obj)) {
                json_object* val;
                if (json_object_object_get_ex(imu_obj, "update_rate_hz", &val))
                    g_config.imu.update_rate_hz = json_object_get_int(val);
                if (json_object_object_get_ex(imu_obj, "accel_range_g", &val))
                    g_config.imu.accel_range_g = json_object_get_int(val);
                if (json_object_object_get_ex(imu_obj, "gyro_range_dps", &val))
                    g_config.imu.gyro_range_dps = json_object_get_int(val);
                if (json_object_object_get_ex(imu_obj, "mag_range_ut", &val))
                    g_config.imu.mag_range_ut = json_object_get_int(val);
                
                // Parse calibration
                json_object* cal_obj;
                if (json_object_object_get_ex(imu_obj, "calibration", &cal_obj)) {
                    json_object* offset_arr;
                    if (json_object_object_get_ex(cal_obj, "accel_offset", &offset_arr)) {
                        for (size_t i = 0; i < 3 && i < json_object_array_length(offset_arr); i++) {
                            g_config.imu.calibration.accel_offset[i] = 
                                json_object_get_double(json_object_array_get_idx(offset_arr, i));
                        }
                    }
                    if (json_object_object_get_ex(cal_obj, "gyro_offset", &offset_arr)) {
                        for (size_t i = 0; i < 3 && i < json_object_array_length(offset_arr); i++) {
                            g_config.imu.calibration.gyro_offset[i] = 
                                json_object_get_double(json_object_array_get_idx(offset_arr, i));
                        }
                    }
                    if (json_object_object_get_ex(cal_obj, "mag_offset", &offset_arr)) {
                        for (size_t i = 0; i < 3 && i < json_object_array_length(offset_arr); i++) {
                            g_config.imu.calibration.mag_offset[i] = 
                                json_object_get_double(json_object_array_get_idx(offset_arr, i));
                        }
                    }
                    if (json_object_object_get_ex(cal_obj, "mag_scale", &offset_arr)) {
                        for (size_t i = 0; i < 3 && i < json_object_array_length(offset_arr); i++) {
                            g_config.imu.calibration.mag_scale[i] = 
                                json_object_get_double(json_object_array_get_idx(offset_arr, i));
                        }
                    }
                }
            }
            
            // Parse GPS config
            json_object* gps_obj;
            if (json_object_object_get_ex(fusion_config, "gps", &gps_obj)) {
                json_object* val;
                if (json_object_object_get_ex(gps_obj, "update_rate_hz", &val))
                    g_config.gps.update_rate_hz = json_object_get_int(val);
                if (json_object_object_get_ex(gps_obj, "timeout_ms", &val))
                    g_config.gps.timeout_ms = json_object_get_int(val);
                if (json_object_object_get_ex(gps_obj, "hdop_threshold", &val))
                    g_config.gps.hdop_threshold = json_object_get_double(val);
                if (json_object_object_get_ex(gps_obj, "min_satellites", &val))
                    g_config.gps.min_satellites = json_object_get_int(val);
            }
            
            // Parse odometry config
            json_object* odom_obj;
            if (json_object_object_get_ex(fusion_config, "odometry", &odom_obj)) {
                json_object* val;
                if (json_object_object_get_ex(odom_obj, "wheel_base", &val))
                    g_config.odometry.wheel_base = json_object_get_double(val);
                if (json_object_object_get_ex(odom_obj, "wheel_radius", &val))
                    g_config.odometry.wheel_radius = json_object_get_double(val);
                if (json_object_object_get_ex(odom_obj, "ticks_per_revolution", &val))
                    g_config.odometry.ticks_per_revolution = json_object_get_int(val);
            }
            
            // Parse fusion parameters
            json_object* fusion_params;
            if (json_object_object_get_ex(fusion_config, "fusion_parameters", &fusion_params)) {
                json_object* val;
                if (json_object_object_get_ex(fusion_params, "update_rate_hz", &val))
                    g_config.fusion_parameters.update_rate_hz = json_object_get_int(val);
                if (json_object_object_get_ex(fusion_params, "publish_rate_hz", &val))
                    g_config.fusion_parameters.publish_rate_hz = json_object_get_int(val);
                
                // Parse noise parameters
                json_object* noise_obj;
                if (json_object_object_get_ex(fusion_params, "process_noise", &noise_obj)) {
                    if (json_object_object_get_ex(noise_obj, "position", &val))
                        g_config.fusion_parameters.process_noise.position = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "velocity", &val))
                        g_config.fusion_parameters.process_noise.velocity = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "orientation", &val))
                        g_config.fusion_parameters.process_noise.orientation = json_object_get_double(val);
                }
                
                if (json_object_object_get_ex(fusion_params, "measurement_noise", &noise_obj)) {
                    if (json_object_object_get_ex(noise_obj, "gps_position", &val))
                        g_config.fusion_parameters.measurement_noise.gps_position = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "imu_accel", &val))
                        g_config.fusion_parameters.measurement_noise.imu_accel = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "imu_gyro", &val))
                        g_config.fusion_parameters.measurement_noise.imu_gyro = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "odometry", &val))
                        g_config.fusion_parameters.measurement_noise.odometry = json_object_get_double(val);
                }
                
                if (json_object_object_get_ex(fusion_params, "filter_alpha", &noise_obj)) {
                    if (json_object_object_get_ex(noise_obj, "velocity", &val))
                        g_config.fusion_parameters.filter_alpha.velocity = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "position", &val))
                        g_config.fusion_parameters.filter_alpha.position = json_object_get_double(val);
                    if (json_object_object_get_ex(noise_obj, "orientation", &val))
                        g_config.fusion_parameters.filter_alpha.orientation = json_object_get_double(val);
                }
            }
            
            // Parse MQTT settings
            json_object* mqtt_obj;
            if (json_object_object_get_ex(fusion_config, "mqtt_settings", &mqtt_obj)) {
                json_object* val;
                if (json_object_object_get_ex(mqtt_obj, "qos", &val))
                    g_config.mqtt_settings.qos = json_object_get_int(val);
                if (json_object_object_get_ex(mqtt_obj, "retain", &val))
                    g_config.mqtt_settings.retain = json_object_get_boolean(val);
                if (json_object_object_get_ex(mqtt_obj, "keepalive", &val))
                    g_config.mqtt_settings.keepalive = json_object_get_int(val);
            }
        }
        
        // Parse logging settings from root
    json_object* logging_obj;
    if (json_object_object_get_ex(root, "fusion_logging", &logging_obj)) {
        json_object* val;
        if (json_object_object_get_ex(logging_obj, "enabled", &val))
            g_config.fusion_logging.enabled = json_object_get_boolean(val);
        if (json_object_object_get_ex(logging_obj, "level", &val))
            g_config.fusion_logging.level = json_object_get_string(val);
        if (json_object_object_get_ex(logging_obj, "file", &val))
            g_config.fusion_logging.file = json_object_get_string(val);
        if (json_object_object_get_ex(logging_obj, "save_raw_data", &val))
            g_config.fusion_logging.save_raw_data = json_object_get_boolean(val);
        if (json_object_object_get_ex(logging_obj, "data_dir", &val))
            g_config.fusion_logging.data_dir = json_object_get_string(val);
    }
    
    // Parse MQTT settings from root
    json_object* mqtt_obj;
    if (json_object_object_get_ex(root, "mqtt_settings", &mqtt_obj)) {
        json_object* val;
        
        // Inizializza i valori di default per MQTT
        g_config.mqtt_settings.broker_host = "localhost";
        g_config.mqtt_settings.broker_port = 1883;
        g_config.mqtt_settings.username = "";
        g_config.mqtt_settings.password = "";
        g_config.mqtt_settings.client_id = "fusion_sensor";
        g_config.mqtt_settings.base_topic = "smartmower/fusion";
        g_config.mqtt_settings.qos = 1;
        g_config.mqtt_settings.retain = false;
        g_config.mqtt_settings.keepalive = 60;
        if (json_object_object_get_ex(mqtt_obj, "qos", &val))
            g_config.mqtt_settings.qos = json_object_get_int(val);
        if (json_object_object_get_ex(mqtt_obj, "retain", &val))
            g_config.mqtt_settings.retain = json_object_get_boolean(val);
        if (json_object_object_get_ex(mqtt_obj, "keepalive", &val))
            g_config.mqtt_settings.keepalive = json_object_get_int(val);
        if (json_object_object_get_ex(mqtt_obj, "broker", &val))
            g_config.mqtt_settings.broker_host = json_object_get_string(val);
        if (json_object_object_get_ex(mqtt_obj, "port", &val))
            g_config.mqtt_settings.broker_port = json_object_get_int(val);
        if (json_object_object_get_ex(mqtt_obj, "username", &val))
            g_config.mqtt_settings.username = json_object_get_string(val);
        if (json_object_object_get_ex(mqtt_obj, "password", &val))
            g_config.mqtt_settings.password = json_object_get_string(val);
    }
    
    json_object_put(root);
    
    // Stampa la configurazione caricata per debug
    std::cout << "Fusion Config loaded:" << std::endl;
    std::cout << "  IMU: " << g_config.imu.update_rate_hz << "Hz, " 
              << g_config.imu.accel_range_g << "g, " << g_config.imu.gyro_range_dps << "dps, " 
              << g_config.imu.mag_range_ut << "uT" << std::endl;
    std::cout << "  IMU Calibration:" << std::endl;
    std::cout << "    Accel offset: [" 
              << g_config.imu.calibration.accel_offset[0] << ", " 
              << g_config.imu.calibration.accel_offset[1] << ", " 
              << g_config.imu.calibration.accel_offset[2] << "]" << std::endl;
    std::cout << "    Gyro offset: [" 
              << g_config.imu.calibration.gyro_offset[0] << ", " 
              << g_config.imu.calibration.gyro_offset[1] << ", " 
              << g_config.imu.calibration.gyro_offset[2] << "]" << std::endl;
    std::cout << "    Mag offset: [" 
              << g_config.imu.calibration.mag_offset[0] << ", " 
              << g_config.imu.calibration.mag_offset[1] << ", " 
              << g_config.imu.calibration.mag_offset[2] << "]" << std::endl;
    std::cout << "    Mag scale: [" 
              << g_config.imu.calibration.mag_scale[0] << ", " 
              << g_config.imu.calibration.mag_scale[1] << ", " 
              << g_config.imu.calibration.mag_scale[2] << "]" << std::endl;
    std::cout << "  GPS: " << g_config.gps.update_rate_hz << "Hz, " 
              << g_config.gps.min_satellites << " sats min" << std::endl;
    std::cout << "  Fusion: " << g_config.fusion_parameters.update_rate_hz << "Hz update, " 
              << g_config.fusion_parameters.publish_rate_hz << "Hz publish" << std::endl;
    std::cout << "  MQTT: " << g_config.mqtt_settings.broker_host << ":" << g_config.mqtt_settings.broker_port
              << " (user: " << g_config.mqtt_settings.username << ")" << std::endl;
    std::cout << "  Logging: " << g_config.fusion_logging.level << " -> " 
              << g_config.fusion_logging.file << std::endl;

    
    return true;
}

// Inizializza la configurazione di default
void init_default_config() {
    // Configurazione IMU di default
    g_config.imu.update_rate_hz = 100;
    g_config.imu.accel_range_g = 4;
    g_config.imu.gyro_range_dps = 500;
    g_config.imu.mag_range_ut = 4800;
    
    // Azzera i valori di calibrazione
    for (int i = 0; i < 3; ++i) {
        g_config.imu.calibration.accel_offset[i] = 0.0;
        g_config.imu.calibration.gyro_offset[i] = 0.0;
        g_config.imu.calibration.mag_offset[i] = 0.0;
        g_config.imu.calibration.mag_scale[i] = 1.0;
    }
    
    // Configurazione GPS di default
    g_config.gps.update_rate_hz = 5;
    g_config.gps.timeout_ms = 2000;
    g_config.gps.hdop_threshold = 3.0;
    g_config.gps.min_satellites = 4;
    
    // Configurazione odometria di default
    g_config.odometry.wheel_base = 0.35;
    g_config.odometry.wheel_radius = 0.1;
    g_config.odometry.ticks_per_revolution = 12;
    g_config.odometry.gear_ratio = 185.0;
    calculate_ticks_per_meter();
    
    // Configurazione parametri di fusione di default
    g_config.fusion_parameters.update_rate_hz = 100;
    g_config.fusion_parameters.publish_rate_hz = 20;
    g_config.fusion_parameters.process_noise.position = 0.1;
    g_config.fusion_parameters.process_noise.velocity = 0.1;
    g_config.fusion_parameters.process_noise.orientation = 0.01;
    g_config.fusion_parameters.measurement_noise.gps_position = 1.0;
    g_config.fusion_parameters.measurement_noise.imu_accel = 0.1;
    g_config.fusion_parameters.measurement_noise.imu_gyro = 0.01;
    g_config.fusion_parameters.measurement_noise.odometry = 0.05;
    g_config.fusion_parameters.filter_alpha.velocity = 0.2;
    g_config.fusion_parameters.filter_alpha.position = 0.1;
    g_config.fusion_parameters.filter_alpha.orientation = 0.3;
    
    // Configurazione MQTT di default
    g_config.mqtt_settings.broker_host = "localhost";
    g_config.mqtt_settings.broker_port = 1883;
    g_config.mqtt_settings.username = "";
    g_config.mqtt_settings.password = "";
    g_config.mqtt_settings.client_id = "fusion_sensor";
    g_config.mqtt_settings.base_topic = "smartmower/fusion";
    g_config.mqtt_settings.qos = 1;
    g_config.mqtt_settings.retain = false;
    g_config.mqtt_settings.keepalive = 60;
    
    // Configurazione logging di default
    g_config.fusion_logging.enabled = true;
    g_config.fusion_logging.level = "info";
    g_config.fusion_logging.file = "/var/log/fusion_sensor.log";
    g_config.fusion_logging.save_raw_data = false;
    g_config.fusion_logging.data_dir = "/var/lib/fusion_sensor";
    
    // Configurazione sicurezza di default
    g_config.safety.emergency_stop_enabled = true;
    g_config.safety.lift_sensor_enabled = true;
    g_config.safety.max_linear_velocity = 0.5;     // 0.5 m/s
    g_config.safety.max_angular_velocity = 1.0;    // 1.0 rad/s
    g_config.safety.max_tilt_angle = 15.0;         // 15 gradi
}

// Signal handler for clean shutdown
void signal_handler(int /*signum*/) {
    std::cout << "Shutting down..." << std::endl;
    g_running = false;
}

// Kalman Filter implementation
class KalmanFilter {
private:
    // State vector: [x, y, z, vx, vy, vz, roll, pitch, yaw]
    VectorXd x_;
    MatrixXd P_;  // State covariance
    MatrixXd Q_;  // Process noise covariance
    MatrixXd R_;  // Measurement noise covariance
    double last_update_time_;
    
public:
    KalmanFilter() {
        // Initialize state vector (9 states)
        x_ = VectorXd::Zero(9);
        
        // Initialize covariance matrices
        P_ = MatrixXd::Identity(9, 9) * 0.1;
        Q_ = MatrixXd::Identity(9, 9) * 0.01;
        R_ = MatrixXd::Identity(6, 6) * 0.1;  // For IMU measurements
        
        last_update_time_ = 0.0;
    }
    
    void predict(double dt) {
        // State transition matrix (simplified constant velocity model)
        MatrixXd F = MatrixXd::Identity(9, 9);
        F(0, 3) = dt; F(1, 4) = dt; F(2, 5) = dt;
        
        // Predict state
        x_ = F * x_;
        
        // Predict covariance
        P_ = F * P_ * F.transpose() + Q_;
        
        last_update_time_ += dt;
    }
    
    void update_imu(const VectorXd& z_imu) {
        // Measurement matrix for IMU (assuming z_imu contains [ax, ay, az, gx, gy, gz])
        MatrixXd H = MatrixXd::Zero(6, 9);
        H(0, 6) = 1.0;  // ax
        H(1, 7) = 1.0;  // ay
        H(2, 8) = 1.0;  // az
        H(3, 3) = 1.0;  // gx
        H(4, 4) = 1.0;  // gy
        H(5, 5) = 1.0;  // gz
        
        // Kalman update
        VectorXd y = z_imu - H * x_;
        MatrixXd S = H * P_ * H.transpose() + R_;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state and covariance
        x_ = x_ + K * y;
        P_ = (MatrixXd::Identity(9, 9) - K * H) * P_;
    }
    
    void update_gps(const Vector3d& position, double position_noise) {
        // Measurement matrix for GPS (position only)
        MatrixXd H = MatrixXd::Zero(3, 9);
        H(0, 0) = 1.0;  // x
        H(1, 1) = 1.0;  // y
        H(2, 2) = 1.0;  // z
        
        // Measurement noise
        MatrixXd R = MatrixXd::Identity(3, 3) * position_noise;
        
        // Kalman update
        VectorXd z = position;
        VectorXd y = z - H * x_;
        MatrixXd S = H * P_ * H.transpose() + R;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state and covariance
        x_ = x_ + K * y;
        P_ = (MatrixXd::Identity(9, 9) - K * H) * P_;
    }
    
    void update_odometry(double left_dist, double right_dist, double dt, double wheel_base) {
        if (dt <= 0) return;
        
        // Calculate linear and angular velocity from wheel distances
        double v = (right_dist + left_dist) / (2.0 * dt);
        double w = (right_dist - left_dist) / (wheel_base * dt);
        
        // Measurement matrix for odometry (velocity only)
        MatrixXd H = MatrixXd::Zero(2, 9);
        H(0, 3) = 1.0;  // vx
        H(1, 4) = 1.0;  // vy
        
        // Measurement noise
        MatrixXd R = MatrixXd::Identity(2, 2) * 0.1;  // Odometry noise
        
        // Convert to global frame using current yaw
        double yaw = x_(8);
        Vector2d velocity_measurement;
        velocity_measurement << v * cos(yaw), v * sin(yaw);
        
        // Kalman update
        VectorXd y = velocity_measurement - H * x_;
        MatrixXd S = H * P_ * H.transpose() + R;
        MatrixXd K = P_ * H.transpose() * S.inverse();
        
        // Update state and covariance
        x_ = x_ + K * y;
        P_ = (MatrixXd::Identity(9, 9) - K * H) * P_;
        
        // Also update yaw with angular velocity
        x_(8) += w * dt;
        // Normalize yaw to [-pi, pi]
        while (x_(8) > M_PI) x_(8) -= 2 * M_PI;
        while (x_(8) < -M_PI) x_(8) += 2 * M_PI;
    }
    
    // Add similar update methods for GPS, odometry, etc.
    
    VectorXd get_state() const { return x_; }
    MatrixXd get_covariance() const { return P_; }
};

// Global Kalman filter instance
KalmanFilter* g_kalman_filter = nullptr;

// Global variables for system status
typedef struct {
    bool comm_ok = false;
    double battery_voltage = 0.0;
    double battery_percent = 0.0;
    double cpu_temp = 0.0;
    uint32_t uptime_seconds = 0;
    uint32_t last_update = 0;
    std::string status = "disconnected";
} PicoStatus;

typedef struct {
    bool uart_connected = false;
    bool comm_ok = false;
    int fix_type = 0;
    int satellites_used = 0;
    double hdop = 0.0;
    uint32_t last_fix_time = 0;
    std::string module_type = "unknown";
    double update_rate = 1.0;
    uint32_t last_update = 0;
    std::string status = "disconnected";
} GpsStatus;

PicoStatus g_pico_status;
GpsStatus g_gps_status;

// Update Kalman filter noise parameters based on system status
void update_kalman_noise_models() {
    if (!g_kalman_filter) return;
    
    // Update GPS noise based on fix quality and HDOP
    double gps_position_noise = g_config.fusion_parameters.measurement_noise.gps_position;
    if (g_gps_status.fix_type < 2) { // No fix or 2D fix
        gps_position_noise *= 5.0; // Higher noise for poor/no fix
    } else if (g_gps_status.hdop > 2.0) { // Poor HDOP
        gps_position_noise *= 1.0 + (g_gps_status.hdop - 1.0); // Scale noise with HDOP
    }
    
    // Update IMU noise based on temperature (if available)
    // Nota: la variabile imu_noise_scale è stata rimossa in quanto non utilizzata
    // La logica di scaling del rumore IMU basata sulla temperatura
    // può essere implementata qui in futuro se necessario
    
    // TODO: Apply these noise parameters to the Kalman filter
    // This would require adding methods to KalmanFilter to adjust noise parameters
}

// MQTT callback implementations
void on_connect(struct mosquitto *mosq, void */*obj*/, int rc) {
    if (rc == 0) {
        std::cout << "Connected to MQTT broker" << std::endl;
        // Subscribe to topics using fusion_mqtt.h constants
        
        // Sensor data topics
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_IMU_DATA, 1);
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_GPS_DATA, 1);
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_ODOMETRY_DATA, 1);
        
        // Status topics
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_PICO_STATUS, 1);
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_GPS_STATUS, 1);
        
        // Vision topics
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_VISION_OBSTACLE, 1);
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_VISION_PERIMETER, 1);
        mosquitto_subscribe(mosq, NULL, FUSION_TOPIC_VISION_GRASS, 1);
    } else {
        std::cerr << "Failed to connect to MQTT broker: " << mosquitto_connack_string(rc) << std::endl;
    }
}

void on_message(struct mosquitto */*mosq*/, void */*obj*/, const struct mosquitto_message *msg) {
    // Parse incoming messages and update sensor data
    std::string topic(msg->topic);
    std::string payload(static_cast<char*>(msg->payload), msg->payloadlen);
    
    std::lock_guard<std::mutex> lock(g_mutex);
    
    // Handle PICO status messages
    if (topic == FUSION_TOPIC_PICO_STATUS) {
        struct json_object *root = json_tokener_parse(payload.c_str());
        if (!root) {
            std::cerr << "Failed to parse PICO status JSON" << std::endl;
            return;
        }
        
        // Parse system status
        struct json_object *system_obj;
        if (json_object_object_get_ex(root, "system", &system_obj)) {
            struct json_object *comm_ok_obj, *uptime_obj, *status_obj;
            
            if (json_object_object_get_ex(system_obj, "communication_ok", &comm_ok_obj)) {
                g_pico_status.comm_ok = json_object_get_boolean(comm_ok_obj);
            }
            
            if (json_object_object_get_ex(system_obj, "uptime_seconds", &uptime_obj)) {
                g_pico_status.uptime_seconds = json_object_get_int64(uptime_obj);
            }
            
            if (json_object_object_get_ex(system_obj, "status", &status_obj)) {
                g_pico_status.status = json_object_get_string(status_obj);
            }
        }
        
        // Parse battery status
        struct json_object *battery_obj;
        if (json_object_object_get_ex(root, "battery", &battery_obj)) {
            struct json_object *voltage_obj, *percent_obj;
            
            if (json_object_object_get_ex(battery_obj, "voltage", &voltage_obj)) {
                g_pico_status.battery_voltage = json_object_get_double(voltage_obj);
            }
            
            if (json_object_object_get_ex(battery_obj, "percentage", &percent_obj)) {
                g_pico_status.battery_percent = json_object_get_double(percent_obj);
            }
        }
        
        // Parse performance metrics
        struct json_object *perf_obj;
        if (json_object_object_get_ex(root, "performance", &perf_obj)) {
            struct json_object *temp_obj;
            if (json_object_object_get_ex(perf_obj, "cpu_temperature", &temp_obj)) {
                g_pico_status.cpu_temp = json_object_get_double(temp_obj);
            }
        }
        
        g_pico_status.last_update = time(nullptr);
        json_object_put(root);
        
        // Update Kalman filter noise models based on new status
        update_kalman_noise_models();
        
        // Debug output
        std::cout << "Updated PICO status - Battery: " << g_pico_status.battery_voltage 
                  << "V (" << g_pico_status.battery_percent << "%), "
                  << "CPU Temp: " << g_pico_status.cpu_temp << "°C, "
                  << "Status: " << g_pico_status.status << std::endl;
        
        return;
    }
    
    // Handle GPS status messages
    if (topic == FUSION_TOPIC_GPS_STATUS) {
        struct json_object *root = json_tokener_parse(payload.c_str());
        if (!root) {
            std::cerr << "Failed to parse GPS status JSON" << std::endl;
            return;
        }
            
            // Parse system status
            struct json_object *system_obj;
            if (json_object_object_get_ex(root, "system", &system_obj)) {
                struct json_object *uart_conn_obj, *comm_ok_obj, *fix_time_obj, *status_obj, *module_obj;
                
                if (json_object_object_get_ex(system_obj, "uart_connected", &uart_conn_obj)) {
                    g_gps_status.uart_connected = json_object_get_boolean(uart_conn_obj);
                }
                
                if (json_object_object_get_ex(system_obj, "communication_ok", &comm_ok_obj)) {
                    g_gps_status.comm_ok = json_object_get_boolean(comm_ok_obj);
                }
                
                if (json_object_object_get_ex(system_obj, "last_fix_time", &fix_time_obj)) {
                    g_gps_status.last_fix_time = json_object_get_int64(fix_time_obj);
                }
                
                if (json_object_object_get_ex(system_obj, "status", &status_obj)) {
                    g_gps_status.status = json_object_get_string(status_obj);
                }
                
                if (json_object_object_get_ex(system_obj, "module_type", &module_obj)) {
                    g_gps_status.module_type = json_object_get_string(module_obj);
                }
            }
            
            // Parse GPS info
            struct json_object *gps_info_obj;
            if (json_object_object_get_ex(root, "gps", &gps_info_obj)) {
                struct json_object *fix_type_obj, *sats_obj, *hdop_obj;
                
                if (json_object_object_get_ex(gps_info_obj, "fix_type", &fix_type_obj)) {
                    g_gps_status.fix_type = json_object_get_int(fix_type_obj);
                }
                
                if (json_object_object_get_ex(gps_info_obj, "satellites_used", &sats_obj)) {
                    g_gps_status.satellites_used = json_object_get_int(sats_obj);
                }
                
                if (json_object_object_get_ex(gps_info_obj, "hdop", &hdop_obj)) {
                    g_gps_status.hdop = json_object_get_double(hdop_obj);
                }
            }
            
            // Parse performance metrics
            struct json_object *perf_obj;
            if (json_object_object_get_ex(root, "performance", &perf_obj)) {
                struct json_object *update_rate_obj;
                if (json_object_object_get_ex(perf_obj, "update_rate_hz", &update_rate_obj)) {
                    g_gps_status.update_rate = json_object_get_double(update_rate_obj);
                }
            }
            
            g_gps_status.last_update = time(nullptr);
            json_object_put(root);
            
            // Update Kalman filter noise models based on new status
            update_kalman_noise_models();
            
            // Debug output
            std::cout << "Updated GPS status - Fix: " << g_gps_status.fix_type 
                      << ", Sats: " << g_gps_status.satellites_used 
                      << ", HDOP: " << g_gps_status.hdop 
                      << ", Status: " << g_gps_status.status << std::endl;
            
            return;
    }
    
    if (topic == FUSION_TOPIC_IMU_DATA) {
        // Parse Pico sensor data JSON structure
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            json_object* type_obj;
            if (json_object_object_get_ex(root, "type", &type_obj) && 
                strcmp(json_object_get_string(type_obj), "sensor_data") == 0) {
                
                // Parse IMU data [ax, ay, az, gx, gy, gz]
                json_object* imu_obj;
                if (json_object_object_get_ex(root, "imu", &imu_obj)) {
                    int imu_len = json_object_array_length(imu_obj);
                    if (imu_len == 6) {
                        VectorXd imu_data(6);
                        for (int i = 0; i < 6; i++) {
                            json_object* val = json_object_array_get_idx(imu_obj, i);
                            imu_data[i] = json_object_get_double(val);
                        }
                        
                        // Applica calibrazione se disponibile
                        if (imu_data.size() >= 6) {
                            for (int i = 0; i < 3; ++i) {
                                imu_data(i) -= g_config.imu.calibration.accel_offset[i];
                            }
                            for (int i = 0; i < 3; ++i) {
                                imu_data(i+3) -= g_config.imu.calibration.gyro_offset[i];
                            }
                        }
                        
                        // Aggiorna il filtro di Kalman con i dati IMU
                        if (g_kalman_filter) {
                            g_kalman_filter->update_imu(imu_data);
                            std::cout << "Updated Kalman filter with IMU data" << std::endl;
                        } else {
                            std::cout << "Received IMU data: [" << imu_data.transpose() << "] (filter not initialized)" << std::endl;
                        }
                    }
                }
                
                // Parse magnetometer data [mx, my, mz]
                json_object* mag_obj;
                if (json_object_object_get_ex(root, "magnetometer", &mag_obj)) {
                    int mag_len = json_object_array_length(mag_obj);
                    if (mag_len == 3) {
                        Vector3d mag_data;
                        for (int i = 0; i < 3; i++) {
                            json_object* val = json_object_array_get_idx(mag_obj, i);
                            mag_data[i] = json_object_get_double(val);
                        }
                        // TODO: Update Kalman filter with magnetometer data
                        std::cout << "Received Magnetometer data: [" << mag_data.transpose() << "]" << std::endl;
                    }
                }
            }
            json_object_put(root);
        }
        
    } else if (topic == FUSION_TOPIC_GPS_DATA) {
        // Parse GPS data JSON structure
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            json_object* type_obj;
            if (json_object_object_get_ex(root, "type", &type_obj) && 
                strcmp(json_object_get_string(type_obj), "gps_data") == 0) {
                
                // Parse GPS position
                json_object* pos_obj;
                if (json_object_object_get_ex(root, "position", &pos_obj)) {
                    json_object* lat_obj, *lon_obj, *alt_obj;
                    if (json_object_object_get_ex(pos_obj, "latitude", &lat_obj) &&
                        json_object_object_get_ex(pos_obj, "longitude", &lon_obj) &&
                        json_object_object_get_ex(pos_obj, "altitude", &alt_obj)) {
                        
                        double lat = json_object_get_double(lat_obj);
                        double lon = json_object_get_double(lon_obj);
                        double alt = json_object_get_double(alt_obj);
                        
                        // Converti da lat/lon a coordinate locali (semplificato)
                        // Nota: in un'implementazione reale, usa una proiezione appropriata
                        Vector3d position;
                        position << lat * 111320.0, lon * 111320.0 * cos(lat * M_PI/180.0), alt;
                        
                        // Aggiorna il filtro di Kalman con la posizione GPS
                        if (g_kalman_filter) {
                            double gps_noise = g_config.fusion_parameters.measurement_noise.gps_position;
                            g_kalman_filter->update_gps(position, gps_noise);
                            std::cout << "Updated Kalman filter with GPS position" << std::endl;
                        } else {
                            std::cout << "Received GPS position: lat=" << lat << ", lon=" << lon << ", alt=" << alt << " (filter not initialized)" << std::endl;
                        }
                    }
                }
                
                // Parse GPS quality
                json_object* quality_obj;
                if (json_object_object_get_ex(root, "quality", &quality_obj)) {
                    json_object* fix_obj, *sat_obj, *hdop_obj;
                    if (json_object_object_get_ex(quality_obj, "fix_type", &fix_obj) &&
                        json_object_object_get_ex(quality_obj, "satellites", &sat_obj) &&
                        json_object_object_get_ex(quality_obj, "hdop", &hdop_obj)) {
                        
                        int fix_type = json_object_get_int(fix_obj);
                        int satellites = json_object_get_int(sat_obj);
                        double hdop = json_object_get_double(hdop_obj);
                        
                        std::cout << "GPS quality: fix=" << fix_type << ", sats=" << satellites << ", hdop=" << hdop << std::endl;
                    }
                }
            }
            json_object_put(root);
        }
        
    } else if (topic == FUSION_TOPIC_ODOMETRY_DATA) {
        // Parse Pico odometry data JSON structure for encoder counts
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            json_object* type_obj;
            if (json_object_object_get_ex(root, "type", &type_obj) && 
                strcmp(json_object_get_string(type_obj), "odometry_data") == 0) {
                
                // Parse encoder counts [left, right, blade1, blade2]
                json_object* encoders_obj;
                if (json_object_object_get_ex(root, "encoders", &encoders_obj)) {
                    int enc_len = json_object_array_length(encoders_obj);
                    if (enc_len >= 2) {
                        json_object* left_enc = json_object_array_get_idx(encoders_obj, 0);
                        json_object* right_enc = json_object_array_get_idx(encoders_obj, 1);
                        
                        uint32_t left_ticks = json_object_get_int64(left_enc);
                        uint32_t right_ticks = json_object_get_int64(right_enc);
                        
                        // Calcola le distanze dai tick degli encoder
                        static uint32_t prev_left_ticks = 0;
                        static uint32_t prev_right_ticks = 0;
                        static auto last_odom_time = high_resolution_clock::now();
                        
                        auto now = high_resolution_clock::now();
                        double dt = duration_cast<milliseconds>(now - last_odom_time).count() / 1000.0;
                        
                        if (dt > 0.001) {  // Evita divisioni per zero
                            // Calcola la differenza nei tick
                            int32_t delta_left = left_ticks - prev_left_ticks;
                            int32_t delta_right = right_ticks - prev_right_ticks;
                            
                            // Converti i tick in distanza
                            double left_dist = (2 * M_PI * g_config.odometry.wheel_radius * delta_left) / 
                                              g_config.odometry.ticks_per_revolution;
                            double right_dist = (2 * M_PI * g_config.odometry.wheel_radius * delta_right) / 
                                               g_config.odometry.ticks_per_revolution;
                            
                            // Aggiorna il filtro di Kalman con i dati odometrici
                            if (g_kalman_filter) {
                                g_kalman_filter->update_odometry(left_dist, right_dist, dt, g_config.odometry.wheel_base);
                                std::cout << "Updated Kalman filter with odometry data" << std::endl;
                            } else {
                                std::cout << "Received encoder ticks: left=" << left_ticks << ", right=" << right_ticks << " (filter not initialized)" << std::endl;
                            }
                            
                            // Aggiorna i valori precedenti
                            prev_left_ticks = left_ticks;
                            prev_right_ticks = right_ticks;
                            last_odom_time = now;
                        }
                    }
                }
                
                // Parse motor RPM for velocity estimation
                json_object* motors_obj;
                if (json_object_object_get_ex(root, "motors", &motors_obj)) {
                    json_object* left_rpm_obj, *right_rpm_obj;
                    if (json_object_object_get_ex(motors_obj, "left_rpm", &left_rpm_obj) &&
                        json_object_object_get_ex(motors_obj, "right_rpm", &right_rpm_obj)) {
                        
                        double left_rpm = json_object_get_double(left_rpm_obj);
                        double right_rpm = json_object_get_double(right_rpm_obj);
                        
                        std::cout << "Motor RPM: left=" << left_rpm << ", right=" << right_rpm << std::endl;
                    }
                }
            }
            json_object_put(root);
        }
        
    } else if (topic == FUSION_TOPIC_VISION_OBSTACLE) {
        // Parse vision obstacle detection JSON structure
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            // TODO: Parse obstacle detection results based on vision_mqtt.h structure
            std::cout << "Received vision obstacle data" << std::endl;
            json_object_put(root);
        }
        
    } else if (topic == FUSION_TOPIC_VISION_PERIMETER) {
        // Parse vision perimeter detection JSON structure
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            // TODO: Parse perimeter detection results based on vision_mqtt.h structure
            std::cout << "Received vision perimeter data" << std::endl;
            json_object_put(root);
        }
        
    } else if (topic == FUSION_TOPIC_VISION_GRASS) {
        // Parse vision grass detection JSON structure
        json_object* root = json_tokener_parse(payload.c_str());
        if (root) {
            // TODO: Parse grass detection results based on vision_mqtt.h structure
            std::cout << "Received vision grass data" << std::endl;
            json_object_put(root);
        }
    }
}

// Main function
int main(int argc, char **argv) {
    // Set up signal handlers
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);
    
    // Load configuration from centralized config
    std::string config_file = "/opt/smartmower/etc/robot_config.json";
    if (argc > 1) {
        config_file = argv[1];
    }
    
    if (!load_config(config_file)) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }
    
    // Initialize MQTT
    mosquitto_lib_init();
    struct mosquitto *mosq = mosquitto_new("fusion_node", true, NULL);
    if (!mosq) {
        std::cerr << "Error: Out of memory when creating mosquitto client" << std::endl;
        return 1;
    }
    
    // Set MQTT callbacks
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);
    
    // Set MQTT authentication if credentials are provided
    if (!g_config.mqtt_settings.username.empty()) {
        if (mosquitto_username_pw_set(mosq, g_config.mqtt_settings.username.c_str(), 
                                     g_config.mqtt_settings.password.c_str()) != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to set MQTT authentication" << std::endl;
            return 1;
        }
    }
    
    // Connect to MQTT broker using config parameters
    if (mosquitto_connect(mosq, g_config.mqtt_settings.broker_host.c_str(), 
                         g_config.mqtt_settings.broker_port, 
                         g_config.mqtt_settings.keepalive) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Unable to connect to MQTT broker" << std::endl;
        return 1;
    }
    
    // Start MQTT loop in a separate thread
    mosquitto_loop_start(mosq);
    
    // Initialize global Kalman filter
    g_kalman_filter = new KalmanFilter();
    std::cout << "Kalman filter initialized" << std::endl;
    
    // Main loop
    auto last_update = high_resolution_clock::now();
    auto last_status_publish = high_resolution_clock::now();
    
    // Publish initial status message
    std::string status_topic = std::string(FUSION_MQTT_BASE_TOPIC) + FUSION_TOPIC_STATUS;
    json_object *j_initial_status = json_object_new_object();
    json_object_object_add(j_initial_status, "type", json_object_new_string(FUSION_JSON_STATUS));
    json_object *j_system = json_object_new_object();
    json_object_object_add(j_system, "running", json_object_new_boolean(true));
    json_object_object_add(j_system, "initialized", json_object_new_boolean(true));
    json_object_object_add(j_initial_status, "system", j_system);
    json_object_object_add(j_initial_status, "timestamp", json_object_new_int64(time(NULL)));
    
    const char *initial_status_json = json_object_to_json_string(j_initial_status);
    mosquitto_publish(mosq, NULL, status_topic.c_str(), 
                     strlen(initial_status_json), initial_status_json, 1, true);
    json_object_put(j_initial_status);
    
    std::cout << "Fusion system initialized and status published" << std::endl;
    
    while (g_running) {
        auto now = high_resolution_clock::now();
        auto dt = duration_cast<milliseconds>(now - last_update).count() / 1000.0;
        last_update = now;
        
        // Predict state usando il filtro globale
        std::lock_guard<std::mutex> lock(g_mutex);
        if (g_kalman_filter) {
            g_kalman_filter->predict(dt);
        }
        
        // Get current state
        VectorXd state = g_kalman_filter ? g_kalman_filter->get_state() : VectorXd::Zero(9);
        MatrixXd covariance = g_kalman_filter ? g_kalman_filter->get_covariance() : MatrixXd::Identity(9, 9);
        
        // Publish complete state with all Kalman filter estimates
        json_object *j_state = json_object_new_object();
        json_object_object_add(j_state, "type", json_object_new_string(FUSION_JSON_DATA));
        json_object_object_add(j_state, "timestamp", json_object_new_int64(duration_cast<milliseconds>(now.time_since_epoch()).count()));
        
        // Position (x, y, z)
        json_object *j_position = json_object_new_object();
        json_object_object_add(j_position, "x", json_object_new_double(state(0)));
        json_object_object_add(j_position, "y", json_object_new_double(state(1)));
        json_object_object_add(j_position, "z", json_object_new_double(state(2)));
        json_object_object_add(j_state, "position", j_position);
        
        // Velocity (vx, vy, vz)
        json_object *j_velocity = json_object_new_object();
        json_object_object_add(j_velocity, "vx", json_object_new_double(state(3)));
        json_object_object_add(j_velocity, "vy", json_object_new_double(state(4)));
        json_object_object_add(j_velocity, "vz", json_object_new_double(state(5)));
        // Calculate speed magnitude
        double speed = sqrt(state(3)*state(3) + state(4)*state(4) + state(5)*state(5));
        json_object_object_add(j_velocity, "speed", json_object_new_double(speed));
        json_object_object_add(j_state, "velocity", j_velocity);
        
        // Orientation (roll, pitch, yaw)
        json_object *j_orientation = json_object_new_object();
        json_object_object_add(j_orientation, "roll", json_object_new_double(state(6)));
        json_object_object_add(j_orientation, "pitch", json_object_new_double(state(7)));
        json_object_object_add(j_orientation, "yaw", json_object_new_double(state(8)));
        json_object_object_add(j_state, "orientation", j_orientation);
        
        // Uncertainty/Covariance (diagonal elements for key states)
        json_object *j_uncertainty = json_object_new_object();
        json_object_object_add(j_uncertainty, "position_x", json_object_new_double(sqrt(covariance(0,0))));
        json_object_object_add(j_uncertainty, "position_y", json_object_new_double(sqrt(covariance(1,1))));
        json_object_object_add(j_uncertainty, "velocity_x", json_object_new_double(sqrt(covariance(3,3))));
        json_object_object_add(j_uncertainty, "velocity_y", json_object_new_double(sqrt(covariance(4,4))));
        json_object_object_add(j_uncertainty, "yaw", json_object_new_double(sqrt(covariance(8,8))));
        json_object_object_add(j_state, "uncertainty", j_uncertainty);
        
        const char *state_json = json_object_to_json_string(j_state);
        mosquitto_publish(mosq, NULL, "smartmower/fusion/data", 
                         strlen(state_json), state_json, 1, true);
        
        json_object_put(j_state);
        
        // Publish status every 5 seconds
        auto status_elapsed = duration_cast<seconds>(now - last_status_publish).count();
        if (status_elapsed >= 5) {
            std::string status_topic = std::string(FUSION_MQTT_BASE_TOPIC) + FUSION_TOPIC_STATUS;
            json_object *j_status = json_object_new_object();
            json_object_object_add(j_status, "type", json_object_new_string(FUSION_JSON_STATUS));
            json_object *j_system = json_object_new_object();
            json_object_object_add(j_system, "running", json_object_new_boolean(true));
            json_object_object_add(j_system, "initialized", json_object_new_boolean(true));
            json_object_object_add(j_status, "system", j_system);
            json_object_object_add(j_status, "timestamp", json_object_new_int64(time(NULL)));
            
            const char *status_json = json_object_to_json_string(j_status);
            mosquitto_publish(mosq, NULL, status_topic.c_str(), 
                             strlen(status_json), status_json, 1, true);
            json_object_put(j_status);
            
            last_status_publish = now;
        }
        
        // Sleep to maintain desired update rate
        std::this_thread::sleep_for(milliseconds(10)); // 100 Hz
    }
    
    // Cleanup
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    // Cleanup Kalman filter
    if (g_kalman_filter) {
        delete g_kalman_filter;
        g_kalman_filter = nullptr;
    }
    
    std::cout << "Fusion node stopped" << std::endl;
    return 0;
}
