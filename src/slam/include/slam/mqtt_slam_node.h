#pragma once

#include "slam/sensor_fusion.h"
#include "slam/visual_odometry.h"
#include "slam/obstacle_detection.h"
#include "mqtt/mqtt_client.h"
#include "pico/pico_protocol.h"
#include <memory>
#include <string>
#include <atomic>
#include <thread>

namespace slam {

/**
 * @brief Nodo SLAM che si integra con MQTT
 */
class MqttSlamNode {
public:
    MqttSlamNode(const std::string& config_file);
    ~MqttSlamNode();

    /**
     * @brief Avvia il nodo SLAM
     */
    void run();

    /**
     * @brief Ferma il nodo SLAM
     */
    void stop();

private:
    // Configurazione
    std::string config_file_;
    
    // Componenti principali
    std::unique_ptr<SensorFusion> sensor_fusion_;
    std::unique_ptr<mqtt::MqttClient> mqtt_client_;
    std::unique_ptr<VisualOdometry> visual_odometry_;
    std::unique_ptr<ObstacleDetector> obstacle_detector_;
    
    // Thread per l'elaborazione
    std::atomic<bool> running_{false};
    std::thread processing_thread_;
    
    // Configurazione MQTT
    struct MqttConfig {
        std::string broker;
        int port;
        std::string client_id;
        std::string username;
        std::string password;
        
        // Topic di sottoscrizione
        std::string imu_topic;
        std::string gps_topic;
        std::string camera_topic;   // opzionale
        std::string odometry_topic; // opzionale
        std::string data_topic;  // Topic aggregato dati dal bridge pico (.../data)
        std::string obstacle_topic; // Vision obstacle detection
        std::string status_topic;   // Topic status del bridge pico (.../status)
        
        // Topic di pubblicazione
        std::string slam_pose_topic;
        std::string slam_map_topic;
    } mqtt_config_;
    
    // Parametri e stato per odometria ruote (Pico)
    struct PicoOdomParams {
        double wheel_radius_m = 0.1;     // raggio ruota [m]
        double wheel_base_m = 0.45;      // carreggiata [m]
        int encoder_cpr = 2048;          // counts per revolution encoder
        int left_motor_index = 0;        // indice nel vettore motors per ruota sinistra
        int right_motor_index = 1;       // indice nel vettore motors per ruota destra
        int encoder_polarity_left = 1;   // +1 o -1
        int encoder_polarity_right = 1;  // +1 o -1
    } pico_odom_params_;

    struct PicoEncoderState {
        bool has_prev = false;
        int64_t prev_left = 0;
        int64_t prev_right = 0;
        uint64_t prev_ts_ms = 0; // timestamp in ms dal payload pico/status
    } pico_enc_state_;

    // Metodi di inizializzazione
    bool loadConfig();
    void initMqtt();
    
    // Callback MQTT
    void onMessage(const std::string& topic, const std::vector<uint8_t>& payload);
    void onDataMessage(const std::vector<uint8_t>& payload);
    void onImuMessage(const std::vector<uint8_t>& payload);
    void onGpsMessage(const std::vector<uint8_t>& payload);
    void onCameraMessage(const std::vector<uint8_t>& payload);
    void onOdometryMessage(const std::vector<uint8_t>& payload);
    void onObstacleMessage(const std::vector<uint8_t>& payload);
    void onPicoStatusMessage(const std::vector<uint8_t>& payload);
    
    // Pubblicazione risultati
    void publishPose(const State& state);
    void publishMap(const std::vector<Vector3d>& trajectory);
    
    // Thread di elaborazione
    void processingLoop();
};

} // namespace slam

