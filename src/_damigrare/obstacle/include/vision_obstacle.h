/**
 * Vision Obstacle Detector - Header File
 * 
 * Questo file contiene le dichiarazioni per il modulo di rilevamento ostacoli.
 * Implementa il rilevamento degli ostacoli utilizzando il flusso ottico e la triangolazione.
 * 
 * Autore: SmartMower Vision System
 * Data: 2025-08-07
 * Versione: 2.0
 */

#ifndef VISION_OBSTACLE_H
#define VISION_OBSTACLE_H

#include <opencv2/opencv.hpp>
#include <string>
#include <vector>
#include <memory>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <chrono>

// Forward declarations
struct mosquitto;

namespace vision_obstacle {

/**
 * @brief Struttura per i punti tracciati
 */
struct TrackedPoint {
    cv::Point2f current_pos;
    cv::Point2f previous_pos;
    double distance_estimate = 0.0;
    int frames_tracked = 0;
    bool is_valid = true;
    
    /**
     * @brief Calcola lo spostamento del punto
     * @return La distanza euclidea tra la posizione corrente e quella precedente
     */
    double getDisplacement() const {
        cv::Point2f diff = current_pos - previous_pos;
        return std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
};

/**
 * @brief Buffer thread-safe per i frame
 */
class FrameBuffer {
private:
    cv::Mat frame_;
    std::chrono::system_clock::time_point timestamp_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool has_new_frame_ = false;

public:
    /**
     * @brief Imposta un nuovo frame nel buffer
     * @param frame Il frame da impostare
     * @param timestamp Il timestamp del frame
     */
    void setFrame(const cv::Mat& frame, const std::chrono::system_clock::time_point& timestamp);
    
    /**
     * @brief Ottiene l'ultimo frame disponibile
     * @param frame Variabile in cui salvare il frame
     * @param timestamp Timestamp del frame
     * @return true se è stato ottenuto un frame valido, false altrimenti
     */
    bool getFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp);
    
    /**
     * @brief Attende un nuovo frame per un massimo di timeout millisecondi
     * @param frame Variabile in cui salvare il frame
     * @param timestamp Timestamp del frame
     * @param timeout Timeout di attesa in millisecondi
     * @return true se è stato ottenuto un frame valido, false in caso di timeout
     */
    bool waitForFrame(cv::Mat& frame, 
                     std::chrono::system_clock::time_point& timestamp, 
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
};

/**
 * @brief Gestore thread-safe della velocità
 */
class VelocityManager {
private:
    std::atomic<double> current_velocity_{0.5};
    mutable std::mutex mutex_;
    std::chrono::system_clock::time_point last_update_;

public:
    /**
     * @brief Imposta la velocità corrente
     * @param velocity La nuova velocità in m/s
     */
    void setVelocity(double velocity);
    
    /**
     * @brief Ottiene la velocità corrente
     * @return La velocità corrente in m/s
     */
    double getVelocity() const;
    
    /**
     * @brief Verifica se la velocità è stata aggiornata di recente
     * @param max_age Età massima consentita in secondi
     * @return true se la velocità è stata aggiornata di recente, false altrimenti
     */
    bool isVelocityRecent(std::chrono::seconds max_age = std::chrono::seconds(5)) const;
};

/**
 * @brief Decoder Base64
 */
class Base64Decoder {
public:
    /**
     * @brief Decodifica una stringa base64
     * @param base64 La stringa base64 da decodificare
     * @return Un vettore di byte con i dati decodificati
     */
    static std::vector<uchar> decode(const std::string& base64);
};

/**
 * @brief Struttura di configurazione
 */
struct Config {
    // MQTT settings
    std::string mqtt_broker;
    int mqtt_port;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_camera_topic;
    std::string mqtt_velocity_topic;
    std::string mqtt_obstacles_topic;
    int mqtt_qos;
    bool mqtt_retain;
    
    // Camera calibration
    double focal_length_x;
    double focal_length_y;
    double principal_point_x;
    double principal_point_y;
    
    // Robot parameters
    double camera_height;
    double max_detection_range;
    double min_obstacle_distance;
    
    // SfM parameters
    int max_corners;
    double quality_level;
    double min_distance;
    int block_size;
    double harris_k;
    
    // Obstacle detection
    int min_frames_tracked;
    double max_optical_flow_error;
    int min_points_threshold;
    double displacement_threshold;
    double publish_threshold;
    
    // Debug
    bool debug_enabled;
};

/**
 * @brief Rilevatore di ostacoli basato su Structure from Motion (SfM)
 */
class SfMObstacleDetector {
public:
    /**
     * @brief Costruttore
     */
    SfMObstacleDetector();
    
    /**
     * @brief Distruttore
     */
    ~SfMObstacleDetector();
    
    /**
     * @brief Inizializza il rilevatore
     * @return true se l'inizializzazione è riuscita, false altrimenti
     */
    bool initialize();
    
    /**
     * @brief Avvia il rilevamento ostacoli
     */
    void start();
    
    /**
     * @brief Arresta il rilevamento ostacoli
     */
    void stop();
    
    /**
     * @brief Verifica se il rilevatore è in esecuzione
     * @return true se è in esecuzione, false altrimenti
     */
    bool isRunning() const;

private:
    // MQTT client
    struct mosquitto* mqtt_client_;
    
    // Configurazione
    Config config_;
    
    // Threading
    std::thread processing_thread_;
    
    // Gestione frame e velocità
    FrameBuffer frame_buffer_;
    VelocityManager velocity_manager_;
    std::atomic<bool> running_{false};
    int frame_count_{0};
    
    // Statistiche
    struct {
        std::atomic<int> frames_processed{0};
        std::atomic<int> obstacles_detected{0};
        std::atomic<double> avg_processing_time{0.0};
    } stats_;
    
    // Metodi privati
    bool setupMQTT();
    void processingLoop();
    void processFrame(const cv::Mat& current_frame, 
                     const std::chrono::system_clock::time_point& current_timestamp,
                     const cv::Mat& previous_frame,
                     const std::chrono::system_clock::time_point& previous_timestamp);
    void detectFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points);
    void publishObstacles(const std::vector<TrackedPoint>& obstacles, double velocity);
    void ensureConnected();
    void printStatistics();
    
    // Gestione messaggi MQTT
    void handleVelocityMessage(const struct mosquitto_message* message);
    void handleImageMessage(const struct mosquitto_message* message);
    
    // Callback MQTT statiche
    static void onMQTTConnect(struct mosquitto* mosq, void* userdata, int result);
    static void onMQTTMessage(struct mosquitto* mosq, void* userdata, const struct mosquitto_message* message);
    static void onMQTTDisconnect(struct mosquitto* mosq, void* userdata, int result);
};

} // namespace vision_obstacle

#endif // VISION_OBSTACLE_H
