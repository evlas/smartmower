#ifndef SFM_OBSTACLE_DETECTOR_H
#define SFM_OBSTACLE_DETECTOR_H

#include <string>
#include <vector>
#include <memory>
#include <atomic>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <chrono>
#include <fstream>
#include <cmath>
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <mosquitto.h>
#include <cjson/cJSON.h>

/**
 * Configuration structure
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
 * Thread-safe frame buffer
 */
class FrameBuffer {
private:
    cv::Mat frame_;
    std::chrono::system_clock::time_point timestamp_;
    mutable std::mutex mutex_;
    std::condition_variable cv_;
    bool has_new_frame_ = false;
    
public:
    void setFrame(const cv::Mat& frame, const std::chrono::system_clock::time_point& timestamp);
    bool getFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp);
    bool waitForFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp, 
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(100));
};

/**
 * Thread-safe velocity manager
 */
class VelocityManager {
private:
    std::atomic<double> current_velocity_{0.5}; // Default velocity
    mutable std::mutex mutex_;
    std::chrono::system_clock::time_point last_update_;
    
public:
    void setVelocity(double velocity);
    double getVelocity() const;
    bool isVelocityRecent(std::chrono::seconds max_age = std::chrono::seconds(5)) const;
};

/**
 * Tracked point structure
 */
struct TrackedPoint {
    cv::Point2f current_pos;
    cv::Point2f previous_pos;
    double distance_estimate = 0.0;
    int frames_tracked = 0;
    bool is_valid = true;
    
    double getDisplacement() const {
        cv::Point2f diff = current_pos - previous_pos;
        return std::sqrt(diff.x * diff.x + diff.y * diff.y);
    }
};

/**
 * Main SfM Obstacle Detector class - Refactored
 */
class SfMObstacleDetector {
public:
    SfMObstacleDetector();
    ~SfMObstacleDetector();
    
    bool loadConfigFromFile(const std::string& filename);
    bool initialize();
    bool setupMQTT();
    void start();
    void stop();

private:
    // Membri atomici e di sincronizzazione - devono essere i primi per l'ordine di inizializzazione
    std::atomic<bool> running_{false};
    
    // Contatori e statistiche - inizializzati nel costruttore
    int frame_count_ = 0;
    struct {
        std::atomic<int> frames_processed{0};
        std::atomic<int> obstacles_detected{0};
        std::atomic<double> avg_processing_time{0.0};
    } stats_;
    
    // Gestione MQTT
    struct mosquitto* mosq_ = nullptr;
    
    // Configurazione
    Config config_;
    
    // Threading
    std::thread processing_thread_;
    
    // Gestione frame e velocità
    FrameBuffer frame_buffer_;
    VelocityManager velocity_manager_;
    cv::Mat previous_frame_;
    std::chrono::system_clock::time_point previous_timestamp_;
    std::vector<TrackedPoint> tracked_points_;
    
    // Funzioni MQTT
    void onMQTTMessage(const struct mosquitto_message* message);
    void onMQTTDisconnect(int result);
    void handleMQTTConnect(struct mosquitto* mosq, int result);
    void handleVelocityMessage(const struct mosquitto_message* message);
    void handleImageMessage(const struct mosquitto_message* message);
    
    // Funzioni di elaborazione
    void processingLoop();
    void processFrame(const cv::Mat& current_frame, 
                     const std::chrono::system_clock::time_point& current_timestamp,
                     const cv::Mat& previous_frame,
                     const std::chrono::system_clock::time_point& previous_timestamp);
    void detectFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points);
    void publishObstacles(const std::vector<TrackedPoint>& obstacles, double velocity);
    void ensureConnected();
    void printStatistics();
    void publishDetectionResults();
    
    // Wrapper statici per le callback MQTT
    static void onMQTTConnectWrapper(struct mosquitto* mosq, void* userdata, int result);
    static void onMQTTMessageWrapper(struct mosquitto* mosq, void* userdata, 
                                   const struct mosquitto_message* message);
    static void onMQTTDisconnectWrapper(struct mosquitto* mosq, void* userdata, int result);
};

#endif // SFM_OBSTACLE_DETECTOR_H
