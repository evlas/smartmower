/**
 * Structure from Motion (SfM) Obstacle Detector - Refactored Version
 * 
 * Implementa il rilevamento ostacoli usando il flusso ottico e la triangolazione.
 * Principio: oggetti più vicini si muovono più velocemente nell'immagine.
 * 
 * Autore: SmartMower Vision System
 * Data: 2025-07-24
 * Versione: 2.0 - Refactored and Optimized
 */

#include <iostream>
#include <vector>
#include <queue>
#include <string>
#include <cmath>
#include <chrono>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>
#include <fstream>
#include <memory>
#include <csignal>
#include <cstdlib>
#include <algorithm>

// OpenCV headers
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// MQTT and JSON
#include <mosquitto.h>
#include <cjson/cJSON.h>

// Vision MQTT definitions
#include "vision_mqtt.h"

/**
 * Configuration structure
 */
struct Config {
    // MQTT settings (read from unified configuration)
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_topic = "smartmower/vision/camera";
    std::string mqtt_velocity_topic = "smartmower/fusion/data";
    
    // Camera calibration
    double focal_length_x = 350.0;
    double focal_length_y = 350.0;
    double principal_point_x = 320.0;
    double principal_point_y = 240.0;
    
    // Robot parameters
    double camera_height = 0.3;
    double max_detection_range = 5.0;
    
    // SfM parameters
    int max_corners = 150;
    double quality_level = 0.005;
    double min_distance = 15.0;
    int block_size = 5;
    double harris_k = 0.04;
    
    // Obstacle detection
    double max_obstacle_distance = 7.0;
    double min_obstacle_distance = 0.1;
    int min_frames_tracked = 2;
    double max_optical_flow_error = 150.0;
    int min_points_threshold = 70;
    double displacement_threshold = 0.08;
    
    // Debug
    bool debug_enabled = true;
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
    void setFrame(const cv::Mat& frame, const std::chrono::system_clock::time_point& timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        frame.copyTo(frame_);
        timestamp_ = timestamp;
        has_new_frame_ = true;
        cv_.notify_one();
    }
    
    bool getFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp) {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!has_new_frame_ || frame_.empty()) {
            return false;
        }
        frame_.copyTo(frame);
        timestamp = timestamp_;
        has_new_frame_ = false;
        return true;
    }
    
    bool waitForFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp, 
                     std::chrono::milliseconds timeout = std::chrono::milliseconds(100)) {
        std::unique_lock<std::mutex> lock(mutex_);
        if (cv_.wait_for(lock, timeout, [this] { return has_new_frame_; })) {
            if (!frame_.empty()) {
                frame_.copyTo(frame);
                timestamp = timestamp_;
                has_new_frame_ = false;
                return true;
            }
        }
        return false;
    }
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
    void setVelocity(double velocity) {
        std::lock_guard<std::mutex> lock(mutex_);
        current_velocity_.store(velocity);
        last_update_ = std::chrono::system_clock::now();
    }
    
    double getVelocity() const {
        return current_velocity_.load();
    }
    
    bool isVelocityRecent(std::chrono::seconds max_age = std::chrono::seconds(5)) const {
        std::lock_guard<std::mutex> lock(mutex_);
        auto now = std::chrono::system_clock::now();
        return (now - last_update_) < max_age;
    }
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
 * Base64 decoder utility
 */
class Base64Decoder {
public:
    static std::vector<uchar> decode(const std::string& base64) {
        static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
        
        std::string decoded;
        int val = 0, valb = -8;
        for (char c : base64) {
            if (chars.find(c) == std::string::npos) break;
            val = (val << 6) + chars.find(c);
            valb += 6;
            if (valb >= 0) {
                decoded.push_back(char((val >> valb) & 0xFF));
                valb -= 8;
            }
        }
        
        std::vector<uchar> data;
        data.assign(decoded.begin(), decoded.end());
        return data;
    }
};

/**
 * Main SfM Obstacle Detector class - Refactored
 */
class SfMObstacleDetector {
private:
    // Configuration
    Config config_;
    
    // MQTT
    struct mosquitto* mqtt_client_ = nullptr;
    
    // Threading
    std::atomic<bool> running_{false};
    std::thread processing_thread_;
    
    // Frame and velocity management
    FrameBuffer frame_buffer_;
    VelocityManager velocity_manager_;
    
    // Computer vision
    cv::Mat previous_frame_;
    std::chrono::system_clock::time_point previous_timestamp_;
    std::vector<TrackedPoint> tracked_points_;
    
    // Statistics
    struct {
        std::atomic<int> frames_processed{0};
        std::atomic<int> obstacles_detected{0};
        std::atomic<double> avg_processing_time{0.0};
    } stats_;

public:
    SfMObstacleDetector() {
        mosquitto_lib_init();
    }
    
    ~SfMObstacleDetector() {
        stop();
        if (mqtt_client_) {
            mosquitto_destroy(mqtt_client_);
        }
        mosquitto_lib_cleanup();
    }
    
    bool loadConfig(const std::string& config_file = "/opt/smartmower/etc/config/robot_config.json") {
        std::ifstream file(config_file);
        if (!file.is_open()) {
            std::cout << "Using default configuration (robot_config.json not found)" << std::endl;
            return true;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        
        cJSON* root = cJSON_Parse(content.c_str());
        if (!root) {
            std::cerr << "Error parsing robot_config.json" << std::endl;
            return false;
        }
        
        // Parse MQTT settings from unified configuration
        cJSON* system = cJSON_GetObjectItemCaseSensitive(root, "system");
        if (system) {
            cJSON* communication = cJSON_GetObjectItemCaseSensitive(system, "communication");
            if (communication) {
                cJSON* mqtt_broker_host = cJSON_GetObjectItemCaseSensitive(communication, "mqtt_broker_host");
                cJSON* mqtt_broker_port = cJSON_GetObjectItemCaseSensitive(communication, "mqtt_broker_port");
                cJSON* mqtt_username = cJSON_GetObjectItemCaseSensitive(communication, "mqtt_username");
                cJSON* mqtt_password = cJSON_GetObjectItemCaseSensitive(communication, "mqtt_password");
                
                if (cJSON_IsString(mqtt_broker_host)) config_.mqtt_broker = mqtt_broker_host->valuestring;
                if (cJSON_IsNumber(mqtt_broker_port)) config_.mqtt_port = mqtt_broker_port->valueint;
                if (cJSON_IsString(mqtt_username)) config_.mqtt_username = mqtt_username->valuestring;
                if (cJSON_IsString(mqtt_password)) config_.mqtt_password = mqtt_password->valuestring;
            }
        }
        
        // Load obstacle detection configuration
        if (cJSON* obstacle_detection = cJSON_GetObjectItem(root, "obstacle_detection")) {
            // Load robot configuration
            if (cJSON* robot = cJSON_GetObjectItem(obstacle_detection, "robot")) {
                if (cJSON* height = cJSON_GetObjectItem(robot, "camera_height"))
                    config_.camera_height = height->valuedouble;
                if (cJSON* range = cJSON_GetObjectItem(robot, "max_detection_range"))
                    config_.max_detection_range = range->valuedouble;
            }
            
            // Load camera configuration
            if (cJSON* camera = cJSON_GetObjectItem(obstacle_detection, "camera")) {
                if (cJSON* fx = cJSON_GetObjectItem(camera, "focal_length_x"))
                    config_.focal_length_x = fx->valuedouble;
                if (cJSON* fy = cJSON_GetObjectItem(camera, "focal_length_y"))
                    config_.focal_length_y = fy->valuedouble;
                if (cJSON* cx = cJSON_GetObjectItem(camera, "principal_point_x"))
                    config_.principal_point_x = cx->valuedouble;
                if (cJSON* cy = cJSON_GetObjectItem(camera, "principal_point_y"))
                    config_.principal_point_y = cy->valuedouble;
                // Note: width and height not stored in Config struct - used directly from camera parameters
            }
            
            // Load SfM parameters
            if (cJSON* sfm = cJSON_GetObjectItem(obstacle_detection, "sfm_parameters")) {
                if (cJSON* corners = cJSON_GetObjectItem(sfm, "max_corners"))
                    config_.max_corners = corners->valueint;
                if (cJSON* quality = cJSON_GetObjectItem(sfm, "quality_level"))
                    config_.quality_level = quality->valuedouble;
                if (cJSON* distance = cJSON_GetObjectItem(sfm, "min_distance"))
                    config_.min_distance = distance->valuedouble;
                if (cJSON* block = cJSON_GetObjectItem(sfm, "block_size"))
                    config_.block_size = block->valueint;
                if (cJSON* harris = cJSON_GetObjectItem(sfm, "harris_k"))
                    config_.harris_k = harris->valuedouble;
                // Note: min_track_frames, max_optical_flow_error, min_displacement_threshold not in Config struct
                // These are handled by the detection_parameters section
            }
            
            // Load detection parameters
            if (cJSON* detection = cJSON_GetObjectItem(obstacle_detection, "detection_parameters")) {
                if (cJSON* max_dist = cJSON_GetObjectItem(detection, "max_distance"))
                    config_.max_obstacle_distance = max_dist->valuedouble;
                if (cJSON* min_dist = cJSON_GetObjectItem(detection, "min_distance"))
                    config_.min_obstacle_distance = min_dist->valuedouble;
                if (cJSON* frames = cJSON_GetObjectItem(detection, "min_frames_tracked"))
                    config_.min_frames_tracked = frames->valueint;
                if (cJSON* error = cJSON_GetObjectItem(detection, "max_optical_flow_error"))
                    config_.max_optical_flow_error = error->valuedouble;
                if (cJSON* points = cJSON_GetObjectItem(detection, "min_points_threshold"))
                    config_.min_points_threshold = points->valueint;
                if (cJSON* threshold = cJSON_GetObjectItem(detection, "displacement_threshold"))
                    config_.displacement_threshold = threshold->valuedouble;
                // Note: publish_close_obstacles_only not in Config struct
            }
        }
        
        // Load external topics configuration
        if (cJSON* external = cJSON_GetObjectItem(root, "external_topics")) {
            if (cJSON* velocity = cJSON_GetObjectItem(external, "velocity_topic"))
                config_.mqtt_velocity_topic = velocity->valuestring;
        }
        
        // Load logging configuration
        if (cJSON* logging = cJSON_GetObjectItem(root, "logging")) {
            if (cJSON* enabled = cJSON_GetObjectItem(logging, "enabled"))
                config_.debug_enabled = cJSON_IsTrue(enabled);
            // Note: log_level, show_windows, log_file, data_dir not in Config struct
            // Only debug_enabled is available in the current Config structure
        }
        
        cJSON_Delete(root);
        std::cout << "Configuration loaded successfully from robot_config.json" << std::endl;
        std::cout << "MQTT: " << config_.mqtt_broker << ":" << config_.mqtt_port 
                  << " (user: " << config_.mqtt_username << ")" << std::endl;
        std::cout << "[CONFIG] Camera topic: '" << config_.mqtt_topic << "'" << std::endl;
        std::cout << "[CONFIG] Velocity topic: '" << config_.mqtt_velocity_topic << "'" << std::endl;
        return true;
    }
    
    bool initialize() {
        if (!loadConfig()) {
            return false;
        }
        
        if (!setupMQTT()) {
            return false;
        }
        
        std::cout << "SfM Obstacle Detector initialized successfully" << std::endl;
        std::cout << "Camera focal length: " << config_.focal_length_x << std::endl;
        std::cout << "Velocity topic: " << config_.mqtt_velocity_topic << std::endl;
        std::cout << "Debug enabled: " << (config_.debug_enabled ? "true" : "false") << std::endl;
        
        return true;
    }
    
    bool setupMQTT() {
        mqtt_client_ = mosquitto_new("sfm_obstacle_detector_v2", true, this);
        if (!mqtt_client_) {
            std::cerr << "Failed to create MQTT client" << std::endl;
            return false;
        }
        
        // Set callbacks
        mosquitto_connect_callback_set(mqtt_client_, onMQTTConnect);
        mosquitto_message_callback_set(mqtt_client_, onMQTTMessage);
        mosquitto_disconnect_callback_set(mqtt_client_, onMQTTDisconnect);
        
        // Set credentials
        if (!config_.mqtt_username.empty() && !config_.mqtt_password.empty()) {
            mosquitto_username_pw_set(mqtt_client_, 
                                    config_.mqtt_username.c_str(), 
                                    config_.mqtt_password.c_str());
            std::cout << "MQTT authentication set for user: " << config_.mqtt_username << std::endl;
        }
        
        return true;
    }
    
    void start() {
        if (running_.load()) {
            return;
        }
        
        // Connect to MQTT broker
        if (mosquitto_connect(mqtt_client_, config_.mqtt_broker.c_str(), config_.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to connect to MQTT broker at " 
                     << config_.mqtt_broker << ":" << config_.mqtt_port << std::endl;
            throw std::runtime_error("Failed to connect to MQTT broker");
        }
        
        std::cout << "Connected to MQTT broker at " << config_.mqtt_broker << ":" << config_.mqtt_port << std::endl;
        
        running_.store(true);
        processing_thread_ = std::thread(&SfMObstacleDetector::processingLoop, this);
        
        std::cout << "SfM Obstacle Detector started" << std::endl;
    }
    
    void stop() {
        if (!running_.load()) {
            return;
        }
        
        running_.store(false);
        
        if (processing_thread_.joinable()) {
            processing_thread_.join();
        }
        
        if (mqtt_client_) {
            mosquitto_disconnect(mqtt_client_);
        }
        
        std::cout << "SfM Obstacle Detector stopped" << std::endl;
        printStatistics();
    }
    
private:
    void processingLoop() {
        mosquitto_loop_start(mqtt_client_);
        
        cv::Mat current_frame, previous_frame;
        std::chrono::system_clock::time_point current_timestamp, previous_timestamp;
        
        while (running_.load()) {
            // Wait for new frame with timeout
            if (!frame_buffer_.waitForFrame(current_frame, current_timestamp, std::chrono::milliseconds(100))) {
                continue;
            }
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            try {
                processFrame(current_frame, current_timestamp, previous_frame, previous_timestamp);
                
                // Update for next iteration
                current_frame.copyTo(previous_frame);
                previous_timestamp = current_timestamp;
                
                stats_.frames_processed++;
                
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in processing loop: " << e.what() << std::endl;
            }
            
            // Update processing time statistics
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            stats_.avg_processing_time.store(duration.count());
            
            // Show debug window if enabled
            if (config_.debug_enabled) {
                cv::namedWindow("SfM Obstacle Detector", cv::WINDOW_NORMAL);
                cv::resizeWindow("SfM Obstacle Detector", 800, 600);
                cv::imshow("SfM Obstacle Detector", current_frame);
                
                int key = cv::waitKey(1);
                if (key == 27) { // ESC
                    running_.store(false);
                    break;
                }
            }
        }
        
        mosquitto_loop_stop(mqtt_client_, true);
    }
    
    void processFrame(const cv::Mat& current_frame, 
                     const std::chrono::system_clock::time_point& current_timestamp,
                     const cv::Mat& previous_frame,
                     const std::chrono::system_clock::time_point& previous_timestamp) {
        
        if (previous_frame.empty()) {
            if (config_.debug_enabled) {
                std::cout << "First frame received, initializing..." << std::endl;
            }
            return;
        }
        
        // Calculate time difference
        auto dt = std::chrono::duration_cast<std::chrono::duration<double>>(
            current_timestamp - previous_timestamp).count();
        
        if (dt <= 0 || dt > 1.0) { // Skip if time difference is invalid
            return;
        }
        
        // Get current velocity
        double velocity = velocity_manager_.getVelocity();
        double baseline = velocity * dt;
        
        if (config_.debug_enabled) {
            std::cout << "Processing frame - Velocity: " << velocity 
                      << " m/s, dt: " << dt << " s, Baseline: " << baseline << " m" << std::endl;
        }
        
        // Detect and track features
        std::vector<cv::Point2f> prev_points, curr_points;
        detectFeatures(previous_frame, prev_points);
        
        if (prev_points.empty()) {
            return;
        }
        
        // Track features using optical flow
        std::vector<uchar> status;
        std::vector<float> error;
        cv::calcOpticalFlowPyrLK(previous_frame, current_frame, prev_points, curr_points, status, error);
        
        // Calculate distances and filter obstacles
        std::vector<TrackedPoint> valid_obstacles;
        for (size_t i = 0; i < prev_points.size(); ++i) {
            if (status[i] && error[i] < config_.max_optical_flow_error) {
                TrackedPoint point;
                point.previous_pos = prev_points[i];
                point.current_pos = curr_points[i];
                point.frames_tracked = 1;
                
                double displacement = point.getDisplacement();
                if (displacement > config_.displacement_threshold) {
                    // Calculate distance using triangulation
                    point.distance_estimate = (config_.focal_length_x * baseline) / displacement;
                    
                    // Filter by distance range
                    if (point.distance_estimate >= config_.min_obstacle_distance && 
                        point.distance_estimate <= config_.max_obstacle_distance) {
                        valid_obstacles.push_back(point);
                    }
                }
            }
        }
        
        if (config_.debug_enabled) {
            std::cout << "Valid obstacles detected: " << valid_obstacles.size() << std::endl;
        }
        
        // Publish obstacles
        publishObstacles(valid_obstacles, velocity);
        stats_.obstacles_detected.store(valid_obstacles.size());
    }
    
    void detectFeatures(const cv::Mat& frame, std::vector<cv::Point2f>& points) {
        cv::Mat gray;
        if (frame.channels() == 3) {
            cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = frame;
        }
        
        cv::goodFeaturesToTrack(gray, points, config_.max_corners, config_.quality_level,
                               config_.min_distance, cv::Mat(), config_.block_size, 
                               false, config_.harris_k);
    }
    
    void publishObstacles(const std::vector<TrackedPoint>& obstacles, double velocity) {
        if (obstacles.empty()) {
            return;
        }
        
        // Create JSON message
        cJSON* root = cJSON_CreateObject();
        cJSON* obstacles_array = cJSON_CreateArray();
        
        for (const auto& obstacle : obstacles) {
            cJSON* obs = cJSON_CreateObject();
            cJSON_AddNumberToObject(obs, "distance", obstacle.distance_estimate);
            cJSON_AddNumberToObject(obs, "x", obstacle.current_pos.x);
            cJSON_AddNumberToObject(obs, "y", obstacle.current_pos.y);
            cJSON_AddItemToArray(obstacles_array, obs);
        }
        
        cJSON_AddItemToObject(root, "obstacles", obstacles_array);
        cJSON_AddNumberToObject(root, "velocity", velocity);
        cJSON_AddNumberToObject(root, "count", obstacles.size());
        
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        cJSON_AddNumberToObject(root, "timestamp", timestamp);
        
        char* json_string = cJSON_Print(root);
        
        // Publish to MQTT
        std::string topic = "smartmower/vision/obstacles";
        mosquitto_publish(mqtt_client_, nullptr, topic.c_str(), strlen(json_string), json_string, 0, false);
        
        if (config_.debug_enabled) {
            std::cout << "Published " << obstacles.size() << " obstacles with velocity: " << velocity << " m/s" << std::endl;
        }
        
        free(json_string);
        cJSON_Delete(root);
    }
    
    void printStatistics() {
        std::cout << "\n=== SfM Obstacle Detector Statistics ===" << std::endl;
        std::cout << "Frames processed: " << stats_.frames_processed.load() << std::endl;
        std::cout << "Obstacles detected: " << stats_.obstacles_detected.load() << std::endl;
        std::cout << "Avg processing time: " << stats_.avg_processing_time.load() << " ms" << std::endl;
        std::cout << "Velocity updates received: " << (velocity_manager_.isVelocityRecent() ? "Recent" : "Old/None") << std::endl;
    }
    
    // MQTT Callbacks
    static void onMQTTConnect(struct mosquitto* mosq, void* userdata, int result) {
        SfMObstacleDetector* detector = static_cast<SfMObstacleDetector*>(userdata);
        
        if (result == 0) {
            std::cout << "Connected to MQTT broker successfully" << std::endl;
            
            // Subscribe to topics with detailed logging
            int camera_result = mosquitto_subscribe(mosq, nullptr, detector->config_.mqtt_topic.c_str(), 0);
            int velocity_result = mosquitto_subscribe(mosq, nullptr, detector->config_.mqtt_velocity_topic.c_str(), 0);
            
            std::cout << "[MQTT] Subscribing to camera topic: '" << detector->config_.mqtt_topic << "' - Result: " << camera_result << std::endl;
            std::cout << "[MQTT] Subscribing to velocity topic: '" << detector->config_.mqtt_velocity_topic << "' - Result: " << velocity_result << std::endl;
            
            if (camera_result == MOSQ_ERR_SUCCESS && velocity_result == MOSQ_ERR_SUCCESS) {
                std::cout << "[MQTT] Successfully subscribed to both topics" << std::endl;
            } else {
                std::cerr << "[MQTT] ERROR: Failed to subscribe to topics!" << std::endl;
            }
        } else {
            std::cerr << "Failed to connect to MQTT broker: " << result << std::endl;
        }
    }
    
    static void onMQTTMessage(struct mosquitto* /*mosq*/, void* userdata, const struct mosquitto_message* message) {
        SfMObstacleDetector* detector = static_cast<SfMObstacleDetector*>(userdata);
        
        std::string topic(message->topic);
        
        // Always log MQTT messages for debugging
        std::cout << "[MQTT] Message received on topic: '" << topic << "' (" << message->payloadlen << " bytes)" << std::endl;
        
        try {
            if (topic == detector->config_.mqtt_velocity_topic) {
                if (detector->config_.debug_enabled) {
                    std::cout << "[MQTT] Processing velocity message" << std::endl;
                }
                detector->handleVelocityMessage(message);
            } else if (topic == detector->config_.mqtt_topic) {
                std::cout << "[MQTT] Processing camera image message" << std::endl;
                detector->handleImageMessage(message);
            } else {
                std::cout << "[MQTT] WARNING: Received message on unexpected topic: '" << topic << "'" << std::endl;
                std::cout << "[MQTT] Expected topics: '" << detector->config_.mqtt_topic << "' or '" << detector->config_.mqtt_velocity_topic << "'" << std::endl;
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in MQTT message handler: " << e.what() << std::endl;
        }
    }
    
    static void onMQTTDisconnect(struct mosquitto* /*mosq*/, void* /*userdata*/, int /*result*/) {
        std::cout << "Disconnected from MQTT broker" << std::endl;
    }
    
    void handleVelocityMessage(const struct mosquitto_message* message) {
        std::string payload(static_cast<char*>(message->payload), message->payloadlen);
        
        if (config_.debug_enabled) {
            std::cout << "[VELOCITY] Received fusion data: " << payload << std::endl;
        }
        
        cJSON* root = cJSON_Parse(payload.c_str());
        if (!root) {
            std::cerr << "[ERROR] Failed to parse fusion data JSON" << std::endl;
            return;
        }
        
        // Parse new expanded fusion data format
        cJSON* type_json = cJSON_GetObjectItem(root, "type");
        if (cJSON_IsString(type_json) && strcmp(type_json->valuestring, "fusion_data") == 0) {
            
            // Extract velocity from new format
            cJSON* velocity_obj = cJSON_GetObjectItem(root, "velocity");
            if (velocity_obj) {
                // Try to get speed magnitude first (preferred)
                cJSON* speed_json = cJSON_GetObjectItem(velocity_obj, "speed");
                if (cJSON_IsNumber(speed_json)) {
                    double speed = speed_json->valuedouble;
                    velocity_manager_.setVelocity(speed);
                    
                    if (config_.debug_enabled) {
                        std::cout << "[VELOCITY] Updated from fusion speed: " << speed << " m/s" << std::endl;
                    }
                } else {
                    // Fallback: calculate speed from vx, vy components
                    cJSON* vx_json = cJSON_GetObjectItem(velocity_obj, "vx");
                    cJSON* vy_json = cJSON_GetObjectItem(velocity_obj, "vy");
                    
                    if (cJSON_IsNumber(vx_json) && cJSON_IsNumber(vy_json)) {
                        double vx = vx_json->valuedouble;
                        double vy = vy_json->valuedouble;
                        double speed = sqrt(vx * vx + vy * vy);
                        velocity_manager_.setVelocity(speed);
                        
                        if (config_.debug_enabled) {
                            std::cout << "[VELOCITY] Calculated from vx=" << vx << ", vy=" << vy 
                                      << " -> speed=" << speed << " m/s" << std::endl;
                        }
                    } else {
                        std::cerr << "[ERROR] No valid velocity components in fusion data" << std::endl;
                    }
                }
            } else {
                std::cerr << "[ERROR] No velocity object in fusion data" << std::endl;
            }
        } else {
            if (config_.debug_enabled) {
                std::cout << "[VELOCITY] Ignoring non-fusion message type" << std::endl;
            }
        }
        
        cJSON_Delete(root);
    }
    
    void handleImageMessage(const struct mosquitto_message* message) {
        std::string payload(static_cast<char*>(message->payload), message->payloadlen);
        
        cJSON* root = cJSON_Parse(payload.c_str());
        if (!root) {
            std::cerr << "[ERROR] Failed to parse image JSON" << std::endl;
            return;
        }
        
        cJSON* image_data = cJSON_GetObjectItemCaseSensitive(root, "image");
        if (!cJSON_IsString(image_data)) {
            std::cerr << "[ERROR] No valid 'image' field in message" << std::endl;
            cJSON_Delete(root);
            return;
        }
        
        // Decode base64 image
        std::string base64_string = image_data->valuestring;
        std::vector<uchar> image_buffer = Base64Decoder::decode(base64_string);
        
        // Convert to OpenCV Mat
        cv::Mat frame = cv::imdecode(image_buffer, cv::IMREAD_COLOR);
        
        if (!frame.empty()) {
            frame_buffer_.setFrame(frame, std::chrono::system_clock::now());
            
            if (config_.debug_enabled) {
                std::cout << "[IMAGE] Frame received: " << frame.cols << "x" << frame.rows << std::endl;
            }
        } else {
            std::cerr << "[ERROR] Failed to decode image" << std::endl;
        }
        
        cJSON_Delete(root);
    }
};

// Signal handler for graceful shutdown
std::unique_ptr<SfMObstacleDetector> g_detector;

void signalHandler(int signal) {
    std::cout << "\nReceived signal " << signal << ", shutting down..." << std::endl;
    if (g_detector) {
        g_detector->stop();
    }
    exit(0);
}

int main(int /*argc*/, char** /*argv*/) {
    // Setup signal handlers
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        g_detector = std::make_unique<SfMObstacleDetector>();
        
        if (!g_detector->initialize()) {
            std::cerr << "Failed to initialize SfM Obstacle Detector" << std::endl;
            return 1;
        }
        
        g_detector->start();
        
        // Keep main thread alive
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
