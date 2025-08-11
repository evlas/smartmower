/**
 * Grass Detector - Refactored and Optimized Version
 * 
 * Rileva e analizza l'erba utilizzando computer vision.
 * Stima l'altezza e la copertura dell'erba tramite analisi del colore HSV.
 * 
 * Autore: SmartMower Vision System
 * Data: 2025-07-24
 * Versione: 2.0 - Refactored Single File
 */

#include <iostream>
#include <vector>
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

// MQTT and JSON
#include <mosquitto.h>
#include <cjson/cJSON.h>

// Vision MQTT definitions
#include "vision_grass.h"

/**
 * Configuration structure
 */
struct Config {
    // MQTT settings (read from unified configuration)
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_client_id = std::string(VISION_MQTT_CLIENT_ID) + "_grass";
    std::string mqtt_subscribe_topic = std::string(VISION_MQTT_BASE_TOPIC) + VISION_TOPIC_CAMERA;
    std::string mqtt_publish_topic = std::string(VISION_MQTT_BASE_TOPIC) + VISION_TOPIC_GRASS;
    
    // Camera parameters
    double focal_length = 600.0;
    double camera_height_cm = 30.0;
    
    // Grass detection parameters
    int grass_low_h = 30;          // Lower H value for green color (HSV)
    int grass_high_h = 90;         // Upper H value for green color (HSV)
    int min_saturation = 40;       // Minimum saturation for green color
    int min_value = 40;            // Minimum value for green color
    int min_grass_area = 100;      // Minimum area to be considered as grass (in pixels)
    
    // Morphological operations
    int morph_kernel_size = 5;     // Size of the kernel for morphological operations
    std::string morph_kernel_type = "ellipse"; // Type of morphological kernel (ellipse/rect/cross)
    
    // Texture analysis
    double min_grass_height = 0.5;  // Minimum grass height in cm
    double max_grass_height = 25.0; // Maximum grass height in cm
    
    // Processing
    int frame_timeout_ms = 100;     // Timeout for frame waiting in ms
    
    // Debug and visualization
    bool debug_enabled = false;    // Enable debug output and visualization
    std::string log_level = "info"; // Logging level
    
    // Visualization parameters
    struct {
        cv::Scalar grass_color = cv::Scalar(0, 200, 0); // BGR color for grass overlay
        double overlay_alpha = 0.3;                      // Alpha for overlay blending
        cv::Scalar text_color = cv::Scalar(255, 255, 255); // Text color
        int text_thickness = 2;                          // Text thickness
        double text_scale = 0.7;                         // Text scale
    } visualization;
    
    bool loadFromFile(const std::string& filename = "/opt/smartmower/etc/config/robot_config.json") {
        std::ifstream file(filename);
        if (!file.is_open()) {
            std::cout << "Using default configuration (" << filename << " not found)" << std::endl;
            return true;
        }
        
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        
        cJSON* root = cJSON_Parse(content.c_str());
        if (!root) {
            std::cerr << "Error parsing " << filename << std::endl;
            return false;
        }
        
        // 1. Parse MQTT settings
        cJSON* mqtt = cJSON_GetObjectItem(root, "mqtt");
        if (mqtt) {
            // Basic MQTT connection settings
            if (cJSON_IsString(cJSON_GetObjectItem(mqtt, "broker"))) 
                this->mqtt_broker = cJSON_GetObjectItem(mqtt, "broker")->valuestring;
            if (cJSON_IsNumber(cJSON_GetObjectItem(mqtt, "port")))
                this->mqtt_port = cJSON_GetObjectItem(mqtt, "port")->valueint;
            if (cJSON_IsString(cJSON_GetObjectItem(mqtt, "username")))
                this->mqtt_username = cJSON_GetObjectItem(mqtt, "username")->valuestring;
            if (cJSON_IsString(cJSON_GetObjectItem(mqtt, "password")))
                this->mqtt_password = cJSON_GetObjectItem(mqtt, "password")->valuestring;
            
            // Load MQTT topics configuration
            cJSON* topics = cJSON_GetObjectItem(mqtt, "topics");
            if (topics) {
                // Camera topic (for subscription)
                cJSON* camera_topic = cJSON_GetObjectItem(topics, "camera");
                if (camera_topic) {
                    std::string base_topic = cJSON_GetObjectItem(camera_topic, "base")->valuestring;
                    cJSON* subtopics = cJSON_GetObjectItem(camera_topic, "subtopics");
                    if (subtopics) {
                        // Use frame subtopic if available, otherwise use base topic
                        cJSON* frame_subtopic = cJSON_GetObjectItem(subtopics, "frame");
                        if (frame_subtopic) {
                            this->mqtt_subscribe_topic = base_topic + "/" + frame_subtopic->valuestring;
                        } else {
                            this->mqtt_subscribe_topic = base_topic;
                        }
                    } else {
                        this->mqtt_subscribe_topic = base_topic;
                    }
                }
                
                // Grass topic (for publishing)
                cJSON* grass_topic = cJSON_GetObjectItem(topics, "grass");
                if (grass_topic) {
                    std::string base_topic = cJSON_GetObjectItem(grass_topic, "base")->valuestring;
                    cJSON* subtopics = cJSON_GetObjectItem(grass_topic, "subtopics");
                    if (subtopics) {
                        // Use detection subtopic if available, otherwise use base topic
                        cJSON* detection_subtopic = cJSON_GetObjectItem(subtopics, "detection");
                        if (detection_subtopic) {
                            this->mqtt_publish_topic = base_topic + "/" + detection_subtopic->valuestring;
                        } else {
                            this->mqtt_publish_topic = base_topic;
                        }
                    } else {
                        this->mqtt_publish_topic = base_topic;
                    }
                }
            }
        }
        
        // 2. Load camera parameters
        if (cJSON* camera = cJSON_GetObjectItem(root, "camera")) {
            // Get camera height (convert from meters to cm)
            if (cJSON* height = cJSON_GetObjectItem(camera, "height_m"))
                camera_height_cm = height->valuedouble * 100.0;
                
            // Get focal length from intrinsics
            if (cJSON* intrinsics = cJSON_GetObjectItem(camera, "intrinsics")) {
                if (cJSON* fl = cJSON_GetObjectItem(intrinsics, "focal_length"))
                    focal_length = fl->valuedouble;
            }
        }
        
        // 3. Load grass detection parameters
        if (cJSON* vision = cJSON_GetObjectItem(root, "vision")) {
            if (cJSON* grass_detection = cJSON_GetObjectItem(vision, "grass_detection")) {
                if (cJSON* params = cJSON_GetObjectItem(grass_detection, "parameters")) {
                    // Get HSV and area parameters
                    if (cJSON* low_h = cJSON_GetObjectItem(params, "grass_low_h"))
                        grass_low_h = low_h->valueint;
                    if (cJSON* high_h = cJSON_GetObjectItem(params, "grass_high_h"))
                        grass_high_h = high_h->valueint;
                    if (cJSON* min_sat = cJSON_GetObjectItem(params, "min_saturation"))
                        min_saturation = min_sat->valueint;
                    if (cJSON* min_val = cJSON_GetObjectItem(params, "min_value"))
                        min_value = min_val->valueint;
                    if (cJSON* min_area = cJSON_GetObjectItem(params, "min_grass_area"))
                        min_grass_area = min_area->valueint;
                    
                    // Load morphological operations parameters
                    if (cJSON* morph = cJSON_GetObjectItem(params, "morphological")) {
                        if (cJSON* ksize = cJSON_GetObjectItem(morph, "kernel_size"))
                            morph_kernel_size = ksize->valueint;
                        if (cJSON* ktype = cJSON_GetObjectItem(morph, "kernel_type"))
                            morph_kernel_type = ktype->valuestring;
                    }
                    
                    // Load grass height parameters
                    if (cJSON* height = cJSON_GetObjectItem(params, "height")) {
                        if (cJSON* min_h = cJSON_GetObjectItem(height, "min_cm"))
                            min_grass_height = min_h->valuedouble;
                        if (cJSON* max_h = cJSON_GetObjectItem(height, "max_cm"))
                            max_grass_height = max_h->valuedouble;
                    }
                    
                    // Load processing parameters
                    if (cJSON* proc = cJSON_GetObjectItem(params, "processing")) {
                        if (cJSON* timeout = cJSON_GetObjectItem(proc, "frame_timeout_ms"))
                            frame_timeout_ms = timeout->valueint;
                    }
                    
                    // Load visualization parameters
                    if (cJSON* vis = cJSON_GetObjectItem(params, "visualization")) {
                        if (cJSON* color = cJSON_GetObjectItem(vis, "grass_color")) {
                            if (cJSON_IsArray(color) && cJSON_GetArraySize(color) == 3) {
                                visualization.grass_color = cv::Scalar(
                                    cJSON_GetArrayItem(color, 0)->valueint,
                                    cJSON_GetArrayItem(color, 1)->valueint,
                                    cJSON_GetArrayItem(color, 2)->valueint
                                );
                            }
                        }
                        if (cJSON* alpha = cJSON_GetObjectItem(vis, "overlay_alpha"))
                            visualization.overlay_alpha = alpha->valuedouble;
                        if (cJSON* text = cJSON_GetObjectItem(vis, "text_color")) {
                            if (cJSON_IsArray(text) && cJSON_GetArraySize(text) == 3) {
                                visualization.text_color = cv::Scalar(
                                    cJSON_GetArrayItem(text, 0)->valueint,
                                    cJSON_GetArrayItem(text, 1)->valueint,
                                    cJSON_GetArrayItem(text, 2)->valueint
                                );
                            }
                        }
                        if (cJSON* thickness = cJSON_GetObjectItem(vis, "text_thickness"))
                            visualization.text_thickness = thickness->valueint;
                        if (cJSON* scale = cJSON_GetObjectItem(vis, "text_scale"))
                            visualization.text_scale = scale->valuedouble;
                    }
                }
            }
        }
        
        // 4. Load logging configuration
        if (cJSON* logging = cJSON_GetObjectItem(root, "logging")) {
            if (cJSON* enabled = cJSON_GetObjectItem(logging, "enabled"))
                debug_enabled = cJSON_IsTrue(enabled);
            if (cJSON* level = cJSON_GetObjectItem(logging, "level"))
                log_level = level->valuestring;
        }
        
        cJSON_Delete(root);
        std::cout << "Configuration loaded successfully from robot_config.json" << std::endl;
        std::cout << "MQTT: " << mqtt_broker << ":" << mqtt_port 
                  << " (user: " << mqtt_username << ")" << std::endl;
        return true;
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
 * Grass analysis result structure
 */
struct GrassInfo {
    double height_cm = 0.0;      // Estimated grass height in cm
    double coverage = 0.0;       // Grass coverage percentage (0-100)
    int grass_pixels = 0;        // Number of grass pixels detected
    int total_pixels = 0;        // Total pixels in frame
    cv::Mat visualization;       // Debug visualization
    std::chrono::system_clock::time_point timestamp;
};

/**
 * Main Grass Detector class - Refactored
 */
class GrassDetector {
private:
    Config config_;
    struct mosquitto* mqtt_client_ = nullptr;
    std::atomic<bool> running_{false};
    std::thread processing_thread_;
    FrameBuffer frame_buffer_;
    
    // Statistics
    struct {
        std::atomic<int> frames_processed{0};
        std::atomic<int> grass_detections{0};
        std::atomic<double> avg_processing_time{0.0};
        std::atomic<double> avg_grass_height{0.0};
        std::atomic<double> avg_coverage{0.0};
    } stats_;

public:
    GrassDetector() {
        mosquitto_lib_init();
    }
    
    ~GrassDetector() {
        stop();
        if (mqtt_client_) {
            mosquitto_destroy(mqtt_client_);
        }
        mosquitto_lib_cleanup();
    }
    
    bool initialize() {
        if (!config_.loadFromFile()) {
            return false;
        }
        
        if (!setupMQTT()) {
            return false;
        }
        
        std::cout << "Grass Detector initialized successfully" << std::endl;
        std::cout << "Camera focal length: " << config_.focal_length << " px" << std::endl;
        std::cout << "Camera height: " << config_.camera_height_cm << " cm" << std::endl;
        std::cout << "Debug enabled: " << (config_.debug_enabled ? "true" : "false") << std::endl;
        
        return true;
    }
    
    bool setupMQTT() {
        mqtt_client_ = mosquitto_new(config_.mqtt_client_id.c_str(), true, this);
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
        processing_thread_ = std::thread(&GrassDetector::processingLoop, this);
        
        std::cout << "Grass Detector started" << std::endl;
        std::cout << "MQTT: " << config_.mqtt_broker << ":" << config_.mqtt_port << std::endl;
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
        
        std::cout << "Grass Detector stopped" << std::endl;
        printStatistics();
    }
    
private:
    void processingLoop() {
        mosquitto_loop_start(mqtt_client_);
        
        cv::Mat current_frame;
        std::chrono::system_clock::time_point current_timestamp;
        
        while (running_.load()) {
            // Wait for new frame with configured timeout
            if (!frame_buffer_.waitForFrame(current_frame, current_timestamp, 
                                          std::chrono::milliseconds(config_.frame_timeout_ms))) {
                continue;
            }
            
            auto start_time = std::chrono::high_resolution_clock::now();
            
            try {
                GrassInfo grass_info = analyzeGrass(current_frame, current_timestamp);
                publishGrassInfo(grass_info);
                
                stats_.frames_processed++;
                stats_.avg_grass_height.store(grass_info.height_cm);
                stats_.avg_coverage.store(grass_info.coverage);
                
                if (grass_info.coverage > 10.0) { // Consider it a grass detection if coverage > 10%
                    stats_.grass_detections++;
                }
                
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in processing loop: " << e.what() << std::endl;
            }
            
            // Update processing time statistics
            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);
            stats_.avg_processing_time.store(duration.count());
            
            // Show debug window if enabled
            if (config_.debug_enabled && !current_frame.empty()) {
                cv::namedWindow("Grass Detector", cv::WINDOW_NORMAL);
                cv::resizeWindow("Grass Detector", 800, 600);
                cv::imshow("Grass Detector", current_frame);
                
                int key = cv::waitKey(1);
                if (key == 27) { // ESC
                    running_.store(false);
                    break;
                }
            }
        }
        
        mosquitto_loop_stop(mqtt_client_, true);
    }
    
    GrassInfo analyzeGrass(const cv::Mat& frame, const std::chrono::system_clock::time_point& timestamp) {
        GrassInfo result;
        result.timestamp = timestamp;
        result.total_pixels = frame.rows * frame.cols;
        
        if (frame.empty()) {
            std::cerr << "[ERROR] Empty frame received" << std::endl;
            return result;
        }
        
        // Create visualization frame
        frame.copyTo(result.visualization);
        
        // Detect grass using HSV color space
        cv::Mat grass_mask = detectGrassHSV(frame);
        
        // Calculate coverage
        result.grass_pixels = cv::countNonZero(grass_mask);
        result.coverage = (result.grass_pixels * 100.0) / result.total_pixels;
        
        // Estimate grass height
        result.height_cm = estimateGrassHeight(frame, grass_mask);
        
        // Create debug visualization
        if (config_.debug_enabled) {
            createVisualization(result, grass_mask);
        }
        
        if (config_.debug_enabled) {
            std::cout << "[GRASS] Height: " << result.height_cm << " cm, Coverage: " 
                      << result.coverage << "%, Pixels: " << result.grass_pixels << std::endl;
        }
        
        return result;
    }
    
    cv::Mat detectGrassHSV(const cv::Mat& frame) {
        cv::Mat hsv, mask;
        
        // Convert to HSV color space
        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);
        
        // Create mask for green colors (grass)
        cv::Scalar lower_green(config_.grass_low_h, config_.min_saturation, config_.min_value);
        cv::Scalar upper_green(config_.grass_high_h, 255, 255);
        cv::inRange(hsv, lower_green, upper_green, mask);
        
        // Apply morphological operations to clean up the mask
        int kernel_shape;
        if (config_.morph_kernel_type == "rect") {
            kernel_shape = cv::MORPH_RECT;
        } else if (config_.morph_kernel_type == "cross") {
            kernel_shape = cv::MORPH_CROSS;
        } else { // default to ellipse
            kernel_shape = cv::MORPH_ELLIPSE;
        }
        
        cv::Mat kernel = cv::getStructuringElement(
            kernel_shape, 
            cv::Size(config_.morph_kernel_size, config_.morph_kernel_size)
        );
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
        
        // Filter small areas
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        cv::Mat filtered_mask = cv::Mat::zeros(mask.size(), CV_8UC1);
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) >= config_.min_grass_area) {
                cv::fillPoly(filtered_mask, std::vector<std::vector<cv::Point>>{contour}, cv::Scalar(255));
            }
        }
        
        return filtered_mask;
    }
    
    double estimateGrassHeight(const cv::Mat& frame, const cv::Mat& mask) {
        if (cv::countNonZero(mask) == 0) {
            return 0.0;
        }
        
        // Find grass regions and estimate height based on texture analysis
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        
        double total_height = 0.0;
        int valid_regions = 0;
        
        for (const auto& contour : contours) {
            if (cv::contourArea(contour) < config_.min_grass_area) {
                continue;
            }
            
            // Get bounding rectangle of grass region
            cv::Rect bbox = cv::boundingRect(contour);
            
            // Extract region of interest
            cv::Mat roi = frame(bbox);
            cv::Mat roi_mask = mask(bbox);
            
            // Analyze texture to estimate height
            double height = analyzeGrassTexture(roi, roi_mask, bbox);
            
            if (height > 0) {
                total_height += height;
                valid_regions++;
            }
        }
        
        return valid_regions > 0 ? total_height / valid_regions : 0.0;
    }
    
    double analyzeGrassTexture(const cv::Mat& roi, const cv::Mat& roi_mask, const cv::Rect& bbox) {
        // Convert to grayscale for texture analysis
        cv::Mat gray;
        cv::cvtColor(roi, gray, cv::COLOR_BGR2GRAY);
        
        // Calculate gradient magnitude to detect grass texture
        cv::Mat grad_x, grad_y, grad_mag;
        cv::Sobel(gray, grad_x, CV_32F, 1, 0, 3);
        cv::Sobel(gray, grad_y, CV_32F, 0, 1, 3);
        cv::magnitude(grad_x, grad_y, grad_mag);
        
        // Calculate texture metrics
        cv::Scalar mean_grad = cv::mean(grad_mag, roi_mask);
        cv::Scalar std_grad;
        cv::meanStdDev(grad_mag, cv::Scalar(), std_grad, roi_mask);
        
        // Calculate grass density (how much of the region is grass)
        double grass_density = (double)cv::countNonZero(roi_mask) / (roi_mask.rows * roi_mask.cols);
        
        // Calculate position in frame (bottom = closer = potentially taller)
        double frame_height = 480.0; // Assuming 640x480 frame
        double center_y = bbox.y + bbox.height / 2.0;
        double distance_from_bottom = (frame_height - center_y) / frame_height;
        
        // Improved height estimation using multiple factors:
        // 1. Gradient intensity (texture complexity)
        // 2. Gradient variation (texture uniformity)
        // 3. Grass density in region
        // 4. Position in frame (perspective)
        
        double texture_complexity = mean_grad[0] / 30.0;  // Normalize to 0-3 range
        double texture_variation = std_grad[0] / 20.0;     // Texture uniformity
        double position_factor = 1.0 - distance_from_bottom; // Closer to camera = higher
        
        // Calculate height factors based on configuration
        double height_range = config_.max_grass_height - config_.min_grass_height;
        
        // Combine factors with weights (all factors are 0-1 range)
        double height_from_texture = texture_complexity * 0.3;         // Texture complexity (0-30% of range)
        double height_from_variation = texture_variation * 0.2;        // Texture variation (0-20% of range)
        double height_from_density = grass_density * 0.2;              // Grass density (0-20% of range)
        double height_from_position = position_factor * 0.3;           // Position in frame (0-30% of range)
        
        // Calculate final height within configured bounds
        double height_factor = height_from_texture + height_from_variation + 
                             height_from_density + height_from_position;
        
        // Scale to configured height range and add minimum height
        double estimated_height = config_.min_grass_height + (height_factor * height_range);
        
        if (config_.debug_enabled) {
            std::cout << "[TEXTURE] Grad:" << texture_complexity 
                      << " Var:" << texture_variation 
                      << " Dens:" << grass_density 
                      << " Pos:" << position_factor 
                      << " -> " << estimated_height << "cm" << std::endl;
        }
        
        return estimated_height;
    }
    
    void createVisualization(GrassInfo& result, const cv::Mat& grass_mask) {
        // Create colored overlay for grass regions
        cv::Mat overlay;
        cv::cvtColor(grass_mask, overlay, cv::COLOR_GRAY2BGR);
        overlay.setTo(config_.visualization.grass_color, grass_mask);
        
        // Blend with original image using configured alpha
        cv::addWeighted(result.visualization, 1.0 - config_.visualization.overlay_alpha, 
                       overlay, config_.visualization.overlay_alpha, 
                       0, result.visualization);
        
        // Add text information
        std::string info = cv::format("Grass: %.1f cm (%.1f%%) - %d pixels", 
                                     result.height_cm, result.coverage, result.grass_pixels);
        cv::putText(result.visualization, info, 
                   cv::Point(10, 30), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   config_.visualization.text_scale, 
                   config_.visualization.text_color, 
                   config_.visualization.text_thickness);
        
        // Add timestamp
        auto time_t = std::chrono::system_clock::to_time_t(result.timestamp);
        std::string time_str = std::ctime(&time_t);
        time_str.pop_back(); // Remove newline
        cv::putText(result.visualization, time_str, 
                   cv::Point(10, result.visualization.rows - 20), 
                   cv::FONT_HERSHEY_SIMPLEX, 
                   config_.visualization.text_scale * 0.7, 
                   config_.visualization.text_color, 
                   config_.visualization.text_thickness);
    }
    
    void publishGrassInfo(const GrassInfo& grass_info) {
        // Create JSON message
        cJSON* root = cJSON_CreateObject();
        
        cJSON_AddNumberToObject(root, "height_cm", grass_info.height_cm);
        cJSON_AddNumberToObject(root, "coverage_percent", grass_info.coverage);
        cJSON_AddNumberToObject(root, "grass_pixels", grass_info.grass_pixels);
        cJSON_AddNumberToObject(root, "total_pixels", grass_info.total_pixels);
        
        // Add timestamp
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            grass_info.timestamp.time_since_epoch()).count();
        cJSON_AddNumberToObject(root, "timestamp", timestamp);
        
        // Add camera parameters for reference
        cJSON* camera = cJSON_CreateObject();
        cJSON_AddNumberToObject(camera, "focal_length", config_.focal_length);
        cJSON_AddNumberToObject(camera, "height_cm", config_.camera_height_cm);
        cJSON_AddItemToObject(root, "camera", camera);
        
        char* json_string = cJSON_Print(root);
        
        // Publish to MQTT
        mosquitto_publish(mqtt_client_, nullptr, config_.mqtt_publish_topic.c_str(), 
                         strlen(json_string), json_string, 0, false);
        
        if (config_.debug_enabled) {
            std::cout << "[PUBLISH] Grass info published to " << config_.mqtt_publish_topic << std::endl;
        }
        
        free(json_string);
        cJSON_Delete(root);
    }
    
    void printStatistics() {
        std::cout << "\n=== Grass Detector Statistics ===" << std::endl;
        std::cout << "Frames processed: " << stats_.frames_processed.load() << std::endl;
        std::cout << "Grass detections: " << stats_.grass_detections.load() << std::endl;
        std::cout << "Avg processing time: " << stats_.avg_processing_time.load() << " ms" << std::endl;
        std::cout << "Avg grass height: " << stats_.avg_grass_height.load() << " cm" << std::endl;
        std::cout << "Avg coverage: " << stats_.avg_coverage.load() << "%" << std::endl;
    }
    
    // MQTT Callbacks
    static void onMQTTConnect(struct mosquitto* mosq, void* userdata, int result) {
        GrassDetector* detector = static_cast<GrassDetector*>(userdata);
        
        if (result == 0) {
            std::cout << "Connected to MQTT broker successfully" << std::endl;
            
            // Subscribe to camera topic
            mosquitto_subscribe(mosq, nullptr, detector->config_.mqtt_subscribe_topic.c_str(), 0);
            
            std::cout << "Subscribed to topic: " << detector->config_.mqtt_subscribe_topic << std::endl;
        } else {
            std::cerr << "Failed to connect to MQTT broker: " << result << std::endl;
        }
    }
    
    static void onMQTTMessage(struct mosquitto* /*mosq*/, void* userdata, const struct mosquitto_message* message) {
        GrassDetector* detector = static_cast<GrassDetector*>(userdata);
        
        std::string topic(message->topic);
        
        if (detector->config_.debug_enabled) {
            std::cout << "[MQTT] Message on topic: " << topic << " (" << message->payloadlen << " bytes)" << std::endl;
        }
        
        try {
            if (topic == detector->config_.mqtt_subscribe_topic) {
                detector->handleImageMessage(message);
            }
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in MQTT message handler: " << e.what() << std::endl;
        }
    }
    
    static void onMQTTDisconnect(struct mosquitto* /*mosq*/, void* /*userdata*/, int /*result*/) {
        std::cout << "Disconnected from MQTT broker" << std::endl;
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

// Global detector instance for signal handling
std::unique_ptr<GrassDetector> g_detector;

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
        g_detector = std::make_unique<GrassDetector>();
        
        if (!g_detector->initialize()) {
            std::cerr << "Failed to initialize Grass Detector" << std::endl;
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
