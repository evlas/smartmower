#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <opencv2/opencv.hpp>
#include <mosquitto.h>
#include <json/json.h>
#include <fstream>
#include <chrono>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <csignal>
#include <filesystem>
#include <atomic>
#include <iomanip>
#include <sstream>

// Vision MQTT definitions
#include "vision_mqtt.h"

// Forward declarations
void signalHandler(int signum);

// Global variables
cv::VideoCapture cap;
std::mutex frameMutex;
std::condition_variable frameCV;
bool frameReady = false;
cv::Mat currentFrame;
struct mosquitto *mosq = nullptr;
std::atomic<bool> running(true);

// Camera configuration structure
struct CameraSettings {
    std::string device;
    int width;
    int height;
    int fps;
};

// MQTT configuration structure
struct MQTTSettings {
    std::string broker;
    int port;
    std::string topic;
    std::string username;
    std::string password;
};

// Application configuration
struct Config {
    MQTTSettings mqtt;
    CameraSettings camera;
} config;

// Function declarations
void handle_signal(int signal);
void cleanup();
int load_config();
int init_mqtt();
int init_camera();
int capture_and_send_frame();
std::string base64_encode(const unsigned char* data, size_t input_length);
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp);

// Signal handler
void handle_signal([[maybe_unused]] int signal) {
    running = 0;
}

// Base64 encoding function
std::string base64_encode(const unsigned char* data, size_t input_length) {
    static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    
    std::string encoded;
    encoded.reserve(4 * ((input_length + 2) / 3));
    
    for (size_t i = 0; i < input_length; i += 3) {
        uint32_t octet_a = i < input_length ? data[i] : 0;
        uint32_t octet_b = i + 1 < input_length ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < input_length ? data[i + 2] : 0;
        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
        
        encoded.push_back(base64_chars[(triple >> 3 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 2 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 1 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 0 * 6) & 0x3F]);
    }
    
    switch (input_length % 3) {
        case 1: 
            encoded[encoded.length() - 1] = '=';
            encoded[encoded.length() - 2] = '='; 
            break;
        case 2: 
            encoded[encoded.length() - 1] = '='; 
            break;
    }
    
    return encoded;
}

// Send image via MQTT
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp) {
    if (!mosq) return -1;
    
    // Create JSON message
    Json::Value json_msg;
    json_msg["data"] = base64_encode(image_data.data(), image_data.size());
    json_msg["timestamp"] = timestamp;
    json_msg["type"] = "image/jpeg";
    
    Json::StreamWriterBuilder writer;
    std::string json_string = Json::writeString(writer, json_msg);
    
    int result = mosquitto_publish(mosq, NULL, config.mqtt.topic.c_str(),
                                  json_string.length(), json_string.c_str(), 1, false);
    
    return (result == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

// Simple frame capture using OpenCV's VideoCapture
int capture_frame_simple() {
    try {
        if (!cap.isOpened()) {
            std::cerr << "[ERROR] Camera is not opened" << std::endl;
            return -1;
        }

        cv::Mat frame;
        if (!cap.read(frame)) {
            std::cerr << "[ERROR] Failed to read frame from camera" << std::endl;
            return -1;
        }

        if (frame.empty()) {
            std::cerr << "[ERROR] Captured frame is empty" << std::endl;
            return -1;
        }

        // Lock and update the current frame
        {
            std::lock_guard<std::mutex> lock(frameMutex);
            currentFrame = frame.clone();
        }

        frameReady = true;
        frameCV.notify_one();
        
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "[EXCEPTION] in capture_frame_simple: " << e.what() << std::endl;
        return -1;
    }
}

// Initialize camera using OpenCV's VideoCapture with V4L2 backend
int init_camera() {
    try {
        // Try different video devices to find the camera
        const std::vector<std::string> device_paths = {
            "/dev/video0", 
            "/dev/video1", 
            "/dev/video2",
            "/dev/v4l/by-path/platform-1f00128000.csi-video-index0"
        };
        
        bool camera_opened = false;
        
        for (const auto& device_path : device_paths) {
            if (!std::filesystem::exists(device_path)) {
                std::cout << "Device " << device_path << " not found, trying next..." << std::endl;
                continue;
            }
            
            std::cout << "Trying to open camera at " << device_path << "..." << std::endl;
            
            // Try to open the camera with V4L2 backend
            cap.open(device_path, cv::CAP_V4L2);
            
            if (!cap.isOpened()) {
                std::cerr << "Failed to open camera at " << device_path << std::endl;
                continue;
            }
        
            // Set camera parameters
            cap.set(cv::CAP_PROP_FRAME_WIDTH, config.camera.width);
            cap.set(cv::CAP_PROP_FRAME_HEIGHT, config.camera.height);
            cap.set(cv::CAP_PROP_FPS, config.camera.fps);
            
            // Verify the settings
            double actual_width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
            double actual_height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
            double actual_fps = cap.get(cv::CAP_PROP_FPS);
            
            std::cout << "Camera opened successfully at " << device_path << std::endl;
            std::cout << "Resolution: " << actual_width << "x" << actual_height << " @ " << actual_fps << " FPS" << std::endl;
            
            camera_opened = true;
            break;
        }
        
        if (!camera_opened) {
            std::cerr << "Failed to open any camera device" << std::endl;
            return -1;
        }
        
        return 0;
    } catch (const std::exception &e) {
        std::cerr << "Exception in init_camera: " << e.what() << std::endl;
        return -1;
    }
}

// Load configuration from JSON file
int load_config(const std::string& filename) {
    try {
        std::ifstream config_file(filename);
        if (!config_file.is_open()) {
            std::cerr << "Error: Could not open config file " << filename << std::endl;
            return -1;
        }
        
        Json::Value root;
        Json::CharReaderBuilder builder;
        std::string errs;
        
        if (!parseFromStream(builder, config_file, &root, &errs)) {
            std::cerr << "Error parsing config file: " << errs << std::endl;
            return -1;
        }
        
        // Parse MQTT settings
        const Json::Value& mqtt = root["system"]["communication"];
        if (!mqtt.isNull()) {
            if (mqtt.isMember("mqtt_broker_host")) 
                config.mqtt.broker = mqtt["mqtt_broker_host"].asString();
            if (mqtt.isMember("mqtt_broker_port")) 
                config.mqtt.port = mqtt["mqtt_broker_port"].asInt();
            if (mqtt.isMember("mqtt_username")) 
                config.mqtt.username = mqtt["mqtt_username"].asString();
            if (mqtt.isMember("mqtt_password")) 
                config.mqtt.password = mqtt["mqtt_password"].asString();
        }
        
        // Parse camera settings
        const Json::Value& camera = root["vision"]["camera"];
        if (!camera.isNull()) {
            if (camera.isMember("device"))
                config.camera.device = camera["device"].asString();
            if (camera.isMember("width"))
                config.camera.width = camera["width"].asInt();
            if (camera.isMember("height"))
                config.camera.height = camera["height"].asInt();
            if (camera.isMember("fps"))
                config.camera.fps = camera["fps"].asInt();
            if (camera.isMember("mqtt_topic"))
                config.mqtt.topic = camera["mqtt_topic"].asString();
        }
        
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Exception loading config: " << e.what() << std::endl;
        return -1;
    }
}

// MQTT client handle
extern struct mosquitto *mosq;

// Initialize MQTT client
int init_mqtt() {
    // Initialize the Mosquitto library
    mosquitto_lib_init();
    
    // Create a new mosquitto client instance with a random client ID
    mosq = mosquitto_new(NULL, true, NULL);
    if (!mosq) {
        std::cerr << "[ERROR] Failed to create MQTT client: Out of memory" << std::endl;
        return -1;
    }
    
    // Set username and password if provided
    if (!config.mqtt.username.empty()) {
        int rc = mosquitto_username_pw_set(
            mosq, 
            config.mqtt.username.c_str(), 
            config.mqtt.password.empty() ? NULL : config.mqtt.password.c_str()
        );
        
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "[WARN] Failed to set MQTT credentials: " << mosquitto_strerror(rc) << std::endl;
        } else {
            std::cout << "MQTT authentication set for user: " << config.mqtt.username << std::endl;
        }
    }
    
    // Connect to MQTT broker with a 5-second timeout
    int rc = mosquitto_connect(mosq, config.mqtt.broker.c_str(), config.mqtt.port, 5);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERROR] Failed to connect to MQTT broker at " 
                  << config.mqtt.broker << ":" << config.mqtt.port 
                  << ": " << mosquitto_strerror(rc) << std::endl;
        mosquitto_destroy(mosq);
        mosq = nullptr;
        return -1;
    }
    
    // Start the network loop in a separate thread
    mosquitto_loop_start(mosq);
    
    std::cout << "Connected to MQTT broker at " << config.mqtt.broker 
              << ":" << config.mqtt.port << std::endl;
    return 0;
}

// Capture and send frame
int capture_and_send_frame() {
    try {
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(frameMutex);
            if (!frameReady) {
                return 0; // No new frame available
            }
            frame = currentFrame.clone();
            frameReady = false;
        }
        
        if (frame.empty()) {
            std::cerr << "[WARN] Empty frame captured" << std::endl;
            return -1;
        }
        
        if (!mosq) {
            std::cerr << "[WARN] MQTT client not initialized, frame not published" << std::endl;
            return -1;
        }
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[EXCEPTION] in capture_and_send_frame: " << e.what() << std::endl;
        return -1;
    }
}

void cleanup() {
    running = false;
    
    // Signal any waiting threads to exit
    running = false;
    frameCV.notify_all();
    
    // Release the camera
    if (cap.isOpened()) {
        std::cout << "Releasing camera..." << std::endl;
        cap.release();
    }
    
    // Cleanup MQTT
    if (mosq) {
        std::cout << "Disconnecting from MQTT broker..." << std::endl;
        mosquitto_disconnect(mosq);
        mosquitto_loop_stop(mosq, true);
        mosquitto_destroy(mosq);
        mosquitto_lib_cleanup();
        mosq = nullptr;
    }
    
    // Clear the current frame
    {
        std::lock_guard<std::mutex> lock(frameMutex);
        currentFrame.release();
    }
    
    std::cout << "Cleanup completed" << std::endl;
}

// Signal handler for graceful shutdown
void signalHandler(int signum) {
    std::cout << "\nInterrupt signal (" << signum << ") received. Cleaning up..." << std::endl;
    running = false;
}

// Main function
int main() {
    // Set up signal handling
    struct sigaction sigIntHandler;
    sigIntHandler.sa_handler = signalHandler;
    sigemptyset(&sigIntHandler.sa_mask);
    sigIntHandler.sa_flags = 0;
    sigaction(SIGINT, &sigIntHandler, NULL);
    sigaction(SIGTERM, &sigIntHandler, NULL);
    
    std::cout << "Starting Vision Camera RPi..." << std::endl;
    
    // Set default configuration
    config.mqtt.broker = "localhost";
    config.mqtt.port = 1883;
    config.mqtt.topic = "vision/camera/frame";
    config.camera.device = "/dev/video0";
    config.camera.width = 1280;
    config.camera.height = 720;
    config.camera.fps = 30;
    
    // Load configuration from unified config file
    if (load_config("/opt/smartmower/etc/config/robot_config.json") != 0) {
        std::cerr << "[WARN] Using default configuration (failed to load config file)" << std::endl;
    }
    
    std::cout << "Camera configuration:" << std::endl;
    std::cout << "  Device: " << config.camera.device << std::endl;
    std::cout << "  Resolution: " << config.camera.width << "x" << config.camera.height << std::endl;
    std::cout << "  FPS: " << config.camera.fps << std::endl;
    std::cout << "MQTT configuration:" << std::endl;
    std::cout << "  Broker: " << config.mqtt.broker << ":" << config.mqtt.port << std::endl;
    std::cout << "  Topic: " << config.mqtt.topic << std::endl;
    
    // Initialize MQTT first (so we can report camera initialization issues)
    if (init_mqtt() != 0) {
        std::cerr << "[ERROR] Failed to initialize MQTT" << std::endl;
        cleanup();
        return 1;
    }
    
    // Initialize camera
    if (init_camera() != 0) {
        std::cerr << "[ERROR] Failed to initialize camera" << std::endl;
        cleanup();
        return 1;
    }
    
    std::cout << "Vision Camera RPi started successfully" << std::endl;
    
    // Main loop
    auto last_capture = std::chrono::steady_clock::now();
    auto frame_interval = std::chrono::milliseconds(1000 / config.camera.fps);
    
    while (running) {
        try {
            auto now = std::chrono::steady_clock::now();
            
            if (now - last_capture >= frame_interval) {
                if (capture_and_send_frame() == 0) {
                    last_capture = now;
                } else {
                    std::cerr << "[WARN] Frame capture failed" << std::endl;
                    std::this_thread::sleep_for(std::chrono::seconds(1));
                }
            }
            
            // Small sleep to prevent busy waiting
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] Exception in main loop: " << e.what() << std::endl;
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    
    std::cout << "Shutting down..." << std::endl;
    cleanup();
    return 0;
}
