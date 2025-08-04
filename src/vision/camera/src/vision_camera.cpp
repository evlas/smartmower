#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <mosquitto.h>
#include <cjson/cJSON.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <stdexcept>
#include <fstream>
#include <thread>
#include <chrono>

// Vision MQTT definitions
#include "vision_mqtt.h"

// Global variables
static volatile sig_atomic_t running = 1;
static struct mosquitto *mosq = nullptr;
static cv::VideoCapture* capture = nullptr;

// Configuration
struct Config {
    // MQTT settings
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_topic = std::string(VISION_MQTT_BASE_TOPIC) + VISION_TOPIC_CAMERA;
    std::string mqtt_client_id = VISION_MQTT_CLIENT_CAMERA;
    int mqtt_qos = 1;
    bool mqtt_retain = false;
    
    // Camera settings
    std::string camera_type = "usb";
    std::string camera_device = "/dev/video0";
    int camera_index = 0;
    int width = 640;
    int height = 480;
    int fps = 15;
    
    // Logging
    std::string log_level = "info";
    std::string log_file = "vision_camera.log";
};

static Config config;

// Function declarations
void handle_signal(int signal);
void cleanup();
int load_config();
int init_mqtt();
int init_camera();
int capture_and_send_frame();
std::string base64_encode(const unsigned char* data, size_t input_length);
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp);

// Base64 encoding
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

    // Add padding
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

// Send image over MQTT
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp) {
    if (!mosq) return -1;

    static uint32_t sequence = 0;
    sequence++;

    // Create JSON message following vision_mqtt.h camera image format
    std::string json_msg = 
        "{"
        "\"type\":\"" + std::string(VISION_JSON_CAMERA) + "\","
        "\"timestamp\":" + timestamp + ","
        "\"camera_id\":\"camera_0\","
        "\"image_info\":{"
            "\"width\":" + std::to_string(config.width) + ","
            "\"height\":" + std::to_string(config.height) + ","
            "\"format\":\"JPEG\","
            "\"sequence\":" + std::to_string(sequence) +
        "},"
        "\"image_data\":\"" + base64_encode(image_data.data(), image_data.size()) + "\""
        "}";

    // Publish message
    int result = mosquitto_publish(mosq, nullptr, config.mqtt_topic.c_str(), 
                                 json_msg.length(), json_msg.c_str(), 0, false);
    
    return (result == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

// Initialize camera
int init_camera() {
    try {
        // Open camera with V4L2 backend
        capture = new cv::VideoCapture(config.camera_index, cv::CAP_V4L2);
        
        if (!capture->isOpened()) {
            std::cerr << "Failed to open camera device " << config.camera_index << std::endl;
            return 0;
        }
        
        // Set camera properties
        capture->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
        capture->set(cv::CAP_PROP_FRAME_WIDTH, config.width);
        capture->set(cv::CAP_PROP_FRAME_HEIGHT, config.height);
        capture->set(cv::CAP_PROP_FPS, config.fps);
        
        // Verify settings
        double actual_width = capture->get(cv::CAP_PROP_FRAME_WIDTH);
        double actual_height = capture->get(cv::CAP_PROP_FRAME_HEIGHT);
        double actual_fps = capture->get(cv::CAP_PROP_FPS);
        
        std::cout << "Camera opened successfully" << std::endl;
        std::cout << "Resolution: " << actual_width << "x" << actual_height << std::endl;
        std::cout << "FPS: " << actual_fps << std::endl;
        
        return 1;
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV error: " << e.what() << std::endl;
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 0;
    }
}

void cleanup() {
    running = 0;
    
    // Cleanup MQTT
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = nullptr;
    }
    mosquitto_lib_cleanup();
    
    // Cleanup camera
    if (capture) {
        if (capture->isOpened()) {
            capture->release();
        }
        delete capture;
        capture = nullptr;
    }
}

void handle_signal(int signal) {
    (void)signal;
    running = 0;
}

// Load configuration from file
int load_config() {
    try {
        std::ifstream config_file("/opt/smartmower/etc/config/robot_config.json");
        if (!config_file.is_open()) {
            std::cerr << "Using default configuration (robot_config.json not found)" << std::endl;
            return 1; // Use defaults
        }

        std::string content((std::istreambuf_iterator<char>(config_file)),
                           std::istreambuf_iterator<char>());
        
        cJSON* root = cJSON_Parse(content.c_str());
        if (!root) {
            std::cerr << "Error parsing robot_config.json: " << cJSON_GetErrorPtr() << std::endl;
            return 0;
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
                
                if (cJSON_IsString(mqtt_broker_host)) config.mqtt_broker = mqtt_broker_host->valuestring;
                if (cJSON_IsNumber(mqtt_broker_port)) config.mqtt_port = mqtt_broker_port->valueint;
                if (cJSON_IsString(mqtt_username)) config.mqtt_username = mqtt_username->valuestring;
                if (cJSON_IsString(mqtt_password)) config.mqtt_password = mqtt_password->valuestring;
            }
        }
        
        // Set MQTT topic and client ID (these can remain from header)
        config.mqtt_topic = VISION_TOPIC_CAMERA;
        config.mqtt_client_id = VISION_MQTT_CLIENT_CAMERA;

        // Parse camera settings from unified structure
        cJSON* camera = cJSON_GetObjectItemCaseSensitive(root, "camera");
        if (camera) {
            // Parse common settings
            cJSON* common = cJSON_GetObjectItemCaseSensitive(camera, "common");
            if (common) {
                cJSON* width = cJSON_GetObjectItemCaseSensitive(common, "width");
                cJSON* height = cJSON_GetObjectItemCaseSensitive(common, "height");
                cJSON* fps = cJSON_GetObjectItemCaseSensitive(common, "fps");
                
                if (cJSON_IsNumber(width)) config.width = width->valueint;
                if (cJSON_IsNumber(height)) config.height = height->valueint;
                if (cJSON_IsNumber(fps)) config.fps = fps->valueint;
            }
            
            // Parse USB-specific settings
            cJSON* usb = cJSON_GetObjectItemCaseSensitive(camera, "usb");
            if (usb) {
                cJSON* type = cJSON_GetObjectItemCaseSensitive(usb, "type");
                cJSON* device = cJSON_GetObjectItemCaseSensitive(usb, "device");
                
                if (cJSON_IsString(type)) config.camera_type = type->valuestring;
                if (cJSON_IsString(device)) config.camera_device = device->valuestring;
                
                // Try to extract camera index from device path (e.g., /dev/video0 -> 0)
                if (config.camera_device.find("/dev/video") == 0) {
                    try {
                        config.camera_index = std::stoi(config.camera_device.substr(10));
                    } catch (...) {
                        // If conversion fails, keep the default index
                    }
                }
            }
        }
        
        // Parse logging settings from unified structure
        cJSON* logging = cJSON_GetObjectItemCaseSensitive(root, "logging");
        if (logging) {
            cJSON* level = cJSON_GetObjectItemCaseSensitive(logging, "level");
            if (cJSON_IsString(level)) config.log_level = level->valuestring;
            
            // Parse USB-specific logging settings
            cJSON* usb = cJSON_GetObjectItemCaseSensitive(logging, "usb");
            if (usb) {
                cJSON* file = cJSON_GetObjectItemCaseSensitive(usb, "file");
                if (cJSON_IsString(file)) config.log_file = file->valuestring;
            }
        }

        cJSON_Delete(root);
        std::cout << "Configuration loaded successfully from robot_config.json" << std::endl;
        std::cout << "MQTT: " << config.mqtt_broker << ":" << config.mqtt_port 
                  << " (user: " << config.mqtt_username << ")" << std::endl;
        return 1;
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return 0;
    }
}

// Initialize MQTT connection
int init_mqtt() {
    mosquitto_lib_init();
    
    mosq = mosquitto_new(config.mqtt_client_id.c_str(), true, NULL);
    if (!mosq) {
        std::cerr << "Failed to create mosquitto instance" << std::endl;
        return 0;
    }
    
    if (!config.mqtt_username.empty() && !config.mqtt_password.empty()) {
        if (mosquitto_username_pw_set(mosq, config.mqtt_username.c_str(), 
                                    config.mqtt_password.c_str()) != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to set MQTT credentials" << std::endl;
            return 0;
        }
        std::cout << "MQTT authentication set for user: " << config.mqtt_username << std::endl;
    }
    
    if (mosquitto_connect(mosq, config.mqtt_broker.c_str(), 
                         config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker at " 
                 << config.mqtt_broker << ":" << config.mqtt_port << std::endl;
        return 0;
    }
    
    std::cout << "Connected to MQTT broker at " << config.mqtt_broker << ":" << config.mqtt_port << std::endl;
    return 1;
}

// Capture and send frame
int capture_and_send_frame() {
    if (!capture || !capture->isOpened()) {
        std::cerr << "Camera not initialized" << std::endl;
        return -1;
    }

    try {
        // Capture frame
        cv::Mat frame;
        if (!capture->read(frame) || frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            return -1;
        }

        // Encode frame as JPEG
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", frame, buffer, params)) {
            std::cerr << "Failed to encode frame" << std::endl;
            return -1;
        }

        // Get current timestamp
        std::time_t now = std::time(nullptr);
        char timestamp[64];
        std::strftime(timestamp, sizeof(timestamp), "%Y-%m-%d %H:%M:%S", std::localtime(&now));

        // Send the image
        return send_image(buffer, timestamp);
    } catch (const cv::Exception& e) {
        std::cerr << "OpenCV exception: " << e.what() << std::endl;
        return -1;
    } catch (const std::exception& e) {
        std::cerr << "Exception: " << e.what() << std::endl;
        return -1;
    }
}

int main(int argc, char **argv) {
    (void)argc;  // Unused parameter
    (void)argv;  // Unused parameter
    
    // Set up signal handling
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Load configuration
    if (!load_config()) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }

    // Initialize MQTT
    if (!init_mqtt()) {
        std::cerr << "Failed to initialize MQTT" << std::endl;
        return 1;
    }

    // Initialize camera with retry logic
    int max_retries = 3;
    int retry_delay = 2; // seconds
    
    for (int i = 0; i < max_retries; i++) {
        if (init_camera()) {
            break;
        }
        
        if (i == max_retries - 1) {
            std::cerr << "Failed to initialize camera after " << max_retries << " attempts" << std::endl;
            cleanup();
            return 1;
        }
        
        std::cerr << "Retrying in " << retry_delay << " seconds... (" << (i+1) << "/" << max_retries << ")" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(retry_delay));
    }

    std::cout << "Vision camera started. Press Ctrl+C to stop." << std::endl;

    // Main loop
    while (running) {
        if (capture_and_send_frame() != 0) {
            std::cerr << "Error capturing/sending frame" << std::endl;
            // Small delay before retrying
            struct timespec ts = {0, 100000000}; // 100ms
            nanosleep(&ts, NULL);
        }
        
        // Small delay to prevent 100% CPU usage
        struct timespec ts = {0, 10000000}; // 10ms
        nanosleep(&ts, NULL);
    }

    // Cleanup
    cleanup();
    std::cout << "Vision camera stopped." << std::endl;
    
    return 0;
}
