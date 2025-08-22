#include <iostream>
#include <csignal>
#include <memory>
#include <thread>
#include <chrono>
#include <unordered_map>
#include <string>
#include <nlohmann/json.hpp>
#include "config/config_manager.h"
#include "camera/camera_interface.h"
#include "camera/usb_camera.h"
#include "camera/rpi_camera.h"
#include "camera/logging.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"

using json = nlohmann::json;

using namespace camera;
using namespace mqtt;
using namespace mqtt::topics;

static bool running = true;
void handle_signal(int) { running = false; }

int main(int argc, char** argv) {
    std::string config_path = "/opt/smartmower/etc/config/robot_config.json";
    if (argc > 1) config_path = argv[1];

    // Initialize logger first
    auto& logger = camera::Logger::getInstance();
    std::unique_ptr<camera::CameraInterface> cam;
    mqtt::MqttClient* client = nullptr;
    
    try {
        config::ConfigManager cfg(config_path);
        if (!cfg.load()) {
            LOG_CRITICAL("Failed to load config: " + config_path);
            return 1;
        }
        
        // Initialize logging from config
        const auto& vision_logging = cfg.getJsonObject("vision_logging");
        if (!logger.initialize(vision_logging)) {
            std::cerr << "Failed to initialize logger" << std::endl;
            return 1;
        }
        
        LOG_INFO("Starting Vision Camera module");
        LOG_DEBUG("Configuration loaded from: " + config_path);

        // Initialize MQTT topics
        auto& topicManager = mqtt::topics::TopicManager::getInstance();
        std::string rootTopic = cfg.getString("mqtt.root_topic", "smartmower");
        
        // Get camera topics
        auto cameraTopics = cfg.getObject("mqtt.topics.camera.subtopics");
        std::unordered_map<std::string, std::string> cameraSubtopics;
        for (const auto& [key, value] : cameraTopics) {
            cameraSubtopics[key] = value;
        }
        
        std::string baseTopic = cfg.getString("mqtt.topics.camera.base", "vision/camera");
        topicManager.initialize(rootTopic, baseTopic, cameraSubtopics);
        
        LOG_INFO("MQTT topics initialized. Base topic: " + rootTopic + "/" + baseTopic);

        // Initialize MQTT client
        std::string mqtt_broker = cfg.getString("mqtt.broker", "localhost");
        int mqtt_port = cfg.getInt("mqtt.port", 1883);
        std::string mqtt_username = cfg.getString("mqtt.username", "");
        std::string mqtt_password = cfg.getString("mqtt.password", "");
        std::string mqtt_client_id = "vision_camera_" + std::to_string(getpid());
        
        LOG_INFO("Initializing MQTT client with broker: " + mqtt_broker + ":" + std::to_string(mqtt_port));
        
        try {
            client = new mqtt::MqttClient(
                mqtt_broker,
                mqtt_port,
                mqtt_client_id,
                mqtt_username,
                mqtt_password
            );
            
            LOG_INFO("Connecting to MQTT broker at " + mqtt_broker + ":" + std::to_string(mqtt_port));
            if (!client->connect()) {
                LOG_CRITICAL("Failed to connect to MQTT broker");
                delete client;
                client = nullptr;
                return 1;
            }
            
            LOG_INFO("Successfully connected to MQTT broker");
            
            // Subscribe to commands
            if (!client->subscribe(topicManager.commands(), 1)) {
                LOG_ERROR("Failed to subscribe to commands topic");
                delete client;
                client = nullptr;
                return 1;
            }
        } catch (const std::exception& e) {
            LOG_CRITICAL("Error initializing MQTT client: " + std::string(e.what()));
            if (client) {
                delete client;
                client = nullptr;
            }
            return 1;
        }

        LOG_INFO("Initializing camera with configuration from vision_config");
        const auto& vision_cfg = cfg.getObject("vision_config.camera");
        
        // Get camera parameters with defaults
        std::string device = vision_cfg.count("device") ? vision_cfg.at("device") : "/dev/video0";
        int width = vision_cfg.count("width") ? std::stoi(vision_cfg.at("width")) : 640;
        int height = vision_cfg.count("height") ? std::stoi(vision_cfg.at("height")) : 480;
        int fps = vision_cfg.count("fps") ? std::stoi(vision_cfg.at("fps")) : 30;
        int rotation = vision_cfg.count("rotation") ? std::stoi(vision_cfg.at("rotation")) : 0;
        std::string flip_mode = vision_cfg.count("flip") ? vision_cfg.at("flip") : "none";
        float height_m = vision_cfg.count("height_m") ? std::stof(vision_cfg.at("height_m")) : 0.3f;
        
        LOG_DEBUG("Camera parameters - Device: " + device + 
                 ", Resolution: " + std::to_string(width) + "x" + std::to_string(height) +
                 ", FPS: " + std::to_string(fps) + 
                 ", Rotation: " + std::to_string(rotation) + 
                 ", Flip: " + flip_mode);
        
        // Get camera intrinsics
        const auto& intrinsics = cfg.getObject("vision_config.camera.intrinsics");
        float fx = intrinsics.count("focal_length_x") ? std::stof(intrinsics.at("focal_length_x")) : 350.0f;
        float fy = intrinsics.count("focal_length_y") ? std::stof(intrinsics.at("focal_length_y")) : 350.0f;
        float cx = intrinsics.count("principal_point_x") ? std::stof(intrinsics.at("principal_point_x")) : 320.0f;
        float cy = intrinsics.count("principal_point_y") ? std::stof(intrinsics.at("principal_point_y")) : 240.0f;
        
        LOG_DEBUG("Camera intrinsics - fx: " + std::to_string(fx) + 
                 ", fy: " + std::to_string(fy) + 
                 ", cx: " + std::to_string(cx) + 
                 ", cy: " + std::to_string(cy));
        
        // Get camera type from config (default to 'usb' if not specified)
        std::string camera_type = vision_cfg.count("type") ? vision_cfg.at("type") : "usb";
        
        // Initialize appropriate camera based on type
        std::unique_ptr<camera::CameraInterface> cam;
        
        if (camera_type == "rpi") {
            // Raspberry Pi camera
            LOG_INFO("Initializing Raspberry Pi camera");
            cam = std::make_unique<camera::RPiCamera>(
                width,   // width
                height,  // height
                fps      // fps
            );
        } else {
            // Default to USB camera
            int device_index = 0;
            try {
                device_index = std::stoi(device.substr(device.find_last_not_of("0123456789") + 1));
            } catch (const std::exception& e) {
                LOG_WARNING("Failed to parse device index from " + device + ", using default 0");
                device_index = 0;
            }
            
            LOG_INFO("Initializing " + camera_type + " camera: " + device + " (index: " + std::to_string(device_index) + ")");
            cam = std::make_unique<camera::USBCamera>(
                device_index,  // device index
                width,         // width
                height,        // height
                fps            // fps
            );
        }

        LOG_INFO("Initializing camera hardware...");
        if (!cam->initialize()) {
            LOG_CRITICAL("Failed to initialize camera");
            return 1;
        }
        LOG_INFO("Camera successfully initialized");

        // Set up signal handling
        std::signal(SIGINT, handle_signal);
        std::signal(SIGTERM, handle_signal);

        // Main loop
        LOG_INFO("Starting main camera loop");
        auto last_status_time = std::chrono::steady_clock::now();
        int frame_count = 0;
        float actual_fps = 0.0f;
    
        while (running) {
            auto loop_start = std::chrono::steady_clock::now();
            
            try {
                // Process MQTT messages
                if (client) {
                    client->loop(0);
                }
                // Capture frame
                std::vector<uint8_t> frame;
                if (cam->captureFrame(frame)) {
                    frame_count++;
                    
                    // Publish frame
                    if (!client->publish(topicManager.data(), frame)) {
                        LOG_ERROR("Failed to publish frame data to MQTT");
                    }
                } else {
                    LOG_ERROR("Failed to capture frame from camera");
                }
            
                // Calculate and publish status at 1Hz
                auto now = std::chrono::steady_clock::now();
                auto status_elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
                    now - last_status_time).count();
                    
                if (status_elapsed >= 1000) {  // 1 second has passed
                    // Calculate actual FPS
                    actual_fps = frame_count * 1000.0f / status_elapsed;
                    
                    LOG_DEBUG("Current FPS: " + std::to_string(actual_fps) + 
                             ", Target FPS: " + std::to_string(fps) +
                             ", Frame count: " + std::to_string(frame_count));
                    
                    // Prepare status message with all camera parameters
                    json status_msg;
                    status_msg["online"] = true;
                    status_msg["device"] = device;
                    status_msg["width"] = width;
                    status_msg["height"] = height;
                    status_msg["fps"] = actual_fps;
                    status_msg["rotation"] = rotation;
                    status_msg["flip"] = flip_mode;
                    status_msg["height_m"] = height_m;
                    
                    // Add camera intrinsics
                    json intrinsics_msg;
                    intrinsics_msg["focal_length_x"] = fx;
                    intrinsics_msg["focal_length_y"] = fy;
                    intrinsics_msg["principal_point_x"] = cx;
                    intrinsics_msg["principal_point_y"] = cy;
                    status_msg["intrinsics"] = intrinsics_msg;
                    
                    // Publish status
                    std::string status_str = status_msg.dump();
                    if (!client->publish(topicManager.status(), status_str)) {
                        LOG_ERROR("Failed to publish status to MQTT");
                    } else {
                        LOG_DEBUG("Published status: " + status_str);
                    }
                    
                    // Reset counters
                    frame_count = 0;
                    last_status_time = now;
                }
                
                // Calculate sleep time to maintain desired FPS
                auto loop_time = std::chrono::duration_cast<std::chrono::milliseconds>(
                    std::chrono::steady_clock::now() - loop_start).count();
                int target_frame_time = 1000 / fps;
                int sleep_time = std::max(1, target_frame_time - static_cast<int>(loop_time));
                
                if (loop_time > target_frame_time * 1.1) {
                    LOG_WARNING("Frame processing time (" + std::to_string(loop_time) + 
                               "ms) exceeds target frame time (" + 
                               std::to_string(target_frame_time) + "ms)");
                }
                
                LOG_DEBUG("Frame processed in " + std::to_string(loop_time) + 
                         "ms, sleeping for " + std::to_string(sleep_time) + "ms");
                
            } catch (const std::exception& e) {
                LOG_ERROR("Error in main loop: " + std::string(e.what()));
                // Attempt to recover by reinitializing the camera
                std::this_thread::sleep_for(std::chrono::seconds(1));
                LOG_INFO("Attempting to reinitialize camera...");
                if (!cam->initialize()) {
                    LOG_CRITICAL("Failed to reinitialize camera");
                    running = false;  // Exit the main loop on critical error
                } else {
                    LOG_INFO("Camera reinitialized successfully");
                }
            }
            
            // Calculate sleep time to maintain desired FPS
            auto loop_end = std::chrono::steady_clock::now();
            auto loop_duration = std::chrono::duration_cast<std::chrono::milliseconds>(
                loop_end - loop_start).count();
            int target_frame_time = 1000 / fps;
            int sleep_time = std::max(1, target_frame_time - static_cast<int>(loop_duration));
            
            if (loop_duration > target_frame_time * 1.1) {
                LOG_WARNING("Frame processing time (" + std::to_string(loop_duration) + 
                          "ms) exceeds target frame time (" + 
                          std::to_string(target_frame_time) + "ms)");
            }

            LOG_DEBUG("Frame processed in " + std::to_string(loop_duration) + 
                     "ms, sleeping for " + std::to_string(sleep_time) + "ms");

            std::this_thread::sleep_for(std::chrono::milliseconds(sleep_time));
        } // End of while(running) loop
        
        // Cleanup
        if (cam) {
            cam->shutdown();
        }
        
        if (client) {
            client->disconnect();
            delete client;
            client = nullptr;
        }
        
        LOG_INFO("Vision Camera module stopped gracefully");
    } catch (const std::exception& e) {
        LOG_CRITICAL("Fatal error: " + std::string(e.what()));
        if (client) {
            client->disconnect();
            delete client;
            client = nullptr;
        }
        return 1;
    }

    return 0;
}
