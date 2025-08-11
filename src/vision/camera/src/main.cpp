#include <iostream>
#include <csignal>
#include <memory>
#include <thread>
#include <chrono>
#include <unordered_map>
#include "config/config_manager.h"
#include "camera/camera_interface.h"
#include "camera/usb_camera.h"
#include "camera/rpi_camera.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"

using namespace camera;
using namespace mqtt;
using namespace mqtt::topics;

static bool running = true;
void handle_signal(int) { running = false; }

int main(int argc, char** argv) {
    std::string config_path = "/opt/smartmower/etc/config/robot_config.json";
    if (argc > 1) config_path = argv[1];

    config::ConfigManager cfg(config_path);
    if (!cfg.load()) {
        std::cerr << "Failed to load config: " << config_path << std::endl;
        return 1;
    }

    // Initialize MQTT topics
    auto& topicManager = mqtt::topics::TopicManager::getInstance();
    std::string rootTopic = cfg.getString("mqtt.root_topic", "smartmower");
    
    // Get camera topics
    auto cameraTopics = cfg.getObject("mqtt.topics.camera.subtopics");
    std::unordered_map<std::string, std::string> cameraSubtopics;
    for (const auto& [key, value] : cameraTopics) {
        cameraSubtopics[key] = value;
    }
    
    topicManager.initialize(
        rootTopic,
        cfg.getString("mqtt.topics.camera.base", "vision/camera"),
        cameraSubtopics
    );

    // Initialize MQTT client
    std::string broker = cfg.getString("mqtt.broker", "localhost");
    int port = cfg.getInt("mqtt.port", 1883);
    std::string clientId = "vision_camera_" + std::to_string(getpid());
    std::string user = cfg.getString("mqtt.username", "");
    std::string pass = cfg.getString("mqtt.password", "");
    
    mqtt::MqttClient client(broker, port, clientId, user, pass);
    if (!client.connect()) {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
        return 1;
    }
    
    // Subscribe to commands
    client.subscribe(topicManager.commands());

    // Initialize camera
    std::string type = cfg.getString("camera.type", "rpi");
    int width = cfg.getInt("camera.width", 1280);
    int height = cfg.getInt("camera.height", 720);
    int fps = cfg.getInt("camera.fps", 15);
    
    std::unique_ptr<camera::CameraInterface> cam;
    if (type == "usb") {
        cam = std::make_unique<camera::USBCamera>(cfg.getInt("camera.index", 0), width, height, fps);
    } else {
        cam = std::make_unique<camera::RPiCamera>(width, height, fps);
    }

    if (!cam->initialize()) {
        std::cerr << "Failed to initialize camera" << std::endl;
        return 1;
    }

    // Set up signal handling
    std::signal(SIGINT, handle_signal);
    std::signal(SIGTERM, handle_signal);

    // Main loop
    while (running) {
        std::vector<uint8_t> frame;
        if (cam->captureFrame(frame)) {
            // Publish frame
            client.publish(topicManager.data(), frame);
            
            // Publish status (fps, resolution, etc.)
            std::string status = R"({"online": true, "fps": )" + 
                               std::to_string(fps) + 
                               ", \"width\": " + std::to_string(width) +
                               ", \"height\": " + std::to_string(height) + "}";
            client.publish(topicManager.status(), status);
        }
        
        // Handle MQTT messages
        client.loop(0);
        
        // Sleep to maintain desired FPS
        std::this_thread::sleep_for(std::chrono::milliseconds(1000 / fps));
    }

    // Cleanup
    cam->shutdown();
    client.disconnect();
    
    return 0;
}
