/**
 * Vision Camera - Header File
 * 
 * This file contains the declarations for the Vision Camera module.
 * It provides MQTT communication and camera capture functionality.
 */

#ifndef VISION_CAMERA_H
#define VISION_CAMERA_H

#include <string>
#include <vector>
#include <cstdint>

// JSON message types
#define VISION_JSON_CAMERA "camera_data"
#define VISION_JSON_INFO "camera_info"
#define VISION_JSON_ERROR "error"

// Forward declarations
struct mosquitto;

namespace vision_camera {

/**
 * @brief Configuration structure for the Vision Camera
 */
struct Config {
    // MQTT settings
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_base_topic = "smartmower";
    std::string mqtt_camera_topic = "vision/camera";
    int mqtt_qos = 1;
    bool mqtt_retain = false;
    
    // Camera settings
    int camera_id = 0;
    int width = 640;
    int height = 480;
    int fps = 15;
    int jpeg_quality = 80;
    
    // Runtime settings
    bool debug = false;
};

/**
 * @brief Main Vision Camera class
 */
class VisionCamera {
public:
    /**
     * @brief Constructor
     * @param config Configuration parameters
     */
    explicit VisionCamera(const Config& config = Config());
    
    /**
     * @brief Destructor
     */
    ~VisionCamera();
    
    /**
     * @brief Initialize the camera and MQTT connection
     * @return true if initialization was successful, false otherwise
     */
    bool initialize();
    
    /**
     * @brief Start the camera capture and publishing loop
     */
    void run();
    
    /**
     * @brief Stop the camera capture and clean up resources
     */
    void stop();
    
    /**
     * @brief Check if the camera is running
     * @return true if running, false otherwise
     */
    bool isRunning() const;

private:
    // MQTT client instance
    struct mosquitto* mosq_;
    
    // Configuration
    Config config_;
    
    // Camera capture
    void* capture_;
    
    // Running flag
    bool running_;
    
    // Private methods
    bool connectToMQTT();
    void disconnectFromMQTT();
    bool captureFrame(std::vector<uint8_t>& buffer);
    bool publishFrame(const std::vector<uint8_t>& frame_data);
};

} // namespace vision_camera

#endif // VISION_CAMERA_H
