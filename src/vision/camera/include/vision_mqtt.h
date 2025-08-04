/**
 * Vision MQTT Definitions - Smart Mower Vision System
 * 
 * Centralizes all MQTT topic definitions, message types, and JSON structures
 * for the vision system components (camera, grass, obstacle, perimeter)
 * 
 * This header serves as the single source of truth for all MQTT communication
 * within the vision subsystem.
 */

#ifndef VISION_MQTT_H
#define VISION_MQTT_H

#include <stdint.h>

// MQTT Configuration - Default values (actual values loaded from robot_config.json)
#define VISION_MQTT_BASE_TOPIC      "smartmower/vision/"
#define VISION_MQTT_KEEPALIVE       60

// NOTE: MQTT broker host, port, username, and password are now loaded
// from the unified configuration file /opt/smartmower/etc/config/robot_config.json
// These defines are kept for reference only:
// - Default broker: "localhost"
// - Default port: 1883
// - Credentials are configured in robot_config.json under system.communication

// MQTT Client IDs
#define VISION_MQTT_CLIENT_CAMERA   "smartmower_vision_camera"
#define VISION_MQTT_CLIENT_GRASS    "smartmower_vision_grass"
#define VISION_MQTT_CLIENT_OBSTACLE "smartmower_vision_obstacle"
#define VISION_MQTT_CLIENT_PERIMETER "smartmower_vision_perimeter"

// MQTT Topics - Camera
#define VISION_TOPIC_CAMERA         "smartmower/vision/camera"
#define VISION_TOPIC_CAMERA_STATUS  "smartmower/vision/camera/status"
#define VISION_TOPIC_CAMERA_CONFIG  "smartmower/vision/camera/config"

// MQTT Topics - Grass Detection
#define VISION_TOPIC_GRASS          "smartmower/vision/grass"
#define VISION_TOPIC_GRASS_STATUS   "smartmower/vision/grass/status"
#define VISION_TOPIC_GRASS_CONFIG   "smartmower/vision/grass/config"

// MQTT Topics - Obstacle Detection
#define VISION_TOPIC_OBSTACLE       "smartmower/vision/obstacle"
#define VISION_TOPIC_OBSTACLE_STATUS "smartmower/vision/obstacle/status"
#define VISION_TOPIC_OBSTACLE_CONFIG "smartmower/vision/obstacle/config"

// MQTT Topics - Perimeter Detection
#define VISION_TOPIC_PERIMETER      "smartmower/vision/perimeter"
#define VISION_TOPIC_PERIMETER_STATUS "smartmower/vision/perimeter/status"
#define VISION_TOPIC_PERIMETER_CONFIG "smartmower/vision/perimeter/config"

// JSON Message Types
#define VISION_JSON_CAMERA          "camera_data"
#define VISION_JSON_GRASS           "grass_detection"
#define VISION_JSON_OBSTACLE        "obstacle_detection"
#define VISION_JSON_PERIMETER       "perimeter_detection"
#define VISION_JSON_STATUS          "status"
#define VISION_JSON_CONFIG          "config"

// Camera Data Structure
typedef struct {
    uint64_t timestamp;
    char camera_type[32];       // "usb" or "rpi"
    int width;
    int height;
    int channels;
    char format[16];            // "BGR", "RGB", "GRAY"
    char encoding[16];          // "base64", "jpeg"
    char image_data[1024000];   // Base64 encoded image data
} vision_camera_data_t;

// Grass Detection Result
typedef struct {
    uint64_t timestamp;
    float grass_coverage;       // Percentage 0.0-100.0
    float grass_height;         // Estimated height in cm
    int grass_density;          // 0=sparse, 1=medium, 2=dense
    bool needs_cutting;
    int confidence;             // 0-100
} vision_grass_result_t;

// Obstacle Detection Result
typedef struct {
    uint64_t timestamp;
    int obstacle_count;
    struct {
        float x, y;             // Position relative to camera
        float width, height;    // Size in meters
        float distance;         // Distance in meters
        int type;               // 0=unknown, 1=static, 2=dynamic
        int confidence;         // 0-100
    } obstacles[10];            // Max 10 obstacles
} vision_obstacle_result_t;

// Perimeter Detection Result
typedef struct {
    uint64_t timestamp;
    bool perimeter_detected;
    float distance_to_perimeter; // Distance in meters
    float perimeter_angle;       // Angle relative to robot heading
    int wire_signal_strength;    // 0-100 if using wire detection
    int confidence;              // 0-100
} vision_perimeter_result_t;

// Status Structure (common for all vision components)
typedef struct {
    uint64_t timestamp;
    char component[32];         // "camera", "grass", "obstacle", "perimeter"
    char status[32];            // "running", "stopped", "error"
    bool camera_connected;
    int fps;                    // Current processing rate
    int cpu_usage;              // CPU usage percentage
    int memory_usage;           // Memory usage in MB
    char last_error[256];
} vision_status_t;

/*
 * JSON Message Examples:
 * 
 * Camera Data:
 * {
 *   "type": "camera_data",
 *   "timestamp": 1234567890123,
 *   "camera_type": "usb",
 *   "width": 640,
 *   "height": 480,
 *   "channels": 3,
 *   "format": "BGR",
 *   "encoding": "base64",
 *   "image_data": "iVBORw0KGgoAAAANSUhEUgAA..."
 * }
 * 
 * Grass Detection:
 * {
 *   "type": "grass_detection",
 *   "timestamp": 1234567890123,
 *   "grass_coverage": 85.5,
 *   "grass_height": 12.3,
 *   "grass_density": 2,
 *   "needs_cutting": true,
 *   "confidence": 92
 * }
 * 
 * Obstacle Detection:
 * {
 *   "type": "obstacle_detection",
 *   "timestamp": 1234567890123,
 *   "obstacle_count": 2,
 *   "obstacles": [
 *     {
 *       "x": 1.5, "y": 0.3,
 *       "width": 0.2, "height": 0.4,
 *       "distance": 1.6,
 *       "type": 1,
 *       "confidence": 87
 *     }
 *   ]
 * }
 * 
 * Perimeter Detection:
 * {
 *   "type": "perimeter_detection",
 *   "timestamp": 1234567890123,
 *   "perimeter_detected": true,
 *   "distance_to_perimeter": 0.5,
 *   "perimeter_angle": 15.0,
 *   "wire_signal_strength": 78,
 *   "confidence": 95
 * }
 * 
 * Status:
 * {
 *   "type": "status",
 *   "timestamp": 1234567890123,
 *   "component": "camera",
 *   "status": "running",
 *   "camera_connected": true,
 *   "fps": 30,
 *   "cpu_usage": 25,
 *   "memory_usage": 128,
 *   "last_error": ""
 * }
 */

#endif // VISION_MQTT_H
