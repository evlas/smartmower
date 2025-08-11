/**
 * MQTT Definitions for SLAM Node
 * 
 * Hand-written header defining MQTT topics, message types, and structures
 * used specifically by the slam_node executable.
 */

#ifndef SLAM_MQTT_H
#define SLAM_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// MQTT Configuration (read from robot_config.json)
#define SLAM_MQTT_BASE_TOPIC           "smartmower/slam"
#define SLAM_MQTT_CLIENT_ID            "slam_node"
#define SLAM_HEARTBEAT_INTERVAL_SEC    5

// =============================================================================
// PUBLISHED TOPICS (SLAM Node -> MQTT)
// =============================================================================

#define SLAM_TOPIC_MAP                 "/map"              // Generated map data
#define SLAM_TOPIC_POSE                "/pose"             // Robot pose estimate
#define SLAM_TOPIC_PATH                "/path"             // Planned path
#define SLAM_TOPIC_STATUS              "/status"           // SLAM system status

// =============================================================================
// SUBSCRIBED TOPICS (MQTT -> SLAM Node)
// =============================================================================

#define SLAM_TOPIC_CMD                 "/cmd"                       // SLAM control commands
#define SLAM_TOPIC_FUSION_DATA         "smartmower/fusion/data"     // Pose from fusion
#define SLAM_TOPIC_GPS_DATA            "smartmower/gps/data"        // GPS data
#define SLAM_TOPIC_PICO_SENSORS        "smartmower/pico/sensors"    // IMU/Magnetometer from Pico
#define SLAM_TOPIC_PICO_ODOMETRY       "smartmower/pico/odometry"   // Odometry from Pico
#define SLAM_TOPIC_VISION_OBSTACLES    "smartmower/vision/obstacles" // Obstacle detection
#define SLAM_TOPIC_VISION_PERIMETER    "smartmower/vision/perimeter" // Perimeter detection
#define SLAM_TOPIC_VISION_GRASS        "smartmower/vision/grass"    // Grass detection

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// JSON message type identifiers
#define SLAM_JSON_MAP                  "slam_map"
#define SLAM_JSON_POSE                 "slam_pose"
#define SLAM_JSON_PATH                 "slam_path"
#define SLAM_JSON_STATUS               "slam_status"

// =============================================================================
// JSON MESSAGE STRUCTURES
// =============================================================================

/*
SLAM MAP MESSAGE (/map):
{
  "type": "slam_map",
  "timestamp": <uint32_t>,
  "map": {
    "width": <int>,                          // Map width in cells
    "height": <int>,                         // Map height in cells
    "resolution": <float>,                   // Meters per cell
    "origin": {
      "x": <double>,                         // Map origin X in meters
      "y": <double>,                         // Map origin Y in meters
      "theta": <float>                       // Map origin orientation in radians
    },
    "data": "<base64_encoded_data>",         // Occupancy grid data (base64 encoded)
    "encoding": "uint8"                      // Data encoding type
  },
  "metadata": {
    "algorithm": <string>,                   // SLAM algorithm used
    "confidence": <float>,                   // Map confidence (0.0 to 1.0)
    "loop_closures": <int>                   // Number of loop closures detected
  }
}

SLAM POSE MESSAGE (/pose):
{
  "type": "slam_pose",
  "timestamp": <uint32_t>,
  "pose": {
    "position": {
      "x": <double>,                         // Position X in meters
      "y": <double>,                         // Position Y in meters
      "z": <double>                          // Position Z in meters
    },
    "orientation": {
      "w": <float>,                          // Quaternion w component
      "x": <float>,                          // Quaternion x component
      "y": <float>,                          // Quaternion y component
      "z": <float>                           // Quaternion z component
    }
  },
  "covariance": [<float>],                   // 6x6 pose covariance matrix
  "quality": {
    "confidence": <float>,                   // Pose confidence (0.0 to 1.0)
    "tracking_quality": <string>             // "good", "poor", "lost"
  }
}

SLAM PATH MESSAGE (/path):
{
  "type": "slam_path",
  "timestamp": <uint32_t>,
  "path": {
    "waypoints": [
      {
        "x": <double>,                       // Waypoint X in meters
        "y": <double>,                       // Waypoint Y in meters
        "theta": <float>                     // Waypoint orientation in radians
      }
    ],
    "total_length": <float>,                 // Total path length in meters
    "algorithm": <string>                    // Path planning algorithm used
  },
  "execution": {
    "current_waypoint": <int>,               // Current waypoint index
    "progress": <float>                      // Progress along path (0.0 to 1.0)
  }
}

SLAM STATUS MESSAGE (/status):
{
  "type": "slam_status",
  "timestamp": <uint32_t>,
  "system": {
    "running": <bool>,                       // SLAM system running status
    "mode": <string>,                        // "mapping", "localization", "slam"
    "algorithm": <string>                    // Current SLAM algorithm
  },
  "mapping": {
    "active": <bool>,                        // Mapping is active
    "map_size": <int>,                       // Current map size in cells
    "explored_area": <float>                 // Explored area in square meters
  },
  "localization": {
    "active": <bool>,                        // Localization is active
    "tracking_quality": <string>,            // "good", "poor", "lost"
    "last_loop_closure": <uint32_t>          // Timestamp of last loop closure
  },
  "performance": {
    "processing_time": <float>,              // Processing time per cycle in ms
    "memory_usage": <int>                    // Memory usage in MB
  }
}

SLAM COMMAND MESSAGE (/cmd):
{
  "command": "<command_string>",             // "start_mapping", "stop_mapping", "save_map", "load_map", "reset"
  "parameters": {
    "map_file": <string>,                    // For save_map/load_map commands
    "algorithm": <string>,                   // For algorithm changes
    "reset_map": <bool>                      // For reset command
  }
}
*/

#endif // SLAM_MQTT_H
