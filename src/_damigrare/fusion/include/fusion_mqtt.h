/**
 * MQTT Definitions for Fusion Sensor
 * 
 * Hand-written header defining MQTT topics, message types, and structures
 * used specifically by the fusion_sensor executable.
 */

#ifndef FUSION_MQTT_H
#define FUSION_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// MQTT Configuration (read from robot_config.json)
#define FUSION_MQTT_BASE_TOPIC         "smartmower/fusion"
#define FUSION_MQTT_CLIENT_ID          "fusion_sensor"
#define FUSION_HEARTBEAT_INTERVAL_SEC  5

// =============================================================================
// PUBLISHED TOPICS (Fusion Sensor -> MQTT)
// =============================================================================

#define FUSION_TOPIC_DATA              "/data"             // Fused sensor output
#define FUSION_TOPIC_STATUS            "/status"           // Fusion system status

// =============================================================================
// SUBSCRIBED TOPICS (MQTT -> Fusion Sensor)
// =============================================================================

#define FUSION_TOPIC_CMD               "/cmd"                           // Fusion control commands
#define FUSION_TOPIC_IMU_DATA          "smartmower/pico/sensors"       // IMU and magnetometer data from Pico
#define FUSION_TOPIC_GPS_DATA          "smartmower/gps/data"           // GPS position and navigation data from GPS bridge
#define FUSION_TOPIC_ODOMETRY_DATA     "smartmower/pico/odometry"      // Motor encoder ticks/odometry data from Pico
#define FUSION_TOPIC_PICO_STATUS       "smartmower/pico/status"        // PICO system status (battery, comms, etc.)
#define FUSION_TOPIC_GPS_STATUS        "smartmower/gps/status"         // GPS module status and health
#define FUSION_TOPIC_VISION_OBSTACLE   "smartmower/vision/obstacles"   // Vision obstacle detection results
#define FUSION_TOPIC_VISION_PERIMETER  "smartmower/vision/perimeter"    // Vision perimeter detection results
#define FUSION_TOPIC_VISION_GRASS      "smartmower/vision/grass"        // Vision grass detection results

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// JSON message type identifiers
#define FUSION_JSON_DATA               "fusion_data"
#define FUSION_JSON_STATUS             "fusion_status"

// =============================================================================
// JSON MESSAGE STRUCTURES
// =============================================================================

/*
FUSION DATA MESSAGE (/data):
{
  "type": "fusion_data",
  "timestamp": <uint64_t>,                   // Timestamp in milliseconds since epoch
  "position": {
    "x": <double>,                          // Position X in meters (local frame)
    "y": <double>,                          // Position Y in meters (local frame)
    "z": <double>                           // Position Z in meters (altitude)
  },
  "velocity": {
    "vx": <double>,                         // Linear velocity X in m/s
    "vy": <double>,                         // Linear velocity Y in m/s
    "vz": <double>,                         // Linear velocity Z in m/s
    "speed": <double>                       // Speed magnitude in m/s
  },
  "orientation": {
    "roll": <double>,                       // Roll angle in radians
    "pitch": <double>,                      // Pitch angle in radians
    "yaw": <double>                         // Yaw angle in radians
  },
  "uncertainty": {
    "position_x": <double>,                 // Position X standard deviation (meters)
    "position_y": <double>,                 // Position Y standard deviation (meters)
    "velocity_x": <double>,                 // Velocity X standard deviation (m/s)
    "velocity_y": <double>,                 // Velocity Y standard deviation (m/s)
    "yaw": <double>                         // Yaw standard deviation (radians)
  }
}

FUSION STATUS MESSAGE (/status):
{
  "type": "fusion_status",
  "timestamp": <uint32_t>,
  "system": {
    "running": <bool>,                       // Fusion system running status
    "filter_initialized": <bool>,            // Kalman filter initialized
    "filter_type": <string>                  // Filter type (EKF, UKF, etc.)
  },
  "input_sources": {
    "imu": {
      "available": <bool>,                   // IMU data available
      "last_update": <uint32_t>,             // Last IMU update timestamp
      "rate": <float>                        // IMU update rate in Hz
    },
    "gps": {
      "available": <bool>,                   // GPS data available
      "last_update": <uint32_t>,             // Last GPS update timestamp
      "quality": <int>                       // GPS fix quality
    },
    "encoders": {
      "available": <bool>,                   // Encoder data available
      "last_update": <uint32_t>              // Last encoder update timestamp
    }
  },
  "performance": {
    "processing_time": <float>,              // Processing time per cycle in ms
    "update_rate": <float>                   // Output update rate in Hz
  }
}

FUSION COMMAND MESSAGE (/cmd):
{
  "command": "<command_string>",             // "reset", "calibrate", "set_mode"
  "parameters": {
    "mode": <string>,                        // For set_mode: "gps", "dead_reckoning", "fusion"
    "reset_position": <bool>                 // For reset: reset position estimate
  }
}
*/

#endif // FUSION_MQTT_H
