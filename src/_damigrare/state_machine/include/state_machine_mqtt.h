/**
 * MQTT Definitions for State Machine
 * 
 * Hand-written header defining MQTT topics, message types, and structures
 * used specifically by the state_machine executable.
 */

#ifndef STATE_MACHINE_MQTT_H
#define STATE_MACHINE_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// MQTT Broker Configuration
#define STATE_MQTT_BROKER              "localhost"
#define STATE_MQTT_PORT                1883
#define STATE_MQTT_USERNAME            "mower"
#define STATE_MQTT_PASSWORD            "smart"
#define STATE_MQTT_BASE_TOPIC          "smartmower/state"
#define STATE_MQTT_CLIENT_ID           "state_machine"
#define STATE_HEARTBEAT_INTERVAL_SEC   2

// =============================================================================
// PUBLISHED TOPICS (State Machine -> MQTT)
// =============================================================================

#define STATE_TOPIC_CURRENT            "/current"          // Current state information
#define STATE_TOPIC_TRANSITIONS        "/transitions"      // State transition events
#define STATE_TOPIC_STATUS             "/status"           // State machine status
#define STATE_TOPIC_COMMANDS           "/commands"         // Commands to other systems

// =============================================================================
// SUBSCRIBED TOPICS (MQTT -> State Machine)
// =============================================================================

#define STATE_TOPIC_CMD                "/cmd"                       // State machine control commands
#define STATE_TOPIC_PICO_SENSORS       "smartmower/pico/sensors"    // IMU/Magnetometer from Pico
#define STATE_TOPIC_PICO_STATUS        "smartmower/pico/status"     // Pico status and odometry
#define STATE_TOPIC_PICO_HEARTBEAT     "smartmower/pico/heartbeat"  // Pico heartbeat
#define STATE_TOPIC_GPS_DATA           "smartmower/gps/data"        // GPS position data
#define STATE_TOPIC_GPS_HEARTBEAT      "smartmower/gps/heartbeat"   // GPS heartbeat
#define STATE_TOPIC_FUSION_DATA        "smartmower/fusion/data"     // Fused sensor data
#define STATE_TOPIC_FUSION_STATUS      "smartmower/fusion/status"   // Fusion system status
#define STATE_TOPIC_VISION_OBSTACLES   "smartmower/vision/obstacles" // Obstacle detection
#define STATE_TOPIC_VISION_PERIMETER   "smartmower/vision/perimeter" // Perimeter detection
#define STATE_TOPIC_VISION_GRASS       "smartmower/vision/grass"    // Grass detection
#define STATE_TOPIC_SLAM_POSE          "smartmower/slam/pose"       // SLAM pose estimate
#define STATE_TOPIC_SLAM_MAP           "smartmower/slam/map"        // SLAM map data
#define STATE_TOPIC_SLAM_STATUS        "smartmower/slam/status"     // SLAM system status

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// JSON message type identifiers
#define STATE_JSON_CURRENT             "state_current"
#define STATE_JSON_TRANSITION          "state_transition"
#define STATE_JSON_STATUS              "state_status"
#define STATE_JSON_COMMAND             "state_command"

// State string definitions for JSON messages
#define STATE_STR_IDLE                 "idle"
#define STATE_STR_MANUAL               "manual"
#define STATE_STR_AUTO_MOWING          "auto_mowing"
#define STATE_STR_RETURNING_HOME       "returning_home"
#define STATE_STR_CHARGING             "charging"
#define STATE_STR_ERROR                "error"
#define STATE_STR_EMERGENCY_STOP       "emergency_stop"
#define STATE_STR_MAINTENANCE          "maintenance"

// =============================================================================
// JSON MESSAGE STRUCTURES
// =============================================================================

/*
CURRENT STATE MESSAGE (/current):
{
  "type": "state_current",
  "timestamp": <uint32_t>,
  "state": {
    "name": <string>,                        // Current state name
    "duration": <uint32_t>,                  // Time in current state (seconds)
    "sub_state": <string>,                   // Sub-state if applicable
    "progress": <float>                      // Progress in current task (0.0 to 1.0)
  },
  "context": {
    "battery_level": <float>,                // Battery level (0.0 to 100.0)
    "mowing_area": <string>,                 // Current mowing area
    "weather_condition": <string>,           // Weather condition
    "safety_status": <string>                // "safe", "caution", "danger"
  },
  "next_actions": [<string>]                 // Possible next actions
}

STATE TRANSITION MESSAGE (/transitions):
{
  "type": "state_transition",
  "timestamp": <uint32_t>,
  "transition": {
    "from_state": <string>,                  // Previous state
    "to_state": <string>,                    // New state
    "trigger": <string>,                     // What triggered the transition
    "reason": <string>                       // Human-readable reason
  },
  "conditions": {
    "battery_level": <float>,                // Battery level at transition
    "safety_checks": <bool>,                 // Safety checks passed
    "weather_ok": <bool>                     // Weather conditions OK
  }
}

STATE MACHINE STATUS MESSAGE (/status):
{
  "type": "state_status",
  "timestamp": <uint32_t>,
  "system": {
    "running": <bool>,                       // State machine running
    "current_state": <string>,               // Current state
    "uptime": <uint32_t>,                    // System uptime in seconds
    "total_transitions": <uint32_t>          // Total state transitions
  },
  "statistics": {
    "mowing_time_today": <uint32_t>,         // Mowing time today in seconds
    "total_mowing_time": <uint32_t>,         // Total mowing time in seconds
    "areas_completed": <int>,                // Areas completed today
    "emergency_stops": <int>                 // Emergency stops today
  },
  "health": {
    "all_systems_ok": <bool>,                // All systems operational
    "warnings": [<string>],                  // Active warnings
    "errors": [<string>]                     // Active errors
  }
}

STATE COMMAND MESSAGE (/commands):
{
  "type": "state_command",
  "timestamp": <uint32_t>,
  "target_system": <string>,                 // "pico", "gps", "fusion", "vision", "slam"
  "command": {
    "action": <string>,                      // Command action
    "parameters": {}                         // Command parameters (varies by system)
  },
  "priority": <string>,                      // "low", "normal", "high", "emergency"
  "timeout": <uint32_t>                      // Command timeout in seconds
}

STATE MACHINE CONTROL MESSAGE (/cmd):
{
  "command": "<command_string>",             // "start", "stop", "pause", "resume", "emergency_stop", "go_home", "set_mode"
  "parameters": {
    "mode": <string>,                        // "auto", "manual", "test"
    "area": <string>,                        // Target mowing area
    "schedule": <bool>                       // Follow schedule
  },
  "override": {
    "safety": <bool>,                        // Override safety checks (dangerous!)
    "weather": <bool>,                       // Override weather checks
    "battery": <bool>                        // Override battery checks
  }
}
*/

#endif // STATE_MACHINE_MQTT_H
