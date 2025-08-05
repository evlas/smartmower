/**
 * MQTT Definitions for Pico Bridge
 * 
 * Hand-written header defining MQTT topics, message types, and structures
 * used specifically by the pico_bridge executable.
 */

#ifndef PICO_MQTT_H
#define PICO_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// MQTT Configuration - Now loaded from robot_config.json
// Default values used if config is not available
#define PICO_MQTT_DEFAULT_BROKER       "localhost"
#define PICO_MQTT_DEFAULT_PORT         1883
#define PICO_MQTT_DEFAULT_USERNAME     "mower"
#define PICO_MQTT_DEFAULT_PASSWORD     "smart"
#define PICO_MQTT_DEFAULT_BASE_TOPIC   "smartmower/pico"
#define PICO_MQTT_DEFAULT_CLIENT_ID    "pico_bridge"
#define PICO_MQTT_DEFAULT_QOS         1
#define PICO_MQTT_DEFAULT_RETAIN      false
#define PICO_MQTT_DEFAULT_KEEPALIVE   60

// Heartbeat interval (seconds)
#define PICO_HEARTBEAT_INTERVAL_SEC    5

// =============================================================================
// Topic names - Actual paths are built using configuration
// These are the default subtopic names that match the config
#define PICO_TOPIC_SENSORS             "sensors"
#define PICO_TOPIC_ODOMETRY            "odometry"
#define PICO_TOPIC_STATUS              "status"
#define PICO_TOPIC_HEARTBEAT           "heartbeat"
#define PICO_TOPIC_CMD_MOTORS          "cmd/motors"
#define PICO_TOPIC_CMD_SYSTEM          "cmd/system"

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// Binary message types (UART communication with Pico)
#define PICO_MSG_SENSOR_DATA           0x01    // Sensor data from Pico
#define PICO_MSG_STATUS_REPORT         0x02    // Status report from Pico
#define PICO_MSG_MOTOR_CMD             0x10    // Motor command to Pico
#define PICO_MSG_SYSTEM_CMD            0x11    // System command to Pico

// JSON message type identifiers
#define PICO_JSON_SENSOR_DATA          "sensor_data"
#define PICO_JSON_ODOMETRY_DATA        "odometry_data"
#define PICO_JSON_STATUS_REPORT        "status_report"
#define PICO_JSON_BRIDGE_HEARTBEAT     "pico_heartbeat"

// System commands
#define PICO_SYS_CMD_EMERGENCY_STOP    0x01    // Emergency stop
#define PICO_SYS_CMD_RESET             0x02    // System reset
#define PICO_SYS_CMD_CALIBRATE         0x03    // Calibration

// =============================================================================
// JSON MESSAGE STRUCTURES
// =============================================================================

/*
SENSOR DATA MESSAGE (/sensors):
{
  "type": "sensor_data",
  "timestamp": <uint32_t>,
  "imu": [ax, ay, az, gx, gy, gz],           // float[6] - accelerometer + gyroscope
  "magnetometer": [mx, my, mz],              // float[3] - magnetometer
  "ultrasonic": [left, center, right],       // float[3] - ultrasonic sensors
  "power": [voltage, current],               // float[2] - power measurements
  "safety": {
    "emergency_stop": <bool>,                // Emergency stop flag
    "rain_sensor": <bool>,                   // Rain sensor flag
    "bumper": <bool>,                        // Bumper sensor flag
    "lift_sensor": <bool>                    // Lift sensor flag
  }
}

ODOMETRY DATA MESSAGE (/odometry):
{
  "type": "odometry_data",
  "timestamp": <uint32_t>,
  "motors": {
    "left_speed": <float>,                   // Left motor speed
    "right_speed": <float>,                  // Right motor speed
    "blade1_speed": <float>,                 // Blade 1 motor speed
    "blade2_speed": <float>,                 // Blade 2 motor speed
    "left_rpm": <float>,                     // Left motor RPM
    "right_rpm": <float>,                    // Right motor RPM
    "blade1_rpm": <float>,                   // Blade 1 motor RPM
    "blade2_rpm": <float>                    // Blade 2 motor RPM
  },
  "encoders": [left, right, blade1, blade2], // uint32_t[4] - encoder counts
  "system": {
    "running": true,                         // System running status
    "relay_state": <bool>                    // Relay state
  }
}

BRIDGE STATUS MESSAGE (/status):
{
  "type": "pico_status",
  "timestamp": <uint32_t>,
  "system": {
    "running": true,                         // Bridge running status
    "uart_connected": <bool>,                // UART connection status
    "communication_ok": <bool>               // Communication with Pico OK
  },
  "performance": {
    "update_rate": 1,                        // Status update rate (Hz)
    "data_age": <uint32_t>                   // Timestamp of last data
  }
}

BRIDGE HEARTBEAT MESSAGE (/bridge/status):
{
  "type": "bridge_heartbeat",
  "timestamp": <timestamp>,
  "system": {
    "running": true,                         // Bridge running status
    "communication_ok": true,                // Communication with Pico OK
    "relay_state": false                     // Relay state
  }
}

MOTOR COMMAND MESSAGE (/cmd/motors):
{
  "left_speed": <float>,                     // Left motor speed (-100 to +100)
  "right_speed": <float>,                    // Right motor speed (-100 to +100)
  "blade1_speed": <float>,                   // Blade 1 speed (0 to 100)
  "blade2_speed": <float>                    // Blade 2 speed (0 to 100)
}

SYSTEM COMMAND MESSAGE (/cmd/system):
{
  "command": "<command_string>",             // "emergency_stop", "reset", "calibrate"
  "value": <float>                           // Optional command value
}
*/

// =============================================================================
// BINARY STRUCTURES (UART Communication)
// =============================================================================

#pragma pack(push, 1)

// Sensor data structure (from Pico to bridge)
typedef struct {
    uint8_t type;           // PICO_MSG_SENSOR_DATA
    uint32_t timestamp;     // Timestamp in ms
    float imu[6];          // Accelerometer + gyroscope [ax,ay,az,gx,gy,gz]
    float magnetometer[3]; // Magnetometer [mx,my,mz]
    float ultrasonic[3];   // 3 ultrasonic sensors [left,center,right]
    float power[2];        // Voltage, current [V,A]
    uint8_t flags[4];      // emergency_stop, rain, bumper, lift
} pico_sensor_data_t;

// Status report structure (from Pico to bridge)
typedef struct {
    uint8_t type;          // PICO_MSG_STATUS_REPORT
    uint32_t timestamp;    // Timestamp in ms
    float motor_speeds[4]; // Current speeds [left,right,blade1,blade2]
    float motor_rpm[4];    // Current RPM [left,right,blade1,blade2]
    uint32_t encoder_counts[4]; // Encoder counters
    uint8_t system_flags;  // System flags
    uint8_t relay_state;   // Relay state
} pico_status_report_t;

// Motor command structure (from bridge to Pico)
typedef struct {
    uint8_t type;          // PICO_MSG_MOTOR_CMD
    float left_speed;      // Left motor speed (-100 to +100)
    float right_speed;     // Right motor speed (-100 to +100)
    float blade1_speed;    // Blade 1 speed (0 to 100)
    float blade2_speed;    // Blade 2 speed (0 to 100)
} pico_motor_command_t;

// System command structure (from bridge to Pico)
typedef struct {
    uint8_t type;          // PICO_MSG_SYSTEM_CMD
    uint8_t command_id;    // PICO_SYS_CMD_*
    float value;           // Optional value
} pico_system_command_t;

#pragma pack(pop)

#endif // PICO_MQTT_H
