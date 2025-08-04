/**
 * MQTT Definitions for GPS Bridge
 * 
 * Hand-written header defining MQTT topics, message types, and structures
 * used specifically by the gps_bridge executable.
 */

#ifndef GPS_MQTT_H
#define GPS_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// CONFIGURATION
// =============================================================================

// MQTT Broker Configuration
#define GPS_MQTT_BROKER                "localhost"
#define GPS_MQTT_PORT                  1883
#define GPS_MQTT_USERNAME              "mower"
#define GPS_MQTT_PASSWORD              "smart"
#define GPS_MQTT_BASE_TOPIC            "smartmower/gps"
#define GPS_MQTT_CLIENT_ID             "gps_bridge"
#define GPS_HEARTBEAT_INTERVAL_SEC     5

// =============================================================================
// PUBLISHED TOPICS (GPS Bridge -> MQTT)
// =============================================================================

#define GPS_TOPIC_DATA                 "/data"             // GPS position and navigation data
#define GPS_TOPIC_STATUS               "/status"           // GPS system status
#define GPS_TOPIC_BRIDGE_HEARTBEAT     "/heartbeat"    // Bridge heartbeat

// =============================================================================
// SUBSCRIBED TOPICS (MQTT -> GPS Bridge)
// =============================================================================

#define GPS_TOPIC_CMD                  "/cmd"              // GPS control commands

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// JSON message type identifiers
#define GPS_JSON_DATA                  "gps_data"
#define GPS_JSON_STATUS                "gps_status"

// =============================================================================
// JSON MESSAGE STRUCTURES
// =============================================================================

/*
GPS DATA MESSAGE (/data):
{
  "type": "gps_data",
  "timestamp": <uint32_t>,                   // Timestamp in milliseconds
  "position": {
    "latitude": <double>,                    // Latitude in degrees
    "longitude": <double>,                   // Longitude in degrees
    "altitude": <float>                      // Altitude in meters above sea level
  },
  "quality": {
    "fix_type": <int>,                       // GPS fix type (0=no fix, 1=GPS, 2=DGPS, 3=RTK)
    "satellites": <int>,                     // Number of satellites in use
    "hdop": <float>,                         // Horizontal dilution of precision
    "vdop": <float>                          // Vertical dilution of precision
  },
  "velocity": {
    "speed": <float>,                        // Speed over ground in m/s
    "heading": <float>,                      // Course over ground in degrees
    "climb": <float>                         // Vertical velocity in m/s
  },
  "accuracy": {
    "horizontal": <float>,                   // Horizontal accuracy in meters
    "vertical": <float>                      // Vertical accuracy in meters
  }
}

GPS STATUS MESSAGE (/status):
{
  "type": "gps_status",
  "timestamp": <uint32_t>,
  "system": {
    "running": <bool>,                       // GPS system running status
    "uart_connected": <bool>,                // UART connection to GPS module status
    "communication_ok": <bool>,              // Communication with GPS module OK
    "last_fix_time": <uint32_t>,             // Timestamp of last valid fix
    "module_type": <string>                  // GPS module type/model
  },
  "performance": {
    "update_rate": <float>,                  // Current update rate in Hz
    "data_age": <float>                      // Age of current data in seconds
  }
}

GPS COMMAND MESSAGE (/cmd):
{
  "command": "<command_string>",             // "reset", "cold_start", "warm_start", "set_rate"
  "parameters": {                            // Optional command parameters
    "rate": <int>                            // For set_rate command (Hz)
  }
}
*/

#endif // GPS_MQTT_H
