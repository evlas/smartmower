#ifndef PATH_PLANNING_MQTT_H
#define PATH_PLANNING_MQTT_H

#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// MQTT CONFIGURATION
// =============================================================================

// MQTT broker configuration
#define PATH_MQTT_BROKER           "localhost"
#define PATH_MQTT_PORT             1883
#define PATH_MQTT_CLIENT_ID        "path_planning_node"
#define PATH_MQTT_USERNAME         "mower"
#define PATH_MQTT_PASSWORD         "smart"
#define PATH_MQTT_BASE_TOPIC       "smartmower/path"

// =============================================================================
// MQTT TOPICS
// =============================================================================

// Published topics (path planning -> other modules)
#define PATH_TOPIC_PLAN            "/plan"              // Current path plan
#define PATH_TOPIC_WAYPOINT        "/waypoint"          // Next waypoint
#define PATH_TOPIC_PROGRESS        "/progress"          // Path execution progress
#define PATH_TOPIC_STATUS          "/status"            // Path planning system status
#define PATH_TOPIC_OBSTACLES       "/obstacles"         // Detected obstacles map

// Subscribed topics (other modules -> path planning)
#define PATH_TOPIC_REQUEST         "/request"           // Path planning requests
#define PATH_TOPIC_POSITION        "smartmower/fusion/data"     // Current robot position (fallback)
#define PATH_TOPIC_SLAM_POSE       "smartmower/slam/pose"       // SLAM robot pose (primary)
#define PATH_TOPIC_SLAM_MAP        "smartmower/slam/map"        // SLAM generated map
#define PATH_TOPIC_VISION_OBSTACLES "smartmower/vision/obstacles" // Vision obstacles
#define PATH_TOPIC_VISION_PERIMETER "smartmower/vision/perimeter" // Perimeter detection
#define PATH_TOPIC_GPS_DATA        "smartmower/gps/data"         // GPS position data
#define PATH_TOPIC_STATE_CMD       "smartmower/state/cmd"        // State machine commands

// =============================================================================
// MESSAGE TYPES
// =============================================================================

// JSON message type identifiers
#define PATH_JSON_PLAN             "path_plan"
#define PATH_JSON_WAYPOINT         "path_waypoint"
#define PATH_JSON_PROGRESS         "path_progress"
#define PATH_JSON_STATUS           "path_status"
#define PATH_JSON_REQUEST          "path_request"
#define PATH_JSON_OBSTACLES        "path_obstacles"
#define PATH_JSON_SLAM_POSE        "slam_pose"
#define PATH_JSON_SLAM_MAP         "slam_map"

// Pattern types
#define PATH_PATTERN_SPIRAL        "spiral"
#define PATH_PATTERN_LINEAR        "linear"
#define PATH_PATTERN_RANDOM        "random"
#define PATH_PATTERN_ADAPTIVE      "adaptive"

// =============================================================================
// DATA STRUCTURES
// =============================================================================

// 2D Point structure
typedef struct {
    double x;
    double y;
} Point2D;

// Waypoint structure
typedef struct {
    Point2D position;
    double heading;           // Target heading in radians
    double speed;            // Target speed in m/s
    uint32_t id;             // Unique waypoint ID
    bool is_mowing;          // Whether to mow at this waypoint
} Waypoint;

// Obstacle structure
typedef struct {
    Point2D center;
    double radius;           // Obstacle radius in meters
    uint32_t id;             // Unique obstacle ID
    bool is_permanent;       // Whether obstacle is permanent
    uint64_t timestamp;      // When obstacle was detected
} Obstacle;

// Area boundary structure
typedef struct {
    Point2D* vertices;       // Array of boundary vertices
    uint32_t vertex_count;   // Number of vertices
    double total_area;       // Total area in square meters
} AreaBoundary;

// Path plan structure
typedef struct {
    Waypoint* waypoints;     // Array of waypoints
    uint32_t waypoint_count; // Number of waypoints
    char pattern_type[32];   // Pattern type used
    double total_distance;   // Total path distance in meters
    double estimated_time;   // Estimated completion time in seconds
    uint32_t plan_id;        // Unique plan ID
    uint64_t timestamp;      // When plan was created
} PathPlan;

// SLAM pose data structure
typedef struct {
    Point2D position;                    // Robot position from SLAM
    double heading;                      // Robot heading in radians
    double confidence;                   // Pose confidence (0.0 to 1.0)
    uint64_t timestamp;                  // Timestamp of pose estimate
    bool valid;                          // Whether pose is valid
} SlamPose;

// SLAM map cell structure
typedef struct {
    uint8_t occupancy;                   // 0=free, 255=occupied, 127=unknown
    uint8_t confidence;                  // Confidence in occupancy value
    bool is_obstacle;                    // Detected obstacle
    bool is_traversable;                 // Safe for mowing
} SlamMapCell;

// SLAM map data structure
typedef struct {
    SlamMapCell* cells;                  // 2D grid of map cells
    uint32_t width;                      // Map width in cells
    uint32_t height;                     // Map height in cells
    double resolution;                   // Meters per cell
    Point2D origin;                      // Map origin in world coordinates
    uint64_t timestamp;                  // Last map update
} SlamMap;

// Path planning configuration
typedef struct {
    char pattern[32];                    // Pattern type
    double cutting_width;                // Cutting width in meters (0.30m)
    double overlap_percentage;           // Overlap percentage (0.1 = 10%)
    double max_speed;                    // Maximum speed in m/s
    double turn_radius;                  // Minimum turn radius in meters
    double obstacle_buffer;              // Buffer around obstacles in meters
    double perimeter_buffer;             // Buffer from perimeter in meters
    bool optimize_battery;               // Whether to optimize for battery
    bool avoid_wet_areas;                // Whether to avoid wet areas
    bool use_slam_data;                  // Whether to use SLAM for localization
    double slam_confidence_threshold;    // Minimum SLAM confidence to use
    uint32_t max_waypoints;              // Maximum waypoints per plan
} PathPlanningConfig;

// =============================================================================
// JSON MESSAGE STRUCTURES (Documentation)
// =============================================================================

/*
PATH PLAN MESSAGE (/plan):
{
  "type": "path_plan",
  "timestamp": <uint64_t>,
  "plan": {
    "id": <uint32_t>,
    "pattern": <string>,                 // "spiral", "linear", "random", "adaptive"
    "total_distance_m": <double>,
    "estimated_time_s": <double>,
    "waypoint_count": <uint32_t>,
    "area_coverage_m2": <double>
  },
  "waypoints": [
    {
      "id": <uint32_t>,
      "position": {
        "x": <double>,
        "y": <double>
      },
      "heading": <double>,               // Radians
      "speed": <double>,                 // m/s
      "is_mowing": <bool>
    }
  ],
  "obstacles": [
    {
      "id": <uint32_t>,
      "center": {
        "x": <double>,
        "y": <double>
      },
      "radius": <double>,
      "is_permanent": <bool>
    }
  ]
}

WAYPOINT MESSAGE (/waypoint):
{
  "type": "path_waypoint",
  "timestamp": <uint64_t>,
  "waypoint": {
    "id": <uint32_t>,
    "position": {
      "x": <double>,
      "y": <double>
    },
    "heading": <double>,
    "speed": <double>,
    "is_mowing": <bool>,
    "distance_to_target": <double>
  },
  "navigation": {
    "next_waypoint_id": <uint32_t>,
    "remaining_waypoints": <uint32_t>,
    "total_progress": <double>           // 0.0 to 1.0
  }
}

PATH PROGRESS MESSAGE (/progress):
{
  "type": "path_progress",
  "timestamp": <uint64_t>,
  "progress": {
    "current_waypoint_id": <uint32_t>,
    "completed_waypoints": <uint32_t>,
    "total_waypoints": <uint32_t>,
    "distance_completed_m": <double>,
    "distance_remaining_m": <double>,
    "area_covered_m2": <double>,
    "area_remaining_m2": <double>,
    "completion_percentage": <double>,   // 0.0 to 100.0
    "estimated_time_remaining_s": <double>
  },
  "performance": {
    "average_speed_mps": <double>,
    "battery_consumption_rate": <double>,
    "obstacles_avoided": <uint32_t>,
    "replanning_count": <uint32_t>
  }
}

PATH STATUS MESSAGE (/status):
{
  "type": "path_status",
  "timestamp": <uint64_t>,
  "system": {
    "running": <bool>,
    "planning_active": <bool>,
    "current_pattern": <string>,
    "last_replan_time": <uint64_t>
  },
  "statistics": {
    "total_plans_generated": <uint32_t>,
    "successful_completions": <uint32_t>,
    "obstacle_detections": <uint32_t>,
    "emergency_replans": <uint32_t>
  }
}

PATH REQUEST MESSAGE (/request):
{
  "type": "path_request",
  "timestamp": <uint64_t>,
  "request": {
    "command": <string>,                 // "plan", "replan", "pause", "resume", "abort"
    "pattern": <string>,                 // "spiral", "linear", "random", "adaptive"
    "priority": <string>,                // "normal", "high", "emergency"
    "area_id": <string>                  // Target area identifier
  },
  "parameters": {
    "start_position": {
      "x": <double>,
      "y": <double>
    },
    "target_area_m2": <double>,
    "max_time_s": <double>,
    "battery_constraint": <double>       // Minimum battery level to maintain
  }
}

OBSTACLES MESSAGE (/obstacles):
{
  "type": "path_obstacles",
  "timestamp": <uint64_t>,
  "obstacles": [
    {
      "id": <uint32_t>,
      "center": {
        "x": <double>,
        "y": <double>
      },
      "radius": <double>,
      "confidence": <double>,            // 0.0 to 1.0
      "is_permanent": <bool>,
      "detection_time": <uint64_t>,
      "source": <string>                // "vision", "lidar", "manual"
    }
  ],
  "map_info": {
    "total_obstacles": <uint32_t>,
    "permanent_obstacles": <uint32_t>,
    "temporary_obstacles": <uint32_t>,
    "last_update": <uint64_t>
  }
}
*/

#endif // PATH_PLANNING_MQTT_H
