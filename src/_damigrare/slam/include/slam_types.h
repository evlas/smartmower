// slam_types.h - Fundamental data structures for SLAM

#ifndef SLAM_TYPES_H
#define SLAM_TYPES_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <string>
#include <vector>
#include <mutex>
#include <atomic>
#include <Eigen/Dense>
#include <json-c/json.h>

// System constants
#define MAP_WIDTH 200          // Map 200x200 cells (20m x 20m if 1 cell=10cm)
#define MAP_HEIGHT 200
#define CELL_SIZE 0.1f         // Each cell = 10cm
#define MAX_LANDMARKS 100
#define MAX_SONAR_RANGE 4.0f   // 4 meters max sonar range
#define GPS_TIMEOUT_MS 5000    // GPS timeout

// GPS fix types
typedef enum {
    GPS_FIX_NONE = 0,    // No fix
    GPS_FIX_TIME_ONLY = 1, // Time only
    GPS_FIX_2D = 2,      // 2D fix (lat/lon)
    GPS_FIX_3D = 3       // 3D fix (lat/lon/alt)
} gps_fix_type_t;

// Basic pose structure
typedef struct {
    float x, y;        // Position in meters
    float theta;       // Orientation in radians
    float uncertainty; // Position uncertainty
    uint64_t timestamp; // Timestamp in milliseconds
} robot_pose_t;

// Basic velocity structure
typedef struct {
    float vx, vy;      // Linear velocity in m/s
    float vtheta;      // Angular velocity in rad/s
} velocity_t;

// GPS data structure
typedef struct {
    double latitude;    // Latitude in degrees
    double longitude;   // Longitude in degrees
    float altitude;     // Altitude in meters
    float hdop;         // Horizontal dilution of precision
    int satellites;     // Number of satellites in view
    int fix;            // GPS fix type (0=no fix, 1=time only, 2=2D, 3=3D)
    float speed;        // Ground speed in m/s
    float course;       // Course over ground in degrees
    uint64_t timestamp; // GPS timestamp
} gps_data_t;

// Camera data structure
typedef struct {
    int width, height;
    std::vector<uint8_t> frame;  // Raw frame data
    uint64_t timestamp;
} camera_data_t;

// Obstacle data structure
typedef struct {
    float x, y;      // Position relative to robot
    float radius;    // Approximate size
    float confidence;
} obstacle_t;

// Obstacles data
typedef struct {
    obstacle_t obstacles[20];  // Fixed-size array for obstacles
    int count;                 // Number of valid obstacles
    uint64_t timestamp;
} obstacles_data_t;

// Perimeter wire detection
typedef struct {
    bool detected;
    float distance;  // Distance to wire
    float angle;     // Angle to wire
    float signal_strength;
    uint64_t timestamp;
} perimeter_data_t;

// Grass detection
typedef struct {
    bool is_grass;
    float confidence;
    uint64_t timestamp;
} grass_data_t;

// Fusion data (from EKF)
typedef struct {
    float x, y, theta;  // Position and orientation
    float vx, vy, vtheta; // Velocities
    float covariance[9];  // 3x3 covariance matrix
    uint64_t timestamp;
} fusion_data_t;

// Raw sensor data
typedef struct sensor_data_t {
    fusion_data_t fusion;
    camera_data_t camera;
    obstacles_data_t obstacles;
    perimeter_data_t perimeter;
    grass_data_t grass;
    gps_data_t gps;
    uint64_t timestamp; // Timestamp for the entire sensor data package
} sensor_data_t;

// Occupancy map cell
typedef struct {
    float probability;  // Occupancy probability [0,1]
    int8_t hits;        // Number of hits
    int8_t misses;      // Number of misses
    bool visited;       // Whether the cell has been explored
    uint32_t update_time; // Last update timestamp
} map_cell_t;

// Occupancy grid map
typedef struct occupancy_map_t {
    map_cell_t* cells;      // Grid cells (row-major order)
    int width, height;      // Dimensions in cells
    float resolution;       // Meters per cell
    float origin_x, origin_y; // Origin position in meters
    std::mutex mutex;       // Mutex for thread-safe access
    uint32_t timestamp;     // Last update timestamp
    bool updated;           // Whether the map has been updated since last publish
} occupancy_map_t;

// Landmark structure for visual features
typedef struct {
    uint16_t id;
    float x, y;           // World position
    float confidence;     // Landmark confidence
    uint8_t descriptor[32]; // Visual descriptor
    uint32_t last_seen;   // Last time this landmark was observed
} landmark_t;

// Odometry state
typedef struct odometry_t {
    // Pose state
    robot_pose_t pose;
    
    // Velocity state
    velocity_t velocity;
    
    // Encoder state
    int32_t left_encoder_prev;
    int32_t right_encoder_prev;
    
    // Distance tracking
    float distance_traveled;
    
    // Timing
    uint64_t timestamp;   // Timestamp in milliseconds
    uint64_t last_update; // Last update timestamp
    
    // Flags
    bool initialized;     // Whether odometry is initialized
} odometry_t;

// Extended Kalman Filter state for localization
typedef struct {
    float state[3];       // [x, y, theta]
    float covariance[3][3]; // State covariance matrix
    float process_noise[3][3]; // Process noise matrix
    float measurement_noise[2][2]; // Measurement noise matrix
} ekf_state_t;

// MQTT topics configuration
typedef struct {
    std::string camera;    // Camera feed topic
    std::string obstacles; // Obstacle detection topic
    std::string perimeter; // Perimeter wire detection topic
    std::string grass;     // Grass detection topic
    std::string fusion;    // Sensor fusion topic
    std::string slam_pose; // Output: SLAM pose topic
    std::string slam_map;  // Output: SLAM map topic
    std::string cmd_vel;   // Input: Velocity commands
} mqtt_topics_t;

// Debug settings
typedef struct {
    bool enable_console_output; // Enable console logging
    bool publish_pose;          // Publish pose updates
    bool publish_map;           // Publish map updates
    int log_level;              // 0=error, 1=warn, 2=info, 3=debug
} debug_settings_t;

// SLAM configuration parameters
typedef struct {
    // MQTT settings
    std::string mqtt_broker;    // MQTT broker address
    int mqtt_port;              // MQTT broker port
    std::string mqtt_username;  // MQTT username for authentication
    std::string mqtt_password;  // MQTT password for authentication
    std::string mqtt_client_id; // Client ID for MQTT connection
    mqtt_topics_t mqtt_topics;  // MQTT topic configuration
    
    // Map parameters
    float map_resolution;       // Meters per cell
    int map_width;              // Map width in cells
    int map_height;             // Map height in cells
    
    // Sensor parameters
    struct {
        float sonar_max_range;    // Maximum reliable sonar range (meters)
        float sonar_fov;          // Sonar field of view (radians)
        float camera_fx;          // Camera focal length x
        float camera_fy;          // Camera focal length y
        float camera_cx;          // Camera principal point x
        float camera_cy;          // Camera principal point y
        float gps_std_dev;        // GPS standard deviation (meters)
        float imu_gyro_std;       // IMU gyroscope standard deviation
        float imu_accel_std;      // IMU accelerometer standard deviation
    } sensors;
    
    // Odometry parameters
    struct {
        float wheel_radius;    // Wheel radius in meters
        float wheel_base;      // Distance between wheels in meters
        int32_t encoder_ticks_per_rev; // Encoder ticks per wheel revolution
        float encoder_std_dev; // Encoder standard deviation
    } odometry;
    
    // EKF parameters
    struct {
        float process_noise[3];     // [x, y, theta]
        float measurement_noise[3]; // [range, bearing, id]
        float initial_covariance[9]; // Initial state covariance
    } ekf;
    
    // SLAM parameters
    struct {
        float update_rate_hz;      // SLAM update rate
        int max_particles;         // For particle filter
        float resample_threshold;  // Resampling threshold
        float max_range;           // Maximum sensor range
        float angular_resolution;  // Angular resolution for scan matching
    } slam;
    
    // GPS parameters
    struct {
        float origin_lat;  // Origin latitude for local coordinate system
        float origin_lon;  // Origin longitude for local coordinate system
    } gps;
    
    // Debug settings
    debug_settings_t debug;
    
} slam_config_t;

// Main SLAM system structure
typedef struct {
    // Core components
    odometry_t odometry;          // Odometry state
    occupancy_map_t map;          // Occupancy grid map
    ekf_state_t ekf;              // EKF state
    sensor_data_t sensors;        // Latest sensor data
    slam_config_t config;         // Configuration parameters
    landmark_t landmarks[MAX_LANDMARKS]; // Visual landmarks
    
    // Status flags
    bool initialized;             // Whether SLAM is initialized
    uint16_t landmark_count;      // Number of valid landmarks
    
    // Thread safety
    std::mutex mutex;             // Mutex for thread-safe access
    
} slam_system_t;

#endif // SLAM_TYPES_H