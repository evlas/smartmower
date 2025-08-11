// slam_node.cpp - Simplified SLAM node for smartmower
// This is a simplified version aligned with slam_types.h structures

#include "slam_mqtt.h"
#include "slam_types.h"
#include "mapping.h"
#include "odometry.h"
#include "slam_functions.h"
#include <iostream>
#include <string>
#include <chrono>
#include <signal.h>
#include <fstream>
#include <mosquitto.h>
#include <json-c/json.h>

using namespace std::chrono;

// Global variables
static struct mosquitto* mosq = nullptr;
static slam_config_t g_config;
static occupancy_map_t g_map;
static odometry_t g_odom;
static robot_pose_t g_robot_pose;
static bool g_running = true;

// Signal handler
void signal_handler(int sig) {
    std::cout << "Received signal " << sig << ", shutting down..." << std::endl;
    g_running = false;
}

// SLAM sensor message parsing using correct JSON structures from other modules
bool parse_sensor_message(const std::string& topic, const std::string& payload, sensor_data_t& data) {
    try {
        json_object* root = json_tokener_parse(payload.c_str());
        if (!root) {
            std::cerr << "Failed to parse JSON payload" << std::endl;
            return false;
        }

        // Set timestamp
        data.timestamp = duration_cast<milliseconds>(system_clock::now().time_since_epoch()).count();

        // Parse fusion data (new expanded format)
        if (topic == SLAM_TOPIC_FUSION_DATA) {
            json_object *type_obj, *position_obj, *velocity_obj, *orientation_obj, *uncertainty_obj;
            
            // Verify message type
            if (json_object_object_get_ex(root, "type", &type_obj)) {
                const char* msg_type = json_object_get_string(type_obj);
                if (strcmp(msg_type, "fusion_data") == 0) {
                    
                    // Parse position data (new format: direct position object)
                    if (json_object_object_get_ex(root, "position", &position_obj)) {
                        json_object *x_obj, *y_obj, *z_obj;
                        if (json_object_object_get_ex(position_obj, "x", &x_obj) &&
                            json_object_object_get_ex(position_obj, "y", &y_obj) &&
                            json_object_object_get_ex(position_obj, "z", &z_obj)) {
                            
                            data.fusion.x = json_object_get_double(x_obj);
                            data.fusion.y = json_object_get_double(y_obj);
                            // z coordinate stored but not used in 2D SLAM
                        }
                    }
                    
                    // Parse orientation data (new format: direct orientation object)
                    if (json_object_object_get_ex(root, "orientation", &orientation_obj)) {
                        json_object *roll_obj, *pitch_obj, *yaw_obj;
                        if (json_object_object_get_ex(orientation_obj, "roll", &roll_obj) &&
                            json_object_object_get_ex(orientation_obj, "pitch", &pitch_obj) &&
                            json_object_object_get_ex(orientation_obj, "yaw", &yaw_obj)) {
                            
                            data.fusion.theta = json_object_get_double(yaw_obj);
                            // roll and pitch available but not used in 2D SLAM
                        }
                    }
                    
                    // Parse velocity data (new format: vx, vy, vz, speed)
                    if (json_object_object_get_ex(root, "velocity", &velocity_obj)) {
                        json_object *vx_obj, *vy_obj, *speed_obj;
                        if (json_object_object_get_ex(velocity_obj, "vx", &vx_obj) &&
                            json_object_object_get_ex(velocity_obj, "vy", &vy_obj)) {
                            
                            data.fusion.vx = json_object_get_double(vx_obj);
                            data.fusion.vy = json_object_get_double(vy_obj);
                            
                            // Calculate angular velocity from linear velocities if needed
                            // For now, assume vtheta is derived from position changes
                            data.fusion.vtheta = 0.0; // Will be calculated from pose changes
                        }
                        
                        // Optional: use speed magnitude if available
                        if (json_object_object_get_ex(velocity_obj, "speed", &speed_obj)) {
                            // Speed value available but not used in SLAM
                            // Speed magnitude available for additional processing
                        }
                    }
                    
                    // Parse uncertainty data (new feature for SLAM quality assessment)
                    if (json_object_object_get_ex(root, "uncertainty", &uncertainty_obj)) {
                        json_object *pos_x_obj, *pos_y_obj, *yaw_obj;
                        if (json_object_object_get_ex(uncertainty_obj, "position_x", &pos_x_obj) &&
                            json_object_object_get_ex(uncertainty_obj, "position_y", &pos_y_obj) &&
                            json_object_object_get_ex(uncertainty_obj, "yaw", &yaw_obj)) {
                            
                            // Store uncertainty for SLAM quality assessment
                            double pos_x_std = json_object_get_double(pos_x_obj);
                            double pos_y_std = json_object_get_double(pos_y_obj);
                            double yaw_std = json_object_get_double(yaw_obj);
                            
                            // Use uncertainty to weight fusion data in SLAM
                            // Higher uncertainty = lower weight in SLAM updates
                            std::cout << "Fusion uncertainty: pos_x=" << pos_x_std << ", pos_y=" << pos_y_std << ", yaw=" << yaw_std << std::endl;
                        }
                    }
                    
                    data.fusion.timestamp = data.timestamp;
                    std::cout << "Parsed fusion data: x=" << data.fusion.x << ", y=" << data.fusion.y << ", theta=" << data.fusion.theta 
                              << ", vx=" << data.fusion.vx << ", vy=" << data.fusion.vy << std::endl;
                }
            }
        }
        // Parse GPS data (from gps_mqtt.h structure)
        else if (topic == SLAM_TOPIC_GPS_DATA) {
            json_object *type_obj, *position_obj;
            
            if (json_object_object_get_ex(root, "type", &type_obj)) {
                const char* msg_type = json_object_get_string(type_obj);
                if (strcmp(msg_type, "gps_data") == 0) {
                    
                    if (json_object_object_get_ex(root, "position", &position_obj)) {
                        json_object *lat_obj, *lon_obj, *alt_obj;
                        if (json_object_object_get_ex(position_obj, "latitude", &lat_obj) &&
                            json_object_object_get_ex(position_obj, "longitude", &lon_obj) &&
                            json_object_object_get_ex(position_obj, "altitude", &alt_obj)) {
                            
                            // Store GPS data for SLAM (would need coordinate conversion)
                            data.gps.latitude = json_object_get_double(lat_obj);
                            data.gps.longitude = json_object_get_double(lon_obj);
                            data.gps.altitude = json_object_get_double(alt_obj);
                            data.gps.timestamp = data.timestamp;
                            
                            std::cout << "Parsed GPS data: lat=" << data.gps.latitude << ", lon=" << data.gps.longitude << std::endl;
                        }
                    }
                }
            }
        }
        // Parse Pico sensor data (from pico_mqtt.h structure) - store in fusion for now
        else if (topic == SLAM_TOPIC_PICO_SENSORS) {
            json_object *type_obj;
            
            if (json_object_object_get_ex(root, "type", &type_obj)) {
                const char* msg_type = json_object_get_string(type_obj);
                if (strcmp(msg_type, "sensor_data") == 0) {
                    // For SLAM, IMU data would be processed for pose estimation
                    // Currently storing timestamp for sensor fusion integration
                    data.timestamp = data.timestamp;
                    std::cout << "Parsed Pico IMU data (stored for fusion processing)" << std::endl;
                }
            }
        }
        // Parse Pico status for odometry (from pico_mqtt.h structure) - use for pose updates
        else if (topic == SLAM_TOPIC_PICO_ODOMETRY) {
            json_object *type_obj, *odometry_obj;
            
            if (json_object_object_get_ex(root, "type", &type_obj)) {
                const char* msg_type = json_object_get_string(type_obj);
                if (strcmp(msg_type, "status_report") == 0) {
                    
                    if (json_object_object_get_ex(root, "odometry", &odometry_obj)) {
                        json_object *encoders_obj;
                        if (json_object_object_get_ex(odometry_obj, "encoders", &encoders_obj)) {
                            json_object *left_obj, *right_obj;
                            if (json_object_object_get_ex(encoders_obj, "left_ticks", &left_obj) &&
                                json_object_object_get_ex(encoders_obj, "right_ticks", &right_obj)) {
                                
                                // Store encoder data for odometry processing
                                // This would be used by the odometry module to update robot pose
                                int64_t left_ticks = json_object_get_int64(left_obj);
                                int64_t right_ticks = json_object_get_int64(right_obj);
                                
                                std::cout << "Parsed odometry: left=" << left_ticks << ", right=" << right_ticks << std::endl;
                            }
                        }
                    }
                }
            }
        }
        // Parse vision obstacle data (from vision_mqtt.h structure)
        else if (topic == SLAM_TOPIC_VISION_OBSTACLES) {
            json_object *type_obj, *obstacles_obj;
            
            if (json_object_object_get_ex(root, "type", &type_obj)) {
                const char* msg_type = json_object_get_string(type_obj);
                if (strcmp(msg_type, "obstacle_detection") == 0) {
                    
                    if (json_object_object_get_ex(root, "obstacles", &obstacles_obj) &&
                        json_object_is_type(obstacles_obj, json_type_array)) {
                        
                        int array_len = json_object_array_length(obstacles_obj);
                        data.obstacles.count = std::min(array_len, 20); // Max 20 obstacles
                        
                        for (int i = 0; i < data.obstacles.count; i++) {
                            json_object* obs_obj = json_object_array_get_idx(obstacles_obj, i);
                            json_object *x_obj, *y_obj, *conf_obj;
                            
                            if (json_object_object_get_ex(obs_obj, "x", &x_obj) &&
                                json_object_object_get_ex(obs_obj, "y", &y_obj) &&
                                json_object_object_get_ex(obs_obj, "confidence", &conf_obj)) {
                                
                                data.obstacles.obstacles[i].x = json_object_get_double(x_obj);
                                data.obstacles.obstacles[i].y = json_object_get_double(y_obj);
                                data.obstacles.obstacles[i].confidence = json_object_get_double(conf_obj);
                                data.obstacles.obstacles[i].radius = 0.1f; // Default radius
                            }
                        }
                        data.obstacles.timestamp = data.timestamp;
                        std::cout << "Parsed " << data.obstacles.count << " obstacles" << std::endl;
                    }
                }
            }
        }
        
        json_object_put(root);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error parsing message: " << e.what() << std::endl;
        return false;
    }
}

// MQTT publishing functions
void publish_slam_pose(struct mosquitto* mosq) {
    if (!mosq) return;
    
    char json_buffer[512];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\": \"slam_pose\","
        "\"timestamp\": %lu,"
        "\"pose\": {"
        "\"x\": %.3f,"
        "\"y\": %.3f,"
        "\"theta\": %.3f,"
        "\"confidence\": %.2f"
        "},"
        "\"covariance\": [%.6f, %.6f, %.6f, %.6f, %.6f, %.6f]"
        "}",
        time(NULL),
        g_robot_pose.x, g_robot_pose.y, g_robot_pose.theta,
        0.95f, // confidence
        0.01f, 0.0f, 0.0f, 0.0f, 0.01f, 0.0f
    );
    
    std::string topic = std::string(SLAM_MQTT_BASE_TOPIC) + SLAM_TOPIC_POSE;
    int rc = mosquitto_publish(mosq, NULL, topic.c_str(), strlen(json_buffer), json_buffer, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to publish SLAM pose: " << mosquitto_strerror(rc) << std::endl;
    }
}

void publish_slam_status(struct mosquitto* mosq) {
    if (!mosq) return;
    
    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\": \"slam_status\","
        "\"timestamp\": %lu,"
        "\"status\": \"active\","
        "\"particles\": %d,"
        "\"map_size\": {\"width\": %d, \"height\": %d},"
        "\"pose_valid\": %s"
        "}",
        time(NULL),
        g_config.slam.max_particles,
        g_config.map_width, g_config.map_height,
        "true"
    );
    
    std::string topic = std::string(SLAM_MQTT_BASE_TOPIC) + SLAM_TOPIC_STATUS;
    int rc = mosquitto_publish(mosq, NULL, topic.c_str(), strlen(json_buffer), json_buffer, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to publish SLAM status: " << mosquitto_strerror(rc) << std::endl;
    }
}

// MQTT message callback
void on_message(struct mosquitto*, void*, const struct mosquitto_message* message) {
    if (!message->payload) return;
    
    std::string topic(message->topic);
    std::string payload(static_cast<char*>(message->payload), message->payloadlen);
    
    sensor_data_t sensor_data;
    if (parse_sensor_message(topic, payload, sensor_data)) {
        // Update odometry with sensor data
        odometry_update(&g_odom, &sensor_data, &g_config);
        
        // Update robot pose from odometry
        g_robot_pose.x = g_odom.pose.x;
        g_robot_pose.y = g_odom.pose.y;
        g_robot_pose.theta = g_odom.pose.theta;
        
        std::cout << "Updated pose: (" << g_robot_pose.x << ", " 
                  << g_robot_pose.y << ", " << g_robot_pose.theta << ")" << std::endl;
                  
        // Publish updated pose to MQTT
        publish_slam_pose(mosq);
    }
}

// MQTT connection callback
void on_connect(struct mosquitto*, void*, int rc) {
    if (rc == 0) {
        std::cout << "Connected to MQTT broker" << std::endl;
        // Subscribe to relevant topics using slam_mqtt.h constants
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_FUSION_DATA, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_GPS_DATA, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_PICO_SENSORS, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_PICO_ODOMETRY, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_VISION_OBSTACLES, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_VISION_PERIMETER, 1);
        mosquitto_subscribe(mosq, nullptr, SLAM_TOPIC_VISION_GRASS, 1);
        
        // Subscribe to SLAM commands
        char cmd_topic[256];
        snprintf(cmd_topic, sizeof(cmd_topic), "%s%s", SLAM_MQTT_BASE_TOPIC, SLAM_TOPIC_CMD);
        mosquitto_subscribe(mosq, nullptr, cmd_topic, 1);
        
        std::cout << "Subscribed to all SLAM topics" << std::endl;
    } else {
        std::cerr << "Failed to connect to MQTT broker: " << mosquitto_connack_string(rc) << std::endl;
    }
}

// Configuration loader using unified config.json
bool load_config(const std::string& config_file) {
    // Initialize default MQTT configuration
    g_config.mqtt_broker = "localhost";
    g_config.mqtt_port = 1883;
    g_config.mqtt_username = "";
    g_config.mqtt_password = "";
    g_config.mqtt_client_id = SLAM_MQTT_CLIENT_ID;
    
    // Load configuration from JSON file
    std::ifstream file(config_file);
    if (!file.is_open()) {
        std::cerr << "Warning: Could not open config file " << config_file << ", using defaults" << std::endl;
        // Use default values
        g_config.map_resolution = 0.1f;
        g_config.map_width = 100;
        g_config.map_height = 100;
        g_config.odometry.wheel_radius = 0.1f;
        g_config.odometry.wheel_base = 0.3f;
        g_config.odometry.encoder_ticks_per_rev = 1024;
        g_config.odometry.encoder_std_dev = 0.01f;
        return true;
    }
    
    try {
        std::string config_content((std::istreambuf_iterator<char>(file)),
                                   std::istreambuf_iterator<char>());
        file.close();
        
        json_object* root = json_tokener_parse(config_content.c_str());
        if (!root) {
            std::cerr << "Error: Failed to parse config JSON" << std::endl;
            return false;
        }
        
        // Parse MQTT configuration from system.communication section
        json_object* system_obj, *communication_obj;
        if (json_object_object_get_ex(root, "system", &system_obj) &&
            json_object_object_get_ex(system_obj, "communication", &communication_obj)) {
            
            json_object* mqtt_broker_obj, *mqtt_port_obj, *mqtt_username_obj, *mqtt_password_obj;
            if (json_object_object_get_ex(communication_obj, "mqtt_broker_host", &mqtt_broker_obj)) {
                g_config.mqtt_broker = json_object_get_string(mqtt_broker_obj);
            }
            if (json_object_object_get_ex(communication_obj, "mqtt_broker_port", &mqtt_port_obj)) {
                g_config.mqtt_port = json_object_get_int(mqtt_port_obj);
            }
            if (json_object_object_get_ex(communication_obj, "mqtt_username", &mqtt_username_obj)) {
                g_config.mqtt_username = json_object_get_string(mqtt_username_obj);
            }
            if (json_object_object_get_ex(communication_obj, "mqtt_password", &mqtt_password_obj)) {
                g_config.mqtt_password = json_object_get_string(mqtt_password_obj);
            }
        }
        
        // Parse slam_config from centralized configuration
        json_object* slam_config_obj;
        if (json_object_object_get_ex(root, "slam_config", &slam_config_obj)) {
            
            // Parse mapping parameters
            json_object* mapping_obj;
            if (json_object_object_get_ex(slam_config_obj, "mapping", &mapping_obj)) {
                json_object* res_obj, *width_obj, *height_obj;
                if (json_object_object_get_ex(mapping_obj, "resolution", &res_obj))
                    g_config.map_resolution = json_object_get_double(res_obj);
                if (json_object_object_get_ex(mapping_obj, "width", &width_obj))
                    g_config.map_width = json_object_get_int(width_obj);
                if (json_object_object_get_ex(mapping_obj, "height", &height_obj))
                    g_config.map_height = json_object_get_int(height_obj);
            }
            
            // Parse localization parameters
            json_object* localization_obj;
            if (json_object_object_get_ex(slam_config_obj, "localization", &localization_obj)) {
                json_object* particle_count_obj, *update_rate_obj;
                if (json_object_object_get_ex(localization_obj, "particle_count", &particle_count_obj))
                    std::cout << "Particle count: " << json_object_get_int(particle_count_obj) << std::endl;
                if (json_object_object_get_ex(localization_obj, "update_rate_hz", &update_rate_obj))
                    std::cout << "Update rate: " << json_object_get_int(update_rate_obj) << "Hz" << std::endl;
            }
            
            // Parse robot_parameters from slam_config
            json_object* robot_params_obj;
            if (json_object_object_get_ex(slam_config_obj, "robot_parameters", &robot_params_obj)) {
                json_object* wheelbase_obj, *wheel_radius_obj, *encoder_ticks_obj;
                if (json_object_object_get_ex(robot_params_obj, "wheelbase", &wheelbase_obj))
                    g_config.odometry.wheel_base = json_object_get_double(wheelbase_obj);
                if (json_object_object_get_ex(robot_params_obj, "wheel_radius", &wheel_radius_obj))
                    g_config.odometry.wheel_radius = json_object_get_double(wheel_radius_obj);
                if (json_object_object_get_ex(robot_params_obj, "encoder_ticks_per_rev", &encoder_ticks_obj))
                    g_config.odometry.encoder_ticks_per_rev = json_object_get_int(encoder_ticks_obj);
            }
        }
        
        // Set default encoder standard deviation
        g_config.odometry.encoder_std_dev = 0.01f;
        
        json_object_put(root);
        std::cout << "Configuration loaded from " << config_file << std::endl;
        std::cout << "MQTT Broker: " << g_config.mqtt_broker << ":" << g_config.mqtt_port << std::endl;
        std::cout << "Map: " << g_config.map_width << "x" << g_config.map_height << " @ " << g_config.map_resolution << "m/cell" << std::endl;
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}

int main(int argc, char** argv) {
    // Initialize signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Load configuration from centralized config file
    std::string config_file = (argc > 1) ? argv[1] : "/opt/smartmower/etc/robot_config.json";
    if (!load_config(config_file)) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }

    // Initialize SLAM components
    mapping_init(&g_map);
    odometry_init(&g_odom, &g_config);
    
    // Initialize robot pose
    g_robot_pose.x = 0.0f;
    g_robot_pose.y = 0.0f;
    g_robot_pose.theta = 0.0f;
    g_robot_pose.uncertainty = 0.0f;
    g_robot_pose.timestamp = 0;
    
    // Initialize MQTT
    mosquitto_lib_init();
    mosq = mosquitto_new(g_config.mqtt_client_id.c_str(), true, nullptr);
    if (!mosq) {
        std::cerr << "Failed to create MQTT client" << std::endl;
        return 1;
    }

    // Set MQTT callbacks
    mosquitto_connect_callback_set(mosq, on_connect);
    mosquitto_message_callback_set(mosq, on_message);

    // Set MQTT authentication if credentials are provided
    if (!g_config.mqtt_username.empty() && !g_config.mqtt_password.empty()) {
        int auth_rc = mosquitto_username_pw_set(mosq, g_config.mqtt_username.c_str(), g_config.mqtt_password.c_str());
        if (auth_rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "Failed to set MQTT authentication: " << mosquitto_strerror(auth_rc) << std::endl;
            return 1;
        }
        std::cout << "MQTT authentication set for user: " << g_config.mqtt_username << std::endl;
    }

    // Connect to MQTT broker
    int rc = mosquitto_connect(mosq, g_config.mqtt_broker.c_str(), g_config.mqtt_port, 60);
    if (rc != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker: " << mosquitto_strerror(rc) << std::endl;
        return 1;
    }

    std::cout << "SLAM node started successfully" << std::endl;

    // Initialize status publishing timer
    time_t last_status_publish = time(NULL);
    const int STATUS_PUBLISH_INTERVAL = 5; // seconds

    // Main loop
    while (g_running) {
        rc = mosquitto_loop(mosq, 100, 1);
        if (rc != MOSQ_ERR_SUCCESS) {
            std::cerr << "MQTT loop error: " << mosquitto_strerror(rc) << std::endl;
            break;
        }
        
        // Publish status periodically
        time_t current_time = time(NULL);
        if (current_time - last_status_publish >= STATUS_PUBLISH_INTERVAL) {
            publish_slam_status(mosq);
            last_status_publish = current_time;
        }
        
        usleep(10000); // 10ms sleep
    }

    // Cleanup
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    
    std::cout << "SLAM node shut down" << std::endl;
    return 0;
}
