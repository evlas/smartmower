#define _GNU_SOURCE
#include "../path_planning_mqtt.h"
#include "../include/pattern_algorithms.h"
#include <stdio.h>
#include <stdlib.h>

// Forward declarations for MQTT publishing functions
void publish_path_status();
void publish_current_waypoint();
void publish_path_progress();
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <mosquitto.h>
#include <json-c/json.h>
#include <pthread.h>
#include <time.h>

// SLAM integration function declarations
Point2D get_best_position_estimate(const SlamPose* slam_pose, const Point2D* fusion_position,
                                  const PathPlanningConfig* config);
bool is_point_safe_slam(const SlamMap* slam_map, const Point2D* point, double safety_buffer);
bool optimize_path_with_slam(PathPlan* plan, const SlamMap* slam_map,
                            const PathPlanningConfig* config);

// Global state structure
typedef struct {
    struct mosquitto* mqtt_client;
    PathPlan current_plan;
    AreaBoundary area_boundary;
    Obstacle* obstacles;
    uint32_t obstacle_count;
    PathPlanningConfig config;
    bool planning_active;
    bool plan_executing;
    uint32_t current_waypoint_idx;
    Point2D current_position;
    
    // SLAM integration
    SlamPose slam_pose;
    SlamMap slam_map;
    bool slam_available;
    Point2D fusion_position;
    
    uint32_t total_plans_generated;
    pthread_mutex_t state_mutex;
    bool running;
} PathPlanningState;

static PathPlanningState g_state = {0};

// Load configuration from centralized robot_config.json
bool load_config(const char* filename) {
    // Set defaults
    strcpy(g_state.config.pattern, "spiral");
    g_state.config.cutting_width = 0.30;
    g_state.config.overlap_percentage = 0.1;
    g_state.config.max_speed = 0.8;
    g_state.config.turn_radius = 0.5;
    g_state.config.obstacle_buffer = 1.0;
    g_state.config.perimeter_buffer = 0.5;
    g_state.config.optimize_battery = true;
    g_state.config.avoid_wet_areas = true;
    g_state.config.use_slam_data = true;
    g_state.config.slam_confidence_threshold = 0.7;
    g_state.config.max_waypoints = 2000;
    
    // Load JSON configuration file
    json_object* root = json_object_from_file(filename);
    if (!root) {
        printf("Path Planning: Config file not found, using defaults\n");
        return true;
    }
    
    // Parse path_planning_config section
    json_object* pp_config;
    if (json_object_object_get_ex(root, "path_planning_config", &pp_config)) {
        
        // Parse cutting_config
        json_object* cutting_config;
        if (json_object_object_get_ex(pp_config, "cutting_config", &cutting_config)) {
            json_object* cutting_width;
            if (json_object_object_get_ex(cutting_config, "cutting_width_m", &cutting_width)) {
                g_state.config.cutting_width = json_object_get_double(cutting_width);
            }
            json_object* overlap;
            if (json_object_object_get_ex(cutting_config, "overlap_percentage", &overlap)) {
                g_state.config.overlap_percentage = json_object_get_double(overlap);
            }
        }
        
        // Parse navigation_config
        json_object* nav_config;
        if (json_object_object_get_ex(pp_config, "navigation_config", &nav_config)) {
            json_object* pattern;
            if (json_object_object_get_ex(nav_config, "default_pattern", &pattern)) {
                strncpy(g_state.config.pattern, json_object_get_string(pattern), sizeof(g_state.config.pattern) - 1);
            }
            json_object* max_speed;
            if (json_object_object_get_ex(nav_config, "max_speed_mps", &max_speed)) {
                g_state.config.max_speed = json_object_get_double(max_speed);
            }
            json_object* turn_radius;
            if (json_object_object_get_ex(nav_config, "turn_radius_m", &turn_radius)) {
                g_state.config.turn_radius = json_object_get_double(turn_radius);
            }
            json_object* max_waypoints;
            if (json_object_object_get_ex(nav_config, "max_waypoints_per_plan", &max_waypoints)) {
                g_state.config.max_waypoints = json_object_get_int(max_waypoints);
            }
        }
        
        // Parse obstacle_config
        json_object* obs_config;
        if (json_object_object_get_ex(pp_config, "obstacle_config", &obs_config)) {
            json_object* detection_buffer;
            if (json_object_object_get_ex(obs_config, "detection_buffer_m", &detection_buffer)) {
                g_state.config.obstacle_buffer = json_object_get_double(detection_buffer);
            }
        }
        
        // Parse area_config
        json_object* area_config;
        if (json_object_object_get_ex(pp_config, "area_config", &area_config)) {
            json_object* perimeter_buffer;
            if (json_object_object_get_ex(area_config, "perimeter_buffer_m", &perimeter_buffer)) {
                g_state.config.perimeter_buffer = json_object_get_double(perimeter_buffer);
            }
        }
        
        // Parse optimization_config
        json_object* opt_config;
        if (json_object_object_get_ex(pp_config, "optimization_config", &opt_config)) {
            json_object* battery_opt;
            if (json_object_object_get_ex(opt_config, "battery_optimization", &battery_opt)) {
                g_state.config.optimize_battery = json_object_get_boolean(battery_opt);
            }
            json_object* avoid_wet;
            if (json_object_object_get_ex(opt_config, "avoid_wet_areas", &avoid_wet)) {
                g_state.config.avoid_wet_areas = json_object_get_boolean(avoid_wet);
            }
        }
        
        // Parse integration_config for SLAM
        json_object* int_config;
        if (json_object_object_get_ex(pp_config, "integration_config", &int_config)) {
            json_object* slam_integration;
            if (json_object_object_get_ex(int_config, "fusion_data_integration", &slam_integration)) {
                g_state.config.use_slam_data = json_object_get_boolean(slam_integration);
            }
        }
    }
    
    json_object_put(root);
    
    printf("Path Planning Config loaded:\n");
    printf("  Pattern: %s\n", g_state.config.pattern);
    printf("  Cutting width: %.2fm\n", g_state.config.cutting_width);
    printf("  Max speed: %.1fm/s\n", g_state.config.max_speed);
    printf("  Turn radius: %.1fm\n", g_state.config.turn_radius);
    printf("  Obstacle buffer: %.1fm\n", g_state.config.obstacle_buffer);
    printf("  Battery optimization: %s\n", g_state.config.optimize_battery ? "enabled" : "disabled");
    printf("  SLAM integration: %s\n", g_state.config.use_slam_data ? "enabled" : "disabled");
    
    return true;
}

// MQTT callback functions
void on_connect(struct mosquitto* mosq, void* userdata __attribute__((unused)), int result) {
    if (result == 0) {
        printf("Connected to MQTT broker\n");
        mosquitto_subscribe(mosq, NULL, PATH_TOPIC_REQUEST, 0);
        mosquitto_subscribe(mosq, NULL, PATH_TOPIC_POSITION, 0);
        mosquitto_subscribe(mosq, NULL, PATH_TOPIC_SLAM_POSE, 0);
        mosquitto_subscribe(mosq, NULL, PATH_TOPIC_SLAM_MAP, 0);
        mosquitto_subscribe(mosq, NULL, PATH_TOPIC_VISION_OBSTACLES, 0);
        printf("Subscribed to path planning and SLAM topics\n");
    }
}

void on_message(struct mosquitto* mosq __attribute__((unused)), void* userdata __attribute__((unused)), const struct mosquitto_message* message) {
    if (!message->payload) return;
    
    char* payload = (char*)message->payload;
    json_object* root = json_tokener_parse(payload);
    if (!root) return;
    
    pthread_mutex_lock(&g_state.state_mutex);
    
    // Handle SLAM pose updates
    if (strcmp(message->topic, PATH_TOPIC_SLAM_POSE) == 0) {
        json_object* type_obj, *pose_obj, *position_obj, *x_obj, *y_obj, *heading_obj, *confidence_obj;
        
        if (json_object_object_get_ex(root, "type", &type_obj) &&
            json_object_object_get_ex(root, "pose", &pose_obj)) {
            
            const char* type = json_object_get_string(type_obj);
            if (strcmp(type, PATH_JSON_SLAM_POSE) == 0) {
                if (json_object_object_get_ex(pose_obj, "position", &position_obj) &&
                    json_object_object_get_ex(position_obj, "x", &x_obj) &&
                    json_object_object_get_ex(position_obj, "y", &y_obj) &&
                    json_object_object_get_ex(pose_obj, "heading", &heading_obj) &&
                    json_object_object_get_ex(pose_obj, "confidence", &confidence_obj)) {
                    
                    g_state.slam_pose.position.x = json_object_get_double(x_obj);
                    g_state.slam_pose.position.y = json_object_get_double(y_obj);
                    g_state.slam_pose.heading = json_object_get_double(heading_obj);
                    g_state.slam_pose.confidence = json_object_get_double(confidence_obj);
                    g_state.slam_pose.timestamp = (uint64_t)time(NULL);
                    g_state.slam_pose.valid = true;
                    g_state.slam_available = true;
                    
                    // Update current position with best estimate
                    g_state.current_position = get_best_position_estimate(&g_state.slam_pose, 
                                                                         &g_state.fusion_position, 
                                                                         &g_state.config);
                    
                    printf("SLAM pose updated: (%.2f, %.2f) confidence=%.2f\n",
                           g_state.slam_pose.position.x, g_state.slam_pose.position.y, 
                           g_state.slam_pose.confidence);
                }
            }
        }
    }
    // Handle SLAM map updates  
    else if (strcmp(message->topic, PATH_TOPIC_SLAM_MAP) == 0) {
        json_object* type_obj;
        
        if (json_object_object_get_ex(root, "type", &type_obj)) {
            const char* type = json_object_get_string(type_obj);
            if (strcmp(type, PATH_JSON_SLAM_MAP) == 0) {
                printf("SLAM map update received (parsing not fully implemented)\n");
                // TODO: Parse full SLAM map data
                // For now, just mark that we have map data
                g_state.slam_map.timestamp = (uint64_t)time(NULL);
            }
        }
    }
    // Handle fusion position updates (fallback) - new expanded format
    else if (strcmp(message->topic, PATH_TOPIC_POSITION) == 0) {
        json_object* type_obj, *position_obj, *velocity_obj, *orientation_obj;
        json_object* x_obj, *y_obj, *yaw_obj;
        
        // Verify message type
        if (json_object_object_get_ex(root, "type", &type_obj)) {
            const char* msg_type = json_object_get_string(type_obj);
            
            if (strcmp(msg_type, "fusion_data") == 0) {
                // Parse position data (new direct format)
                if (json_object_object_get_ex(root, "position", &position_obj) &&
                    json_object_object_get_ex(position_obj, "x", &x_obj) &&
                    json_object_object_get_ex(position_obj, "y", &y_obj)) {
                    
                    g_state.fusion_position.x = json_object_get_double(x_obj);
                    g_state.fusion_position.y = json_object_get_double(y_obj);
                    
                    // Parse orientation for heading information
                    if (json_object_object_get_ex(root, "orientation", &orientation_obj) &&
                        json_object_object_get_ex(orientation_obj, "yaw", &yaw_obj)) {
                        // Store yaw for path planning heading calculations
                        // Yaw value available but not used in path planning
                        // g_state.fusion_heading = yaw; // if heading field exists
                    }
                    
                    // Parse velocity for motion planning
                    if (json_object_object_get_ex(root, "velocity", &velocity_obj)) {
                        json_object *vx_obj, *vy_obj, *speed_obj;
                        if (json_object_object_get_ex(velocity_obj, "vx", &vx_obj) &&
                            json_object_object_get_ex(velocity_obj, "vy", &vy_obj)) {
                            // Store velocity for dynamic path planning
                            // g_state.fusion_velocity.vx = json_object_get_double(vx_obj);
                            // g_state.fusion_velocity.vy = json_object_get_double(vy_obj);
                        }
                        
                        if (json_object_object_get_ex(velocity_obj, "speed", &speed_obj)) {
                            // Store speed for path timing calculations
                            // g_state.fusion_speed = json_object_get_double(speed_obj);
                        }
                    }
                    
                    // Update current position with best estimate
                    g_state.current_position = get_best_position_estimate(&g_state.slam_pose, 
                                                                         &g_state.fusion_position, 
                                                                         &g_state.config);
                    
                    printf("Path planning updated position: x=%.3f, y=%.3f\n", 
                           g_state.fusion_position.x, g_state.fusion_position.y);
                }
            }
        }
    }
    // Handle path planning requests
    else if (strcmp(message->topic, PATH_TOPIC_REQUEST) == 0) {
        json_object* type_obj, *request_obj, *command_obj;
        
        if (json_object_object_get_ex(root, "type", &type_obj) &&
            json_object_object_get_ex(root, "request", &request_obj) &&
            json_object_object_get_ex(request_obj, "command", &command_obj)) {
            
            const char* command = json_object_get_string(command_obj);
            
            if (strcmp(command, "plan") == 0) {
                // Generate spiral pattern for 6000mq area
                printf("Generating spiral pattern for 6000mq area\n");
                
                free_path_plan(&g_state.current_plan);
                
                bool success = generate_spiral_pattern(&g_state.area_boundary,
                                                     g_state.obstacles,
                                                     g_state.obstacle_count,
                                                     &g_state.config,
                                                     &g_state.current_plan);
                
                if (success) {
                    g_state.planning_active = true;
                    g_state.current_waypoint_idx = 0;
                    g_state.total_plans_generated++;
                    printf("Path plan generated: %u waypoints, %.1fm total distance\n",
                           g_state.current_plan.waypoint_count,
                           g_state.current_plan.total_distance);
                    
                    // Publish new plan status and first waypoint
                    publish_path_status();
                    publish_current_waypoint();
                    publish_path_progress();
                }
            }
        }
    }
    
    pthread_mutex_unlock(&g_state.state_mutex);
    json_object_put(root);
}

// MQTT Publishing Functions
void publish_path_status() {
    if (!g_state.mqtt_client) return;
    
    char json_buffer[512];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\": \"path_status\","
        "\"timestamp\": %lu,"
        "\"status\": \"%s\","
        "\"planning_active\": %s,"
        "\"current_waypoint\": %u,"
        "\"total_waypoints\": %u,"
        "\"total_distance\": %.2f,"
        "\"plans_generated\": %u"
        "}",
        time(NULL),
        g_state.planning_active ? "active" : "idle",
        g_state.planning_active ? "true" : "false",
        g_state.current_waypoint_idx,
        g_state.current_plan.waypoint_count,
        g_state.current_plan.total_distance,
        g_state.total_plans_generated
    );
    
    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", PATH_MQTT_BASE_TOPIC, PATH_TOPIC_STATUS);
    
    int rc = mosquitto_publish(g_state.mqtt_client, NULL, topic, strlen(json_buffer), json_buffer, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        printf("Failed to publish path status: %s\n", mosquitto_strerror(rc));
    }
}

void publish_current_waypoint() {
    if (!g_state.mqtt_client || !g_state.planning_active || 
        g_state.current_waypoint_idx >= g_state.current_plan.waypoint_count) return;
    
    Waypoint* wp = &g_state.current_plan.waypoints[g_state.current_waypoint_idx];
    
    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\": \"path_waypoint\","
        "\"timestamp\": %lu,"
        "\"waypoint\": {"
        "\"index\": %u,"
        "\"x\": %.3f,"
        "\"y\": %.3f,"
        "\"heading\": %.3f,"
        "\"speed\": %.2f"
        "}"
        "}",
        time(NULL),
        g_state.current_waypoint_idx,
        wp->position.x, wp->position.y, wp->heading, wp->speed
    );
    
    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", PATH_MQTT_BASE_TOPIC, PATH_TOPIC_WAYPOINT);
    
    int rc = mosquitto_publish(g_state.mqtt_client, NULL, topic, strlen(json_buffer), json_buffer, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        printf("Failed to publish current waypoint: %s\n", mosquitto_strerror(rc));
    }
}

void publish_path_progress() {
    if (!g_state.mqtt_client || !g_state.planning_active) return;
    
    float progress_percent = 0.0f;
    if (g_state.current_plan.waypoint_count > 0) {
        progress_percent = (float)g_state.current_waypoint_idx / g_state.current_plan.waypoint_count * 100.0f;
    }
    
    char json_buffer[256];
    snprintf(json_buffer, sizeof(json_buffer),
        "{"
        "\"type\": \"path_progress\","
        "\"timestamp\": %lu,"
        "\"progress\": {"
        "\"current_waypoint\": %u,"
        "\"total_waypoints\": %u,"
        "\"percent_complete\": %.1f,"
        "\"distance_remaining\": %.2f"
        "}"
        "}",
        time(NULL),
        g_state.current_waypoint_idx,
        g_state.current_plan.waypoint_count,
        progress_percent,
        g_state.current_plan.total_distance
    );
    
    char topic[128];
    snprintf(topic, sizeof(topic), "%s%s", PATH_MQTT_BASE_TOPIC, PATH_TOPIC_PROGRESS);
    
    int rc = mosquitto_publish(g_state.mqtt_client, NULL, topic, strlen(json_buffer), json_buffer, 0, false);
    if (rc != MOSQ_ERR_SUCCESS) {
        printf("Failed to publish path progress: %s\n", mosquitto_strerror(rc));
    }
}

// Initialize path planning system for 6000mq area
bool initialize_path_planning() {
    pthread_mutex_init(&g_state.state_mutex, NULL);
    
    // Load configuration from centralized config file
    if (!load_config("/opt/smartmower/etc/robot_config.json")) {
        printf("Failed to load path planning configuration\n");
        return false;
    }
    
    // Initialize SLAM state
    g_state.slam_available = false;
    g_state.slam_pose.valid = false;
    g_state.slam_map.cells = NULL;
    
    // Create 6000mq rectangular boundary (77.46m x 77.46m = 6000mq)
    if (!create_rectangular_boundary(77.46, 77.46, &g_state.area_boundary)) {
        printf("Failed to create area boundary\n");
        return false;
    }
    
    // Create default obstacles for testing
    if (!create_default_obstacles_6000mq(&g_state.obstacles, &g_state.obstacle_count)) {
        printf("Failed to create default obstacles\n");
        return false;
    }
    
    printf("Path planning initialized for 6000mq area with %u obstacles\n", 
           g_state.obstacle_count);
    
    return true;
}

// Main function
int main() {
    printf("Starting Path Planning Node for 6000mq Smart Mower\n");
    
    if (!initialize_path_planning()) {
        printf("Failed to initialize path planning\n");
        return 1;
    }
    
    // Initialize MQTT
    mosquitto_lib_init();
    g_state.mqtt_client = mosquitto_new(PATH_MQTT_CLIENT_ID, true, NULL);
    
    if (!g_state.mqtt_client) {
        printf("Failed to create MQTT client\n");
        return 1;
    }
    
    mosquitto_connect_callback_set(g_state.mqtt_client, on_connect);
    mosquitto_message_callback_set(g_state.mqtt_client, on_message);
    
    if (mosquitto_connect(g_state.mqtt_client, PATH_MQTT_BROKER, PATH_MQTT_PORT, 60) != MOSQ_ERR_SUCCESS) {
        printf("Failed to connect to MQTT broker\n");
        return 1;
    }
    
    g_state.running = true;
    
    // Initialize status publishing timer
    time_t last_status_publish = time(NULL);
    const int STATUS_PUBLISH_INTERVAL = 10; // seconds
    
    // Main loop
    while (g_state.running) {
        mosquitto_loop(g_state.mqtt_client, 100, 1);
        
        // Publish status periodically
        time_t current_time = time(NULL);
        if (current_time - last_status_publish >= STATUS_PUBLISH_INTERVAL) {
            publish_path_status();
            if (g_state.planning_active) {
                publish_path_progress();
            }
            last_status_publish = current_time;
        }
        
        usleep(100000); // 100ms
    }
    
    // Cleanup
    mosquitto_disconnect(g_state.mqtt_client);
    mosquitto_destroy(g_state.mqtt_client);
    mosquitto_lib_cleanup();
    
    free_path_plan(&g_state.current_plan);
    if (g_state.area_boundary.vertices) {
        free(g_state.area_boundary.vertices);
    }
    if (g_state.obstacles) {
        free(g_state.obstacles);
    }
    
    pthread_mutex_destroy(&g_state.state_mutex);
    
    printf("Path Planning Node stopped\n");
    return 0;
}
