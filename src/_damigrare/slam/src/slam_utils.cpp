// slam_utils.cpp - SLAM configuration utilities

#include "slam_types.h"
#include <iostream>
#include <fstream>
#include <json-c/json.h>
#include <cstring>

// Simplified configuration loader
bool load_config(const std::string& config_file, slam_config_t* config) {
    if (!config) return false;
    
    try {
        std::ifstream file(config_file);
        if (!file.is_open()) {
            std::cout << "Config file not found, using defaults" << std::endl;
            // Initialize with default values
            config->mqtt_broker = "localhost";
            config->mqtt_port = 1883;
            config->mqtt_client_id = "slam_node";
            
            // MQTT topics
            config->mqtt_topics.fusion = "smartmower/fusion";
            config->mqtt_topics.obstacles = "smartmower/obstacles";
            config->mqtt_topics.camera = "smartmower/camera";
            config->mqtt_topics.perimeter = "smartmower/perimeter";
            config->mqtt_topics.grass = "smartmower/grass";
            
            // Map parameters
            config->map_resolution = 0.1f;
            config->map_width = 200;
            config->map_height = 200;
            
            // Odometry parameters
            config->odometry.wheel_radius = 0.05f;
            config->odometry.wheel_base = 0.3f;
            config->odometry.encoder_ticks_per_rev = 1000;
            config->odometry.encoder_std_dev = 0.01f;
            
            // SLAM parameters
            config->slam.update_rate_hz = 10.0f;
            config->slam.max_particles = 100;
            config->slam.resample_threshold = 0.5f;
            config->slam.max_range = 4.0f;
            config->slam.angular_resolution = 0.1f;
            
            return true;
        }
        
        // Read entire file
        std::string content((std::istreambuf_iterator<char>(file)),
                           std::istreambuf_iterator<char>());
        file.close();
        
        // Parse JSON
        json_object* root = json_tokener_parse(content.c_str());
        if (!root) {
            std::cerr << "Failed to parse config JSON" << std::endl;
            return false;
        }
        
        // Parse MQTT settings
        json_object* mqtt_obj;
        if (json_object_object_get_ex(root, "mqtt", &mqtt_obj)) {
            json_object* broker_obj;
            if (json_object_object_get_ex(mqtt_obj, "broker", &broker_obj)) {
                config->mqtt_broker = json_object_get_string(broker_obj);
            }
            
            json_object* port_obj;
            if (json_object_object_get_ex(mqtt_obj, "port", &port_obj)) {
                config->mqtt_port = json_object_get_int(port_obj);
            }
            
            json_object* client_id_obj;
            if (json_object_object_get_ex(mqtt_obj, "client_id", &client_id_obj)) {
                config->mqtt_client_id = json_object_get_string(client_id_obj);
            }
            
            // Parse MQTT topics
            json_object* topics_obj;
            if (json_object_object_get_ex(mqtt_obj, "topics", &topics_obj)) {
                json_object* fusion_obj;
                if (json_object_object_get_ex(topics_obj, "fusion", &fusion_obj)) {
                    config->mqtt_topics.fusion = json_object_get_string(fusion_obj);
                }
                
                json_object* obstacles_obj;
                if (json_object_object_get_ex(topics_obj, "obstacles", &obstacles_obj)) {
                    config->mqtt_topics.obstacles = json_object_get_string(obstacles_obj);
                }
                
                json_object* camera_obj;
                if (json_object_object_get_ex(topics_obj, "camera", &camera_obj)) {
                    config->mqtt_topics.camera = json_object_get_string(camera_obj);
                }
                
                json_object* perimeter_obj;
                if (json_object_object_get_ex(topics_obj, "perimeter", &perimeter_obj)) {
                    config->mqtt_topics.perimeter = json_object_get_string(perimeter_obj);
                }
                
                json_object* grass_obj;
                if (json_object_object_get_ex(topics_obj, "grass", &grass_obj)) {
                    config->mqtt_topics.grass = json_object_get_string(grass_obj);
                }
            }
        }
        
        // Parse map parameters
        json_object* map_obj;
        if (json_object_object_get_ex(root, "map", &map_obj)) {
            json_object* resolution_obj;
            if (json_object_object_get_ex(map_obj, "resolution", &resolution_obj)) {
                config->map_resolution = json_object_get_double(resolution_obj);
            }
            
            json_object* width_obj;
            if (json_object_object_get_ex(map_obj, "width", &width_obj)) {
                config->map_width = json_object_get_int(width_obj);
            }
            
            json_object* height_obj;
            if (json_object_object_get_ex(map_obj, "height", &height_obj)) {
                config->map_height = json_object_get_int(height_obj);
            }
        }
        
        json_object_put(root);
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error loading config: " << e.what() << std::endl;
        return false;
    }
}
