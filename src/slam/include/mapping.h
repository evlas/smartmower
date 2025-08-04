// mapping.h - Occupancy grid mapping for SLAM

#ifndef MAPPING_H
#define MAPPING_H

#include "slam_types.h"
#include "math_utils.h"

// Forward declarations are not needed since slam_types.h is included

// Initialize the occupancy grid map
void mapping_init(occupancy_map_t* map);

// Convert world coordinates to grid coordinates
bool world_to_grid(const occupancy_map_t* map, float x, float y, int* grid_x, int* grid_y);

// Convert grid coordinates to world coordinates
void grid_to_world(const occupancy_map_t* map, int grid_x, int grid_y, float* x, float* y);

// Update the map with new sensor data
void mapping_update_sonar(occupancy_map_t* map, robot_pose_t* pose, 
                         sensor_data_t* sensors, slam_config_t* config);

// Get the occupancy probability of a cell
float get_occupancy(const occupancy_map_t* map, int x, int y);

// Set the occupancy probability of a cell
void set_occupancy(occupancy_map_t* map, int x, int y, float prob);

// Check if a cell is occupied (probability > threshold)
bool is_occupied(const occupancy_map_t* map, int x, int y, float threshold = 0.65f);

// Check if a cell is free (probability < threshold)
bool is_free(const occupancy_map_t* map, int x, int y, float threshold = 0.35f);

// Check if a cell is unknown (probability == 0.5)
bool is_unknown(const occupancy_map_t* map, int x, int y);

// Save the map to a file
bool save_map(const occupancy_map_t* map, const char* filename);

// Load the map from a file
bool load_map(occupancy_map_t* map, const char* filename);

#endif // MAPPING_H
