// slam_functions.h - Function declarations for SLAM system

#ifndef SLAM_FUNCTIONS_H
#define SLAM_FUNCTIONS_H

#include "slam_types.h"

// Odometry functions
void odometry_init(odometry_t* odom, slam_config_t* config);
void odometry_update(odometry_t* odom, sensor_data_t* sensors, slam_config_t* config);

// Mapping functions
void mapping_init(occupancy_map_t* map);
bool world_to_grid(occupancy_map_t* map, float x, float y, int* grid_x, int* grid_y);
void mapping_update_sonar(occupancy_map_t* map, robot_pose_t* pose, sensor_data_t* sensors, slam_config_t* config);

// GPS functions
void gps_to_local(float lat, float lon, float origin_lat, float origin_lon, float* x, float* y);
void gps_correction(robot_pose_t* pose, sensor_data_t* sensors, slam_config_t* config);

// SLAM system functions
void slam_init(slam_system_t* system, slam_config_t* config);
void slam_update(slam_system_t* system, sensor_data_t* sensors);
void slam_save_map(const char* filename, const occupancy_map_t* map);
void slam_load_map(const char* filename, occupancy_map_t* map);

#endif // SLAM_FUNCTIONS_H
