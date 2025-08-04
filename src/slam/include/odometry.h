// odometry.h - Odometry handling for SLAM

#ifndef ODOMETRY_H
#define ODOMETRY_H

#include "slam_types.h"

// Forward declarations are not needed since slam_types.h is included

// Initialize odometry system
void odometry_init(odometry_t* odom, slam_config_t* config);

// Update odometry with new sensor data
void odometry_update(odometry_t* odom, sensor_data_t* sensors, slam_config_t* config);

// Reset odometry to initial state
void odometry_reset(odometry_t* odom);

// Get the current pose from odometry
void odometry_get_pose(const odometry_t* odom, float* x, float* y, float* theta);

// Set the current pose in odometry
void odometry_set_pose(odometry_t* odom, float x, float y, float theta);

// Get the current velocity from odometry
void odometry_get_velocity(const odometry_t* odom, float* vx, float* vy, float* vtheta);

// Get the current pose covariance
void odometry_get_covariance(const odometry_t* odom, float* covariance);

// Predict the pose at a future time based on current velocity
void odometry_predict_pose(const odometry_t* odom, float dt, float* x, float* y, float* theta);

#endif // ODOMETRY_H
