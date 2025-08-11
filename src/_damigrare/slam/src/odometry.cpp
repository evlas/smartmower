// odometry.cpp - Odometry functionality for SLAM

#include "odometry.h"
#include "slam_types.h"
#include <math.h>
#include <cstring> // For memcpy

// Normalize angle to [-π, π]
static inline float normalize_angle(float angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
    return angle;
}

// Initialize odometry system
void odometry_init(odometry_t* odom, slam_config_t* config) {
    if (!odom || !config) return;
    
    // Initialize pose
    odom->pose.x = 0.0f;
    odom->pose.y = 0.0f;
    odom->pose.theta = 0.0f;
    odom->pose.uncertainty = 0.0f;
    odom->pose.timestamp = 0;
    
    // Initialize velocity
    odom->velocity.vx = 0.0f;
    odom->velocity.vy = 0.0f;
    odom->velocity.vtheta = 0.0f;
    
    // Initialize encoder state
    odom->left_encoder_prev = 0;
    odom->right_encoder_prev = 0;
    odom->last_update = 0;
    
    // Note: robot_pose_t doesn't have covariance member in slam_types.h
    // Covariance is handled separately if needed
    
    // Note: odometry_t doesn't store wheel parameters - they're accessed from config
    // Parameters are in config->odometry.wheel_base, config->odometry.wheel_radius, etc.
}

// Update odometry with new sensor data
void odometry_update(odometry_t* odom, sensor_data_t* sensors, slam_config_t* config) {
    if (!odom || !sensors || !config) return;
    
    // Calculate time delta in seconds
    float dt = (sensors->timestamp - odom->last_update) / 1000.0f;
    if (dt <= 0) return; // Invalid time delta
    
    // Use fusion data instead of raw encoder data
    // The fusion data already contains processed pose and velocity information
    const fusion_data_t& fusion = sensors->fusion;
    
    // Calculate displacement from previous pose
    float dx = fusion.x - odom->pose.x;
    float dy = fusion.y - odom->pose.y;
    
    // Update pose with fusion data
    odom->pose.x = fusion.x;
    odom->pose.y = fusion.y;
    odom->pose.theta = fusion.theta;
    odom->pose.timestamp = sensors->timestamp;
    
    // Update velocity with fusion data
    odom->velocity.vx = fusion.vx;
    odom->velocity.vy = fusion.vy;
    odom->velocity.vtheta = fusion.vtheta;
    
    // Update distance traveled
    float distance = sqrtf(dx * dx + dy * dy);
    odom->distance_traveled += distance;
    
    // Update timestamps
    odom->last_update = sensors->timestamp;
}

// Reset odometry to initial state
void odometry_reset(odometry_t* odom) {
    if (!odom) return;
    
    odom->pose.x = 0.0f;
    odom->pose.y = 0.0f;
    odom->pose.theta = 0.0f;
    odom->pose.uncertainty = 0.0f;
    odom->pose.timestamp = 0;
    
    odom->velocity.vx = 0.0f;
    odom->velocity.vy = 0.0f;
    odom->velocity.vtheta = 0.0f;
    
    odom->left_encoder_prev = 0;
    odom->right_encoder_prev = 0;
    odom->last_update = 0;
    odom->distance_traveled = 0.0f;
    
    // Note: robot_pose_t doesn't have covariance member in slam_types.h
    // Covariance is handled separately if needed
}

// Get the current pose from odometry
void odometry_get_pose(const odometry_t* odom, float* x, float* y, float* theta) {
    if (!odom) return;
    
    if (x) *x = odom->pose.x;
    if (y) *y = odom->pose.y;
    if (theta) *theta = odom->pose.theta;
}

// Set the current pose in odometry
void odometry_set_pose(odometry_t* odom, float x, float y, float theta) {
    if (!odom) return;
    
    odom->pose.x = x;
    odom->pose.y = y;
    odom->pose.theta = theta;
    
    // Reset velocity to avoid jumps
    odom->velocity.vx = 0.0f;
    odom->velocity.vy = 0.0f;
    odom->velocity.vtheta = 0.0f;
}

// Get the current velocity from odometry
void odometry_get_velocity(const odometry_t* odom, float* vx, float* vy, float* vtheta) {
    if (!odom) return;
    
    if (vx) *vx = odom->velocity.vx;
    if (vy) *vy = odom->velocity.vy;
    if (vtheta) *vtheta = odom->velocity.vtheta;
}

// Get the current pose covariance
void odometry_get_covariance(const odometry_t* odom, float* covariance) {
    if (!odom || !covariance) return;
    
    // Note: robot_pose_t doesn't have covariance member in slam_types.h
    // Return identity matrix as default covariance
    for (int i = 0; i < 9; i++) {
        covariance[i] = (i % 4 == 0) ? 1.0f : 0.0f;
    }
}

// Predict the pose at a future time based on current velocity
void odometry_predict_pose(const odometry_t* odom, float dt, float* x, float* y, float* theta) {
    if (!odom) return;
    
    float pred_theta = odom->pose.theta + odom->velocity.vtheta * dt;
    // Normalize angle to [-pi, pi]
    while (pred_theta > M_PI) pred_theta -= 2.0f * M_PI;
    while (pred_theta < -M_PI) pred_theta += 2.0f * M_PI;
    
    if (x) *x = odom->pose.x + odom->velocity.vx * cosf(pred_theta) * dt;
    if (y) *y = odom->pose.y + odom->velocity.vx * sinf(pred_theta) * dt;
    if (theta) *theta = pred_theta;
}
