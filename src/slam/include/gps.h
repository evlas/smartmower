// gps.h - GPS utilities for SLAM

#ifndef GPS_UTILS_H
#define GPS_UTILS_H

#include "slam_types.h"
#include "math_utils.h"

// Earth's radius in meters
constexpr double EARTH_RADIUS = 6371000.0;

// Convert latitude/longitude to local coordinates (UTM)
void gps_to_utm(double lat, double lon, double* x, double* y);

// Convert local coordinates (UTM) to latitude/longitude
void utm_to_gps(double x, double y, double* lat, double* lon);

// Calculate distance between two GPS coordinates (in meters)
double haversine_distance(double lat1, double lon1, double lat2, double lon2);

// Calculate bearing between two GPS coordinates (in radians)
double calculate_bearing(double lat1, double lon1, double lat2, double lon2);

// Convert GPS coordinates to local coordinates using a reference point
void gps_to_local(double ref_lat, double ref_lon, double lat, double lon, double* x, double* y);

// Convert local coordinates back to GPS coordinates using a reference point
void local_to_gps(double ref_lat, double ref_lon, double x, double y, double* lat, double* lon);

// Initialize GPS system with reference point
void gps_init(double ref_latitude, double ref_longitude);

// Update robot position using GPS data
void gps_update_position(robot_pose_t* pose, const sensor_data_t* sensors);

// Check if GPS signal is valid
bool is_gps_valid(const sensor_data_t* sensors);

// Get GPS accuracy in meters
float get_gps_accuracy(const sensor_data_t* sensors);

// Convert degrees to meters (approximate)
double degrees_to_meters(double degrees);

// Convert meters to degrees (approximate)
double meters_to_degrees(double meters);

#endif // GPS_UTILS_H
