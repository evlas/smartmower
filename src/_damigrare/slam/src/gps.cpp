// gps.cpp - GPS utilities for SLAM

#include "gps.h"
#include <cmath>
#include <iostream>

// Initialize GPS system with reference point
void gps_init(double ref_latitude, double ref_longitude) {
    (void)ref_latitude;   // Suppress unused parameter warning
    (void)ref_longitude;  // Suppress unused parameter warning
    // Implementation depends on your GPS hardware and requirements
    std::cout << "GPS initialized" << std::endl;
}

// Update robot position using GPS data
void gps_update_position(robot_pose_t* pose, const sensor_data_t* sensors) {
    if (!pose || !sensors) return;
    
    // Only update if we have a valid GPS fix
    if (sensors->gps.fix < GPS_FIX_2D) return;
    
    // Convert GPS coordinates to local coordinates
    double x, y;
    gps_to_local(0, 0, sensors->gps.latitude, sensors->gps.longitude, &x, &y);
    
    // Update pose (you might want to add filtering here)
    pose->x = static_cast<float>(x);
    pose->y = static_cast<float>(y);
    
    // Update heading if we have course information
    if (sensors->gps.fix == GPS_FIX_3D && sensors->gps.speed > 0.1f) {
        pose->theta = static_cast<float>(sensors->gps.course * M_PI / 180.0);
    }
}

// Check if GPS signal is valid
bool is_gps_valid(const sensor_data_t* sensors) {
    if (!sensors) return false;
    
    // Consider GPS valid if we have at least a 2D fix and reasonable HDOP
    return (sensors->gps.fix >= GPS_FIX_2D && 
            sensors->gps.hdop < 5.0f && 
            sensors->gps.satellites >= 4);
}

// Get GPS accuracy in meters (approximate based on HDOP)
float get_gps_accuracy(const sensor_data_t* sensors) {
    if (!sensors || sensors->gps.fix < GPS_FIX_2D) {
        return 9999.0f; // Large value indicating no fix
    }
    
    // Simple accuracy estimation based on HDOP
    // Typical HDOP values:
    // <1: Excellent
    // 1-2: Good
    // 2-5: Moderate
    // 5-10: Fair
    // >10: Poor
    return sensors->gps.hdop * 5.0f; // Approximate error in meters
}

// Convert degrees to radians
static inline double to_radians(double degrees) {
    return degrees * M_PI / 180.0;
}

// Convert latitude/longitude to local coordinates (UTM)
void gps_to_utm(double lat, double lon, double* x, double* y) {
    // This is a simplified implementation
    // In a real application, you would use a proper UTM conversion library
    *x = to_radians(lon) * EARTH_RADIUS;
    *y = std::log(std::tan(to_radians(lat)/2.0 + M_PI/4.0)) * EARTH_RADIUS;
}

// Convert local coordinates (UTM) to latitude/longitude
void utm_to_gps(double x, double y, double* lat, double* lon) {
    // This is a simplified implementation
    // In a real application, you would use a proper UTM conversion library
    *lon = x * 180.0 / (M_PI * EARTH_RADIUS);
    *lat = (2.0 * std::atan(std::exp(y / EARTH_RADIUS)) - M_PI/2.0) * 180.0 / M_PI;
}

// Calculate distance between two GPS coordinates (in meters)
double haversine_distance(double lat1, double lon1, double lat2, double lon2) {
    double dLat = to_radians(lat2 - lat1);
    double dLon = to_radians(lon2 - lon1);
    double a = std::sin(dLat/2) * std::sin(dLat/2) +
               std::cos(to_radians(lat1)) * std::cos(to_radians(lat2)) * 
               std::sin(dLon/2) * std::sin(dLon/2);
    double c = 2 * std::atan2(std::sqrt(a), std::sqrt(1-a));
    return EARTH_RADIUS * c;
}

// Calculate bearing between two GPS coordinates (in radians)
double calculate_bearing(double lat1, double lon1, double lat2, double lon2) {
    double dLon = to_radians(lon2 - lon1);
    double y = std::sin(dLon) * std::cos(to_radians(lat2));
    double x = std::cos(to_radians(lat1)) * std::sin(to_radians(lat2)) - 
               std::sin(to_radians(lat1)) * std::cos(to_radians(lat2)) * std::cos(dLon);
    return std::atan2(y, x);
}

// Convert GPS coordinates to local coordinates using a reference point
void gps_to_local(double ref_lat, double ref_lon, double lat, double lon, double* x, double* y) {
    // Simple flat earth approximation for small distances
    // For more accurate results, use a proper projection library like Proj or GeographicLib
    *x = (lon - ref_lon) * (M_PI/180.0) * EARTH_RADIUS * std::cos(to_radians(ref_lat));
    *y = (lat - ref_lat) * (M_PI/180.0) * EARTH_RADIUS;
}

// Convert local coordinates back to GPS coordinates using a reference point
void local_to_gps(double ref_lat, double ref_lon, double x, double y, double* lat, double* lon) {
    // Inverse of the flat earth approximation
    *lon = ref_lon + (x / (EARTH_RADIUS * std::cos(to_radians(ref_lat)) * (M_PI/180.0)));
    *lat = ref_lat + (y / (EARTH_RADIUS * (M_PI/180.0)));
}

// Convert degrees to meters (approximate)
double degrees_to_meters(double degrees) {
    return degrees * (M_PI/180.0) * EARTH_RADIUS;
}

// Convert meters to degrees (approximate)
double meters_to_degrees(double meters) {
    return (meters / EARTH_RADIUS) * (180.0/M_PI);
}
