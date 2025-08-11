// math_utils.h - Utilities for mathematical operations in SLAM

#ifndef MATH_UTILS_H
#define MATH_UTILS_H

#include <cmath>
#include <limits>

namespace math_utils {

// Constants
constexpr double PI = 3.14159265358979323846;
constexpr double DEG_TO_RAD = PI / 180.0;
constexpr double RAD_TO_DEG = 180.0 / PI;
constexpr double EPSILON = 1e-6;

// Normalize angle to [-pi, pi]
inline double normalize_angle(double angle) {
    while (angle > PI) angle -= 2.0 * PI;
    while (angle < -PI) angle += 2.0 * PI;
    return angle;
}

// Convert degrees to radians
inline double deg2rad(double deg) {
    return deg * DEG_TO_RAD;
}

// Convert radians to degrees
inline double rad2deg(double rad) {
    return rad * RAD_TO_DEG;
}

// Wrap value between min and max
inline double wrap(double value, double min, double max) {
    const double range = max - min;
    return value - range * std::floor((value - min) / range);
}

// Check if two floating point numbers are approximately equal
inline bool approx_equal(double a, double b, double epsilon = EPSILON) {
    return std::abs(a - b) < epsilon;
}

// Calculate Euclidean distance between two 2D points
inline double distance(double x1, double y1, double x2, double y2) {
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

// Calculate angle between two 2D points
inline double angle_between(double x1, double y1, double x2, double y2) {
    return std::atan2(y2 - y1, x2 - x1);
}

} // namespace math_utils

#endif // MATH_UTILS_H
