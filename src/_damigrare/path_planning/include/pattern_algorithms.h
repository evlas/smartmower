#ifndef PATTERN_ALGORITHMS_H
#define PATTERN_ALGORITHMS_H

#include "../path_planning_mqtt.h"

// =============================================================================
// PATTERN ALGORITHM INTERFACE
// =============================================================================

/**
 * Generate a spiral pattern for the given area
 * Optimized for 6000 mq areas with obstacle avoidance
 * 
 * @param boundary Area boundary definition
 * @param obstacles Array of obstacles to avoid
 * @param obstacle_count Number of obstacles
 * @param config Path planning configuration
 * @param plan Output path plan (allocated by function)
 * @return true if successful, false on error
 */
bool generate_spiral_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan);

/**
 * Generate a linear parallel pattern for the given area
 * Efficient for regular rectangular areas
 * 
 * @param boundary Area boundary definition
 * @param obstacles Array of obstacles to avoid
 * @param obstacle_count Number of obstacles
 * @param config Path planning configuration
 * @param plan Output path plan (allocated by function)
 * @return true if successful, false on error
 */
bool generate_linear_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan);

/**
 * Generate a random pattern with intelligent bias
 * Good for irregular areas and avoiding soil compaction
 * 
 * @param boundary Area boundary definition
 * @param obstacles Array of obstacles to avoid
 * @param obstacle_count Number of obstacles
 * @param config Path planning configuration
 * @param plan Output path plan (allocated by function)
 * @return true if successful, false on error
 */
bool generate_random_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan);

/**
 * Generate an adaptive pattern based on historical data
 * Optimizes for grass growth patterns and battery efficiency
 * 
 * @param boundary Area boundary definition
 * @param obstacles Array of obstacles to avoid
 * @param obstacle_count Number of obstacles
 * @param config Path planning configuration
 * @param historical_data Historical mowing data (can be NULL)
 * @param plan Output path plan (allocated by function)
 * @return true if successful, false on error
 */
bool generate_adaptive_pattern(const AreaBoundary* boundary,
                              const Obstacle* obstacles,
                              uint32_t obstacle_count,
                              const PathPlanningConfig* config,
                              void* historical_data,
                              PathPlan* plan);

/**
 * Create default obstacle configuration for 6000 mq area testing
 * Generates realistic obstacles for residential lawn scenarios
 * 
 * @param obstacles Pointer to obstacle array (allocated by function)
 * @param obstacle_count Pointer to obstacle count (updated)
 * @return true if successful, false on error
 */
bool create_default_obstacles_6000mq(Obstacle** obstacles, uint32_t* obstacle_count);

/**
 * Validate path plan for safety and completeness
 * Checks boundary compliance, obstacle avoidance, and performance metrics
 * 
 * @param plan Path plan to validate
 * @param boundary Area boundary
 * @param obstacles Array of obstacles
 * @param obstacle_count Number of obstacles
 * @param config Planning configuration
 * @return true if plan is valid and safe
 */
bool validate_path_plan(const PathPlan* plan, const AreaBoundary* boundary,
                       const Obstacle* obstacles, uint32_t obstacle_count,
                       const PathPlanningConfig* config);

// =============================================================================
// UTILITY FUNCTIONS
// =============================================================================

/**
 * Check if a point is inside the given boundary
 * 
 * @param point Point to check
 * @param boundary Area boundary
 * @return true if point is inside boundary
 */
bool point_in_boundary(const Point2D* point, const AreaBoundary* boundary);

/**
 * Check if a point collides with any obstacle
 * 
 * @param point Point to check
 * @param obstacles Array of obstacles
 * @param obstacle_count Number of obstacles
 * @param buffer Additional buffer around obstacles
 * @return true if point collides with obstacle
 */
bool point_collides_obstacle(const Point2D* point,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            double buffer);

/**
 * Calculate distance between two points
 * 
 * @param p1 First point
 * @param p2 Second point
 * @return Distance in meters
 */
double point_distance(const Point2D* p1, const Point2D* p2);

/**
 * Calculate angle between two points
 * 
 * @param from Starting point
 * @param to Target point
 * @return Angle in radians
 */
double point_angle(const Point2D* from, const Point2D* to);

/**
 * Optimize path for battery efficiency
 * Reorders waypoints to minimize energy consumption
 * 
 * @param plan Path plan to optimize
 * @param battery_capacity Current battery capacity (0.0 to 1.0)
 * @return true if optimization successful
 */
bool optimize_path_battery(PathPlan* plan, double battery_capacity);

/**
 * Smooth path to reduce sharp turns
 * 
 * @param plan Path plan to smooth
 * @param turn_radius Minimum turn radius
 * @return true if smoothing successful
 */
bool smooth_path(PathPlan* plan, double turn_radius);

/**
 * Estimate completion time for path plan
 * 
 * @param plan Path plan
 * @param config Planning configuration
 * @return Estimated time in seconds
 */
double estimate_completion_time(const PathPlan* plan, const PathPlanningConfig* config);

/**
 * Calculate total area coverage for path plan
 * 
 * @param plan Path plan
 * @param cutting_width Cutting width in meters
 * @return Covered area in square meters
 */
double calculate_area_coverage(const PathPlan* plan, double cutting_width);

/**
 * Free memory allocated for path plan
 * 
 * @param plan Path plan to free
 */
void free_path_plan(PathPlan* plan);

/**
 * Create a default 6000 mq rectangular boundary
 * Useful for testing and standard configurations
 * 
 * @param width Area width in meters
 * @param height Area height in meters
 * @param boundary Output boundary (allocated by function)
 * @return true if successful
 */
bool create_rectangular_boundary(double width, double height, AreaBoundary* boundary);

/**
 * Add obstacle to obstacle array
 * Helper function for dynamic obstacle management
 * 
 * @param obstacles Pointer to obstacle array (may be reallocated)
 * @param obstacle_count Pointer to obstacle count (updated)
 * @param center Obstacle center position
 * @param radius Obstacle radius
 * @param is_permanent Whether obstacle is permanent
 * @return true if successful
 */
bool add_obstacle(Obstacle** obstacles, uint32_t* obstacle_count,
                 const Point2D* center, double radius, bool is_permanent);

#endif // PATTERN_ALGORITHMS_H
