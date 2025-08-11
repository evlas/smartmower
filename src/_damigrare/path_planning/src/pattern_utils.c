#include "../include/pattern_algorithms.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <time.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// =============================================================================
// UTILITY FUNCTIONS IMPLEMENTATION
// =============================================================================

/**
 * Check if a point is inside the given boundary using ray casting algorithm
 */
bool point_in_boundary(const Point2D* point, const AreaBoundary* boundary) {
    if (!point || !boundary || boundary->vertex_count < 3) {
        return false;
    }
    
    bool inside = false;
    uint32_t j = boundary->vertex_count - 1;
    
    for (uint32_t i = 0; i < boundary->vertex_count; i++) {
        if (((boundary->vertices[i].y > point->y) != (boundary->vertices[j].y > point->y)) &&
            (point->x < (boundary->vertices[j].x - boundary->vertices[i].x) * 
             (point->y - boundary->vertices[i].y) / (boundary->vertices[j].y - boundary->vertices[i].y) + 
             boundary->vertices[i].x)) {
            inside = !inside;
        }
        j = i;
    }
    
    return inside;
}

/**
 * Check if a point collides with any obstacle
 */
bool point_collides_obstacle(const Point2D* point,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            double buffer) {
    if (!point || !obstacles) {
        return false;
    }
    
    for (uint32_t i = 0; i < obstacle_count; i++) {
        double distance = point_distance(point, &obstacles[i].center);
        if (distance <= (obstacles[i].radius + buffer)) {
            return true;
        }
    }
    
    return false;
}

/**
 * Calculate distance between two points
 */
double point_distance(const Point2D* p1, const Point2D* p2) {
    if (!p1 || !p2) {
        return 0.0;
    }
    
    double dx = p2->x - p1->x;
    double dy = p2->y - p1->y;
    return sqrt(dx * dx + dy * dy);
}

/**
 * Calculate angle between two points
 */
double point_angle(const Point2D* from, const Point2D* to) {
    if (!from || !to) {
        return 0.0;
    }
    
    double dx = to->x - from->x;
    double dy = to->y - from->y;
    return atan2(dy, dx);
}

/**
 * Optimize path for battery efficiency using nearest neighbor heuristic
 */
bool optimize_path_battery(PathPlan* plan, double battery_capacity __attribute__((unused))) {
    if (!plan || plan->waypoint_count < 2) {
        return false;
    }
    
    // Create array to track visited waypoints
    bool* visited = calloc(plan->waypoint_count, sizeof(bool));
    if (!visited) {
        return false;
    }
    
    // Create new optimized waypoint array
    Waypoint* optimized = malloc(plan->waypoint_count * sizeof(Waypoint));
    if (!optimized) {
        free(visited);
        return false;
    }
    
    // Start from first waypoint
    optimized[0] = plan->waypoints[0];
    visited[0] = true;
    
    // Find nearest unvisited waypoint for each step
    for (uint32_t step = 1; step < plan->waypoint_count; step++) {
        uint32_t nearest_idx = 0;
        double min_distance = INFINITY;
        
        for (uint32_t i = 0; i < plan->waypoint_count; i++) {
            if (!visited[i]) {
                double distance = point_distance(&optimized[step-1].position,
                                               &plan->waypoints[i].position);
                if (distance < min_distance) {
                    min_distance = distance;
                    nearest_idx = i;
                }
            }
        }
        
        optimized[step] = plan->waypoints[nearest_idx];
        visited[nearest_idx] = true;
    }
    
    // Replace original waypoints with optimized ones
    memcpy(plan->waypoints, optimized, plan->waypoint_count * sizeof(Waypoint));
    
    free(visited);
    free(optimized);
    
    printf("Path optimized for battery efficiency\n");
    return true;
}

/**
 * Smooth path to reduce sharp turns using simple averaging
 */
bool smooth_path(PathPlan* plan, double turn_radius __attribute__((unused))) {
    if (!plan || plan->waypoint_count < 3) {
        return false;
    }
    
    // Create smoothed waypoint array
    Waypoint* smoothed = malloc(plan->waypoint_count * sizeof(Waypoint));
    if (!smoothed) {
        return false;
    }
    
    // Copy first and last waypoints unchanged
    smoothed[0] = plan->waypoints[0];
    smoothed[plan->waypoint_count - 1] = plan->waypoints[plan->waypoint_count - 1];
    
    // Smooth intermediate waypoints
    for (uint32_t i = 1; i < plan->waypoint_count - 1; i++) {
        Point2D prev = plan->waypoints[i-1].position;
        Point2D curr = plan->waypoints[i].position;
        Point2D next = plan->waypoints[i+1].position;
        
        // Calculate turn angle
        double angle1 = point_angle(&prev, &curr);
        double angle2 = point_angle(&curr, &next);
        double turn_angle = fabs(angle2 - angle1);
        
        // Normalize angle
        while (turn_angle > M_PI) turn_angle -= 2 * M_PI;
        while (turn_angle < -M_PI) turn_angle += 2 * M_PI;
        
        // If turn is too sharp, smooth the position
        if (fabs(turn_angle) > M_PI/4) { // 45 degrees
            double smooth_factor = 0.3; // Smoothing strength
            
            smoothed[i] = plan->waypoints[i];
            smoothed[i].position.x = curr.x + smooth_factor * 
                                   ((prev.x + next.x) / 2.0 - curr.x);
            smoothed[i].position.y = curr.y + smooth_factor * 
                                   ((prev.y + next.y) / 2.0 - curr.y);
            
            // Update heading to match smoothed direction
            if (i < plan->waypoint_count - 1) {
                smoothed[i].heading = point_angle(&smoothed[i].position,
                                                &plan->waypoints[i+1].position);
            }
        } else {
            smoothed[i] = plan->waypoints[i];
        }
    }
    
    // Replace original waypoints with smoothed ones
    memcpy(plan->waypoints, smoothed, plan->waypoint_count * sizeof(Waypoint));
    free(smoothed);
    
    printf("Path smoothed to reduce sharp turns\n");
    return true;
}

/**
 * Estimate completion time for path plan
 */
double estimate_completion_time(const PathPlan* plan, const PathPlanningConfig* config) {
    if (!plan || !config || plan->waypoint_count == 0) {
        return 0.0;
    }
    
    double total_time = 0.0;
    double turn_time_penalty = 2.0; // Extra seconds per turn
    
    for (uint32_t i = 1; i < plan->waypoint_count; i++) {
        double distance = point_distance(&plan->waypoints[i-1].position,
                                       &plan->waypoints[i].position);
        
        // Base travel time
        double speed = plan->waypoints[i].speed;
        if (speed <= 0) speed = config->max_speed;
        
        total_time += distance / speed;
        
        // Add turn penalty if significant direction change
        if (i > 1) {
            double angle1 = point_angle(&plan->waypoints[i-2].position,
                                      &plan->waypoints[i-1].position);
            double angle2 = point_angle(&plan->waypoints[i-1].position,
                                      &plan->waypoints[i].position);
            double turn_angle = fabs(angle2 - angle1);
            
            if (turn_angle > M_PI/6) { // 30 degrees
                total_time += turn_time_penalty;
            }
        }
    }
    
    // Add overhead for obstacle avoidance and battery management
    total_time *= 1.2; // 20% overhead
    
    return total_time;
}

/**
 * Calculate total area coverage for path plan
 */
double calculate_area_coverage(const PathPlan* plan, double cutting_width) {
    if (!plan || plan->waypoint_count < 2 || cutting_width <= 0) {
        return 0.0;
    }
    
    double total_coverage = 0.0;
    
    for (uint32_t i = 1; i < plan->waypoint_count; i++) {
        if (plan->waypoints[i].is_mowing) {
            double segment_length = point_distance(&plan->waypoints[i-1].position,
                                                 &plan->waypoints[i].position);
            total_coverage += segment_length * cutting_width;
        }
    }
    
    return total_coverage;
}

/**
 * Free memory allocated for path plan
 */
void free_path_plan(PathPlan* plan) {
    if (plan && plan->waypoints) {
        free(plan->waypoints);
        plan->waypoints = NULL;
        plan->waypoint_count = 0;
    }
}

/**
 * Create a default rectangular boundary for testing
 * Optimized for 6000 mq area (approximately 77m x 77m)
 */
bool create_rectangular_boundary(double width, double height, AreaBoundary* boundary) {
    if (!boundary || width <= 0 || height <= 0) {
        return false;
    }
    
    // Allocate vertices for rectangle (4 corners)
    boundary->vertices = malloc(4 * sizeof(Point2D));
    if (!boundary->vertices) {
        return false;
    }
    
    boundary->vertex_count = 4;
    boundary->total_area = width * height;
    
    // Define rectangle corners (centered at origin)
    double half_width = width / 2.0;
    double half_height = height / 2.0;
    
    boundary->vertices[0] = (Point2D){-half_width, -half_height}; // Bottom-left
    boundary->vertices[1] = (Point2D){half_width, -half_height};  // Bottom-right
    boundary->vertices[2] = (Point2D){half_width, half_height};   // Top-right
    boundary->vertices[3] = (Point2D){-half_width, half_height};  // Top-left
    
    printf("Created rectangular boundary: %.1fm x %.1fm (%.0f m²)\n",
           width, height, boundary->total_area);
    
    return true;
}

/**
 * Add obstacle to obstacle array with dynamic reallocation
 */
bool add_obstacle(Obstacle** obstacles, uint32_t* obstacle_count,
                 const Point2D* center, double radius, bool is_permanent) {
    if (!obstacles || !obstacle_count || !center || radius <= 0) {
        return false;
    }
    
    // Reallocate array for new obstacle
    Obstacle* new_obstacles = realloc(*obstacles, (*obstacle_count + 1) * sizeof(Obstacle));
    if (!new_obstacles) {
        return false;
    }
    
    *obstacles = new_obstacles;
    
    // Add new obstacle
    Obstacle* new_obstacle = &(*obstacles)[*obstacle_count];
    new_obstacle->center = *center;
    new_obstacle->radius = radius;
    new_obstacle->id = *obstacle_count + 1;
    new_obstacle->is_permanent = is_permanent;
    new_obstacle->timestamp = (uint64_t)time(NULL);
    
    (*obstacle_count)++;
    
    printf("Added %s obstacle at (%.1f, %.1f) with radius %.1fm\n",
           is_permanent ? "permanent" : "temporary",
           center->x, center->y, radius);
    
    return true;
}

/**
 * Create default obstacle configuration for 6000 mq area testing
 */
bool create_default_obstacles_6000mq(Obstacle** obstacles, uint32_t* obstacle_count) {
    if (!obstacles || !obstacle_count) {
        return false;
    }
    
    *obstacles = NULL;
    *obstacle_count = 0;
    
    // Add some typical obstacles for a 6000 mq residential area
    
    // Trees (permanent obstacles)
    Point2D tree1 = {-20.0, 15.0};
    add_obstacle(obstacles, obstacle_count, &tree1, 2.5, true);
    
    Point2D tree2 = {25.0, -10.0};
    add_obstacle(obstacles, obstacle_count, &tree2, 3.0, true);
    
    Point2D tree3 = {-15.0, -25.0};
    add_obstacle(obstacles, obstacle_count, &tree3, 2.0, true);
    
    // Garden shed (permanent)
    Point2D shed = {30.0, 30.0};
    add_obstacle(obstacles, obstacle_count, &shed, 4.0, true);
    
    // Flower beds (permanent)
    Point2D flowerbed1 = {-30.0, 20.0};
    add_obstacle(obstacles, obstacle_count, &flowerbed1, 1.5, true);
    
    Point2D flowerbed2 = {10.0, 25.0};
    add_obstacle(obstacles, obstacle_count, &flowerbed2, 2.0, true);
    
    // Temporary obstacles (furniture, toys, etc.)
    Point2D furniture = {5.0, -5.0};
    add_obstacle(obstacles, obstacle_count, &furniture, 1.0, false);
    
    Point2D playground = {-10.0, 5.0};
    add_obstacle(obstacles, obstacle_count, &playground, 1.5, false);
    
    printf("Created default obstacle set for 6000mq area (%u obstacles)\n", 
           *obstacle_count);
    
    return true;
}

/**
 * Validate path plan for safety and completeness
 */
bool validate_path_plan(const PathPlan* plan, const AreaBoundary* boundary,
                       const Obstacle* obstacles, uint32_t obstacle_count,
                       const PathPlanningConfig* config) {
    if (!plan || !boundary || !config) {
        printf("ERROR: Invalid path plan parameters\n");
        return false;
    }
    
    if (plan->waypoint_count == 0) {
        printf("ERROR: Path plan has no waypoints\n");
        return false;
    }
    
    uint32_t invalid_waypoints = 0;
    uint32_t collision_waypoints = 0;
    
    // Check each waypoint
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        const Waypoint* wp = &plan->waypoints[i];
        
        // Check if waypoint is within boundary
        if (!point_in_boundary(&wp->position, boundary)) {
            invalid_waypoints++;
        }
        
        // Check obstacle collisions
        if (point_collides_obstacle(&wp->position, obstacles, obstacle_count,
                                   config->obstacle_buffer)) {
            collision_waypoints++;
        }
        
        // Check speed limits
        if (wp->speed > config->max_speed) {
            printf("WARNING: Waypoint %u exceeds max speed (%.2f > %.2f)\n",
                   wp->id, wp->speed, config->max_speed);
        }
    }
    
    // Report validation results
    printf("Path plan validation results:\n");
    printf("- Total waypoints: %u\n", plan->waypoint_count);
    printf("- Invalid waypoints (outside boundary): %u\n", invalid_waypoints);
    printf("- Collision waypoints: %u\n", collision_waypoints);
    printf("- Total distance: %.1f m\n", plan->total_distance);
    printf("- Estimated time: %.1f minutes\n", plan->estimated_time / 60.0);
    
    bool is_valid = (invalid_waypoints == 0 && collision_waypoints == 0);
    printf("- Plan is %s\n", is_valid ? "VALID" : "INVALID");
    
    return is_valid;
}
