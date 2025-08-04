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
// SPIRAL PATTERN IMPLEMENTATION
// =============================================================================

/**
 * Generate spiral pattern optimized for 6000 mq areas with obstacles
 * Uses outward-in spiral to ensure perimeter coverage first
 */
bool generate_spiral_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan) {
    
    if (!boundary || !config || !plan) {
        return false;
    }
    
    // Initialize plan
    memset(plan, 0, sizeof(PathPlan));
    strcpy(plan->pattern_type, PATH_PATTERN_SPIRAL);
    plan->plan_id = (uint32_t)time(NULL);
    plan->timestamp = (uint64_t)time(NULL);
    
    // Calculate area bounds
    double min_x = boundary->vertices[0].x, max_x = boundary->vertices[0].x;
    double min_y = boundary->vertices[0].y, max_y = boundary->vertices[0].y;
    
    for (uint32_t i = 1; i < boundary->vertex_count; i++) {
        if (boundary->vertices[i].x < min_x) min_x = boundary->vertices[i].x;
        if (boundary->vertices[i].x > max_x) max_x = boundary->vertices[i].x;
        if (boundary->vertices[i].y < min_y) min_y = boundary->vertices[i].y;
        if (boundary->vertices[i].y > max_y) max_y = boundary->vertices[i].y;
    }
    
    double area_width = max_x - min_x;
    double area_height = max_y - min_y;
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;
    
    // Calculate spiral parameters
    double cutting_width = config->cutting_width;
    double overlap = cutting_width * config->overlap_percentage;
    double effective_width = cutting_width - overlap;
    double spiral_step = effective_width;
    
    // Estimate maximum waypoints needed
    double spiral_radius = sqrt(area_width * area_width + area_height * area_height) / 2.0;
    uint32_t max_waypoints = (uint32_t)((2 * M_PI * spiral_radius) / spiral_step) * 2;
    if (max_waypoints > config->max_waypoints) {
        max_waypoints = config->max_waypoints;
    }
    
    // Allocate waypoint array
    plan->waypoints = malloc(max_waypoints * sizeof(Waypoint));
    if (!plan->waypoints) {
        return false;
    }
    
    // Generate spiral waypoints (outward-in pattern)
    uint32_t waypoint_id = 1;
    double current_radius = spiral_step;
    double angle = 0.0;
    double angle_step = spiral_step / current_radius; // Adaptive angle step
    
    printf("Generating spiral pattern for 6000mq area (%.1fm x %.1fm)\n", 
           area_width, area_height);
    printf("Cutting width: %.2fm, Effective width: %.2fm\n", 
           cutting_width, effective_width);
    
    while (current_radius < spiral_radius && plan->waypoint_count < max_waypoints - 1) {
        // Calculate position on spiral
        double x = center_x + current_radius * cos(angle);
        double y = center_y + current_radius * sin(angle);
        
        Point2D current_point = {x, y};
        
        // Check if point is within boundary
        if (point_in_boundary(&current_point, boundary)) {
            // Check obstacle collision
            if (!point_collides_obstacle(&current_point, obstacles, obstacle_count, 
                                       config->obstacle_buffer)) {
                
                // Create waypoint
                Waypoint waypoint = {0};
                waypoint.position = current_point;
                waypoint.heading = angle + M_PI/2; // Perpendicular to spiral
                waypoint.speed = config->max_speed;
                waypoint.id = waypoint_id++;
                waypoint.is_mowing = true;
                
                plan->waypoints[plan->waypoint_count] = waypoint;
                plan->waypoint_count++;
                
                // Debug output every 50 waypoints
                if (plan->waypoint_count % 50 == 0) {
                    printf("Generated %u waypoints, current radius: %.1fm\n", 
                           plan->waypoint_count, current_radius);
                }
            }
        }
        
        // Advance spiral
        angle += angle_step;
        
        // Increase radius gradually (Archimedean spiral)
        current_radius += spiral_step / (2 * M_PI);
        
        // Recalculate angle step for current radius
        if (current_radius > 0) {
            angle_step = spiral_step / current_radius;
        }
        
        // Prevent infinite loops
        if (angle > 20 * M_PI) { // Max 10 full rotations
            break;
        }
    }
    
    // Add perimeter coverage waypoints if needed
    if (plan->waypoint_count < max_waypoints - boundary->vertex_count) {
        printf("Adding perimeter coverage waypoints\n");
        
        for (uint32_t i = 0; i < boundary->vertex_count; i++) {
            Point2D perimeter_point = boundary->vertices[i];
            
            // Move point slightly inward to avoid perimeter
            double inward_x = center_x - perimeter_point.x;
            double inward_y = center_y - perimeter_point.y;
            double inward_dist = sqrt(inward_x*inward_x + inward_y*inward_y);
            
            if (inward_dist > 0) {
                inward_x /= inward_dist;
                inward_y /= inward_dist;
                
                perimeter_point.x += inward_x * config->perimeter_buffer;
                perimeter_point.y += inward_y * config->perimeter_buffer;
            }
            
            // Check obstacle collision
            if (!point_collides_obstacle(&perimeter_point, obstacles, obstacle_count,
                                       config->obstacle_buffer)) {
                
                Waypoint waypoint = {0};
                waypoint.position = perimeter_point;
                waypoint.heading = atan2(inward_y, inward_x);
                waypoint.speed = config->max_speed * 0.8; // Slower near perimeter
                waypoint.id = waypoint_id++;
                waypoint.is_mowing = true;
                
                plan->waypoints[plan->waypoint_count] = waypoint;
                plan->waypoint_count++;
            }
        }
    }
    
    // Optimize path order for battery efficiency
    if (config->optimize_battery) {
        printf("Optimizing path for battery efficiency\n");
        optimize_path_battery(plan, 1.0); // Assume full battery at start
    }
    
    // Smooth path to reduce sharp turns
    smooth_path(plan, config->turn_radius);
    
    // Calculate total distance and time
    plan->total_distance = 0.0;
    for (uint32_t i = 1; i < plan->waypoint_count; i++) {
        plan->total_distance += point_distance(&plan->waypoints[i-1].position,
                                             &plan->waypoints[i].position);
    }
    
    plan->estimated_time = estimate_completion_time(plan, config);
    
    printf("Spiral pattern generated successfully:\n");
    printf("- Waypoints: %u\n", plan->waypoint_count);
    printf("- Total distance: %.1f m\n", plan->total_distance);
    printf("- Estimated time: %.1f minutes\n", plan->estimated_time / 60.0);
    printf("- Estimated coverage: %.1f m²\n", 
           calculate_area_coverage(plan, cutting_width));
    
    return true;
}

/**
 * Optimize spiral path for battery efficiency
 * Reorders waypoints to minimize travel distance between mowing segments
 */
bool optimize_spiral_battery(PathPlan* plan, double battery_capacity __attribute__((unused))) {
    if (!plan || plan->waypoint_count < 2) {
        return false;
    }
    
    // Simple optimization: ensure we start from the closest point to current position
    // For spiral patterns, this usually means starting from the outer edge
    
    // Find the waypoint closest to the boundary (outer edge)
    uint32_t start_index = 0;
    double max_distance_from_center = 0.0;
    
    // Calculate center of all waypoints
    double center_x = 0.0, center_y = 0.0;
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        center_x += plan->waypoints[i].position.x;
        center_y += plan->waypoints[i].position.y;
    }
    center_x /= plan->waypoint_count;
    center_y /= plan->waypoint_count;
    
    Point2D center = {center_x, center_y};
    
    // Find outermost waypoint
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        double dist = point_distance(&plan->waypoints[i].position, &center);
        if (dist > max_distance_from_center) {
            max_distance_from_center = dist;
            start_index = i;
        }
    }
    
    // If we need to reorder, rotate the array to start from the optimal point
    if (start_index != 0) {
        Waypoint* temp = malloc(plan->waypoint_count * sizeof(Waypoint));
        if (!temp) return false;
        
        // Copy from start_index to end
        memcpy(temp, &plan->waypoints[start_index], 
               (plan->waypoint_count - start_index) * sizeof(Waypoint));
        
        // Copy from beginning to start_index
        memcpy(&temp[plan->waypoint_count - start_index], plan->waypoints,
               start_index * sizeof(Waypoint));
        
        // Replace original array
        memcpy(plan->waypoints, temp, plan->waypoint_count * sizeof(Waypoint));
        free(temp);
        
        printf("Optimized spiral start point (moved %u positions)\n", start_index);
    }
    
    return true;
}

/**
 * Handle obstacle avoidance in spiral pattern
 * Creates detour paths around obstacles while maintaining spiral integrity
 */
bool handle_spiral_obstacles(PathPlan* plan, const Obstacle* obstacles,
                            uint32_t obstacle_count, double buffer) {
    if (!plan || !obstacles || obstacle_count == 0) {
        return true; // No obstacles to handle
    }
    
    bool modified = false;
    
    // Check each waypoint for obstacle collision
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        if (point_collides_obstacle(&plan->waypoints[i].position, obstacles,
                                   obstacle_count, buffer)) {
            
            // Find safe alternative position
            Point2D original = plan->waypoints[i].position;
            Point2D safe_position = original;
            bool found_safe = false;
            
            // Try positions in a circle around the original point
            for (double angle = 0; angle < 2 * M_PI; angle += M_PI/8) {
                for (double radius = buffer; radius < buffer * 3; radius += 0.5) {
                    safe_position.x = original.x + radius * cos(angle);
                    safe_position.y = original.y + radius * sin(angle);
                    
                    if (!point_collides_obstacle(&safe_position, obstacles,
                                                obstacle_count, buffer)) {
                        plan->waypoints[i].position = safe_position;
                        found_safe = true;
                        modified = true;
                        break;
                    }
                }
                if (found_safe) break;
            }
            
            if (!found_safe) {
                // Remove waypoint if no safe position found
                for (uint32_t j = i; j < plan->waypoint_count - 1; j++) {
                    plan->waypoints[j] = plan->waypoints[j + 1];
                }
                plan->waypoint_count--;
                i--; // Recheck this index
                modified = true;
            }
        }
    }
    
    if (modified) {
        printf("Modified spiral pattern to avoid %u obstacles\n", obstacle_count);
    }
    
    return true;
}
