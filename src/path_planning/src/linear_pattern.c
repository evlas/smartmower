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
// LINEAR PATTERN IMPLEMENTATION
// =============================================================================

/**
 * Generate linear parallel pattern optimized for large rectangular areas
 * Uses boustrophedon (alternating direction) for efficiency
 */
bool generate_linear_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan) {
    
    if (!boundary || !config || !plan) {
        return false;
    }
    
    // Initialize plan
    memset(plan, 0, sizeof(PathPlan));
    strcpy(plan->pattern_type, PATH_PATTERN_LINEAR);
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
    
    // Calculate line parameters
    double cutting_width = config->cutting_width;
    double overlap = cutting_width * config->overlap_percentage;
    double effective_width = cutting_width - overlap;
    
    // Determine optimal direction (minimize number of lines)
    bool horizontal_lines = (area_width > area_height);
    
    double line_spacing = effective_width;
    uint32_t num_lines;
    double line_length;
    
    if (horizontal_lines) {
        num_lines = (uint32_t)ceil(area_height / line_spacing);
        line_length = area_width;
        printf("Using horizontal lines: %u lines of %.1fm each\n", num_lines, line_length);
    } else {
        num_lines = (uint32_t)ceil(area_width / line_spacing);
        line_length = area_height;
        printf("Using vertical lines: %u lines of %.1fm each\n", num_lines, line_length);
    }
    
    // Estimate waypoints needed (points per line + turn points)
    double waypoint_spacing = 5.0; // 5m between waypoints on each line
    uint32_t waypoints_per_line = (uint32_t)(line_length / waypoint_spacing) + 1;
    uint32_t max_waypoints = num_lines * waypoints_per_line + num_lines; // +1 for turns
    
    if (max_waypoints > config->max_waypoints) {
        max_waypoints = config->max_waypoints;
        waypoint_spacing = line_length / (max_waypoints / num_lines - 1);
        printf("Adjusted waypoint spacing to %.1fm due to limit\n", waypoint_spacing);
    }
    
    // Allocate waypoint array
    plan->waypoints = malloc(max_waypoints * sizeof(Waypoint));
    if (!plan->waypoints) {
        return false;
    }
    
    uint32_t waypoint_id = 1;
    bool left_to_right = true; // Direction alternates for boustrophedon
    
    // Generate lines
    for (uint32_t line = 0; line < num_lines && plan->waypoint_count < max_waypoints - 1; line++) {
        double line_position;
        
        if (horizontal_lines) {
            // Horizontal lines (Y varies, X is the line direction)
            line_position = min_y + line * line_spacing + line_spacing/2;
            
            // Generate waypoints along this horizontal line
            if (left_to_right) {
                // Left to right
                for (double x = min_x; x <= max_x && plan->waypoint_count < max_waypoints - 1; 
                     x += waypoint_spacing) {
                    Point2D point = {x, line_position};
                    
                    // Check boundary and obstacles
                    if (point_in_boundary(&point, boundary) &&
                        !point_collides_obstacle(&point, obstacles, obstacle_count,
                                                config->obstacle_buffer)) {
                        
                        Waypoint waypoint = {0};
                        waypoint.position = point;
                        waypoint.heading = 0.0; // East
                        waypoint.speed = config->max_speed;
                        waypoint.id = waypoint_id++;
                        waypoint.is_mowing = true;
                        
                        plan->waypoints[plan->waypoint_count] = waypoint;
                        plan->waypoint_count++;
                    }
                }
            } else {
                // Right to left
                for (double x = max_x; x >= min_x && plan->waypoint_count < max_waypoints - 1; 
                     x -= waypoint_spacing) {
                    Point2D point = {x, line_position};
                    
                    if (point_in_boundary(&point, boundary) &&
                        !point_collides_obstacle(&point, obstacles, obstacle_count,
                                                config->obstacle_buffer)) {
                        
                        Waypoint waypoint = {0};
                        waypoint.position = point;
                        waypoint.heading = M_PI; // West
                        waypoint.speed = config->max_speed;
                        waypoint.id = waypoint_id++;
                        waypoint.is_mowing = true;
                        
                        plan->waypoints[plan->waypoint_count] = waypoint;
                        plan->waypoint_count++;
                    }
                }
            }
        } else {
            // Vertical lines (X varies, Y is the line direction)
            line_position = min_x + line * line_spacing + line_spacing/2;
            
            // Generate waypoints along this vertical line
            if (left_to_right) {
                // Bottom to top
                for (double y = min_y; y <= max_y && plan->waypoint_count < max_waypoints - 1; 
                     y += waypoint_spacing) {
                    Point2D point = {line_position, y};
                    
                    if (point_in_boundary(&point, boundary) &&
                        !point_collides_obstacle(&point, obstacles, obstacle_count,
                                                config->obstacle_buffer)) {
                        
                        Waypoint waypoint = {0};
                        waypoint.position = point;
                        waypoint.heading = M_PI/2; // North
                        waypoint.speed = config->max_speed;
                        waypoint.id = waypoint_id++;
                        waypoint.is_mowing = true;
                        
                        plan->waypoints[plan->waypoint_count] = waypoint;
                        plan->waypoint_count++;
                    }
                }
            } else {
                // Top to bottom
                for (double y = max_y; y >= min_y && plan->waypoint_count < max_waypoints - 1; 
                     y -= waypoint_spacing) {
                    Point2D point = {line_position, y};
                    
                    if (point_in_boundary(&point, boundary) &&
                        !point_collides_obstacle(&point, obstacles, obstacle_count,
                                                config->obstacle_buffer)) {
                        
                        Waypoint waypoint = {0};
                        waypoint.position = point;
                        waypoint.heading = -M_PI/2; // South
                        waypoint.speed = config->max_speed;
                        waypoint.id = waypoint_id++;
                        waypoint.is_mowing = true;
                        
                        plan->waypoints[plan->waypoint_count] = waypoint;
                        plan->waypoint_count++;
                    }
                }
            }
        }
        
        // Add turn waypoint between lines (except for last line)
        if (line < num_lines - 1 && plan->waypoint_count < max_waypoints - 1) {
            Point2D turn_point;
            
            if (horizontal_lines) {
                // Turn at end of horizontal line
                double turn_x = left_to_right ? max_x : min_x;
                double next_y = min_y + (line + 1) * line_spacing + line_spacing/2;
                turn_point = (Point2D){turn_x, (line_position + next_y) / 2};
            } else {
                // Turn at end of vertical line
                double turn_y = left_to_right ? max_y : min_y;
                double next_x = min_x + (line + 1) * line_spacing + line_spacing/2;
                turn_point = (Point2D){(line_position + next_x) / 2, turn_y};
            }
            
            // Add turn waypoint if safe
            if (point_in_boundary(&turn_point, boundary) &&
                !point_collides_obstacle(&turn_point, obstacles, obstacle_count,
                                        config->obstacle_buffer)) {
                
                Waypoint turn_waypoint = {0};
                turn_waypoint.position = turn_point;
                turn_waypoint.heading = left_to_right ? M_PI : 0.0; // Turn direction
                turn_waypoint.speed = config->max_speed * 0.5; // Slower for turns
                turn_waypoint.id = waypoint_id++;
                turn_waypoint.is_mowing = false; // No mowing during turns
                
                plan->waypoints[plan->waypoint_count] = turn_waypoint;
                plan->waypoint_count++;
            }
        }
        
        // Alternate direction for next line (boustrophedon pattern)
        left_to_right = !left_to_right;
        
        // Progress update
        if ((line + 1) % 5 == 0) {
            printf("Generated %u/%u lines (%u waypoints)\n", 
                   line + 1, num_lines, plan->waypoint_count);
        }
    }
    
    // Optimize path if requested
    if (config->optimize_battery) {
        printf("Optimizing linear pattern for battery efficiency\n");
        optimize_path_battery(plan, 1.0);
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
    
    printf("Linear pattern generated successfully:\n");
    printf("- Pattern: %s lines\n", horizontal_lines ? "Horizontal" : "Vertical");
    printf("- Lines: %u\n", num_lines);
    printf("- Waypoints: %u\n", plan->waypoint_count);
    printf("- Total distance: %.1f m\n", plan->total_distance);
    printf("- Estimated time: %.1f minutes\n", plan->estimated_time / 60.0);
    printf("- Estimated coverage: %.1f m²\n", 
           calculate_area_coverage(plan, cutting_width));
    
    return true;
}

/**
 * Handle obstacle avoidance in linear pattern
 * Creates gaps in lines where obstacles are present
 */
bool handle_linear_obstacles(PathPlan* plan, const Obstacle* obstacles,
                            uint32_t obstacle_count, double buffer) {
    if (!plan || !obstacles || obstacle_count == 0) {
        return true;
    }
    
    bool modified = false;
    uint32_t removed_waypoints = 0;
    
    // Remove waypoints that collide with obstacles
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        if (point_collides_obstacle(&plan->waypoints[i].position, obstacles,
                                   obstacle_count, buffer)) {
            
            // Remove this waypoint by shifting array
            for (uint32_t j = i; j < plan->waypoint_count - 1; j++) {
                plan->waypoints[j] = plan->waypoints[j + 1];
            }
            plan->waypoint_count--;
            removed_waypoints++;
            i--; // Recheck this index
            modified = true;
        }
    }
    
    if (modified) {
        printf("Modified linear pattern: removed %u waypoints due to obstacles\n", 
               removed_waypoints);
        
        // Recalculate headings for remaining waypoints
        for (uint32_t i = 1; i < plan->waypoint_count; i++) {
            plan->waypoints[i].heading = point_angle(&plan->waypoints[i-1].position,
                                                   &plan->waypoints[i].position);
        }
    }
    
    return true;
}

/**
 * Optimize linear pattern for large areas (6000+ mq)
 * Adjusts line spacing and waypoint density based on area size
 */
bool optimize_linear_large_area(PathPlan* plan, double area_size, 
                               const PathPlanningConfig* config __attribute__((unused))) {
    if (!plan || area_size <= 0) {
        return false;
    }
    
    // For areas larger than 5000 mq, optimize waypoint density
    if (area_size > 5000.0) {
        printf("Optimizing linear pattern for large area (%.0f m²)\n", area_size);
        
        // Reduce waypoint density for large areas to improve efficiency
        double density_factor = 5000.0 / area_size; // Reduce density as area increases
        if (density_factor < 0.5) density_factor = 0.5; // Minimum 50% density
        
        uint32_t target_waypoints = (uint32_t)(plan->waypoint_count * density_factor);
        
        if (target_waypoints < plan->waypoint_count) {
            // Create optimized waypoint array
            Waypoint* optimized = malloc(target_waypoints * sizeof(Waypoint));
            if (!optimized) return false;
            
            // Select waypoints with even spacing
            double step = (double)plan->waypoint_count / target_waypoints;
            
            for (uint32_t i = 0; i < target_waypoints; i++) {
                uint32_t source_idx = (uint32_t)(i * step);
                if (source_idx >= plan->waypoint_count) {
                    source_idx = plan->waypoint_count - 1;
                }
                optimized[i] = plan->waypoints[source_idx];
                optimized[i].id = i + 1; // Renumber waypoints
            }
            
            // Replace waypoints
            free(plan->waypoints);
            plan->waypoints = optimized;
            plan->waypoint_count = target_waypoints;
            
            printf("Reduced waypoints from %u to %u for large area efficiency\n",
                   (uint32_t)(target_waypoints / density_factor), target_waypoints);
        }
    }
    
    return true;
}
