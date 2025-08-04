#include "../path_planning_mqtt.h"
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
// SLAM INTEGRATION FOR PATH PLANNING
// =============================================================================

/**
 * Check if a point is safe for navigation using SLAM map data
 */
bool is_point_safe_slam(const SlamMap* slam_map, const Point2D* point, double safety_buffer) {
    if (!slam_map || !slam_map->cells || !point) {
        return false; // No map data, assume unsafe
    }
    
    // Convert world coordinates to map coordinates
    int32_t map_x = (int32_t)((point->x - slam_map->origin.x) / slam_map->resolution);
    int32_t map_y = (int32_t)((point->y - slam_map->origin.y) / slam_map->resolution);
    
    // Check if point is within map bounds
    if (map_x < 0 || map_x >= (int32_t)slam_map->width ||
        map_y < 0 || map_y >= (int32_t)slam_map->height) {
        return false; // Outside map, assume unsafe
    }
    
    // Calculate safety buffer in map cells
    int32_t buffer_cells = (int32_t)ceil(safety_buffer / slam_map->resolution);
    
    // Check area around point for obstacles
    for (int32_t dy = -buffer_cells; dy <= buffer_cells; dy++) {
        for (int32_t dx = -buffer_cells; dx <= buffer_cells; dx++) {
            int32_t check_x = map_x + dx;
            int32_t check_y = map_y + dy;
            
            // Skip if outside map
            if (check_x < 0 || check_x >= (int32_t)slam_map->width ||
                check_y < 0 || check_y >= (int32_t)slam_map->height) {
                continue;
            }
            
            // Check if within safety radius
            double dist = sqrt(dx*dx + dy*dy) * slam_map->resolution;
            if (dist <= safety_buffer) {
                SlamMapCell* cell = &slam_map->cells[check_y * slam_map->width + check_x];
                
                // Consider occupied or unknown cells as unsafe
                if (cell->occupancy > 200 || // Definitely occupied
                    cell->is_obstacle ||     // Marked as obstacle
                    !cell->is_traversable || // Not safe for mowing
                    (cell->occupancy == 127 && cell->confidence < 50)) { // Unknown with low confidence
                    return false;
                }
            }
        }
    }
    
    return true; // Point is safe
}

/**
 * Get the best position estimate using SLAM pose with fusion fallback
 */
Point2D get_best_position_estimate(const SlamPose* slam_pose, const Point2D* fusion_position,
                                  const PathPlanningConfig* config) {
    if (!config) {
        return fusion_position ? *fusion_position : (Point2D){0.0, 0.0};
    }
    
    // Use SLAM if enabled and pose is valid with sufficient confidence
    if (config->use_slam_data && slam_pose && slam_pose->valid &&
        slam_pose->confidence >= config->slam_confidence_threshold) {
        
        printf("Using SLAM pose: (%.2f, %.2f) confidence=%.2f\n",
               slam_pose->position.x, slam_pose->position.y, slam_pose->confidence);
        return slam_pose->position;
    }
    
    // Fallback to fusion data
    if (fusion_position) {
        printf("Using Fusion pose: (%.2f, %.2f) [SLAM unavailable]\n",
               fusion_position->x, fusion_position->y);
        return *fusion_position;
    }
    
    // Last resort: return origin
    printf("WARNING: No position data available, using origin\n");
    return (Point2D){0.0, 0.0};
}

/**
 * Extract obstacles from SLAM map for path planning
 */
uint32_t extract_obstacles_from_slam(const SlamMap* slam_map, Obstacle** obstacles,
                                    double min_obstacle_size, double max_obstacles) {
    if (!slam_map || !slam_map->cells || !obstacles) {
        return 0;
    }
    
    printf("Extracting obstacles from SLAM map (%ux%u cells)\n", 
           slam_map->width, slam_map->height);
    
    // Temporary storage for obstacle candidates
    typedef struct {
        Point2D center;
        uint32_t cell_count;
        double min_x, max_x, min_y, max_y;
    } ObstacleCandidate;
    
    ObstacleCandidate* candidates = malloc(max_obstacles * sizeof(ObstacleCandidate));
    if (!candidates) {
        return 0;
    }
    
    uint32_t candidate_count = 0;
    
    // Create visited array to track processed cells
    bool* visited = calloc(slam_map->width * slam_map->height, sizeof(bool));
    if (!visited) {
        free(candidates);
        return 0;
    }
    
    // Scan map for obstacle clusters
    for (uint32_t y = 0; y < slam_map->height && candidate_count < max_obstacles; y++) {
        for (uint32_t x = 0; x < slam_map->width && candidate_count < max_obstacles; x++) {
            uint32_t cell_idx = y * slam_map->width + x;
            
            if (visited[cell_idx]) continue;
            
            SlamMapCell* cell = &slam_map->cells[cell_idx];
            
            // Check if this cell represents an obstacle
            if (cell->occupancy > 200 || cell->is_obstacle) {
                // Start flood fill to find connected obstacle cells
                ObstacleCandidate candidate = {0};
                candidate.min_x = candidate.max_x = x * slam_map->resolution + slam_map->origin.x;
                candidate.min_y = candidate.max_y = y * slam_map->resolution + slam_map->origin.y;
                
                // Simple flood fill (stack-based to avoid recursion)
                typedef struct { uint32_t x, y; } StackItem;
                StackItem* stack = malloc(1000 * sizeof(StackItem));
                if (!stack) continue;
                
                uint32_t stack_size = 0;
                stack[stack_size++] = (StackItem){x, y};
                visited[cell_idx] = true;
                
                while (stack_size > 0 && candidate.cell_count < 1000) {
                    StackItem item = stack[--stack_size];
                    candidate.cell_count++;
                    
                    // Update bounding box
                    double world_x = item.x * slam_map->resolution + slam_map->origin.x;
                    double world_y = item.y * slam_map->resolution + slam_map->origin.y;
                    
                    if (world_x < candidate.min_x) candidate.min_x = world_x;
                    if (world_x > candidate.max_x) candidate.max_x = world_x;
                    if (world_y < candidate.min_y) candidate.min_y = world_y;
                    if (world_y > candidate.max_y) candidate.max_y = world_y;
                    
                    // Check 4-connected neighbors
                    int32_t neighbors[4][2] = {{-1,0}, {1,0}, {0,-1}, {0,1}};
                    for (int i = 0; i < 4; i++) {
                        uint32_t nx = item.x + neighbors[i][0];
                        uint32_t ny = item.y + neighbors[i][1];
                        
                        if (nx < slam_map->width && ny < slam_map->height) {
                            uint32_t neighbor_idx = ny * slam_map->width + nx;
                            
                            if (!visited[neighbor_idx]) {
                                SlamMapCell* neighbor_cell = &slam_map->cells[neighbor_idx];
                                
                                if (neighbor_cell->occupancy > 200 || neighbor_cell->is_obstacle) {
                                    visited[neighbor_idx] = true;
                                    if (stack_size < 999) {
                                        stack[stack_size++] = (StackItem){nx, ny};
                                    }
                                }
                            }
                        }
                    }
                }
                
                free(stack);
                
                // Calculate obstacle properties
                double width = candidate.max_x - candidate.min_x;
                double height = candidate.max_y - candidate.min_y;
                double size = sqrt(width * width + height * height);
                
                // Only keep obstacles above minimum size
                if (size >= min_obstacle_size) {
                    candidate.center.x = (candidate.min_x + candidate.max_x) / 2.0;
                    candidate.center.y = (candidate.min_y + candidate.max_y) / 2.0;
                    
                    candidates[candidate_count] = candidate;
                    candidate_count++;
                    
                    printf("Found obstacle cluster: center=(%.1f,%.1f) size=%.1fm cells=%u\n",
                           candidate.center.x, candidate.center.y, size, candidate.cell_count);
                }
            }
        }
    }
    
    // Convert candidates to obstacles
    *obstacles = malloc(candidate_count * sizeof(Obstacle));
    if (!*obstacles) {
        free(candidates);
        free(visited);
        return 0;
    }
    
    for (uint32_t i = 0; i < candidate_count; i++) {
        ObstacleCandidate* candidate = &candidates[i];
        Obstacle* obstacle = &(*obstacles)[i];
        
        obstacle->center = candidate->center;
        
        // Calculate radius as half the diagonal of bounding box
        double width = candidate->max_x - candidate->min_x;
        double height = candidate->max_y - candidate->min_y;
        obstacle->radius = sqrt(width * width + height * height) / 2.0;
        
        // Add some padding for safety
        obstacle->radius += 0.5;
        
        obstacle->id = i + 1;
        obstacle->is_permanent = true; // SLAM obstacles are considered permanent
        obstacle->timestamp = slam_map->timestamp;
    }
    
    free(candidates);
    free(visited);
    
    printf("Extracted %u obstacles from SLAM map\n", candidate_count);
    return candidate_count;
}

/**
 * Validate waypoint using SLAM map data
 */
bool validate_waypoint_slam(const Waypoint* waypoint, const SlamMap* slam_map,
                           const PathPlanningConfig* config) {
    if (!waypoint || !config) {
        return false;
    }
    
    // If SLAM is disabled, always validate
    if (!config->use_slam_data || !slam_map) {
        return true;
    }
    
    // Check if waypoint position is safe according to SLAM map
    return is_point_safe_slam(slam_map, &waypoint->position, config->obstacle_buffer);
}

/**
 * Optimize path using SLAM map data for better obstacle avoidance
 */
bool optimize_path_with_slam(PathPlan* plan, const SlamMap* slam_map,
                            const PathPlanningConfig* config) {
    if (!plan || !config || !config->use_slam_data || !slam_map) {
        return true; // No optimization needed/possible
    }
    
    printf("Optimizing path with SLAM map data\n");
    
    uint32_t optimized_waypoints = 0;
    uint32_t removed_waypoints = 0;
    
    // Check each waypoint and optimize if needed
    for (uint32_t i = 0; i < plan->waypoint_count; i++) {
        Waypoint* wp = &plan->waypoints[i];
        
        if (!validate_waypoint_slam(wp, slam_map, config)) {
            // Try to find a nearby safe position
            bool found_safe = false;
            double search_radius = 2.0; // Start with 2m search radius
            
            for (int attempts = 0; attempts < 3 && !found_safe; attempts++) {
                // Try positions in a circle around the original waypoint
                for (double angle = 0; angle < 2 * M_PI; angle += M_PI/8) {
                    Point2D test_point = {
                        wp->position.x + search_radius * cos(angle),
                        wp->position.y + search_radius * sin(angle)
                    };
                    
                    if (is_point_safe_slam(slam_map, &test_point, config->obstacle_buffer)) {
                        wp->position = test_point;
                        found_safe = true;
                        optimized_waypoints++;
                        break;
                    }
                }
                search_radius += 1.0; // Expand search radius
            }
            
            if (!found_safe) {
                // Remove unsafe waypoint
                for (uint32_t j = i; j < plan->waypoint_count - 1; j++) {
                    plan->waypoints[j] = plan->waypoints[j + 1];
                }
                plan->waypoint_count--;
                removed_waypoints++;
                i--; // Recheck this index
            }
        }
    }
    
    printf("SLAM optimization complete: %u waypoints optimized, %u removed\n",
           optimized_waypoints, removed_waypoints);
    
    return true;
}

/**
 * Update path planning configuration with SLAM-specific settings
 */
void configure_slam_integration(PathPlanningConfig* config, bool enable_slam,
                               double confidence_threshold) {
    if (!config) return;
    
    config->use_slam_data = enable_slam;
    config->slam_confidence_threshold = confidence_threshold;
    
    if (enable_slam) {
        // Adjust other parameters for SLAM integration
        config->obstacle_buffer += 0.2; // Extra safety margin with SLAM
        printf("SLAM integration enabled (confidence threshold: %.2f)\n", confidence_threshold);
    } else {
        printf("SLAM integration disabled\n");
    }
}

/**
 * Get map coverage statistics for path planning optimization
 */
typedef struct {
    double mapped_area_percentage;
    double obstacle_density;
    double unknown_area_percentage;
    uint32_t total_obstacles;
    bool map_complete;
} SlamMapStats;

SlamMapStats analyze_slam_map(const SlamMap* slam_map) {
    SlamMapStats stats = {0};
    
    if (!slam_map || !slam_map->cells) {
        return stats;
    }
    
    uint32_t free_cells = 0;
    uint32_t occupied_cells = 0;
    uint32_t unknown_cells = 0;
    uint32_t obstacle_cells = 0;
    
    for (uint32_t i = 0; i < slam_map->width * slam_map->height; i++) {
        SlamMapCell* cell = &slam_map->cells[i];
        
        if (cell->occupancy < 50) {
            free_cells++;
        } else if (cell->occupancy > 200) {
            occupied_cells++;
        } else {
            unknown_cells++;
        }
        
        if (cell->is_obstacle) {
            obstacle_cells++;
        }
    }
    
    uint32_t total_cells = slam_map->width * slam_map->height;
    
    stats.mapped_area_percentage = ((double)(free_cells + occupied_cells) / total_cells) * 100.0;
    stats.unknown_area_percentage = ((double)unknown_cells / total_cells) * 100.0;
    stats.obstacle_density = ((double)obstacle_cells / total_cells) * 100.0;
    stats.total_obstacles = obstacle_cells;
    stats.map_complete = (stats.unknown_area_percentage < 10.0); // <10% unknown = complete
    
    printf("SLAM Map Analysis:\n");
    printf("- Mapped area: %.1f%%\n", stats.mapped_area_percentage);
    printf("- Unknown area: %.1f%%\n", stats.unknown_area_percentage);
    printf("- Obstacle density: %.1f%%\n", stats.obstacle_density);
    printf("- Map complete: %s\n", stats.map_complete ? "Yes" : "No");
    
    return stats;
}
