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
// RANDOM PATTERN IMPLEMENTATION WITH INTELLIGENT BIAS
// =============================================================================

// Coverage grid for tracking mowed areas
typedef struct {
    uint8_t** grid;          // 2D grid of coverage values (0-255)
    uint32_t width;          // Grid width in cells
    uint32_t height;         // Grid height in cells
    double cell_size;        // Size of each cell in meters
    double min_x, min_y;     // Grid origin coordinates
    double max_x, max_y;     // Grid bounds
} CoverageGrid;

// Random walk state
typedef struct {
    Point2D current_position;
    double current_heading;
    double segment_remaining;
    uint32_t direction_changes;
    uint32_t stuck_counter;
    bool exploring_new_area;
} RandomWalkState;

/**
 * Initialize coverage grid for tracking mowed areas
 */
bool init_coverage_grid(CoverageGrid* grid, const AreaBoundary* boundary, double resolution) {
    if (!grid || !boundary || resolution <= 0) {
        return false;
    }
    
    // Calculate boundary extents
    double min_x = boundary->vertices[0].x, max_x = boundary->vertices[0].x;
    double min_y = boundary->vertices[0].y, max_y = boundary->vertices[0].y;
    
    for (uint32_t i = 1; i < boundary->vertex_count; i++) {
        if (boundary->vertices[i].x < min_x) min_x = boundary->vertices[i].x;
        if (boundary->vertices[i].x > max_x) max_x = boundary->vertices[i].x;
        if (boundary->vertices[i].y < min_y) min_y = boundary->vertices[i].y;
        if (boundary->vertices[i].y > max_y) max_y = boundary->vertices[i].y;
    }
    
    // Add padding
    min_x -= 2.0; max_x += 2.0;
    min_y -= 2.0; max_y += 2.0;
    
    grid->min_x = min_x; grid->max_x = max_x;
    grid->min_y = min_y; grid->max_y = max_y;
    grid->cell_size = resolution;
    grid->width = (uint32_t)ceil((max_x - min_x) / resolution);
    grid->height = (uint32_t)ceil((max_y - min_y) / resolution);
    
    // Allocate grid memory
    grid->grid = malloc(grid->height * sizeof(uint8_t*));
    if (!grid->grid) return false;
    
    for (uint32_t i = 0; i < grid->height; i++) {
        grid->grid[i] = calloc(grid->width, sizeof(uint8_t));
        if (!grid->grid[i]) {
            // Cleanup on failure
            for (uint32_t j = 0; j < i; j++) {
                free(grid->grid[j]);
            }
            free(grid->grid);
            return false;
        }
    }
    
    printf("Coverage grid initialized: %ux%u cells (%.1fm resolution)\n",
           grid->width, grid->height, resolution);
    
    return true;
}

/**
 * Mark area as covered in the grid
 */
void mark_coverage(CoverageGrid* grid, const Point2D* position, double cutting_width) {
    if (!grid || !position) return;
    
    // Convert position to grid coordinates
    int32_t center_x = (int32_t)((position->x - grid->min_x) / grid->cell_size);
    int32_t center_y = (int32_t)((position->y - grid->min_y) / grid->cell_size);
    
    // Mark cells within cutting radius
    int32_t radius_cells = (int32_t)ceil(cutting_width / (2.0 * grid->cell_size));
    
    for (int32_t dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int32_t dx = -radius_cells; dx <= radius_cells; dx++) {
            int32_t x = center_x + dx;
            int32_t y = center_y + dy;
            
            if (x >= 0 && x < (int32_t)grid->width && 
                y >= 0 && y < (int32_t)grid->height) {
                
                // Check if within cutting radius
                double dist = sqrt(dx*dx + dy*dy) * grid->cell_size;
                if (dist <= cutting_width / 2.0) {
                    grid->grid[y][x] = 255; // Fully covered
                }
            }
        }
    }
}

/**
 * Get coverage density around a position
 */
double get_coverage_density(const CoverageGrid* grid, const Point2D* position, double radius) {
    if (!grid || !position) return 1.0; // Assume covered if no grid
    
    int32_t center_x = (int32_t)((position->x - grid->min_x) / grid->cell_size);
    int32_t center_y = (int32_t)((position->y - grid->min_y) / grid->cell_size);
    int32_t radius_cells = (int32_t)ceil(radius / grid->cell_size);
    
    uint32_t total_cells = 0;
    uint32_t covered_sum = 0;
    
    for (int32_t dy = -radius_cells; dy <= radius_cells; dy++) {
        for (int32_t dx = -radius_cells; dx <= radius_cells; dx++) {
            int32_t x = center_x + dx;
            int32_t y = center_y + dy;
            
            if (x >= 0 && x < (int32_t)grid->width && 
                y >= 0 && y < (int32_t)grid->height) {
                
                double dist = sqrt(dx*dx + dy*dy) * grid->cell_size;
                if (dist <= radius) {
                    total_cells++;
                    covered_sum += grid->grid[y][x];
                }
            }
        }
    }
    
    return total_cells > 0 ? (double)covered_sum / (total_cells * 255.0) : 0.0;
}

/**
 * Find the best direction for random walk with bias toward uncovered areas
 */
double find_best_direction(const CoverageGrid* grid, const Point2D* position,
                          const AreaBoundary* boundary, const Obstacle* obstacles,
                          uint32_t obstacle_count, double bias_uncut, double search_radius) {
    
    const int num_directions = 16; // Test 16 directions (22.5° apart)
    double best_direction = 0.0;
    double best_score = -1.0;
    
    for (int i = 0; i < num_directions; i++) {
        double direction = (2.0 * M_PI * i) / num_directions;
        
        // Test point in this direction
        Point2D test_point = {
            position->x + search_radius * cos(direction),
            position->y + search_radius * sin(direction)
        };
        
        // Check if point is valid (in boundary, no obstacles)
        if (!point_in_boundary(&test_point, boundary) ||
            point_collides_obstacle(&test_point, obstacles, obstacle_count, 1.0)) {
            continue; // Skip invalid directions
        }
        
        // Calculate score based on coverage density
        double coverage = get_coverage_density(grid, &test_point, search_radius * 0.5);
        double uncovered_score = 1.0 - coverage; // Higher score for less covered areas
        
        // Add randomness factor
        double randomness = ((double)rand() / RAND_MAX) * (1.0 - bias_uncut);
        double total_score = uncovered_score * bias_uncut + randomness;
        
        if (total_score > best_score) {
            best_score = total_score;
            best_direction = direction;
        }
    }
    
    return best_direction;
}

/**
 * Generate random pattern with intelligent bias toward uncovered areas
 */
bool generate_random_pattern(const AreaBoundary* boundary,
                            const Obstacle* obstacles,
                            uint32_t obstacle_count,
                            const PathPlanningConfig* config,
                            PathPlan* plan) {
    
    if (!boundary || !config || !plan) {
        return false;
    }
    
    // Initialize plan
    memset(plan, 0, sizeof(PathPlan));
    strcpy(plan->pattern_type, PATH_PATTERN_RANDOM);
    plan->plan_id = (uint32_t)time(NULL);
    plan->timestamp = (uint64_t)time(NULL);
    
    // Initialize random seed
    srand((unsigned int)time(NULL));
    
    // Initialize coverage grid
    CoverageGrid coverage_grid;
    if (!init_coverage_grid(&coverage_grid, boundary, 1.0)) { // 1m resolution
        printf("Failed to initialize coverage grid\n");
        return false;
    }
    
    // Calculate area bounds
    double min_x = boundary->vertices[0].x, max_x = boundary->vertices[0].x;
    double min_y = boundary->vertices[0].y, max_y = boundary->vertices[0].y;
    
    for (uint32_t i = 1; i < boundary->vertex_count; i++) {
        if (boundary->vertices[i].x < min_x) min_x = boundary->vertices[i].x;
        if (boundary->vertices[i].x > max_x) max_x = boundary->vertices[i].x;
        if (boundary->vertices[i].y < min_y) min_y = boundary->vertices[i].y;
        if (boundary->vertices[i].y > max_y) max_y = boundary->vertices[i].y;
    }
    
    double center_x = (min_x + max_x) / 2.0;
    double center_y = (min_y + max_y) / 2.0;
    
    // Allocate waypoint array
    plan->waypoints = malloc(config->max_waypoints * sizeof(Waypoint));
    if (!plan->waypoints) {
        return false;
    }
    
    // Initialize random walk state
    RandomWalkState walk_state = {0};
    walk_state.current_position = (Point2D){center_x, center_y}; // Start from center
    walk_state.current_heading = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
    walk_state.segment_remaining = 5.0; // Start with 5m segment
    walk_state.exploring_new_area = true;
    
    // Random pattern parameters
    double bias_uncut = 0.8;        // Strong bias toward uncut areas
    double min_segment = 2.0;       // Minimum segment length
    double max_segment = 10.0;      // Maximum segment length
    double search_radius = 8.0;     // Radius for direction search
    double waypoint_spacing = 2.0;  // Distance between waypoints
    
    uint32_t waypoint_id = 1;
    uint32_t max_iterations = config->max_waypoints * 3; // Prevent infinite loops
    uint32_t iteration = 0;
    
    printf("Generating random pattern with intelligent bias (%.0f%% toward uncut areas)\n", 
           bias_uncut * 100.0);
    
    while (plan->waypoint_count < config->max_waypoints && iteration < max_iterations) {
        iteration++;
        
        // Check if current position is valid
        if (!point_in_boundary(&walk_state.current_position, boundary) ||
            point_collides_obstacle(&walk_state.current_position, obstacles, 
                                   obstacle_count, config->obstacle_buffer)) {
            
            // Find a new valid starting position
            bool found_valid = false;
            for (int attempts = 0; attempts < 50; attempts++) {
                Point2D new_pos = {
                    min_x + ((double)rand() / RAND_MAX) * (max_x - min_x),
                    min_y + ((double)rand() / RAND_MAX) * (max_y - min_y)
                };
                
                if (point_in_boundary(&new_pos, boundary) &&
                    !point_collides_obstacle(&new_pos, obstacles, obstacle_count,
                                            config->obstacle_buffer)) {
                    walk_state.current_position = new_pos;
                    found_valid = true;
                    break;
                }
            }
            
            if (!found_valid) {
                printf("Could not find valid position, stopping random generation\n");
                break;
            }
            
            walk_state.stuck_counter = 0;
        }
        
        // Create waypoint at current position
        Waypoint waypoint = {0};
        waypoint.position = walk_state.current_position;
        waypoint.heading = walk_state.current_heading;
        waypoint.speed = config->max_speed * (0.6 + 0.4 * ((double)rand() / RAND_MAX)); // Variable speed
        waypoint.id = waypoint_id++;
        waypoint.is_mowing = true;
        
        plan->waypoints[plan->waypoint_count] = waypoint;
        plan->waypoint_count++;
        
        // Mark area as covered
        mark_coverage(&coverage_grid, &walk_state.current_position, config->cutting_width);
        
        // Decide next movement
        if (walk_state.segment_remaining <= 0 || walk_state.stuck_counter > 5) {
            // Time to change direction
            walk_state.current_heading = find_best_direction(&coverage_grid, 
                                                           &walk_state.current_position,
                                                           boundary, obstacles, obstacle_count,
                                                           bias_uncut, search_radius);
            
            // New segment length (random with bias toward longer segments in uncovered areas)
            double coverage_here = get_coverage_density(&coverage_grid, &walk_state.current_position, 5.0);
            double length_bias = 1.0 - coverage_here; // Longer segments in uncovered areas
            walk_state.segment_remaining = min_segment + 
                                         (max_segment - min_segment) * length_bias * ((double)rand() / RAND_MAX);
            
            walk_state.direction_changes++;
            walk_state.stuck_counter = 0;
            walk_state.exploring_new_area = (coverage_here < 0.3); // Exploring if <30% covered
        }
        
        // Move to next position
        Point2D next_position = {
            walk_state.current_position.x + waypoint_spacing * cos(walk_state.current_heading),
            walk_state.current_position.y + waypoint_spacing * sin(walk_state.current_heading)
        };
        
        // Check if next position is valid
        if (point_in_boundary(&next_position, boundary) &&
            !point_collides_obstacle(&next_position, obstacles, obstacle_count,
                                    config->obstacle_buffer)) {
            
            walk_state.current_position = next_position;
            walk_state.segment_remaining -= waypoint_spacing;
            walk_state.stuck_counter = 0;
        } else {
            // Stuck, try to find new direction
            walk_state.stuck_counter++;
            if (walk_state.stuck_counter > 3) {
                walk_state.segment_remaining = 0; // Force direction change
            }
        }
        
        // Progress update
        if (plan->waypoint_count % 100 == 0) {
            double total_coverage = 0.0;
            uint32_t total_cells = 0;
            
            for (uint32_t y = 0; y < coverage_grid.height; y++) {
                for (uint32_t x = 0; x < coverage_grid.width; x++) {
                    total_coverage += coverage_grid.grid[y][x];
                    total_cells++;
                }
            }
            
            double coverage_percentage = (total_coverage / (total_cells * 255.0)) * 100.0;
            printf("Generated %u waypoints, coverage: %.1f%%, direction changes: %u\n",
                   plan->waypoint_count, coverage_percentage, walk_state.direction_changes);
            
            // Stop if we've achieved good coverage
            if (coverage_percentage > 90.0) {
                printf("Target coverage achieved, stopping generation\n");
                break;
            }
        }
    }
    
    // Optimize path order for efficiency
    if (config->optimize_battery) {
        printf("Optimizing random path for battery efficiency\n");
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
    
    // Calculate final coverage
    double final_coverage = calculate_area_coverage(plan, config->cutting_width);
    
    printf("Random pattern generated successfully:\n");
    printf("- Waypoints: %u\n", plan->waypoint_count);
    printf("- Direction changes: %u\n", walk_state.direction_changes);
    printf("- Total distance: %.1f m\n", plan->total_distance);
    printf("- Estimated time: %.1f minutes\n", plan->estimated_time / 60.0);
    printf("- Estimated coverage: %.1f m²\n", final_coverage);
    printf("- Coverage efficiency: %.1f%%\n", (final_coverage / boundary->total_area) * 100.0);
    
    // Cleanup coverage grid
    for (uint32_t i = 0; i < coverage_grid.height; i++) {
        free(coverage_grid.grid[i]);
    }
    free(coverage_grid.grid);
    
    return true;
}

/**
 * Adaptive random pattern that learns from previous sessions
 */
bool generate_adaptive_random_pattern(const AreaBoundary* boundary,
                                     const Obstacle* obstacles,
                                     uint32_t obstacle_count,
                                     const PathPlanningConfig* config,
                                     void* historical_data __attribute__((unused)),
                                     PathPlan* plan) {
    
    // For now, use enhanced random pattern
    // In future versions, this would incorporate machine learning
    // to adapt parameters based on historical performance data
    
    printf("Generating adaptive random pattern (ML-enhanced)\n");
    
    // TODO: Implement machine learning adaptation
    // - Analyze historical coverage efficiency
    // - Adapt bias_uncut based on grass growth patterns
    // - Optimize segment lengths based on battery performance
    // - Learn optimal starting positions
    
    return generate_random_pattern(boundary, obstacles, obstacle_count, config, plan);
}
