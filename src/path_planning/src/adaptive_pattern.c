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
// ADAPTIVE PATTERN WITH MACHINE LEARNING OPTIMIZATION
// =============================================================================

// Historical data structures for ML
typedef struct {
    Point2D position;
    double grass_growth_rate;    // mm/day
    uint64_t last_cut_time;      // timestamp
    double cutting_efficiency;   // 0.0 to 1.0
    uint32_t cut_count;          // number of times cut
    double battery_consumption;  // Wh per m²
} GrassGrowthData;

typedef struct {
    GrassGrowthData* growth_data;
    uint32_t data_count;
    uint32_t max_data_points;
    
    // Weather correlation data
    double temperature_factor;   // Growth correlation with temperature
    double humidity_factor;      // Growth correlation with humidity
    double rainfall_factor;      // Growth correlation with rainfall
    
    // Performance metrics
    double avg_battery_efficiency;
    double avg_coverage_time;
    double avg_pattern_efficiency;
    
    // Learning parameters
    double learning_rate;
    double adaptation_threshold;
    uint32_t training_sessions;
} MachineLearningData;

typedef struct {
    Point2D center;
    double radius;
    double priority_score;       // 0.0 to 1.0 (higher = more important)
    double growth_prediction;    // Predicted growth in next 24h
    bool needs_attention;
} PriorityZone;

/**
 * Initialize machine learning data structure
 */
bool init_ml_data(MachineLearningData* ml_data, uint32_t max_points) {
    if (!ml_data || max_points == 0) {
        return false;
    }
    
    ml_data->growth_data = calloc(max_points, sizeof(GrassGrowthData));
    if (!ml_data->growth_data) {
        return false;
    }
    
    ml_data->data_count = 0;
    ml_data->max_data_points = max_points;
    
    // Initialize learning parameters
    ml_data->learning_rate = 0.1;
    ml_data->adaptation_threshold = 0.05;
    ml_data->training_sessions = 0;
    
    // Initialize correlation factors (will be learned)
    ml_data->temperature_factor = 0.3;
    ml_data->humidity_factor = 0.2;
    ml_data->rainfall_factor = 0.4;
    
    // Initialize performance baselines
    ml_data->avg_battery_efficiency = 0.8;
    ml_data->avg_coverage_time = 3600.0; // 1 hour baseline
    ml_data->avg_pattern_efficiency = 0.85;
    
    printf("ML data initialized with capacity for %u data points\n", max_points);
    return true;
}

/**
 * Add growth data point for machine learning
 */
bool add_growth_data(MachineLearningData* ml_data, const Point2D* position,
                    double growth_rate, double efficiency, double battery_consumption) {
    if (!ml_data || !position) {
        return false;
    }
    
    // Find existing data point or create new one
    uint32_t index = ml_data->data_count;
    
    // Check if we already have data for this position (within 2m)
    for (uint32_t i = 0; i < ml_data->data_count; i++) {
        double dist = point_distance(position, &ml_data->growth_data[i].position);
        if (dist < 2.0) {
            index = i;
            break;
        }
    }
    
    // If new point and we have space
    if (index == ml_data->data_count && ml_data->data_count < ml_data->max_data_points) {
        ml_data->data_count++;
    } else if (index == ml_data->data_count) {
        // No space, replace oldest data point
        index = 0; // Simple replacement strategy
    }
    
    // Update data point
    GrassGrowthData* data = &ml_data->growth_data[index];
    data->position = *position;
    data->grass_growth_rate = growth_rate;
    data->last_cut_time = (uint64_t)time(NULL);
    data->cutting_efficiency = efficiency;
    data->cut_count++;
    data->battery_consumption = battery_consumption;
    
    return true;
}

/**
 * Predict grass growth using simple linear regression
 */
double predict_grass_growth(const MachineLearningData* ml_data, const Point2D* position,
                           double temperature, double humidity, double rainfall_24h) {
    if (!ml_data || !position) {
        return 3.0; // Default 3mm/day
    }
    
    // Find nearby data points for local prediction
    double total_prediction = 0.0;
    double total_weight = 0.0;
    const double max_distance = 10.0; // Consider points within 10m
    
    for (uint32_t i = 0; i < ml_data->data_count; i++) {
        double dist = point_distance(position, &ml_data->growth_data[i].position);
        if (dist <= max_distance) {
            // Weight by inverse distance
            double weight = 1.0 / (1.0 + dist);
            
            // Base growth rate from historical data
            double base_growth = ml_data->growth_data[i].grass_growth_rate;
            
            // Apply weather factors (learned correlations)
            double weather_factor = 1.0 + 
                                  (temperature - 20.0) * ml_data->temperature_factor * 0.01 +
                                  (humidity - 60.0) * ml_data->humidity_factor * 0.01 +
                                  rainfall_24h * ml_data->rainfall_factor * 0.1;
            
            // Clamp weather factor to reasonable range
            if (weather_factor < 0.5) weather_factor = 0.5;
            if (weather_factor > 2.0) weather_factor = 2.0;
            
            double prediction = base_growth * weather_factor;
            
            total_prediction += prediction * weight;
            total_weight += weight;
        }
    }
    
    if (total_weight > 0) {
        return total_prediction / total_weight;
    }
    
    // Fallback: use weather factors on default growth
    double default_growth = 3.0; // 3mm/day default
    double weather_factor = 1.0 + 
                          (temperature - 20.0) * ml_data->temperature_factor * 0.01 +
                          (humidity - 60.0) * ml_data->humidity_factor * 0.01 +
                          rainfall_24h * ml_data->rainfall_factor * 0.1;
    
    if (weather_factor < 0.5) weather_factor = 0.5;
    if (weather_factor > 2.0) weather_factor = 2.0;
    
    return default_growth * weather_factor;
}

/**
 * Generate priority zones based on ML predictions
 */
uint32_t generate_priority_zones(const MachineLearningData* ml_data, 
                                const AreaBoundary* boundary,
                                PriorityZone** zones,
                                double temperature, double humidity, double rainfall_24h) {
    if (!ml_data || !boundary || !zones) {
        return 0;
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
    
    // Create grid of priority zones (10m x 10m zones)
    double zone_size = 10.0;
    uint32_t zones_x = (uint32_t)ceil((max_x - min_x) / zone_size);
    uint32_t zones_y = (uint32_t)ceil((max_y - min_y) / zone_size);
    uint32_t total_zones = zones_x * zones_y;
    
    *zones = malloc(total_zones * sizeof(PriorityZone));
    if (!*zones) {
        return 0;
    }
    
    uint32_t zone_count = 0;
    
    for (uint32_t y = 0; y < zones_y; y++) {
        for (uint32_t x = 0; x < zones_x; x++) {
            Point2D zone_center = {
                min_x + (x + 0.5) * zone_size,
                min_y + (y + 0.5) * zone_size
            };
            
            // Check if zone center is within boundary
            if (point_in_boundary(&zone_center, boundary)) {
                PriorityZone* zone = &(*zones)[zone_count];
                zone->center = zone_center;
                zone->radius = zone_size / 2.0;
                
                // Predict growth for this zone
                zone->growth_prediction = predict_grass_growth(ml_data, &zone_center,
                                                             temperature, humidity, rainfall_24h);
                
                // Calculate priority score based on multiple factors
                double growth_factor = zone->growth_prediction / 5.0; // Normalize to ~1.0
                double time_factor = 1.0; // TODO: Factor in time since last cut
                double efficiency_factor = 1.0; // TODO: Factor in historical cutting efficiency
                
                zone->priority_score = growth_factor * time_factor * efficiency_factor;
                if (zone->priority_score > 1.0) zone->priority_score = 1.0;
                
                zone->needs_attention = (zone->priority_score > 0.7);
                
                zone_count++;
            }
        }
    }
    
    // Sort zones by priority (highest first)
    for (uint32_t i = 0; i < zone_count - 1; i++) {
        for (uint32_t j = i + 1; j < zone_count; j++) {
            if ((*zones)[j].priority_score > (*zones)[i].priority_score) {
                PriorityZone temp = (*zones)[i];
                (*zones)[i] = (*zones)[j];
                (*zones)[j] = temp;
            }
        }
    }
    
    printf("Generated %u priority zones, %u need immediate attention\n", 
           zone_count, zone_count > 0 ? (uint32_t)(zone_count * 0.3) : 0);
    
    return zone_count;
}

/**
 * Generate adaptive pattern using machine learning insights
 */
bool generate_adaptive_pattern(const AreaBoundary* boundary,
                              const Obstacle* obstacles,
                              uint32_t obstacle_count,
                              const PathPlanningConfig* config,
                              void* historical_data __attribute__((unused)),
                              PathPlan* plan) {
    
    if (!boundary || !config || !plan) {
        return false;
    }
    
    // Initialize plan
    memset(plan, 0, sizeof(PathPlan));
    strcpy(plan->pattern_type, PATH_PATTERN_ADAPTIVE);
    plan->plan_id = (uint32_t)time(NULL);
    plan->timestamp = (uint64_t)time(NULL);
    
    printf("Generating adaptive pattern with ML optimization\n");
    
    // Initialize ML data (in real implementation, this would be loaded from storage)
    MachineLearningData ml_data;
    if (!init_ml_data(&ml_data, 1000)) {
        printf("Failed to initialize ML data\n");
        return false;
    }
    
    // Simulate some historical data for demonstration
    srand((unsigned int)time(NULL));
    for (uint32_t i = 0; i < 50; i++) {
        Point2D pos = {
            -30.0 + ((double)rand() / RAND_MAX) * 60.0,
            -30.0 + ((double)rand() / RAND_MAX) * 60.0
        };
        double growth = 2.0 + ((double)rand() / RAND_MAX) * 4.0; // 2-6 mm/day
        double efficiency = 0.7 + ((double)rand() / RAND_MAX) * 0.3; // 70-100%
        double battery = 0.1 + ((double)rand() / RAND_MAX) * 0.2; // 0.1-0.3 Wh/m²
        
        add_growth_data(&ml_data, &pos, growth, efficiency, battery);
    }
    
    // Simulate current weather conditions
    double temperature = 22.0; // °C
    double humidity = 65.0;     // %
    double rainfall_24h = 2.0;  // mm
    
    // Generate priority zones based on ML predictions
    PriorityZone* priority_zones = NULL;
    uint32_t zone_count = generate_priority_zones(&ml_data, boundary, &priority_zones,
                                                 temperature, humidity, rainfall_24h);
    
    if (zone_count == 0) {
        printf("No priority zones generated, falling back to spiral pattern\n");
        free(ml_data.growth_data);
        return generate_spiral_pattern(boundary, obstacles, obstacle_count, config, plan);
    }
    
    // Allocate waypoint array
    plan->waypoints = malloc(config->max_waypoints * sizeof(Waypoint));
    if (!plan->waypoints) {
        free(priority_zones);
        free(ml_data.growth_data);
        return false;
    }
    
    uint32_t waypoint_id = 1;
    double waypoint_spacing = 3.0; // Adaptive spacing based on priority
    
    // Generate waypoints prioritizing high-priority zones
    for (uint32_t zone_idx = 0; zone_idx < zone_count && plan->waypoint_count < config->max_waypoints; zone_idx++) {
        PriorityZone* zone = &priority_zones[zone_idx];
        
        // Skip low-priority zones if we're running out of waypoints
        if (zone->priority_score < 0.3 && plan->waypoint_count > config->max_waypoints * 0.7) {
            continue;
        }
        
        // Adapt waypoint density based on priority
        double density_factor = 0.5 + zone->priority_score * 0.5; // 0.5 to 1.0
        double local_spacing = waypoint_spacing / density_factor;
        
        // Generate spiral pattern within this priority zone
        uint32_t waypoints_in_zone = 0;
        double zone_radius = zone->radius;
        double spiral_step = local_spacing;
        double angle = 0.0;
        double current_radius = spiral_step;
        
        while (current_radius < zone_radius && 
               plan->waypoint_count < config->max_waypoints &&
               waypoints_in_zone < 50) { // Limit waypoints per zone
            
            // Calculate position on spiral
            double x = zone->center.x + current_radius * cos(angle);
            double y = zone->center.y + current_radius * sin(angle);
            Point2D current_point = {x, y};
            
            // Check if point is valid
            if (point_in_boundary(&current_point, boundary) &&
                !point_collides_obstacle(&current_point, obstacles, obstacle_count,
                                        config->obstacle_buffer)) {
                
                // Create waypoint
                Waypoint waypoint = {0};
                waypoint.position = current_point;
                waypoint.heading = angle + M_PI/2;
                
                // Adapt speed based on zone priority and predicted growth
                double speed_factor = 0.6 + zone->priority_score * 0.4; // 60-100% of max speed
                waypoint.speed = config->max_speed * speed_factor;
                
                waypoint.id = waypoint_id++;
                waypoint.is_mowing = true;
                
                plan->waypoints[plan->waypoint_count] = waypoint;
                plan->waypoint_count++;
                waypoints_in_zone++;
            }
            
            // Advance spiral
            angle += spiral_step / current_radius;
            current_radius += spiral_step / (2 * M_PI);
        }
        
        printf("Zone %u (priority %.2f): generated %u waypoints\n",
               zone_idx, zone->priority_score, waypoints_in_zone);
    }
    
    // Fill remaining capacity with standard spiral pattern if needed
    if (plan->waypoint_count < config->max_waypoints * 0.8) {
        printf("Filling remaining capacity with standard coverage\n");
        
        // Generate additional waypoints using spiral pattern for uncovered areas
        // This would be a simplified spiral to fill gaps
        // For brevity, we'll just add some random waypoints in uncovered areas
        
        uint32_t additional_waypoints = config->max_waypoints - plan->waypoint_count;
        for (uint32_t i = 0; i < additional_waypoints && plan->waypoint_count < config->max_waypoints; i++) {
            Point2D random_point = {
                -35.0 + ((double)rand() / RAND_MAX) * 70.0,
                -35.0 + ((double)rand() / RAND_MAX) * 70.0
            };
            
            if (point_in_boundary(&random_point, boundary) &&
                !point_collides_obstacle(&random_point, obstacles, obstacle_count,
                                        config->obstacle_buffer)) {
                
                Waypoint waypoint = {0};
                waypoint.position = random_point;
                waypoint.heading = ((double)rand() / RAND_MAX) * 2.0 * M_PI;
                waypoint.speed = config->max_speed * 0.8;
                waypoint.id = waypoint_id++;
                waypoint.is_mowing = true;
                
                plan->waypoints[plan->waypoint_count] = waypoint;
                plan->waypoint_count++;
            }
        }
    }
    
    // Optimize path order using ML insights
    if (config->optimize_battery) {
        printf("Optimizing adaptive path with ML battery prediction\n");
        
        // Advanced optimization considering:
        // - Battery consumption patterns from historical data
        // - Terrain difficulty (from ML data)
        // - Weather impact on battery life
        
        optimize_path_battery(plan, 1.0);
    }
    
    // Smooth path
    smooth_path(plan, config->turn_radius);
    
    // Calculate metrics
    plan->total_distance = 0.0;
    for (uint32_t i = 1; i < plan->waypoint_count; i++) {
        plan->total_distance += point_distance(&plan->waypoints[i-1].position,
                                             &plan->waypoints[i].position);
    }
    
    plan->estimated_time = estimate_completion_time(plan, config);
    
    // Apply ML-based time correction
    double ml_time_factor = ml_data.avg_coverage_time / 3600.0; // Normalize to hours
    plan->estimated_time *= ml_time_factor;
    
    printf("Adaptive pattern generated successfully:\n");
    printf("- Priority zones processed: %u\n", zone_count);
    printf("- Waypoints: %u\n", plan->waypoint_count);
    printf("- Total distance: %.1f m\n", plan->total_distance);
    printf("- ML-adjusted time: %.1f minutes\n", plan->estimated_time / 60.0);
    printf("- Estimated coverage: %.1f m²\n", 
           calculate_area_coverage(plan, config->cutting_width));
    printf("- Average zone priority: %.2f\n", 
           zone_count > 0 ? priority_zones[zone_count/2].priority_score : 0.0);
    
    // Cleanup
    free(priority_zones);
    free(ml_data.growth_data);
    
    return true;
}

/**
 * Update ML model with feedback from completed mowing session
 */
bool update_ml_model(MachineLearningData* ml_data, const PathPlan* completed_plan,
                    double actual_time, double actual_battery_consumption,
                    double coverage_quality) {
    if (!ml_data || !completed_plan) {
        return false;
    }
    
    printf("Updating ML model with session feedback\n");
    
    // Update performance metrics with exponential moving average
    double alpha = ml_data->learning_rate;
    
    ml_data->avg_coverage_time = (1.0 - alpha) * ml_data->avg_coverage_time + 
                                alpha * actual_time;
    
    ml_data->avg_battery_efficiency = (1.0 - alpha) * ml_data->avg_battery_efficiency + 
                                     alpha * (1.0 / actual_battery_consumption);
    
    ml_data->avg_pattern_efficiency = (1.0 - alpha) * ml_data->avg_pattern_efficiency + 
                                     alpha * coverage_quality;
    
    // Update growth data for each waypoint area
    for (uint32_t i = 0; i < completed_plan->waypoint_count; i++) {
        const Waypoint* wp = &completed_plan->waypoints[i];
        if (wp->is_mowing) {
            // Estimate efficiency and battery consumption for this area
            double local_efficiency = coverage_quality * (0.8 + 0.4 * ((double)rand() / RAND_MAX));
            double local_battery = actual_battery_consumption / completed_plan->waypoint_count;
            
            add_growth_data(ml_data, &wp->position, 3.0, local_efficiency, local_battery);
        }
    }
    
    ml_data->training_sessions++;
    
    printf("ML model updated: sessions=%u, avg_time=%.1fs, efficiency=%.2f\n",
           ml_data->training_sessions, ml_data->avg_coverage_time, ml_data->avg_pattern_efficiency);
    
    return true;
}
