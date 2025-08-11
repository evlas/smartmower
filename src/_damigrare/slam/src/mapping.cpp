// mapping.cpp - Mapping functionality for SLAM

#include "mapping.h"
#include "slam_types.h"
#include <math.h>
#include <mutex>
#include <fstream>
#include <stdexcept>

// Initialize the occupancy grid map
void mapping_init(occupancy_map_t* map) {
    if (!map) return;
    
    map->width = MAP_WIDTH;
    map->height = MAP_HEIGHT;
    map->resolution = CELL_SIZE;
    map->origin_x = -MAP_WIDTH * CELL_SIZE / 2.0f;
    map->origin_y = -MAP_HEIGHT * CELL_SIZE / 2.0f;
    
    // Initialize map cells
    map->cells = new map_cell_t[MAP_WIDTH * MAP_HEIGHT];
    for (int i = 0; i < MAP_WIDTH * MAP_HEIGHT; i++) {
        map->cells[i].probability = 0.5f;  // Unknown
        map->cells[i].hits = 0;
        map->cells[i].misses = 0;
    }
    
    map->updated = false;
}

// Convert world coordinates to grid coordinates
bool world_to_grid(const occupancy_map_t* map, float x, float y, int* grid_x, int* grid_y) {
    if (!map || !grid_x || !grid_y) return false;
    
    *grid_x = static_cast<int>((x - map->origin_x) / map->resolution);
    *grid_y = static_cast<int>((y - map->origin_y) / map->resolution);
    
    return (*grid_x >= 0 && *grid_x < map->width && 
            *grid_y >= 0 && *grid_y < map->height);
}

// Convert grid coordinates to world coordinates
void grid_to_world(const occupancy_map_t* map, int grid_x, int grid_y, float* x, float* y) {
    if (!map || !x || !y) return;
    
    *x = map->origin_x + (grid_x + 0.5f) * map->resolution;
    *y = map->origin_y + (grid_y + 0.5f) * map->resolution;
}

// Update the map with sonar sensor data
void mapping_update_sonar(occupancy_map_t* map, robot_pose_t* pose, 
                         sensor_data_t* sensors, slam_config_t* config) {
    if (!map || !pose || !sensors || !config) return;
    
    std::lock_guard<std::mutex> lock(map->mutex);
    
    // Use obstacles data from sensor_data_t
    obstacles_data_t& obstacles = sensors->obstacles;
    
    // Process each detected obstacle
    for (int i = 0; i < obstacles.count && i < 20; i++) {
        const obstacle_t& obs = obstacles.obstacles[i];
        
        // Convert obstacle position from robot-relative to world coordinates
        float obs_world_x = pose->x + obs.x * cosf(pose->theta) - obs.y * sinf(pose->theta);
        float obs_world_y = pose->y + obs.x * sinf(pose->theta) + obs.y * cosf(pose->theta);
    
        // Update map with obstacle
        int grid_x, grid_y;
        if (world_to_grid(map, obs_world_x, obs_world_y, &grid_x, &grid_y)) {
            int cell_idx = grid_y * map->width + grid_x;
            if (cell_idx >= 0 && cell_idx < map->width * map->height) {
                map->cells[cell_idx].probability = 0.9f; // Occupied
                map->cells[cell_idx].hits++;
                map->updated = true;
            }
        }
        
        // Mark free space along the line from robot to obstacle
        float distance = sqrtf(obs.x * obs.x + obs.y * obs.y);
        float step = map->resolution / 2.0f;
        for (float r = 0.1f; r < distance; r += step) {
            float ratio = r / distance;
            float free_world_x = pose->x + ratio * (obs_world_x - pose->x);
            float free_world_y = pose->y + ratio * (obs_world_y - pose->y);
            
            if (world_to_grid(map, free_world_x, free_world_y, &grid_x, &grid_y)) {
                int cell_idx = grid_y * map->width + grid_x;
                if (cell_idx >= 0 && cell_idx < map->width * map->height) {
                    if (map->cells[cell_idx].probability > 0.2f) { // Only update if not strongly occupied
                        map->cells[cell_idx].probability = 0.1f; // Free
                        map->cells[cell_idx].misses++;
                        map->updated = true;
                    }
                }
            }
        }
    }
    map->timestamp = sensors->timestamp;
}

// Get the occupancy probability of a cell
float get_occupancy(const occupancy_map_t* map, int x, int y) {
    if (!map || x < 0 || x >= map->width || y < 0 || y >= map->height) {
        return -1.0f; // Invalid cell
    }
    int cell_idx = y * map->width + x;
    return map->cells[cell_idx].probability;
}

// Set the occupancy probability of a cell
void set_occupancy(occupancy_map_t* map, int x, int y, float prob) {
    if (!map || x < 0 || x >= map->width || y < 0 || y >= map->height) {
        return; // Invalid cell
    }
    int cell_idx = y * map->width + x;
    map->cells[cell_idx].probability = std::max(0.0f, std::min(1.0f, prob));
    map->updated = true;
}

// Check if a cell is occupied (probability > threshold)
bool is_occupied(const occupancy_map_t* map, int x, int y, float threshold) {
    return get_occupancy(map, x, y) > threshold;
}

// Check if a cell is free (probability < threshold)
bool is_free(const occupancy_map_t* map, int x, int y, float threshold) {
    return get_occupancy(map, x, y) < threshold;
}

// Check if a cell is unknown (probability == 0.5)
bool is_unknown(const occupancy_map_t* map, int x, int y) {
    float occ = get_occupancy(map, x, y);
    return occ >= 0.45f && occ <= 0.55f; // Allow for floating point imprecision
}

// Save the map to a file
bool save_map(const occupancy_map_t* map, const char* filename) {
    if (!map || !filename) return false;
    
    try {
        std::ofstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        // Write header
        file.write(reinterpret_cast<const char*>(&map->width), sizeof(int));
        file.write(reinterpret_cast<const char*>(&map->height), sizeof(int));
        file.write(reinterpret_cast<const char*>(&map->resolution), sizeof(float));
        file.write(reinterpret_cast<const char*>(&map->origin_x), sizeof(float));
        file.write(reinterpret_cast<const char*>(&map->origin_y), sizeof(float));
        
        // Write grid data
        for (int i = 0; i < map->width * map->height; i++) {
            file.write(reinterpret_cast<const char*>(&map->cells[i].probability), sizeof(float));
            file.write(reinterpret_cast<const char*>(&map->cells[i].hits), sizeof(int8_t));
            file.write(reinterpret_cast<const char*>(&map->cells[i].misses), sizeof(int8_t));
        }
        
        return file.good();
    } catch (...) {
        return false;
    }
}

// Load the map from a file
bool load_map(occupancy_map_t* map, const char* filename) {
    if (!map || !filename) return false;
    
    try {
        std::ifstream file(filename, std::ios::binary);
        if (!file.is_open()) return false;
        
        // Read header
        int width, height;
        file.read(reinterpret_cast<char*>(&width), sizeof(int));
        file.read(reinterpret_cast<char*>(&height), sizeof(int));
        
        if (width != map->width || height != map->height) {
            return false; // Map dimensions don't match
        }
        
        file.read(reinterpret_cast<char*>(&map->resolution), sizeof(float));
        file.read(reinterpret_cast<char*>(&map->origin_x), sizeof(float));
        file.read(reinterpret_cast<char*>(&map->origin_y), sizeof(float));
        
        // Read grid data
        for (int i = 0; i < map->width * map->height; i++) {
            file.read(reinterpret_cast<char*>(&map->cells[i].probability), sizeof(float));
            file.read(reinterpret_cast<char*>(&map->cells[i].hits), sizeof(int8_t));
            file.read(reinterpret_cast<char*>(&map->cells[i].misses), sizeof(int8_t));
        }
        
        return file.good();
    } catch (...) {
        return false;
    }
}
