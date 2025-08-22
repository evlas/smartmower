#include "camera/logging.h"
#include <iomanip>
#include <sstream>
#include <iostream>
#include <filesystem>
#include <chrono>
#include <algorithm>
#include <unordered_map>

namespace camera {

bool Logger::initialize(const std::unordered_map<std::string, std::string>& config) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    try {
        // Check if logging is enabled
        auto it = config.find("enabled");
        enabled_ = (it != config.end()) ? (it->second == "true") : true;
        if (!enabled_) {
            return true; // Logging is explicitly disabled
        }
        
        // Get log level and convert to lowercase
        it = config.find("level");
        log_level_ = (it != config.end()) ? it->second : "info";
        std::transform(log_level_.begin(), log_level_.end(), log_level_.begin(), 
                      [](unsigned char c){ return std::tolower(c); });
        
        // Determine data directory and explicit file overrides (flat map variant)
        // Base data_dir
        it = config.find("data_dir");
        data_dir_ = (it != config.end()) ? it->second : "/var/log/smartmower/vision";

        // Optional overrides using flattened keys if provided
        auto it_cam_dir = config.find("camera.data_dir");
        if (it_cam_dir != config.end() && !it_cam_dir->second.empty()) {
            data_dir_ = it_cam_dir->second;
        }

        // If an explicit file is provided, it takes precedence
        auto it_cam_file = config.find("camera.file");
        if (it_cam_file != config.end() && !it_cam_file->second.empty()) {
            try {
                std::filesystem::path p(it_cam_file->second);
                if (!p.parent_path().empty()) {
                    std::filesystem::create_directories(p.parent_path());
                }
                log_file_ = p.string();
            } catch (const std::exception& e) {
                std::cerr << "Failed to prepare log file path " << it_cam_file->second
                          << ": " << e.what() << std::endl;
                return false;
            }
        } else {
            // No explicit file: ensure data_dir exists and build a timestamped filename
            try {
                std::filesystem::create_directories(data_dir_);
            } catch (const std::exception& e) {
                std::cerr << "Failed to create log directory " << data_dir_ 
                         << ": " << e.what() << std::endl;
                return false;
            }
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
            log_file_ = data_dir_ + "/camera_" + ss.str() + ".log";
        }
        
        // Open log file
        log_stream_.open(log_file_, std::ios::out | std::ios::app);
        if (!log_stream_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_ << std::endl;
            return false;
        }
        
        // Check if we should show log messages in the console
        it = config.find("show_windows");
        show_windows_ = (it != config.end()) ? (it->second == "true") : false;
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing logger from map: " << e.what() << std::endl;
        return false;
    }
}

bool Logger::initialize(const nlohmann::json& config) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    try {
        // Check if logging is enabled
        enabled_ = config.value("enabled", true);
        if (!enabled_) {
            return true; // Logging is explicitly disabled
        }
        
        // Get log level and convert to lowercase
        log_level_ = config.value("level", "info");
        std::transform(log_level_.begin(), log_level_.end(), log_level_.begin(), 
                      [](unsigned char c){ return std::tolower(c); });
        
        // Determine data directory and explicit file overrides
        data_dir_ = config.value("data_dir", "/var/log/smartmower/vision");
        std::string explicit_file;
        if (config.contains("camera") && config["camera"].is_object()) {
            const auto& cam = config["camera"];
            // Optional camera-specific data_dir
            if (cam.contains("data_dir") && cam["data_dir"].is_string()) {
                data_dir_ = cam["data_dir"].get<std::string>();
            }
            // Optional explicit file
            if (cam.contains("file") && cam["file"].is_string()) {
                explicit_file = cam["file"].get<std::string>();
            }
        }

        if (!explicit_file.empty()) {
            try {
                std::filesystem::path p(explicit_file);
                if (!p.parent_path().empty()) {
                    std::filesystem::create_directories(p.parent_path());
                }
                log_file_ = p.string();
            } catch (const std::exception& e) {
                std::cerr << "Failed to prepare log file path " << explicit_file
                          << ": " << e.what() << std::endl;
                return false;
            }
        } else {
            // No explicit file: ensure data_dir exists and build a timestamped filename
            try {
                std::filesystem::create_directories(data_dir_);
            } catch (const std::exception& e) {
                std::cerr << "Failed to create log directory " << data_dir_ 
                         << ": " << e.what() << std::endl;
                return false;
            }
            auto now = std::chrono::system_clock::now();
            auto in_time_t = std::chrono::system_clock::to_time_t(now);
            std::stringstream ss;
            ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
            log_file_ = data_dir_ + "/camera_" + ss.str() + ".log";
        }
        
        // Open log file
        log_stream_.open(log_file_, std::ios::out | std::ios::app);
        if (!log_stream_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_ << std::endl;
            return false;
        }
        
        // Check if we should show log messages in the console
        show_windows_ = config.value("show_windows", false);
        
        return true;
    } catch (const std::exception& e) {
        std::cerr << "Error initializing logger: " << e.what() << std::endl;
        return false;
    }
}

Logger::~Logger() {
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_stream_.is_open()) {
        log_stream_.close();
    }
}

void Logger::log(const std::string& level, const std::string& message) {
    if (!enabled_ || log_level_to_int(level) < log_level_to_int(log_level_)) {
        return;
    }
    
    std::string log_entry = get_timestamp() + " [" + level + "] " + message;
    
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    // Write to console if enabled
    if (show_windows_) {
        std::cout << log_entry << std::endl;
    }
    
    // Write to log file if open
    if (log_stream_.is_open()) {
        log_stream_ << log_entry << std::endl;
        log_stream_.flush();
    }
}

int Logger::log_level_to_int(const std::string& level) const {
    if (level == "debug") return 0;
    if (level == "info") return 1;
    if (level == "warning") return 2;
    if (level == "error") return 3;
    if (level == "critical") return 4;
    return 1; // Default to info
}

std::string Logger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

} // namespace camera
