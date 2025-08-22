#include "camera/logging.h"
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <unistd.h>
#include <filesystem>
#include <algorithm>

namespace camera {

Logger::~Logger() {
    if (log_stream_.is_open()) {
        log_stream_.close();
    }
}

bool Logger::initialize(const std::unordered_map<std::string, std::string>& config) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    try {
        // Get logging configuration
        auto it = config.find("enabled");
        enabled_ = (it != config.end()) ? (it->second == "true") : true;
        if (!enabled_) {
            return true; // Logging is disabled
        }
        
        // Get log level (default to "info" if not specified)
        it = config.find("level");
        log_level_ = (it != config.end()) ? it->second : "info";
        
        // Get show_windows flag (default to false if not specified)
        it = config.find("show_windows");
        show_windows_ = (it != config.end()) ? (it->second == "true") : false;
        
        // Get camera-specific logging config
        std::string camera_prefix = "camera.";
        for (const auto& [key, value] : config) {
            if (key.find(camera_prefix) == 0) {
                std::string subkey = key.substr(camera_prefix.length());
                if (subkey == "file") {
                    log_file_ = value;
                } else if (subkey == "data_dir") {
                    data_dir_ = value;
                }
            }
        }
        
        // Set defaults if not found in config
        if (log_file_.empty()) log_file_ = "/opt/smartmower/log/vision_camera.log";
        if (data_dir_.empty()) data_dir_ = "/opt/smartmower/data/vision/camera";
        
        // Create data directory if it doesn't exist
        std::filesystem::create_directories(data_dir_);
        std::filesystem::path log_path(log_file_);
        std::filesystem::create_directories(log_path.parent_path());
        
        // Open log file
        log_stream_.open(log_file_, std::ios::out | std::ios::app);
        if (!log_stream_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_ << std::endl;
            return false;
        }
        
        // Log initialization message
        log_stream_ << "\n" << get_timestamp() << " [INFO] Vision Camera Logger initialized (map config)" << std::endl;
        log_stream_ << "Log level: " << log_level_ << std::endl;
        log_stream_ << "Log file: " << log_file_ << std::endl;
        log_stream_ << "Data directory: " << data_dir_ << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing logger from map: " << e.what() << std::endl;
        return false;
    }
}

bool Logger::initialize(const nlohmann::json& config) {
    std::lock_guard<std::mutex> lock(log_mutex_);
    
    try {
        // Get logging configuration
        enabled_ = config.value("enabled", true);
        if (!enabled_) {
            return true; // Logging is disabled
        }
        
        log_level_ = config.value("level", std::string("info"));
        show_windows_ = config.value("show_windows", false);
        
        // Get camera-specific logging config
        const auto& camera_logging = config.value("camera", nlohmann::json::object());
        log_file_ = camera_logging.value("file", "/opt/smartmower/log/vision_camera.log");
        data_dir_ = camera_logging.value("data_dir", "/opt/smartmower/data/vision/camera");
        
        // Create data directory if it doesn't exist
        std::filesystem::create_directories(data_dir_);
        std::filesystem::path log_path(log_file_);
        std::filesystem::create_directories(log_path.parent_path());
        
        // Open log file
        log_stream_.open(log_file_, std::ios::out | std::ios::app);
        if (!log_stream_.is_open()) {
            std::cerr << "Failed to open log file: " << log_file_ << std::endl;
            return false;
        }
        
        // Log initialization message
        log_stream_ << "\n" << get_timestamp() << " [INFO] Vision Camera Logger initialized (JSON config)" << std::endl;
        log_stream_ << "Log level: " << log_level_ << std::endl;
        log_stream_ << "Log file: " << log_file_ << std::endl;
        log_stream_ << "Data directory: " << data_dir_ << std::endl;
        
        return true;
        
    } catch (const std::exception& e) {
        std::cerr << "Error initializing logger: " << e.what() << std::endl;
        return false;
    }
}

void Logger::log(const std::string& level, const std::string& message) {
    if (!enabled_ || log_level_to_int(level) < log_level_to_int(log_level_)) {
        return;
    }
    
    std::string level_upper = level;
    std::transform(level_upper.begin(), level_upper.end(), level_upper.begin(), 
                  [](unsigned char c){ return std::toupper(c); });
    
    std::string log_entry = get_timestamp() + " [" + level_upper + "] " + message;
    
    // Log to console
    if (level == "error" || level == "critical") {
        std::cerr << log_entry << std::endl;
    } else {
        std::cout << log_entry << std::endl;
    }
    
    // Log to file
    std::lock_guard<std::mutex> lock(log_mutex_);
    if (log_stream_.is_open()) {
        log_stream_ << log_entry << std::endl;
    }
}

int Logger::log_level_to_int(const std::string& level) const {
    if (level == "debug") return 0;
    if (level == "info") return 1;
    if (level == "warning") return 2;
    if (level == "error") return 3;
    if (level == "critical") return 4;
    return 1; // default to info
}

std::string Logger::get_timestamp() const {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&time), "%Y-%m-%d %H:%M:%S");
    ss << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

} // namespace camera
