#ifndef CAMERA_LOGGING_H
#define CAMERA_LOGGING_H

#include <string>
#include <fstream>
#include <memory>
#include <mutex>
#include <nlohmann/json.hpp>

namespace camera {

class Logger {
public:
    static Logger& getInstance() {
        static Logger instance;
        return instance;
    }

    // Delete copy constructor and assignment operator
    Logger(const Logger&) = delete;
    void operator=(const Logger&) = delete;

    // Initialize with JSON configuration
    bool initialize(const nlohmann::json& config);
    // Initialize with string map configuration (for backward compatibility)
    bool initialize(const std::unordered_map<std::string, std::string>& config);
    void log(const std::string& level, const std::string& message);
    bool isEnabled() const { return enabled_; }
    
    // Convenience methods
    void debug(const std::string& message) { log("debug", message); }
    void info(const std::string& message) { log("info", message); }
    void warning(const std::string& message) { log("warning", message); }
    void error(const std::string& message) { log("error", message); }
    void critical(const std::string& message) { log("critical", message); }

private:
    Logger() = default;
    ~Logger();

    bool enabled_ = false;
    std::string log_level_ = "info";
    std::string log_file_;
    std::string data_dir_;
    bool show_windows_ = false;
    std::ofstream log_stream_;
    std::mutex log_mutex_;
    
    int log_level_to_int(const std::string& level) const;
    std::string get_timestamp() const;
};

// Global logger instance
inline Logger& getLogger() {
    return Logger::getInstance();
}

// Logging macros
#define LOG_DEBUG(msg) camera::getLogger().debug(msg)
#define LOG_INFO(msg) camera::getLogger().info(msg)
#define LOG_WARNING(msg) camera::getLogger().warning(msg)
#define LOG_ERROR(msg) camera::getLogger().error(msg)
#define LOG_CRITICAL(msg) camera::getLogger().critical(msg)

} // namespace camera

#endif // CAMERA_LOGGING_H
