#include <iostream>
#include <string>
#include <vector>
#include <cstring>
#include <cstdlib>
#include <ctime>
#include <csignal>
#include <unistd.h>
#include <mosquitto.h>
#include <cjson/cJSON.h>
#include <opencv2/opencv.hpp>
#include <fstream>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <libcamera/libcamera.h>
#include <libcamera/transform.h>
#include <libcamera/stream.h>

// Vision MQTT definitions
#include "vision_mqtt.h"

// Global variables
static volatile sig_atomic_t running = 1;
static struct mosquitto *mosq = nullptr;
static std::unique_ptr<libcamera::Camera> camera;
static std::unique_ptr<libcamera::CameraManager> cameraManager;
static std::unique_ptr<libcamera::CameraConfiguration> config;
static libcamera::Stream *stream = nullptr;
static std::mutex cameraMutex;
static cv::Mat currentFrame;

// Camera configuration parameters
struct CameraConfig {
    // Image dimensions and format
    int width = 1280;
    int height = 720;
    int fps = 30;
    
    // Image adjustments
    float brightness = 0.0f;
    float contrast = 1.0f;
    float saturation = 1.0f;
    float sharpness = 1.0f;
    
    // Exposure and white balance
    float exposure = 0.0f;
    float awb_red = 1.0f;
    float awb_blue = 1.0f;
};

// Application configuration
struct Config {
    // MQTT settings
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_topic = std::string(VISION_MQTT_BASE_TOPIC) + VISION_TOPIC_CAMERA;
    std::string mqtt_client_id = VISION_MQTT_CLIENT_CAMERA "_rpi";
    
    // Camera configuration
    CameraConfig camera_config;
};

static Config config;

// Function declarations
void handle_signal(int signal);
void cleanup();
int load_config();
int init_mqtt();
int init_camera();
int capture_and_send_frame();
std::string base64_encode(const unsigned char* data, size_t input_length);
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp);

// Base64 encoding
std::string base64_encode(const unsigned char* data, size_t input_length) {
    static const std::string base64_chars =
        "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
        "abcdefghijklmnopqrstuvwxyz"
        "0123456789+/";
    std::string encoded;
    encoded.reserve(4 * ((input_length + 2) / 3));
    for (size_t i = 0; i < input_length; i += 3) {
        uint32_t octet_a = i < input_length ? data[i] : 0;
        uint32_t octet_b = i + 1 < input_length ? data[i + 1] : 0;
        uint32_t octet_c = i + 2 < input_length ? data[i + 2] : 0;
        uint32_t triple = (octet_a << 0x10) + (octet_b << 0x08) + octet_c;
        encoded.push_back(base64_chars[(triple >> 3 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 2 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 1 * 6) & 0x3F]);
        encoded.push_back(base64_chars[(triple >> 0 * 6) & 0x3F]);
    }
    switch (input_length % 3) {
        case 1: encoded[encoded.length() - 1] = '=';
                encoded[encoded.length() - 2] = '='; break;
        case 2: encoded[encoded.length() - 1] = '='; break;
    }
    return encoded;
}

// Send image over MQTT
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp) {
    if (!mosq) return -1;
    static uint32_t sequence = 0;
    sequence++;

    std::string json_msg = 
        "{"
        "\"type\":\"" + std::string(VISION_JSON_CAMERA) + "\","
        "\"timestamp\":" + timestamp + ","
        "\"camera_id\":\"rpi_camera\","
        "\"image_info\":{"
            "\"width\":" + std::to_string(config.width) + ","
            "\"height\":" + std::to_string(config.height) + ","
            "\"format\":\"JPEG\","
            "\"sequence\":" + std::to_string(sequence) + ","
            "\"device\":\"Raspberry Pi Camera\""
        "},"
        "\"image_data\":\"" + base64_encode(image_data.data(), image_data.size()) + "\""
        "}";

    return (mosquitto_publish(mosq, nullptr, config.mqtt_topic.c_str(), 
                            json_msg.length(), json_msg.c_str(), 0, false) == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

// Initialize Raspberry Pi camera with libcamera
int init_camera() {
    try {
        // Initialize camera manager
        cameraManager = std::make_unique<libcamera::CameraManager>();
        if (cameraManager->start()) {
            std::cerr << "Failed to start camera manager" << std::endl;
            return 0;
        }

        // Get the first available camera
        if (cameraManager->cameras().empty()) {
            std::cerr << "No cameras available" << std::endl;
            return 0;
        }

        std::string cameraId = cameraManager->cameras()[0]->id();
        camera = cameraManager->get(cameraId);
        if (!camera) {
            std::cerr << "Failed to acquire camera " << cameraId << std::endl;
            return 0;
        }

        if (camera->acquire()) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return 0;
        }

        // Configure the camera
        config = camera->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (!config) {
            std::cerr << "Failed to generate camera configuration" << std::endl;
            return 0;
        }

        // Set stream configuration
        libcamera::StreamConfiguration &streamConfig = config->at(0);
        streamConfig.pixelFormat = libcamera::formats::BGR888;
        streamConfig.size.width = config.camera_config.width;
        streamConfig.size.height = config.camera_config.height;
        streamConfig.bufferCount = 2;

        // Set frame rate
        int ret = streamConfig.setFrameDuration(1.0 / config.camera_config.fps);
        if (ret) {
            std::cerr << "Failed to set frame rate" << std::endl;
            return 0;
        }

        // Validate and apply configuration
        if (config->validate() == libcamera::CameraConfiguration::Invalid) {
            std::cerr << "Failed to validate stream configuration" << std::endl;
            return 0;
        }

        if (camera->configure(config.get())) {
            std::cerr << "Failed to configure camera" << std::endl;
            return 0;
        }

        // Get the stream and allocate buffers
        stream = config->at(0).stream();
        if (camera->allocator(stream)->allocate(stream->configuration().bufferCount) < 0) {
            std::cerr << "Failed to allocate buffers" << std::endl;
            return 0;
        }

        // Apply camera controls
        libcamera::ControlList controls(camera->controls());
        controls.set(libcamera::controls::Brightness, config.camera_config.brightness);
        controls.set(libcamera::controls::Contrast, config.camera_config.contrast);
        controls.set(libcamera::controls::Saturation, config.camera_config.saturation);
        controls.set(libcamera::controls::Sharpness, config.camera_config.sharpness);
        controls.set(libcamera::controls::ExposureValue, config.camera_config.exposure);
        controls.set(libcamera::controls::AwbEnable, true);
        controls.set(libcamera::controls::AwbMode, libcamera::controls::AwbAuto);

        // Start the camera
        if (camera->start(&controls)) {
            std::cerr << "Failed to start camera" << std::endl;
            return 0;
        }

        // Camera warm-up
        std::this_thread::sleep_for(std::chrono::seconds(2));
        return 1;

    } catch (const std::exception& e) {
        std::cerr << "Error initializing camera: " << e.what() << std::endl;
        return 0;
    }
}

// Request completion handler
static void requestComplete(libcamera::Request *request) {
    if (request->status() == libcamera::Request::RequestCancelled) {
        return;
    }

    // Get the buffer for the stream
    const libcamera::Request::BufferMap &buffers = request->buffers();
    for (auto it = buffers.begin(); it != buffers.end(); ++it) {
        libcamera::Stream *stream = it->first;
        libcamera::FrameBuffer *buffer = it->second;
        
        // Get the frame data
        const libcamera::FrameBuffer::Plane &plane = buffer->planes().front();
        const libcamera::FrameMetadata &metadata = buffer->metadata();
        
        // Lock the frame mutex
        std::lock_guard<std::mutex> lock(cameraMutex);
        
        // Create or update the OpenCV Mat with the frame data
        const libcamera::StreamConfiguration &cfg = stream->configuration();
        currentFrame = cv::Mat(cfg.size.height, cfg.size.width, CV_8UC3, 
                             const_cast<uint8_t*>(static_cast<const uint8_t*>(plane.data)));
    }
    
    // Requeue the request
    request->reuse(libcamera::Request::ReuseBuffers);
    camera->queueRequest(request);
}

// Capture and send frame
int capture_and_send_frame() {
    try {
        // Create requests for each buffer
        std::vector<std::unique_ptr<libcamera::Request>> requests;
        
        // Get the buffer allocation
        libcamera::FrameBufferAllocator *allocator = camera->allocator(stream);
        if (!allocator) {
            std::cerr << "Failed to get buffer allocator" << std::endl;
            return -1;
        }
        
        // Create requests for each buffer
        for (unsigned int i = 0; i < allocator->buffers(stream).size(); ++i) {
            std::unique_ptr<libcamera::Request> request = camera->createRequest();
            if (!request) {
                std::cerr << "Can't create request" << std::endl;
                return -1;
            }
            
            // Get the buffer
            const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator->buffers(stream);
            if (i >= buffers.size()) {
                std::cerr << "Not enough buffers available" << std::endl;
                return -1;
            }
            
            // Add buffer to request
            if (request->addBuffer(stream, buffers[i].get()) < 0) {
                std::cerr << "Can't set buffer for request" << std::endl;
                return -1;
            }
            
            // Set completion callback
            request->setCompletionCallback(std::bind(&requestComplete, request.get()));
            
            // Queue the request
            if (camera->queueRequest(request.get()) < 0) {
                std::cerr << "Failed to queue request" << std::endl;
                return -1;
            }
            
            // Keep the request alive
            requests.push_back(std::move(request));
        }
        
        // Get current timestamp
        auto now = std::chrono::system_clock::now();
        auto timestamp = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()).count();
        
        // Wait for a frame to be available
        cv::Mat frame;
        {
            std::unique_lock<std::mutex> lock(cameraMutex);
            auto start = std::chrono::steady_clock::now();
            
            // Wait up to 1 second for a frame
            while (currentFrame.empty()) {
                if (std::chrono::steady_clock::now() - start > std::chrono::seconds(1)) {
                    std::cerr << "Timeout waiting for frame" << std::endl;
                    return -1;
                }
                std::this_thread::sleep_for(std::chrono::milliseconds(10));
            }
            
            frame = currentFrame.clone();
            currentFrame.release();
        }
        
        if (frame.empty()) {
            std::cerr << "Failed to capture frame" << std::endl;
            return -1;
        }
        
        // Convert to JPEG
        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", frame, buffer, params)) {
            std::cerr << "Failed to encode frame" << std::endl;
            return -1;
        }
        
        // Send frame
        return send_image(buffer, std::to_string(timestamp));
        
    } catch (const std::exception& e) {
        std::cerr << "Error in capture_and_send_frame: " << e.what() << std::endl;
        return -1;
    }
}

void cleanup() {
    running = 0;
    
    // Stop and release camera resources
    if (camera) {
        camera->stop();
        camera->release();
        camera.reset();
    }
    
    if (cameraManager) {
        cameraManager->stop();
        cameraManager.reset();
    }
    if (mosq) {
        mosquitto_disconnect(mosq);
        mosquitto_destroy(mosq);
        mosq = nullptr;
    }
    mosquitto_lib_cleanup();
    
    if (camera) {
        if (camera->isOpened()) camera->release();
        delete camera;
        camera = nullptr;
    }
}

void handle_signal(int signal) {
    (void)signal;
    running = 0;
}

// Load configuration from file
int load_config() {
    try {
        std::ifstream config_file("/opt/smartmower/etc/config/robot_config.json");
        if (!config_file.is_open()) return 1;

        std::string content((std::istreambuf_iterator<char>(config_file)),
                           std::istreambuf_iterator<char>());
        
        cJSON* root = cJSON_Parse(content.c_str());
        if (!root) return 0;

        // Parse MQTT settings
        cJSON* system = cJSON_GetObjectItemCaseSensitive(root, "system");
        if (system) {
            cJSON* comm = cJSON_GetObjectItemCaseSensitive(system, "communication");
            if (comm) {
                cJSON* item;
                if ((item = cJSON_GetObjectItemCaseSensitive(comm, "mqtt_broker_host")) && cJSON_IsString(item)) 
                    config.mqtt_broker = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(comm, "mqtt_broker_port")) && cJSON_IsNumber(item)) 
                    config.mqtt_port = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(comm, "mqtt_username")) && cJSON_IsString(item)) 
                    config.mqtt_username = item->valuestring;
                if ((item = cJSON_GetObjectItemCaseSensitive(comm, "mqtt_password")) && cJSON_IsString(item)) 
                    config.mqtt_password = item->valuestring;
            }
        }
        
        // Parse camera settings
        cJSON* camera = cJSON_GetObjectItemCaseSensitive(root, "camera");
        if (camera) {
            cJSON* common = cJSON_GetObjectItemCaseSensitive(camera, "common");
            if (common) {
                cJSON* item;
                if ((item = cJSON_GetObjectItemCaseSensitive(common, "width")) && cJSON_IsNumber(item)) 
                    config.width = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(common, "height")) && cJSON_IsNumber(item)) 
                    config.height = item->valueint;
                if ((item = cJSON_GetObjectItemCaseSensitive(common, "fps")) && cJSON_IsNumber(item)) 
                    config.fps = item->valueint;
            }
            
            cJSON* rpi = cJSON_GetObjectItemCaseSensitive(camera, "rpi");
            if (rpi) {
                cJSON* item;
                #define SET_IF_EXISTS(field, name) \
                    if ((item = cJSON_GetObjectItemCaseSensitive(rpi, name)) && cJSON_IsNumber(item)) \
                        config.field = item->valueint;
                
                SET_IF_EXISTS(brightness, "brightness");
                SET_IF_EXISTS(contrast, "contrast");
                SET_IF_EXISTS(saturation, "saturation");
                SET_IF_EXISTS(sharpness, "sharpness");
                SET_IF_EXISTS(iso, "iso");
                SET_IF_EXISTS(shutter_speed, "shutter_speed");
                SET_IF_EXISTS(awb_mode, "awb_mode");
                SET_IF_EXISTS(exposure_mode, "exposure_mode");
                #undef SET_IF_EXISTS
            }
        }
        
        cJSON_Delete(root);
        return 1;
    } catch (...) {
        return 0;
    }
}

// Initialize MQTT connection
int init_mqtt() {
    mosquitto_lib_init();
    mosq = mosquitto_new(config.mqtt_client_id.c_str(), true, NULL);
    if (!mosq) return 0;
    
    if (!config.mqtt_username.empty() && !config.mqtt_password.empty()) {
        if (mosquitto_username_pw_set(mosq, config.mqtt_username.c_str(), 
                                    config.mqtt_password.c_str()) != MOSQ_ERR_SUCCESS) {
            return 0;
        }
    }
    
    return (mosquitto_connect(mosq, config.mqtt_broker.c_str(), 
                            config.mqtt_port, 60) == MOSQ_ERR_SUCCESS) ? 1 : 0;
}

// Capture and send frame
int capture_and_send_frame() {
    if (!camera || !camera->isOpened()) return -1;

    try {
        cv::Mat frame;
        if (!camera->grab()) return -1;
        camera->retrieve(frame);
        if (frame.empty()) return -1;

        std::vector<uchar> buffer;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, 80};
        if (!cv::imencode(".jpg", frame, buffer, params)) return -1;

        auto now = std::chrono::system_clock::now();
        auto now_time_t = std::chrono::system_clock::to_time_t(now);
        auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
            now.time_since_epoch()) % 1000;
            
        std::stringstream ss;
        ss << std::put_time(std::localtime(&now_time_t), "%Y-%m-%d %H:%M:%S");
        ss << '.' << std::setfill('0') << std::setw(3) << now_ms.count();
        
        return send_image(buffer, ss.str());
    } catch (...) {
        return -1;
    }
}

int main(int argc, char **argv) {
    // Suppress GStreamer debug output
    setenv("GST_DEBUG", "*:1", 1);  // Set to 1 for errors only, 0 for none
    setenv("GST_DEBUG_NO_COLOR", "1", 1);
    
    // Set up signal handling
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);

    // Load configuration
    if (load_config() != 0) {
        std::cerr << "Failed to load configuration" << std::endl;
        return 1;
    }

    if (!init_mqtt()) {
        std::cerr << "Failed to initialize MQTT" << std::endl;
        return 1;
    }

    int max_retries = 5;
    for (int i = 0; i < max_retries; i++) {
        if (init_camera()) break;
        if (i == max_retries - 1) {
            std::cerr << "Failed to initialize camera" << std::endl;
            cleanup();
            return 1;
        }
        std::this_thread::sleep_for(std::chrono::seconds(2));
    }

    std::cout << "RPi Camera started. Resolution: " << config.width 
              << "x" << config.height << " @ " << config.fps << " FPS" << std::endl;

    while (running) {
        if (capture_and_send_frame() != 0) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(1000/config.fps));
    }

    cleanup();
    return 0;
}
