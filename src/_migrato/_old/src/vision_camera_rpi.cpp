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
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <libcamera/libcamera.h>
#include <libcamera/transform.h>
#include <libcamera/stream.h>
#include <stdexcept>
#include <fstream>
#include <thread>
#include <chrono>
#include <memory>
#include <sys/mman.h>
#include <nlohmann/json.hpp>

using json = nlohmann::json;

// Vision MQTT definitions
#include "vision_camera.h"

// Global variables
static volatile sig_atomic_t running = 1;
static struct mosquitto *mosq = nullptr;
static std::shared_ptr<libcamera::Camera> camera;
static std::unique_ptr<libcamera::CameraManager> cameraManager;
static std::unique_ptr<libcamera::CameraConfiguration> cameraConfig;
static libcamera::Stream *stream = nullptr;
static std::vector<std::unique_ptr<libcamera::Request>> requests;
static std::unique_ptr<libcamera::FrameBufferAllocator> allocator;

// Configuration
struct Config {
    // MQTT settings
    std::string mqtt_broker = "localhost";
    int mqtt_port = 1883;
    std::string mqtt_username;
    std::string mqtt_password;
    std::string mqtt_base_topic = "smartmower";
    std::string mqtt_topic = "smartmower/vision/camera/frame";
    std::string mqtt_status_topic = "smartmower/vision/camera/status";
    std::string mqtt_commands_topic = "smartmower/vision/camera/commands";
    std::string mqtt_client_id = "vision_camera_rpi";
    int mqtt_qos = 1;
    bool mqtt_retain = false;
    
    // Camera settings
    int width = 640;
    int height = 480;
    int fps = 15;
    float brightness = 0.0f;
    float contrast = 1.0f;
    float saturation = 1.0f;
    float sharpness = 1.0f;
    float exposure = 0.0f;
    
    // Logging
    std::string log_level = "info";
    std::string log_file = "vision_camera_rpi.log";
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
static void request_complete(libcamera::Request *request);

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

    // Add padding
    switch (input_length % 3) {
        case 1:
            encoded[encoded.length() - 1] = '=';
            encoded[encoded.length() - 2] = '=';
            break;
        case 2:
            encoded[encoded.length() - 1] = '=';
            break;
    }

    return encoded;
}

// Send image over MQTT
int send_image(const std::vector<uchar>& image_data, const std::string& timestamp) {
    if (!mosq) return -1;
    
    static uint32_t sequence = 0;
    sequence++;
    
    // Create JSON object
    cJSON *root = cJSON_CreateObject();
    
    // Add type and timestamp
    cJSON_AddStringToObject(root, "type", VISION_JSON_CAMERA);
    cJSON_AddStringToObject(root, "timestamp", timestamp.c_str());
    cJSON_AddStringToObject(root, "camera_id", "camera_0");
    
    // Add image info object
    cJSON *image_info = cJSON_CreateObject();
    cJSON_AddNumberToObject(image_info, "width", config.width);
    cJSON_AddNumberToObject(image_info, "height", config.height);
    cJSON_AddStringToObject(image_info, "format", "JPEG");
    cJSON_AddNumberToObject(image_info, "sequence", sequence);
    cJSON_AddItemToObject(root, "image_info", image_info);
    
    // Add base64 encoded image
    std::string base64_img = base64_encode(image_data.data(), image_data.size());
    cJSON_AddStringToObject(root, "image_data", base64_img.c_str());
    
    // Convert to JSON string
    char *json_str = cJSON_PrintUnformatted(root);
    if (!json_str) {
        std::cerr << "Error creating JSON string" << std::endl;
        cJSON_Delete(root);
        return -1;
    }
    
    // Publish message
    int result = mosquitto_publish(mosq, NULL, config.mqtt_topic.c_str(), 
                                  strlen(json_str), json_str, 
                                  config.mqtt_qos, config.mqtt_retain);
    
    // Cleanup
    free(json_str);
    cJSON_Delete(root);
    
    // Log result
    if (result != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to publish MQTT message: " << mosquitto_strerror(result) << std::endl;
    }
    
    return (result == MOSQ_ERR_SUCCESS) ? 0 : -1;
}

// Request complete callback for libcamera
static void request_complete(libcamera::Request *request) {
    // Gestione stato della richiesta
    if (request->status() == libcamera::Request::RequestCancelled) {
        std::cerr << "Richiesta annullata" << std::endl;
        return;
    } else if (request->status() == libcamera::Request::RequestComplete) {
        // La richiesta è stata completata con successo
        std::cout << "Richiesta completata con successo" << std::endl;
    } else {
        // Altri stati non gestiti
        std::cerr << "Stato richiesta non gestito: " << request->status() << std::endl;
        return;
    }

    const libcamera::Request::BufferMap &buffers = request->buffers();
    
    for (const auto &pair : buffers) {
        const libcamera::Stream *stream = pair.first;
        libcamera::FrameBuffer *buffer = pair.second;
        
        // Verifica del buffer
        if (!buffer) {
            std::cerr << "Buffer nullo ricevuto" << std::endl;
            continue;
        }
        
        // Verifica dei piani del buffer
        if (buffer->planes().empty()) {
            std::cerr << "Nessun piano disponibile nel buffer" << std::endl;
            continue;
        }
        
        // Get the frame data
        const libcamera::FrameBuffer::Plane &plane = buffer->planes().front();
        const libcamera::FrameMetadata &metadata = buffer->metadata();
        
        // Log dettagli sullo stato del frame
        std::cout << "Frame ricevuto - Stato: " << metadata.status 
                  << ", Sequenza: " << metadata.sequence
                  << ", Timestamp: " << metadata.timestamp << std::endl;
        
        if (metadata.status != libcamera::FrameMetadata::FrameSuccess) {
            std::string status_str;
            switch (metadata.status) {
                case libcamera::FrameMetadata::FrameError: status_str = "Errore nel frame"; break;
                case libcamera::FrameMetadata::FrameCancelled: status_str = "Frame annullato"; break;
                case libcamera::FrameMetadata::FrameStartup: status_str = "Frame di startup"; break;
                default: status_str = "Stato sconosciuto";
            }
            std::cerr << "Errore nel frame: " << status_str << std::endl;
            continue;
        }
        
        const libcamera::StreamConfiguration &cfg = stream->configuration();
        
        // Map the buffer data con gestione errori migliorata
        void *data = mmap(NULL, plane.length, PROT_READ, MAP_SHARED, plane.fd.get(), 0);
        if (data == MAP_FAILED) {
            std::cerr << "Errore nel mapping del buffer: " << strerror(errno) << std::endl;
            continue;
        }
        
        try {
            // Verifica dimensione attesa del frame
            size_t expected_size = cfg.size.width * cfg.size.height * 3; // 3 canali per BGR888
            if (plane.length < expected_size) {
                std::cerr << "Dimensione buffer insufficiente: " << plane.length 
                          << " bytes, attesi almeno " << expected_size << " bytes" << std::endl;
                throw std::runtime_error("Dimensione buffer non valida");
            }
            
            // Convert to OpenCV Mat con verifica dimensione
            cv::Mat frame(cfg.size.height, cfg.size.width, CV_8UC3, data);
            
            if (frame.empty()) {
                std::cerr << "Errore: frame vuoto dopo la creazione" << std::endl;
                throw std::runtime_error("Frame vuoto");
            }
            
            // Verifica che i dati dell'immagine non siano nulli
            if (!frame.data) {
                std::cerr << "Errore: dati immagine nulli" << std::endl;
                throw std::runtime_error("Dati immagine nulli");
            }
            
            // Get current timestamp con formato ISO 8601
            auto now = std::chrono::system_clock::now();
            auto now_time_t = std::chrono::system_clock::to_time_t(now);
            std::string timestamp = std::to_string(now_time_t);
            
            // Log info frame
            std::cout << "Elaborazione frame " << metadata.sequence 
                      << ", Dimensione: " << frame.cols << "x" << frame.rows << std::endl;
            
            // Encode as JPEG con gestione errori
            std::vector<uchar> jpeg_buffer;
            if (!cv::imencode(".jpg", frame, jpeg_buffer, {cv::IMWRITE_JPEG_QUALITY, 80})) {
                std::cerr << "Errore nella codifica JPEG" << std::endl;
                throw std::runtime_error("Errore codifica JPEG");
            }
            
            std::cout << "Dimensione JPEG: " << jpeg_buffer.size() << " bytes" << std::endl;
            
            // Send via MQTT con verifica
            int result = send_image(jpeg_buffer, timestamp);
            if (result != MOSQ_ERR_SUCCESS) {
                std::cerr << "Errore nell'invio MQTT: " << mosquitto_strerror(result) << std::endl;
            } else {
                std::cout << "Frame inviato con successo via MQTT" << std::endl;
            }
            
        } catch (const cv::Exception& e) {
            std::cerr << "Errore OpenCV: " << e.what() << std::endl;
        } catch (const std::exception& e) {
            std::cerr << "Errore nell'elaborazione del frame: " << e.what() << std::endl;
        }
        
        // Unmap del buffer
        if (munmap(data, plane.length) == -1) {
            std::cerr << "Errore nell'unmapping del buffer: " << strerror(errno) << std::endl;
        }
    }
    
    // Requeue the request
    request->reuse(libcamera::Request::ReuseBuffers);
    camera->queueRequest(request);
}

// Initialize Raspberry Pi camera with libcamera
int init_camera() {
    try {
        // Create camera manager
        cameraManager = std::make_unique<libcamera::CameraManager>();
        cameraManager->start();

        // Get the first available camera
        if (cameraManager->cameras().empty()) {
            std::cerr << "No cameras available" << std::endl;
            return -1;
        }
        
        std::string cameraId = cameraManager->cameras()[0]->id();
        std::cout << "Using camera: " << cameraId << std::endl;
        
        camera = cameraManager->get(cameraId);
        
        if (!camera) {
            std::cerr << "Failed to get camera" << std::endl;
            return -1;
        }
        
        if (camera->acquire()) {
            std::cerr << "Failed to acquire camera" << std::endl;
            return -1;
        }
        
        // Configure the camera
        cameraConfig = camera->generateConfiguration({libcamera::StreamRole::Viewfinder});
        if (!cameraConfig) {
            std::cerr << "Failed to generate configuration" << std::endl;
            return -1;
        }
        
        // Configure the stream
        libcamera::StreamConfiguration &streamConfig = cameraConfig->at(0);
        streamConfig.size.width = config.width;
        streamConfig.size.height = config.height;
        streamConfig.pixelFormat = libcamera::formats::BGR888;
        streamConfig.bufferCount = 4;
        
        std::cout << "Camera configuration: " << config.width << "x" << config.height << std::endl;
        
        // Validate configuration
        if (cameraConfig->validate() == libcamera::CameraConfiguration::Invalid) {
            std::cerr << "Failed to validate configuration" << std::endl;
            return -1;
        }
        
        // Apply configuration
        if (camera->configure(cameraConfig.get()) < 0) {
            std::cerr << "Failed to configure camera" << std::endl;
            return -1;
        }
        
        stream = cameraConfig->at(0).stream();
        
        // Set camera controls
        libcamera::ControlList controls(camera->controls());
        controls.set(libcamera::controls::Brightness, config.brightness);
        controls.set(libcamera::controls::Contrast, config.contrast);
        controls.set(libcamera::controls::Saturation, config.saturation);
        controls.set(libcamera::controls::Sharpness, config.sharpness);
        controls.set(libcamera::controls::ExposureValue, config.exposure);
        controls.set(libcamera::controls::AwbEnable, true);
        controls.set(libcamera::controls::AwbMode, libcamera::controls::AwbAuto);
        
        // Allocate buffers
        allocator = std::make_unique<libcamera::FrameBufferAllocator>(camera);
        if (allocator->allocate(stream) < 0) {
            std::cerr << "Failed to allocate buffers" << std::endl;
            return -1;
        }
        
        // Create requests
        const std::vector<std::unique_ptr<libcamera::FrameBuffer>> &buffers = allocator->buffers(stream);
        
        for (const auto &buffer : buffers) {
            std::unique_ptr<libcamera::Request> request = camera->createRequest();
            if (!request) {
                std::cerr << "Failed to create request" << std::endl;
                return -1;
            }
            
            if (request->addBuffer(stream, buffer.get()) < 0) {
                std::cerr << "Failed to add buffer to request" << std::endl;
                return -1;
            }
            
            requests.push_back(std::move(request));
        }
        
        // Set request completed callback
        camera->requestCompleted.connect(request_complete);
        
        std::cout << "Camera initialized successfully" << std::endl;
        return 0;
        
    } catch (const std::exception &e) {
        std::cerr << "Camera initialization error: " << e.what() << std::endl;
        return -1;
    }
}

// Start camera capture
int capture_and_send_frame() {
    if (!camera) {
        std::cerr << "Camera not initialized" << std::endl;
        return -1;
    }
    
    // Start the camera
    if (camera->start()) {
        std::cerr << "Failed to start camera" << std::endl;
        return -1;
    }
    
    // Queue all requests
    for (auto &request : requests) {
        if (camera->queueRequest(request.get()) < 0) {
            std::cerr << "Failed to queue request" << std::endl;
            return -1;
        }
    }
    
    std::cout << "Camera capture started" << std::endl;
    return 0;
}

void cleanup() {
    running = 0;
    
    if (camera) {
        camera->stop();
        camera->release();
        camera.reset();
    }
    
    if (allocator) {
        allocator.reset();
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
    std::cout << "Cleanup completed" << std::endl;
}

void handle_signal(int signal) {
    std::cout << "Received signal " << signal << ", shutting down..." << std::endl;
    running = 0;
}

// Load configuration from JSON file
int load_config() {
    const char* config_path = "/opt/smartmower/etc/config/robot_config.json";
    std::ifstream config_file(config_path);
    
    if (!config_file.is_open()) {
        std::cerr << "Errore: impossibile aprire il file di configurazione " << config_path << std::endl;
        std::cerr << "Utilizzo dei valori di default" << std::endl;
        return 0;
    }
    
    try {
        nlohmann::json json_config;
        config_file >> json_config;
        
        // MQTT settings
        if (json_config.contains("mqtt")) {
            auto& mqtt = json_config["mqtt"];
            if (mqtt.contains("broker")) config.mqtt_broker = mqtt["broker"].get<std::string>();
            if (mqtt.contains("port")) config.mqtt_port = mqtt["port"].get<int>();
            if (mqtt.contains("username")) config.mqtt_username = mqtt["username"].get<std::string>();
            if (mqtt.contains("password")) config.mqtt_password = mqtt["password"].get<std::string>();
            if (mqtt.contains("base_topic")) config.mqtt_base_topic = mqtt["base_topic"].get<std::string>();
            
            // Costruisci i topic completi
            config.mqtt_topic = config.mqtt_base_topic + "/vision/camera/frame";
            config.mqtt_status_topic = config.mqtt_base_topic + "/vision/camera/status";
            config.mqtt_commands_topic = config.mqtt_base_topic + "/vision/camera/commands";
            
            if (mqtt.contains("client_id")) {
                config.mqtt_client_id = mqtt["client_id"].get<std::string>();
            } else {
                // Genera un client ID univoco se non specificato
                std::ostringstream ss;
                ss << "vision_camera_" << std::hex << time(nullptr);
                config.mqtt_client_id = ss.str();
            }
        }
        
        // Camera settings
        if (json_config.contains("camera")) {
            auto& camera = json_config["camera"];
            if (camera.contains("width")) config.width = camera["width"].get<int>();
            if (camera.contains("height")) config.height = camera["height"].get<int>();
            if (camera.contains("fps")) config.fps = camera["fps"].get<int>();
            if (camera.contains("brightness")) config.brightness = camera["brightness"].get<float>();
            if (camera.contains("contrast")) config.contrast = camera["contrast"].get<float>();
            if (camera.contains("saturation")) config.saturation = camera["saturation"].get<float>();
            if (camera.contains("sharpness")) config.sharpness = camera["sharpness"].get<float>();
            if (camera.contains("exposure")) config.exposure = camera["exposure"].get<float>();
        }
        
        // Log settings
        if (json_config.contains("logging")) {
            auto& logging = json_config["logging"];
            if (logging.contains("level")) config.log_level = logging["level"].get<std::string>();
            if (logging.contains("file")) config.log_file = logging["file"].get<std::string>();
        }
        
        std::cout << "Configurazione caricata da " << config_path << std::endl;
        return 0;
        
    } catch (const std::exception& e) {
        std::cerr << "Errore nel parsing del file di configurazione: " << e.what() << std::endl;
        std::cerr << "Utilizzo dei valori di default" << std::endl;
        return 0;
    }
}

// Initialize MQTT connection
int init_mqtt() {
    mosquitto_lib_init();
    
    mosq = mosquitto_new(config.mqtt_client_id.c_str(), true, nullptr);
    if (!mosq) {
        std::cerr << "Failed to create mosquitto client" << std::endl;
        return -1;
    }
    
    if (!config.mqtt_username.empty()) {
        mosquitto_username_pw_set(mosq, config.mqtt_username.c_str(), 
                                 config.mqtt_password.c_str());
    }
    
    if (mosquitto_connect(mosq, config.mqtt_broker.c_str(), config.mqtt_port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker" << std::endl;
        return -1;
    }
    
    std::cout << "Connected to MQTT broker: " << config.mqtt_broker << ":" << config.mqtt_port << std::endl;
    return 0;
}

int main(int /*argc*/, char** /*argv*/) {
    std::cout << "Vision Camera RPI - Starting..." << std::endl;
    
    // Set up signal handlers
    signal(SIGINT, handle_signal);
    signal(SIGTERM, handle_signal);
    
    // Load configuration
    if (load_config() < 0) {
        std::cerr << "Failed to load configuration" << std::endl;
        return -1;
    }
    
    // Initialize MQTT
    if (init_mqtt() < 0) {
        std::cerr << "Failed to initialize MQTT" << std::endl;
        return -1;
    }
    
    // Initialize camera
    if (init_camera() < 0) {
        std::cerr << "Failed to initialize camera" << std::endl;
        cleanup();
        return -1;
    }
    
    // Start capture
    if (capture_and_send_frame() < 0) {
        std::cerr << "Failed to start capture" << std::endl;
        cleanup();
        return -1;
    }
    
    // Main loop
    std::cout << "Vision Camera RPI running... Press Ctrl+C to stop" << std::endl;
    while (running) {
        mosquitto_loop(mosq, 100, 1);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    cleanup();
    std::cout << "Vision Camera RPI stopped" << std::endl;
    return 0;
}
