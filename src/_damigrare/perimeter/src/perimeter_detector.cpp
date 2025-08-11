#include <mosquitto.h>
#include <cjson/cJSON.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <mutex>
#include <fstream>
#include <thread>
#include <cstring>

// Vision MQTT definitions
#include "vision_perimeter.h"

// Configuration structure
struct Config {
    // MQTT settings
    std::string broker;
    int port;
    std::string username;
    std::string password;
    std::string subscribe_topic;
    std::string publish_topic;
    std::string client_id;
    
    // Camera parameters
    double focal_length;
    double height_cm;
    int width_px;
    int height_px;
    
    // Detection parameters
    int min_contour_area;
    int threshold_value;
    int blur_size;
    int erode_iterations;
    int dilate_iterations;
    int hough_threshold;
    int min_line_length;
    int max_line_gap;
    
    // Color detection
    cv::Scalar wire_color_low;
    cv::Scalar wire_color_high;
    
    // Validation
    int min_wire_segments;
    double max_wire_distance;
    
    // Distance thresholds
    double warning_distance_m;
    double alert_distance_m;
    double max_distance_m;
    
    // Debug
    bool debug_enabled;
    bool show_windows;
};

// Variabili globali
Config config;
cv::Mat current_frame;
std::mutex frame_mutex;
std::atomic<bool> running(true);
struct mosquitto* mosq = nullptr;

// Carica la configurazione
bool load_config(const std::string& filename) {
    std::ifstream file(filename);
    if (!file) {
        std::cerr << "Impossibile aprire il file di configurazione" << std::endl;
        return false;
    }
    
    std::string content((std::istreambuf_iterator<char>(file)), 
                       (std::istreambuf_iterator<char>()));
    
    cJSON* root = cJSON_Parse(content.c_str());
    if (!root) {
        std::cerr << "Errore nel parsing del JSON" << std::endl;
        return false;
    }

    // Parse MQTT settings from configuration
    cJSON* mqtt = cJSON_GetObjectItemCaseSensitive(root, "mqtt");
    if (mqtt) {
        // Get MQTT broker settings
        cJSON* broker = cJSON_GetObjectItem(mqtt, "broker");
        cJSON* port = cJSON_GetObjectItem(mqtt, "port");
        cJSON* username = cJSON_GetObjectItem(mqtt, "username");
        cJSON* password = cJSON_GetObjectItem(mqtt, "password");
        
        if (cJSON_IsString(broker)) config.broker = broker->valuestring;
        if (cJSON_IsNumber(port)) config.port = port->valueint;
        if (cJSON_IsString(username)) config.username = username->valuestring;
        if (cJSON_IsString(password)) config.password = password->valuestring;
        
        // Get MQTT topics
        cJSON* topics = cJSON_GetObjectItem(mqtt, "topics");
        if (topics) {
            cJSON* camera = cJSON_GetObjectItem(topics, "camera");
            cJSON* perimeter = cJSON_GetObjectItem(topics, "perimeter");
            
            if (cJSON_IsObject(camera)) {
                cJSON* base = cJSON_GetObjectItem(camera, "base");
                if (cJSON_IsString(base)) {
                    config.subscribe_topic = base->valuestring;
                }
            }
            
            if (cJSON_IsObject(perimeter)) {
                cJSON* base = cJSON_GetObjectItem(perimeter, "base");
                if (cJSON_IsString(base)) {
                    config.publish_topic = base->valuestring;
                }
            }
        }
    } // Chiude if(mqtt)
    
    // Load camera configuration from root
    cJSON* camera = cJSON_GetObjectItem(root, "camera");
    if (camera) {
        config.width_px = cJSON_GetObjectItem(camera, "width")->valueint;
        config.height_px = cJSON_GetObjectItem(camera, "height")->valueint;
        
        cJSON* intrinsics = cJSON_GetObjectItem(camera, "intrinsics");
        if (intrinsics) {
            cJSON* fx = cJSON_GetObjectItem(intrinsics, "focal_length_x");
            if (fx) config.focal_length = fx->valuedouble;
        }
        
        cJSON* height = cJSON_GetObjectItem(camera, "height_m");
        if (height) {
            // Converti da metri a centimetri
            config.height_cm = height->valuedouble * 100.0;
        }
    }
        
    // Load perimeter detection configuration
    cJSON* perimeter_detection = cJSON_GetObjectItem(root, "perimeter_detection");
    if (perimeter_detection) {
        
        // Load detection parameters
        cJSON* detection = cJSON_GetObjectItem(perimeter_detection, "detection_parameters");
        if (detection) {
            cJSON* item;
            if ((item = cJSON_GetObjectItem(detection, "min_contour_area"))) 
                config.min_contour_area = item->valueint;
            if ((item = cJSON_GetObjectItem(detection, "threshold_value"))) 
                config.threshold_value = item->valueint;
            if ((item = cJSON_GetObjectItem(detection, "blur_size"))) 
                config.blur_size = item->valueint;
            if ((item = cJSON_GetObjectItem(detection, "erode_iterations"))) 
                config.erode_iterations = item->valueint;
            if ((item = cJSON_GetObjectItem(detection, "dilate_iterations"))) 
                config.dilate_iterations = item->valueint;
        }
        
        // Load distance thresholds
        cJSON* thresholds = cJSON_GetObjectItem(perimeter_detection, "distance_thresholds");
        if (thresholds) {
            cJSON* item;
            if ((item = cJSON_GetObjectItem(thresholds, "warning_distance_m"))) 
                config.warning_distance_m = item->valuedouble;
            if ((item = cJSON_GetObjectItem(thresholds, "alert_distance_m"))) 
                config.alert_distance_m = item->valuedouble;
            if ((item = cJSON_GetObjectItem(thresholds, "max_distance_m"))) 
                config.max_distance_m = item->valuedouble;
        }
    }
    
    // Load logging configuration
    cJSON* logging = cJSON_GetObjectItem(root, "logging");
    if (logging) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(logging, "enabled"))) 
            config.debug_enabled = cJSON_IsTrue(item);
        if ((item = cJSON_GetObjectItem(logging, "show_windows"))) 
            config.show_windows = cJSON_IsTrue(item);
    }

    cJSON_Delete(root);
    std::cout << "Configuration loaded successfully from robot_config.json" << std::endl;
    std::cout << "MQTT: " << config.broker << ":" << config.port 
              << " (user: " << config.username << ")" << std::endl;
    return true;
}

// Rileva il perimetro nell'immagine
void detect_perimeter(const cv::Mat& frame) {
    if (frame.empty()) return;
    
    // Converti in scala di grigi
    cv::Mat gray;
    cv::cvtColor(frame, gray, cv::COLOR_BGR2GRAY);
    
    // Applica blur
    int blur_size = config.blur_size | 1; // Assicura numero dispari
    cv::GaussianBlur(gray, gray, cv::Size(blur_size, blur_size), 0);
    
    // Thresholding
    cv::Mat thresh;
    cv::threshold(gray, thresh, config.threshold_value, 255, cv::THRESH_BINARY_INV);
    
    // Operazioni morfologiche
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(thresh, thresh, kernel, cv::Point(-1, -1), config.erode_iterations);
    cv::dilate(thresh, thresh, kernel, cv::Point(-1, -1), config.dilate_iterations);
    
    // Trova contorni
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(thresh.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    
    // Filtra contorni per area minima
    std::vector<std::vector<cv::Point>> valid_contours;
    for (const auto& contour : contours) {
        if (cv::contourArea(contour) > config.min_contour_area) {
            valid_contours.push_back(contour);
        }
    }
    
    if (config.debug_enabled) {
        std::cout << "Trovati " << valid_contours.size() << " contorni validi" << std::endl;
        
        if (config.show_windows) {
            // Mostra immagini di debug
            cv::imshow("Original", frame);
            cv::imshow("Gray", gray);
            cv::imshow("Threshold", thresh);
            
            // Disegna contorni
            cv::Mat contour_img = frame.clone();
            cv::drawContours(contour_img, valid_contours, -1, cv::Scalar(0, 255, 0), 2);
            cv::imshow("Contours", contour_img);
            
            cv::waitKey(1);
        }
    }
    
    // Calcola distanze e pubblica risultati
    if (!valid_contours.empty()) {
        // Trova il contorno più grande (più vicino)
        double max_area = 0;
        for (size_t i = 0; i < valid_contours.size(); i++) {
            double area = cv::contourArea(valid_contours[i]);
            if (area > max_area) {
                max_area = area;
            }
        }
        
        // Calcola distanza approssimativa basata sull'area
        double distance = config.focal_length * config.height_cm / sqrt(max_area);
        distance = std::min(distance, config.max_distance_m);
        
        std::cout << "Perimetro rilevato a distanza: " << distance << "m" << std::endl;
        
        // Crea messaggio MQTT
        cJSON* json = cJSON_CreateObject();
        cJSON_AddNumberToObject(json, "distance", distance);
        cJSON_AddBoolToObject(json, "warning", distance < config.warning_distance_m);
        cJSON_AddBoolToObject(json, "alert", distance < config.alert_distance_m);
        cJSON_AddNumberToObject(json, "contours", valid_contours.size());
        
        char* json_string = cJSON_Print(json);
        if (json_string) {
            mosquitto_publish(mosq, nullptr, config.publish_topic.c_str(), 
                            strlen(json_string), json_string, 0, false);
            free(json_string);
        }
        cJSON_Delete(json);
    }
}

// Callback per i messaggi MQTT
void on_message(struct mosquitto*, void*, const struct mosquitto_message* msg) {
    if (!msg->payload || msg->payloadlen <= 0) {
        std::cerr << "Messaggio vuoto ricevuto" << std::endl;
        return;
    }

    try {
        std::string payload(static_cast<const char*>(msg->payload), msg->payloadlen);
        cJSON* root = cJSON_Parse(payload.c_str());
        if (!root) {
            std::cerr << "Errore nel parsing del JSON" << std::endl;
            return;
        }

        cJSON* img_json = cJSON_GetObjectItem(root, "image");
        if (img_json && cJSON_IsString(img_json)) {
            std::string base64 = img_json->valuestring;
            
            // Decodifica base64 usando la stessa logica di mqtt_viewer
            std::vector<uchar> data;
            try {
                static const std::string chars = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
                
                std::string decoded;
                int val = 0, valb = -8;
                for (char c : base64) {
                    if (chars.find(c) == std::string::npos) break;
                    val = (val << 6) + chars.find(c);
                    valb += 6;
                    if (valb >= 0) {
                        decoded.push_back(char((val >> valb) & 0xFF));
                        valb -= 8;
                    }
                }
                
                data.assign(decoded.begin(), decoded.end());
            } catch (const std::exception& e) {
                std::cerr << "Errore nella decodifica base64: " << e.what() << std::endl;
                cJSON_Delete(root);
                return;
            }

            cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR);
            if (frame.empty()) {
                std::cerr << "Impossibile decodificare l'immagine" << std::endl;
            } else {
                std::lock_guard<std::mutex> lock(frame_mutex);
                frame.copyTo(current_frame);
                std::cout << "Frame ricevuto: " << frame.cols << "x" << frame.rows << std::endl;
                
                // Rileva il perimetro
                detect_perimeter(frame);
            }
        } else {
            std::cerr << "Nessun dato immagine trovato nel messaggio" << std::endl;
        }
        cJSON_Delete(root);
    } catch (const std::exception& e) {
        std::cerr << "Eccezione: " << e.what() << std::endl;
    }
}

int main() {
    std::cout << "Avvio del rilevatore di perimetro" << std::endl;
    
    if (!load_config("/opt/smartmower/etc/config/robot_config.json")) {
        std::cerr << "Errore nel caricamento della configurazione" << std::endl;
        return 1;
    }

    mosquitto_lib_init();
    mosq = mosquitto_new(config.client_id.c_str(), true, NULL);
    if (!mosq) {
        std::cerr << "Errore nell'inizializzazione di Mosquitto" << std::endl;
        return 1;
    }

    if (!config.username.empty()) {
        mosquitto_username_pw_set(mosq, config.username.c_str(), config.password.c_str());
        std::cout << "MQTT authentication set for user: " << config.username << std::endl;
    }

    mosquitto_message_callback_set(mosq, on_message);

    std::cout << "Connessione a " << config.broker << ":" << config.port << "..." << std::endl;
    if (mosquitto_connect(mosq, config.broker.c_str(), config.port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Failed to connect to MQTT broker at " 
                 << config.broker << ":" << config.port << std::endl;
        return 1;
    }
    
    std::cout << "Connected to MQTT broker at " << config.broker << ":" << config.port << std::endl;

    std::cout << "Sottoscrizione al topic: " << config.subscribe_topic << std::endl;
    if (mosquitto_subscribe(mosq, NULL, config.subscribe_topic.c_str(), 0) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore nella sottoscrizione al topic" << std::endl;
        return 1;
    }

    std::cout << "In ascolto dei messaggi (premi Ctrl+C per uscire)..." << std::endl;
    while (running) {
        int rc = mosquitto_loop(mosq, 100, 1);
        if (rc) {
            std::cerr << "Errore nel loop MQTT: " << mosquitto_strerror(rc) << std::endl;
            running = false;
        }
        
        // Gestisci eventi delle finestre OpenCV
        if (config.show_windows) {
            int key = cv::waitKey(1);
            if (key == 27) {  // ESC per uscire
                running = false;
            }
        }
    }

    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();
    cv::destroyAllWindows();

    return 0;
}
