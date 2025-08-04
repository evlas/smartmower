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

// Configurazione
struct Config {
    std::string broker = "localhost";
    int port = 1883;
    std::string topic = "smartmower/vision/camera";
    std::string client_id = "mqtt_viewer";
    std::string username;
    std::string password;
    bool show_window = true;
};

// Variabili globali
Config config;
cv::Mat current_frame;
std::mutex frame_mutex;
std::atomic<bool> running(true);

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

    cJSON* mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (mqtt) {
        cJSON* item;
        if ((item = cJSON_GetObjectItem(mqtt, "broker"))) 
            config.broker = item->valuestring;
        if ((item = cJSON_GetObjectItem(mqtt, "port"))) 
            config.port = item->valueint;
        if ((item = cJSON_GetObjectItem(mqtt, "subscribe_topic"))) 
            config.topic = item->valuestring;
        if ((item = cJSON_GetObjectItem(mqtt, "username"))) 
            config.username = item->valuestring;
        if ((item = cJSON_GetObjectItem(mqtt, "password"))) 
            config.password = item->valuestring;
    }

    cJSON_Delete(root);
    return true;
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
            
            // Decodifica base64 usando OpenCV
            std::vector<uchar> data;
            cv::Mat decoded_data;
            cv::Mat base64_mat(1, base64.length(), CV_8UC1, (void*)base64.data());
            
            try {
                cv::Mat decoded_mat;
                // Decodifica manuale base64
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
            }
        } else {
            std::cerr << "Nessun dato immagine trovato nel messaggio" << std::endl;
        }
        cJSON_Delete(root);
    } catch (const std::exception& e) {
        std::cerr << "Eccezione: " << e.what() << std::endl;
    }
}

// Visualizza il frame corrente
void display_loop() {
    cv::namedWindow("MQTT Stream", cv::WINDOW_NORMAL);
    while (running) {
        cv::Mat frame;
        {
            std::lock_guard<std::mutex> lock(frame_mutex);
            if (!current_frame.empty()) {
                frame = current_frame.clone();
            }
        }
        
        if (!frame.empty()) {
            cv::imshow("MQTT Stream", frame);
        } else {
            std::cout << "In attesa di frame..." << std::endl;
        }
        
        int key = cv::waitKey(100);
        if (key == 27) {  // ESC per uscire
            running = false;
        }
    }
    cv::destroyAllWindows();
}

int main() {
    std::cout << "Avvio del visualizzatore MQTT" << std::endl;
    
    if (!load_config("../config.json")) {
        std::cerr << "Errore nel caricamento della configurazione" << std::endl;
        return 1;
    }

    mosquitto_lib_init();
    struct mosquitto* mosq = mosquitto_new(config.client_id.c_str(), true, NULL);
    if (!mosq) {
        std::cerr << "Errore nell'inizializzazione di Mosquitto" << std::endl;
        return 1;
    }

    if (!config.username.empty()) {
        mosquitto_username_pw_set(mosq, config.username.c_str(), config.password.c_str());
    }

    mosquitto_message_callback_set(mosq, on_message);

    std::cout << "Connessione a " << config.broker << ":" << config.port << "..." << std::endl;
    if (mosquitto_connect(mosq, config.broker.c_str(), config.port, 60) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore nella connessione al broker" << std::endl;
        return 1;
    }

    std::cout << "Sottoscrizione al topic: " << config.topic << std::endl;
    if (mosquitto_subscribe(mosq, NULL, config.topic.c_str(), 0) != MOSQ_ERR_SUCCESS) {
        std::cerr << "Errore nella sottoscrizione al topic" << std::endl;
        return 1;
    }

    std::thread display_thread(display_loop);

    std::cout << "In ascolto dei messaggi (premi ESC per uscire)..." << std::endl;
    while (running) {
        int rc = mosquitto_loop(mosq, 100, 1);
        if (rc) {
            std::cerr << "Errore nel loop MQTT: " << mosquitto_strerror(rc) << std::endl;
            running = false;
        }
    }

    display_thread.join();
    mosquitto_disconnect(mosq);
    mosquitto_destroy(mosq);
    mosquitto_lib_cleanup();

    return 0;
}
