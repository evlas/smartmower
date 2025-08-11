#include "sfm_obstacle_detector_fixed.h"
#include <cstring>
#include <csignal>
#include <fstream>

// Classe di utilità per la decodifica Base64
class Base64Decoder {
private:
    static const std::string base64_chars;
    
    static inline bool is_base64(unsigned char c) {
        return (isalnum(c) || (c == '+') || (c == '/'));
    }
    
public:
    static std::vector<unsigned char> decode(const std::string& encoded_string) {
        int in_len = encoded_string.size();
        int i = 0;
        int j = 0;
        int in_ = 0;
        unsigned char char_array_4[4], char_array_3[3];
        std::vector<unsigned char> ret;

        while (in_len-- && (encoded_string[in_] != '=') && is_base64(encoded_string[in_])) {
            char_array_4[i++] = encoded_string[in_]; in_++;
            if (i == 4) {
                for (i = 0; i < 4; i++)
                    char_array_4[i] = base64_chars.find(char_array_4[i]);

                char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
                char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
                char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

                for (i = 0; (i < 3); i++)
                    ret.push_back(char_array_3[i]);
                i = 0;
            }
        }

        if (i) {
            for (j = i; j < 4; j++)
                char_array_4[j] = 0;

            for (j = 0; j < 4; j++)
                char_array_4[j] = base64_chars.find(char_array_4[j]);

            char_array_3[0] = (char_array_4[0] << 2) + ((char_array_4[1] & 0x30) >> 4);
            char_array_3[1] = ((char_array_4[1] & 0xf) << 4) + ((char_array_4[2] & 0x3c) >> 2);
            char_array_3[2] = ((char_array_4[2] & 0x3) << 6) + char_array_4[3];

            for (j = 0; (j < i - 1); j++)
                ret.push_back(char_array_3[j]);
        }

        return ret;
    }
};

const std::string Base64Decoder::base64_chars = 
    "ABCDEFGHIJKLMNOPQRSTUVWXYZ"
    "abcdefghijklmnopqrstuvwxyz"
    "0123456789+/";

// Implementazione di FrameBuffer
void FrameBuffer::setFrame(const cv::Mat& frame, const std::chrono::system_clock::time_point& timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    frame.copyTo(frame_);
    timestamp_ = timestamp;
    has_new_frame_ = true;
    cv_.notify_one();
}

bool FrameBuffer::getFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp) {
    std::lock_guard<std::mutex> lock(mutex_);
    if (!has_new_frame_ || frame_.empty()) {
        return false;
    }
    frame_.copyTo(frame);
    timestamp = timestamp_;
    has_new_frame_ = false;
    return true;
}

// Costruttore
SfMObstacleDetector::SfMObstacleDetector() 
    : running_(false),       // Inizializza prima i membri atomici
      frame_count_(0),       // Poi i contatori
      mosq_(nullptr) {       // Infine i puntatori e altri membri
    // Inizializza la configurazione predefinita
    config_.mqtt_broker = "localhost";
    config_.mqtt_port = 1883;
    config_.mqtt_camera_topic = "smartmower/vision/camera/frame";
    config_.mqtt_velocity_topic = "smartmower/vision/velocity";
    config_.debug_enabled = true;
    
    // Inizializza le statistiche
    stats_.frames_processed = 0;
    stats_.obstacles_detected = 0;
    stats_.avg_processing_time = 0.0;
}

// Distruttore
SfMObstacleDetector::~SfMObstacleDetector() {
    stop();
}

// Carica la configurazione da file
bool SfMObstacleDetector::loadConfigFromFile(const std::string& filename) {
    std::ifstream config_file(filename);
    if (!config_file.is_open()) {
        std::cerr << "[ERROR] Impossibile aprire il file di configurazione: " << filename << std::endl;
        return false;
    }
    
    std::string json_str((std::istreambuf_iterator<char>(config_file)),
                        std::istreambuf_iterator<char>());
    
    cJSON* root = cJSON_Parse(json_str.c_str());
    if (!root) {
        std::cerr << "[ERROR] Errore nel parsing del file di configurazione JSON" << std::endl;
        return false;
    }
    
    // 1. Caricamento configurazione MQTT
    cJSON* mqtt = cJSON_GetObjectItem(root, "mqtt");
    if (mqtt) {
        std::cout << "[CONFIG] Trovata sezione MQTT nel file di configurazione" << std::endl;
        // Impostazioni di base MQTT
        cJSON* broker = cJSON_GetObjectItem(mqtt, "broker");
        cJSON* port = cJSON_GetObjectItem(mqtt, "port");
        cJSON* username = cJSON_GetObjectItem(mqtt, "username");
        cJSON* password = cJSON_GetObjectItem(mqtt, "password");
        
        // Usa i valori di default se non specificati
        config_.mqtt_broker = broker ? broker->valuestring : "localhost";
        config_.mqtt_port = port ? port->valueint : 1883;
        config_.mqtt_username = username ? username->valuestring : "mower";
        config_.mqtt_password = password ? password->valuestring : "smart";
        
        // Log dei parametri MQTT
        std::cout << "[CONFIG] Parametri MQTT:" << std::endl;
        std::cout << "  Broker: " << config_.mqtt_broker << ":" << config_.mqtt_port << std::endl;
        std::cout << "  Username: " << (config_.mqtt_username.empty() ? "[none]" : config_.mqtt_username) << std::endl;
        std::cout << "  Password: " << (config_.mqtt_password.empty() ? "[none]" : "[set]") << std::endl;
        
        // Configurazione dei topic
        cJSON* topics = cJSON_GetObjectItem(mqtt, "topics");
        if (topics) {
            cJSON* obstacle = cJSON_GetObjectItem(topics, "obstacle");
            if (obstacle) {
                // Topic base per obstacle
                cJSON* base_topic = cJSON_GetObjectItem(obstacle, "base_topic");
                if (base_topic) {
                    std::string base = base_topic->valuestring;
                    config_.mqtt_camera_topic = base + "/camera";
                    config_.mqtt_obstacles_topic = base + "/obstacles";
                }
                
                // QoS e altri parametri
                cJSON* qos = cJSON_GetObjectItem(obstacle, "qos");
                cJSON* retain = cJSON_GetObjectItem(obstacle, "retain");
                cJSON* pub_thresh = cJSON_GetObjectItem(obstacle, "publish_threshold");
                
                config_.mqtt_qos = qos ? qos->valueint : 1;
                config_.mqtt_retain = retain ? cJSON_IsTrue(retain) : false;
                config_.publish_threshold = pub_thresh ? pub_thresh->valuedouble : 0.08;
            }
        }
    }
    
    // 2. Caricamento configurazione telecamera
    cJSON* camera = cJSON_GetObjectItem(root, "camera");
    if (camera) {
        cJSON* height = cJSON_GetObjectItem(camera, "height");
        cJSON* intrinsics = cJSON_GetObjectItem(camera, "intrinsics");
        
        if (height) config_.camera_height = height->valuedouble;
        
        if (intrinsics) {
            cJSON* fx = cJSON_GetObjectItem(intrinsics, "focal_length_x");
            cJSON* fy = cJSON_GetObjectItem(intrinsics, "focal_length_y");
            cJSON* cx = cJSON_GetObjectItem(intrinsics, "principal_point_x");
            cJSON* cy = cJSON_GetObjectItem(intrinsics, "principal_point_y");
            
            if (fx) config_.focal_length_x = fx->valuedouble;
            if (fy) config_.focal_length_y = fy->valuedouble;
            if (cx) config_.principal_point_x = cx->valuedouble;
            if (cy) config_.principal_point_y = cy->valuedouble;
        }
    }
    
    // 3. Caricamento parametri di rilevamento
    cJSON* detection = cJSON_GetObjectItem(root, "detection");
    if (detection) {
        cJSON* max_range = cJSON_GetObjectItem(detection, "max_detection_range");
        cJSON* min_dist = cJSON_GetObjectItem(detection, "min_obstacle_distance");
        cJSON* min_points = cJSON_GetObjectItem(detection, "min_points_threshold");
        cJSON* disp_thresh = cJSON_GetObjectItem(detection, "displacement_threshold");
        
        if (max_range) config_.max_detection_range = max_range->valuedouble;
        if (min_dist) config_.min_obstacle_distance = min_dist->valuedouble;
        if (min_points) config_.min_points_threshold = min_points->valueint;
        if (disp_thresh) config_.displacement_threshold = disp_thresh->valuedouble;
        
        // Parametri SfM
        cJSON* sfm_params = cJSON_GetObjectItem(detection, "sfm_parameters");
        if (sfm_params) {
            cJSON* max_corners = cJSON_GetObjectItem(sfm_params, "max_corners");
            cJSON* quality = cJSON_GetObjectItem(sfm_params, "quality_level");
            cJSON* min_dist = cJSON_GetObjectItem(sfm_params, "min_distance");
            cJSON* block_size = cJSON_GetObjectItem(sfm_params, "block_size");
            cJSON* harris_k = cJSON_GetObjectItem(sfm_params, "harris_k");
            cJSON* min_frames = cJSON_GetObjectItem(sfm_params, "min_track_frames");
            cJSON* max_error = cJSON_GetObjectItem(sfm_params, "max_optical_flow_error");
            
            if (max_corners) config_.max_corners = max_corners->valueint;
            if (quality) config_.quality_level = quality->valuedouble;
            if (min_dist) config_.min_distance = min_dist->valuedouble;
            if (block_size) config_.block_size = block_size->valueint;
            if (harris_k) config_.harris_k = harris_k->valuedouble;
            if (min_frames) config_.min_frames_tracked = min_frames->valueint;
            if (max_error) config_.max_optical_flow_error = max_error->valuedouble;
        }
    }
    
    // 4. Configurazione di debug
    cJSON* debug = cJSON_GetObjectItem(root, "debug");
    if (debug) {
        cJSON* enabled = cJSON_GetObjectItem(debug, "enabled");
        // log_level non esiste nella struttura Config
        
        if (enabled) config_.debug_enabled = cJSON_IsTrue(enabled);
    }
    
    cJSON_Delete(root);
    return true;
}

// Inizializza il rilevatore
bool SfMObstacleDetector::initialize() {
    return setupMQTT();
}

// Configura la connessione MQTT
bool SfMObstacleDetector::setupMQTT() {
    // Inizializza la libreria Mosquitto
    mosquitto_lib_init();
    
    // Crea un'istanza del client MQTT
    mosq_ = mosquitto_new(nullptr, true, this);
    if (!mosq_) {
        std::cerr << "[ERROR] Failed to create MQTT client instance" << std::endl;
        return false;
    }
    
    // Imposta le callback
    mosquitto_connect_callback_set(mosq_, onMQTTConnectWrapper);
    mosquitto_message_callback_set(mosq_, onMQTTMessageWrapper);
    mosquitto_disconnect_callback_set(mosq_, onMQTTDisconnectWrapper);
    
    // Imposta le credenziali MQTT
    std::cout << "[DEBUG] Configurazione credenziali MQTT..." << std::endl;
    if (!config_.mqtt_username.empty() && !config_.mqtt_password.empty()) {
        std::cout << "[DEBUG] Tentativo di autenticazione con username: " << config_.mqtt_username << std::endl;
        int auth_result = mosquitto_username_pw_set(mosq_, 
                                                  config_.mqtt_username.c_str(), 
                                                  config_.mqtt_password.c_str());
        if (auth_result != MOSQ_ERR_SUCCESS) {
            std::cerr << "[ERROR] Impostazione credenziali MQTT fallita: " 
                     << mosquitto_strerror(auth_result) << " (codice: " << auth_result << ")" << std::endl;
            mosquitto_destroy(mosq_);
            mosq_ = nullptr;
            return false;
        } else {
            std::cout << "[DEBUG] Credenziali MQTT impostate correttamente" << std::endl;
        }
    } else {
        std::cout << "[DEBUG] Nessuna credenziale MQTT specificata, connessione anonima" << std::endl;
    }
    
    // Configura la connessione MQTT
    std::cout << "[MQTT] Configurazione connessione a " << config_.mqtt_broker 
              << ":" << config_.mqtt_port << "..." << std::endl;
    
    // Forza l'uso di IPv4
    mosquitto_int_option(mosq_, MOSQ_OPT_PROTOCOL_VERSION, MQTT_PROTOCOL_V31);
    
    // Configura il client per il ripristino automatico della connessione
    mosquitto_reconnect_delay_set(mosq_, 1, 30, true);
    
    std::cout << "[DEBUG] Connessione al broker MQTT..." << std::endl;
    
    // Imposta il numero massimo di tentativi di riconnessione
    mosquitto_max_inflight_messages_set(mosq_, 20);
    
    // Connessione al broker
    int result = mosquitto_connect(mosq_, config_.mqtt_broker.c_str(), 
                                 config_.mqtt_port, 60);
    
    if (result != MOSQ_ERR_SUCCESS) {
        std::cerr << "[ERROR] Connessione al broker MQTT fallita: " 
                 << mosquitto_strerror(result) << " (codice: " << result << ")" << std::endl;
        std::cerr << "[DEBUG] Dettagli connessione:" << std::endl;
        std::cerr << "  Broker: " << config_.mqtt_broker << std::endl;
        std::cerr << "  Porta: " << config_.mqtt_port << std::endl;
        std::cerr << "  Username: " << (config_.mqtt_username.empty() ? "[none]" : config_.mqtt_username) << std::endl;
        std::cerr << "  Password: " << (config_.mqtt_password.empty() ? "[none]" : "[set]") << std::endl;
        
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }
    
    // Piccolo ritardo per assicurare che la connessione sia stabile
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    
    // Sottoscrizione ai topic con gestione errori
    int max_retries = 3;
    int retry_delay_ms = 200;
    bool subscription_ok = false;
    
    for (int attempt = 1; attempt <= max_retries; ++attempt) {
        std::cout << "[MQTT] Tentativo di sottoscrizione " << attempt 
                 << "/" << max_retries << "..." << std::endl;
        
        int camera_result = mosquitto_subscribe(mosq_, nullptr, 
                                             config_.mqtt_camera_topic.c_str(), 1); // QoS 1
        
        int velocity_result = mosquitto_subscribe(mosq_, nullptr, 
                                               config_.mqtt_velocity_topic.c_str(), 1); // QoS 1
        
        if (camera_result == MOSQ_ERR_SUCCESS && velocity_result == MOSQ_ERR_SUCCESS) {
            std::cout << "[MQTT] Sottoscrizione ai topic completata con successo" << std::endl;
            std::cout << "[MQTT] Camera topic: " << config_.mqtt_camera_topic << std::endl;
            std::cout << "[MQTT] Velocity topic: " << config_.mqtt_velocity_topic << std::endl;
            subscription_ok = true;
            break;
        } else {
            std::cerr << "[ERROR] Tentativo " << attempt << " fallito: " 
                     << "camera_result=" << mosquitto_strerror(camera_result) << ", "
                     << "velocity_result=" << mosquitto_strerror(velocity_result) << std::endl;
            
            if (attempt < max_retries) {
                std::this_thread::sleep_for(std::chrono::milliseconds(retry_delay_ms));
                retry_delay_ms *= 2; // Backoff esponenziale
            }
        }
    }
    
    if (!subscription_ok) {
        std::cerr << "[ERROR] Impossibile sottoscriversi ai topic dopo " 
                 << max_retries << " tentativi" << std::endl;
        mosquitto_disconnect(mosq_);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
        return false;
    }
    
    // Avvia il loop MQTT in un thread separato
    mosquitto_loop_start(mosq_);
    
    std::cout << "[MQTT] Connesso al broker " << config_.mqtt_broker 
              << ":" << config_.mqtt_port << std::endl;
    
    return true;
}

// Avvia il rilevamento ostacoli
void SfMObstacleDetector::start() {
    if (running_.exchange(true)) {
        std::cerr << "[WARNING] SfMObstacleDetector is already running" << std::endl;
        return;
    }
    
    // Avvia il thread di elaborazione
    processing_thread_ = std::thread(&SfMObstacleDetector::processingLoop, this);
    
    if (config_.debug_enabled) {
        std::cout << "[START] SfMObstacleDetector started successfully" << std::endl;
    }
}

// Ferma il rilevamento ostacoli
void SfMObstacleDetector::stop() {
    if (!running_.exchange(false)) {
        return; // Già fermo
    }
    
    // Ferma il thread di elaborazione
    if (processing_thread_.joinable()) {
        processing_thread_.join();
    }
    
    // Disconnessione MQTT
    if (mosq_) {
        mosquitto_disconnect(mosq_);
        mosquitto_loop_stop(mosq_, true);
        mosquitto_destroy(mosq_);
        mosq_ = nullptr;
    }
    
    mosquitto_lib_cleanup();
    
    if (config_.debug_enabled) {
        std::cout << "[STOP] SfMObstacleDetector stopped" << std::endl;
    }
}

// Loop principale di elaborazione
void SfMObstacleDetector::processingLoop() {
    cv::Mat current_frame;
    std::chrono::system_clock::time_point frame_timestamp;
    bool first_frame = true;
    
    while (running_) {
        // Attendi un nuovo frame con un timeout
        if (frame_buffer_.waitForFrame(current_frame, frame_timestamp, std::chrono::milliseconds(100))) {
            try {
                if (first_frame) {
                    // Se è il primo frame, salvalo come frame precedente e attendi il prossimo
                    previous_frame_ = current_frame.clone();
                    previous_timestamp_ = frame_timestamp;
                    first_frame = false;
                } else {
                    // Elabora il frame con il frame precedente e il timestamp
                    processFrame(current_frame, frame_timestamp, previous_frame_, previous_timestamp_);
                    
                    // Aggiorna il frame precedente e il timestamp
                    previous_frame_ = current_frame.clone();
                    previous_timestamp_ = frame_timestamp;
                    
                    // Incrementa il contatore frame
                    frame_count_++;
                }
            } catch (const std::exception& e) {
                std::cerr << "[ERROR] Exception in processing loop: " << e.what() << std::endl;
            }
        } else if (!running_) {
            break; // Uscita pulita
        }
    }
}

// Elabora un singolo frame
void SfMObstacleDetector::processFrame(const cv::Mat& current_frame,
                                     const std::chrono::system_clock::time_point& current_timestamp,
                                     const cv::Mat& previous_frame,
                                     const std::chrono::system_clock::time_point& previous_timestamp) {
    // Silenzia i warning dei parametri non utilizzati
    (void)current_timestamp;
    (void)previous_frame;
    (void)previous_timestamp;
    // Implementa qui la logica di elaborazione del frame
    if (config_.debug_enabled && (frame_count_ % 30 == 0)) {
        std::cout << "[PROCESSING] Frame size: " << current_frame.cols << "x" << current_frame.rows 
                 << ", channels: " << current_frame.channels() << std::endl;
    }
    
    // Esempio: pubblica un messaggio MQTT con i risultati
    if (mosq_ && (frame_count_ % 10 == 0)) {
        // Implementa qui la logica di rilevamento ostacoli
        // ...
        
        // Pubblica i risultati
        publishDetectionResults();
    }
    
    frame_count_++;
}

// Pubblica i risultati del rilevamento
void SfMObstacleDetector::publishDetectionResults() {
    if (!mosq_) return;
    
    cJSON* root = cJSON_CreateObject();
    if (!root) return;
    
    cJSON_AddStringToObject(root, "type", "obstacle_detection");
    cJSON_AddNumberToObject(root, "frame_count", frame_count_);
    cJSON_AddNumberToObject(root, "timestamp", std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
    
    // Aggiungi qui i risultati del rilevamento
    // Aggiungi qui la logica per pubblicare gli ostacoli rilevati
    // cJSON* obstacles = cJSON_AddArrayToObject(root, "obstacles");
    (void)root;  // Silenzia il warning della variabile non utilizzata
    // Esempio: aggiungi ostacoli rilevati
    
    char* json_str = cJSON_PrintUnformatted(root);
    if (json_str) {
        mosquitto_publish(mosq_, nullptr, "smartmower/vision/obstacles", 
                         strlen(json_str), json_str, 0, false);
        free(json_str);
    }
    
    cJSON_Delete(root);
}

// Funzione principale
int main(int argc, char** argv) {
    try {
        // Crea il rilevatore di ostacoli
        SfMObstacleDetector detector;
        
        // Carica la configurazione se specificata come argomento
        if (argc > 1) {
            if (!detector.loadConfigFromFile(argv[1])) {
                std::cerr << "[ERROR] Failed to load configuration from " << argv[1] << std::endl;
                return 1;
            }
        }
        
        // Inizializza il rilevatore
        if (!detector.initialize()) {
            std::cerr << "[ERROR] Failed to initialize obstacle detector" << std::endl;
            return 1;
        }
        
        // Gestisci il segnale di terminazione
        std::signal(SIGINT, [](int) { 
            std::cout << "\nRicevuto segnale di interruzione, terminazione in corso..." << std::endl; 
            exit(0); 
        });
        
        // Avvia il rilevatore
        detector.start();
        
        std::cout << "SfM Obstacle Detector avviato. Premi Ctrl+C per terminare." << std::endl;
        
        // Attendi la terminazione
        while (true) {
            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "[FATAL ERROR] " << e.what() << std::endl;
        return 1;
    }
}

bool FrameBuffer::waitForFrame(cv::Mat& frame, std::chrono::system_clock::time_point& timestamp, 
                              std::chrono::milliseconds timeout) {
    std::unique_lock<std::mutex> lock(mutex_);
    if (cv_.wait_for(lock, timeout, [this] { return has_new_frame_; })) {
        if (!frame_.empty()) {
            frame_.copyTo(frame);
            timestamp = timestamp_;
            has_new_frame_ = false;
            return true;
        }
    }
    return false;
}

// Implementazione di VelocityManager
void VelocityManager::setVelocity(double velocity) {
    std::lock_guard<std::mutex> lock(mutex_);
    current_velocity_.store(velocity);
    last_update_ = std::chrono::system_clock::now();
}

double VelocityManager::getVelocity() const {
    return current_velocity_.load();
}

bool VelocityManager::isVelocityRecent(std::chrono::seconds max_age) const {
    std::lock_guard<std::mutex> lock(mutex_);
    auto now = std::chrono::system_clock::now();
    return (now - last_update_) < max_age;
}

// Implementazione delle funzioni MQTT
void SfMObstacleDetector::onMQTTConnectWrapper(struct mosquitto* mosq, void* userdata, int result) {
    SfMObstacleDetector* detector = static_cast<SfMObstacleDetector*>(userdata);
    if (detector) {
        detector->handleMQTTConnect(mosq, result);
    }
}

void SfMObstacleDetector::onMQTTMessageWrapper(struct mosquitto* /*mosq*/, void* userdata, 
                                             const struct mosquitto_message* message) {
    SfMObstacleDetector* detector = static_cast<SfMObstacleDetector*>(userdata);
    if (detector && message) {
        detector->onMQTTMessage(message);
    }
}

void SfMObstacleDetector::onMQTTDisconnectWrapper(struct mosquitto* /*mosq*/, void* userdata, int result) {
    SfMObstacleDetector* detector = static_cast<SfMObstacleDetector*>(userdata);
    if (detector) {
        detector->onMQTTDisconnect(result);
    }
}

void SfMObstacleDetector::handleMQTTConnect(struct mosquitto* mosq, int result) {
    if (result == 0) {
        // Connessione riuscita, la sottoscrizione viene ora gestita direttamente in setupMQTT
        std::cout << "[MQTT] Connessione al broker stabilita con successo" << std::endl;
    } else {
        std::cerr << "[ERROR] Connessione al broker MQTT fallita: " 
                 << mosquitto_strerror(result) << " (codice: " << result << ")" << std::endl;
    }
}

void SfMObstacleDetector::onMQTTDisconnect(int result) {
    if (config_.debug_enabled) {
        std::cout << "[MQTT] Disconnected from broker: " << mosquitto_strerror(result) << std::endl;
    }
}

void SfMObstacleDetector::onMQTTMessage(const struct mosquitto_message* message) {
    if (!message || !message->topic) {
        std::cerr << "[ERROR] Invalid MQTT message (null topic)" << std::endl;
        return;
    }
    
    std::string topic = message->topic;
    
    try {
        if (topic == config_.mqtt_camera_topic) {
            handleImageMessage(message);
        } else if (topic == config_.mqtt_velocity_topic) {
            handleVelocityMessage(message);
        } else if (config_.debug_enabled) {
            std::cout << "[DEBUG] Received message on topic: " << topic << std::endl;
        }
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Exception in MQTT message handler: " << e.what() << std::endl;
    }
}

void SfMObstacleDetector::handleVelocityMessage(const struct mosquitto_message* message) {
    if (!message || !message->payload || message->payloadlen <= 0) {
        std::cerr << "[ERROR] Invalid velocity message received" << std::endl;
        return;
    }
    
    std::string payload(static_cast<const char*>(message->payload), message->payloadlen);
    cJSON* root = cJSON_Parse(payload.c_str());
    
    if (!root) {
        std::cerr << "[ERROR] Failed to parse velocity JSON" << std::endl;
        return;
    }
    
    // Estrai i dati di velocità
    cJSON* type_json = cJSON_GetObjectItem(root, "type");
    if (cJSON_IsString(type_json) && strcmp(type_json->valuestring, "fusion_data") == 0) {
        cJSON* velocity_obj = cJSON_GetObjectItem(root, "velocity");
        if (velocity_obj) {
            // Prova a ottenere la velocità diretta
            cJSON* speed_json = cJSON_GetObjectItem(velocity_obj, "speed");
            if (cJSON_IsNumber(speed_json)) {
                double speed = speed_json->valuedouble;
                velocity_manager_.setVelocity(speed);
                if (config_.debug_enabled) {
                    std::cout << "[VELOCITY] Set speed: " << speed << " m/s" << std::endl;
                }
            } else {
                // Fallback: calcola la velocità dalle componenti vx, vy
                cJSON* vx_json = cJSON_GetObjectItem(velocity_obj, "vx");
                cJSON* vy_json = cJSON_GetObjectItem(velocity_obj, "vy");
                
                if (cJSON_IsNumber(vx_json) && cJSON_IsNumber(vy_json)) {
                    double vx = vx_json->valuedouble;
                    double vy = vy_json->valuedouble;
                    double speed = sqrt(vx * vx + vy * vy);
                    velocity_manager_.setVelocity(speed);
                    if (config_.debug_enabled) {
                        std::cout << "[VELOCITY] Calculated speed from vx/vy: " << speed << " m/s" << std::endl;
                    }
                } else {
                    std::cerr << "[ERROR] No valid velocity components in fusion data" << std::endl;
                }
            }
        } else {
            std::cerr << "[ERROR] No velocity object in fusion data" << std::endl;
        }
    } else if (config_.debug_enabled) {
        std::cout << "[VELOCITY] Ignoring non-fusion message type" << std::endl;
    }
    
    cJSON_Delete(root);
}

void SfMObstacleDetector::handleImageMessage(const struct mosquitto_message* message) {
    if (!message || !message->payload || message->payloadlen <= 0) {
        std::cerr << "[ERROR] Invalid image message received" << std::endl;
        return;
    }
    
    std::string payload(static_cast<const char*>(message->payload), message->payloadlen);
    cJSON* root = cJSON_Parse(payload.c_str());
    
    if (!root) {
        std::cerr << "[ERROR] Failed to parse image JSON" << std::endl;
        return;
    }
    
    // Estrai i dati dell'immagine
    cJSON* image_data = cJSON_GetObjectItem(root, "image_data");
    if (!cJSON_IsString(image_data)) {
        std::cerr << "[ERROR] No image_data field in message" << std::endl;
        cJSON_Delete(root);
        return;
    }
    
    // Decodifica l'immagine base64
    std::string base64_data = image_data->valuestring;
    std::vector<uchar> image_buffer = Base64Decoder::decode(base64_data);
    
    if (image_buffer.empty()) {
        std::cerr << "[ERROR] Failed to decode base64 image data" << std::endl;
        cJSON_Delete(root);
        return;
    }
    
    // Decodifica l'immagine in formato OpenCV
    cv::Mat frame = cv::imdecode(image_buffer, cv::IMREAD_COLOR);
    
    if (!frame.empty()) {
        // Salva il frame nel buffer per l'elaborazione
        frame_buffer_.setFrame(frame, std::chrono::system_clock::now());
        
        // Log di debug ogni 30 frame
        if (config_.debug_enabled && (frame_count_++ % 30 == 0)) {
            std::cout << "[DEBUG] Processed frame: " << frame.cols << "x" << frame.rows 
                     << " channels: " << frame.channels() << std::endl;
        }
    } else {
        std::cerr << "[ERROR] Failed to decode image data" << std::endl;
        
        // Salva i dati grezzi per il debug in caso di errore
        if (config_.debug_enabled) {
            std::ofstream out("/tmp/last_received_frame.bin", std::ios::binary);
            if (out.is_open()) {
                out.write(reinterpret_cast<const char*>(image_buffer.data()), image_buffer.size());
                out.close();
                std::cerr << "[DEBUG] Raw image data saved to /tmp/last_received_frame.bin (" 
                         << image_buffer.size() << " bytes)" << std::endl;
            } else {
                std::cerr << "[ERROR] Failed to save debug image data" << std::endl;
            }
        }
    }
    
    cJSON_Delete(root);
}
