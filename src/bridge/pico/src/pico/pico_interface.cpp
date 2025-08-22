#include "pico/pico_interface.h"
#include "pico/serial_interface.h"
#include "pico/pico_protocol.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <sstream>
#include <cstring>
#include <algorithm>
#include <cmath>

using json = nlohmann::json;

namespace pico {

PicoInterface::PicoInterface(const std::shared_ptr<ConfigManager>& config)
    : config_(config) {
    // Inizializza qui i membri
}

PicoInterface::~PicoInterface() {
    shutdown();
}

bool PicoInterface::initialize() {
    if (!initializeMqtt()) {
        std::cerr << "Errore nell'inizializzazione di MQTT" << std::endl;
        return false;
    }
    
    if (!initializeSerial()) {
        std::cerr << "Errore nell'inizializzazione della porta seriale" << std::endl;
        return false;
    }
    
    // Avvia il thread di elaborazione
    running_ = true;
    process_thread_ = std::thread(&PicoInterface::processThread, this);
    
    return true;
}

void PicoInterface::run() {
    // Loop principale
    while (running_) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        
        // Qui puoi aggiungere logica periodica se necessario
        if (messages_received_ > 0 && messages_received_ % 100 == 0) {
            std::cout << "Messaggi ricevuti: " << messages_received_ 
                      << ", pubblicati: " << messages_published_ << std::endl;
        }
    }
}

void PicoInterface::shutdown() {
    running_ = false;
    cv_.notify_all();
    
    if (process_thread_.joinable()) {
        process_thread_.join();
    }
    
    if (serial_) {
        serial_->close();
    }
    
    if (mqtt_client_) {
        mqtt_client_->disconnect();
    }
}

bool PicoInterface::initializeMqtt() {
    try {
        std::string broker = config_->getString("mqtt.broker", "localhost");
        int port = config_->getInt("mqtt.port", 1883);
        std::string client_id = "pico_bridge";
        
        mqtt_client_ = std::make_unique<mqtt::MqttClient>(
            broker,
            port,
            client_id,
            config_->getString("mqtt.username", ""),
            config_->getString("mqtt.password", "")
        );
        
        // Sottoscrizione ai topic di comando
        std::string root = config_->getString("mqtt.root_topic", "smartmower");
        std::string base = config_->getString("mqtt.topics.pico.base", "bridge/pico");
        std::string base_topic = root + "/" + base;
        std::string commands_topic = base_topic + "/commands/#";
        
        // Usa un wrapper per la callback MQTT
        auto mqtt_cb = [this](const std::string& topic, const std::string& payload) {
            this->processMqttMessage(topic, payload);
        };
        
        // Registra la callback
        mqtt_callback_ = mqtt_cb;
        
        // Iscriviti al topic con QoS 1
        mqtt_client_->subscribe(commands_topic, 1);
        
        return mqtt_client_->connect();
    } catch (const std::exception& e) {
        std::cerr << "Errore MQTT: " << e.what() << std::endl;
        return false;
    }
}

bool PicoInterface::initializeSerial() {
    try {
        // Usa pico_config.uart_device invece di pico.serial.device per allinearsi al file di configurazione
        std::string device = config_->getString("pico_config.uart_device", "/dev/ttyAMA1");
        int baudrate = config_->getInt("pico_config.baudrate", 115200);
        int timeout_ms = config_->getInt("pico_config.uart_timeout_ms", 1000);
        
        serial_ = std::make_unique<SerialInterface>(device, baudrate, timeout_ms);
        
        // Imposta il callback per i dati ricevuti
        serial_callback_ = [this](const std::vector<uint8_t>& data) {
            if (!data.empty()) {
                this->processReceivedData(data.data(), data.size());
            }
        };
        serial_->setDataCallback(serial_callback_);
        
        return serial_->open();
    } catch (const std::exception& e) {
        std::cerr << "Errore inizializzazione seriale: " << e.what() << std::endl;
        return false;
    }
}

void PicoInterface::processMqttMessage(const std::string& topic, const std::string& payload) {
    try {
        std::cout << "Comando ricevuto su " << topic << ": " << payload << std::endl;
        
        // Estrai il comando dal topic
        std::string root = config_->getString("mqtt.root_topic", "smartmower");
        std::string base = config_->getString("mqtt.topics.pico.base", "bridge/pico");
        std::string base_topic = root + "/" + base;
        std::string command = topic.substr(base_topic.length() + 9); // +9 per "/commands/"
        
        if (command == "motors") {
            auto json = json::parse(payload);
            MotorCommand cmd{};
            cmd.type = static_cast<uint8_t>(MessageType::MOTOR_COMMAND);
            cmd.left_speed = json.value("left", 0.0f);
            cmd.right_speed = json.value("right", 0.0f);
            cmd.blade1_speed = json.value("blade1", 0.0f);
            cmd.blade2_speed = json.value("blade2", 0.0f);
            
            if (!sendMotorCommand(cmd)) {
                std::cerr << "Errore nell'invio del comando motori" << std::endl;
            }
        } else if (command == "system") {
            auto json = json::parse(payload);
            std::string action = json.value("action", "");
            
            SystemCommand cmd{};
            cmd.type = static_cast<uint8_t>(MessageType::SYSTEM_COMMAND);
            
            if (action == "emergency_stop") {
                cmd.command_id = static_cast<uint8_t>(SystemCommandId::EMERGENCY_STOP);
                cmd.value = 0.0f;
            } else if (action == "reset" || action == "estop_reset") {
                cmd.command_id = static_cast<uint8_t>(SystemCommandId::RESET);
                cmd.value = 0.0f;
            } else if (action == "set_relay") {
                cmd.command_id = static_cast<uint8_t>(SystemCommandId::SET_RELAY);
                cmd.value = json.value("value", 0.0f);
            } else if (action == "calibrate") {
                cmd.command_id = static_cast<uint8_t>(SystemCommandId::CALIBRATE);
                cmd.value = 0.0f;
            } else if (action == "reset_encoders") {
                cmd.command_id = static_cast<uint8_t>(SystemCommandId::RESET_ENCODERS);
                cmd.value = 0.0f;
            } else {
                std::cerr << "Comando di sistema non riconosciuto: " << action << std::endl;
                return;
            }
            
            if (!sendSystemCommand(cmd)) {
                std::cerr << "Errore nell'invio del comando di sistema" << std::endl;
            }
        }
    } catch (const std::exception& e) {
        std::cerr << "Errore nell'elaborazione del messaggio MQTT: " << e.what() << std::endl;
    }
}

void PicoInterface::processReceivedData(const uint8_t* data, size_t length) {
    std::lock_guard<std::mutex> lock(buffer_mutex_);
    
    // Aggiungi i nuovi dati al buffer
    size_t old_size = receive_buffer_.size();
    receive_buffer_.resize(old_size + length);
    std::memcpy(receive_buffer_.data() + old_size, data, length);
    
    // Notifica il thread di elaborazione
    cv_.notify_one();
}

void PicoInterface::processThread() {
    while (running_) {
        std::unique_lock<std::mutex> lock(buffer_mutex_);
        
        // Attendi nuovi dati o l'uscita
        cv_.wait_for(lock, std::chrono::milliseconds(100), [this] {
            return !receive_buffer_.empty() || !running_;
        });
        
        if (!running_) break;
        
        // Elabora i dati ricevuti con framing: [SOF][LENlo][LENhi][PAYLOAD][CHK]
        while (receive_buffer_.size() >= 3) { // almeno SOF + LEN
            // Cerca SOF
            auto it = std::find(receive_buffer_.begin(), receive_buffer_.end(), PICO_SOF);
            if (it == receive_buffer_.end()) {
                receive_buffer_.clear();
                break;
            }
            // Scarta bytes prima di SOF
            if (it != receive_buffer_.begin()) {
                receive_buffer_.erase(receive_buffer_.begin(), it);
                if (receive_buffer_.size() < 3) break;
            }
            // Ora buffer[0] = SOF, controlla lunghezza disponibile
            if (receive_buffer_.size() < 3) break;
            uint16_t frame_len = static_cast<uint16_t>(receive_buffer_[1]) |
                                 (static_cast<uint16_t>(receive_buffer_[2]) << 8);
            size_t total_needed = 1 + 2 + frame_len; // SOF+LEN+payload+chk
            if (receive_buffer_.size() < total_needed) {
                // Attendi altri byte
                break;
            }
            // Puntatore al payload e lunghezza del payload+chk
            const uint8_t* payload = receive_buffer_.data() + 3;
            size_t payload_plus_chk = frame_len;
            if (payload_plus_chk < 2) {
                // Lunghezza non valida, scarta SOF
                receive_buffer_.erase(receive_buffer_.begin());
                continue;
            }
            // Valida checksum sul payload (ultimo byte è chk)
            if (!ProtocolParser::validateChecksum(payload, payload_plus_chk)) {
                // frame corrotto, scarta SOF e ricomincia
                receive_buffer_.erase(receive_buffer_.begin());
                continue;
            }
            // Deserializza dal payload (include checksum alla fine)
            MessageType msg_type;
            uint8_t msg_buffer[256];
            if (ProtocolParser::deserializeMessage(payload, payload_plus_chk, msg_type, msg_buffer)) {
                switch (msg_type) {
                    case MessageType::SENSOR_DATA: {
                        auto* data = reinterpret_cast<SensorData*>(msg_buffer);
                        publishSensorData(*data);
                        messages_received_++;
                        break;
                    }
                    case MessageType::STATUS_REPORT: {
                        auto* report = reinterpret_cast<StatusReport*>(msg_buffer);
                        publishStatusReport(*report);
                        messages_received_++;
                        break;
                    }
                    default:
                        std::cerr << "Tipo di messaggio non gestito: " << static_cast<int>(msg_type) << std::endl;
                        break;
                }
            }
            // Rimuovi l'intero frame dal buffer
            receive_buffer_.erase(receive_buffer_.begin(), receive_buffer_.begin() + total_needed);
        }
    }
}

bool PicoInterface::sendMotorCommand(const MotorCommand& cmd) {
    if (!serial_ || !serial_->isOpen()) {
        std::cerr << "Errore: connessione seriale non disponibile" << std::endl;
        return false;
    }
    
    // Serializza payload (struct + checksum)
    uint8_t payload[256];
    size_t payload_len = ProtocolParser::serializeMotorCommand(cmd, payload, sizeof(payload));
    
    if (payload_len == 0) {
        std::cerr << "Errore nella serializzazione del comando motori" << std::endl;
        return false;
    }
    // Incapsula con framing
    uint8_t frame[260];
    if (payload_len + 3 > sizeof(frame)) return false;
    frame[0] = PICO_SOF;
    frame[1] = static_cast<uint8_t>(payload_len & 0xFF);
    frame[2] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
    std::memcpy(frame + 3, payload, payload_len);
    size_t frame_len = 3 + payload_len;
    ssize_t written = serial_->write(frame, frame_len);
    return written == static_cast<ssize_t>(frame_len);
}

bool PicoInterface::sendSystemCommand(const SystemCommand& cmd) {
    if (!serial_ || !serial_->isOpen()) {
        std::cerr << "Errore: connessione seriale non disponibile" << std::endl;
        return false;
    }
    
    uint8_t payload[256];
    size_t payload_len = ProtocolParser::serializeSystemCommand(cmd, payload, sizeof(payload));
    
    if (payload_len == 0) {
        std::cerr << "Errore nella serializzazione del comando di sistema" << std::endl;
        return false;
    }
    uint8_t frame[260];
    if (payload_len + 3 > sizeof(frame)) return false;
    frame[0] = PICO_SOF;
    frame[1] = static_cast<uint8_t>(payload_len & 0xFF);
    frame[2] = static_cast<uint8_t>((payload_len >> 8) & 0xFF);
    std::memcpy(frame + 3, payload, payload_len);
    size_t frame_len = 3 + payload_len;
    ssize_t written = serial_->write(frame, frame_len);
    return written == static_cast<ssize_t>(frame_len);
}

void PicoInterface::publishSensorData(const SensorData& data) {
    if (!mqtt_client_) return;
    
    try {
        std::string root = config_->getString("mqtt.root_topic", "smartmower");
        std::string base = config_->getString("mqtt.topics.pico.base", "bridge/pico");
        std::string base_topic = root + "/" + base;
        
        // Pubblica i dati aggregati su .../data con anche tilt e safety
        json data_json;
        data_json["timestamp"] = data.timestamp;

        json accel = json::object();
        accel["x"] = data.accel[0];
        accel["y"] = data.accel[1];
        accel["z"] = data.accel[2];
        data_json["accel"] = accel;

        json gyro = json::object();
        gyro["x"] = data.gyro[0];
        gyro["y"] = data.gyro[1];
        gyro["z"] = data.gyro[2];
        data_json["gyro"] = gyro;

        json mag = json::object();
        mag["x"] = data.mag[0];
        mag["y"] = data.mag[1];
        mag["z"] = data.mag[2];
        data_json["mag"] = mag;

        json us = json::object();
        us["left"] = data.us_distances[0];
        us["center"] = data.us_distances[1];
        us["right"] = data.us_distances[2];
        data_json["ultrasonic"] = us;

        json power = json::object();
        power["bus_voltage"] = data.bus_voltage;
        power["current"] = data.current;
        data_json["power"] = power;

        // Calcolo tilt (solo booleano richiesto nel payload finale)
        float pitch = atan2(-data.accel[0], sqrt(data.accel[1] * data.accel[1] + data.accel[2] * data.accel[2]));
        float roll = atan2(data.accel[1], data.accel[2]);
        pitch = pitch * 180.0f / M_PI;
        roll = roll * 180.0f / M_PI;
        float tilt_angle_limit = config_->getDouble("sensors.tilt_angle_limit", 30.0);
        bool is_tilted = (std::abs(pitch) > tilt_angle_limit || std::abs(roll) > tilt_angle_limit);

        // Safety flags
        bool emergency = data.safety_flags[0] != 0;
        bool rain = data.safety_flags[1] != 0;
        bool bumper = data.safety_flags[2] != 0;
        bool lift = data.safety_flags[3] != 0;
        json safety = json::object();
        safety["emergency"] = emergency;
        safety["rain"] = rain;
        safety["bumper"] = bumper;
        safety["lift"] = lift;
        safety["tilt"] = is_tilted;
        data_json["safety"] = safety;

        mqtt_client_->publish(base_topic + "/data", data_json.dump());

        // Aggiorna il contatore per un solo messaggio pubblicato
        messages_published_ += 1;
    } catch (const std::exception& e) {
        std::cerr << "Errore nella pubblicazione dei dati sensori: " << e.what() << std::endl;
    }
}

void PicoInterface::publishStatusReport(const StatusReport& report) {
    if (!mqtt_client_) return;
    
    try {
        json j;
        j["timestamp"] = report.timestamp;
        
        // Dati motori
        json motors = json::array();
        for (int i = 0; i < 4; ++i) {
            json motor;
            motor["speed"] = report.motor_speeds[i];
            motor["rpm"] = report.motor_rpm[i];
            motor["encoder"] = report.encoder_counts[i];
            motors.push_back(motor);
        }
        j["motors"] = motors;
        
        // Flag di sistema e stato relè
        j["system_flags"] = report.system_flags;
        j["relay_state"] = (report.relay_state != 0);
        
        // Pubblica sul topic appropriato
        std::string root = config_->getString("mqtt.root_topic", "smartmower");
        std::string base = config_->getString("mqtt.topics.pico.base", "bridge/pico");
        std::string base_topic = root + "/" + base;
        std::string topic = base_topic + "/status";
        
        mqtt_client_->publish(topic, j.dump());
        messages_published_++;
    } catch (const std::exception& e) {
        std::cerr << "Errore nella pubblicazione del report di stato: " << e.what() << std::endl;
    }
}

std::string PicoInterface::getCurrentTimeString() const {
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y-%m-%d %X");
    return ss.str();
}

} // namespace pico
