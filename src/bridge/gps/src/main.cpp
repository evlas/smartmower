#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include <iomanip>
#include <sstream>
#include "gps/gps_interface.h"
#include "gps/nmea_parser.h"
#include "mqtt/mqtt_client.h"
#include "mqtt/mqtt_topics.h"
#include "config/config_manager.h"
#include <nlohmann/json.hpp>

using namespace std::chrono_literals;

// Variabile atomica per gestire l'uscita pulita
std::atomic<bool> g_running{true};

// Gestore del segnale di interruzione
void signalHandler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "Ricevuto segnale di terminazione. Uscita..." << std::endl;
        g_running = false;
    }
}

int main(int argc, char* argv[]) {
    // Configura i gestori di segnale
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);
    
    std::cout << "Avvio GPS Bridge..." << std::endl;
    
    try {
        // Carica la configurazione
        auto config = createConfigManager();
        const char* config_path = "/opt/smartmower/etc/config/robot_config.json";
        if (!config->loadFromFile(config_path)) {
            std::cerr << "Errore nel caricamento della configurazione da " << config_path << std::endl;
            return 1;
        }
        
        // Inizializza i topic MQTT unificando root + base (coerenza con Pico/SLAM)
        {
            std::string root = config->getString("mqtt.root_topic", "smartmower");
            std::string gps_base = config->getString("mqtt.topics.gps.base", "gps");
            if (!root.empty() && root.back() == '/') root.pop_back();
            while (!gps_base.empty() && gps_base.front() == '/') gps_base.erase(gps_base.begin());
            std::string full_base = root + "/" + gps_base; // es. smartmower/gps
            MqttTopics::initialize(full_base);
        }
        
        // Crea il client MQTT
        mqtt::MqttClient mqtt(
            config->getString("mqtt.broker", "localhost"),
            config->getInt("mqtt.port", 1883),
            "gps_bridge",
            config->getString("mqtt.username", ""),
            config->getString("mqtt.password", "")
        );
        
        // Connetti al broker MQTT
        std::cout << "Connessione al broker MQTT..." << std::endl;
        
        if (!mqtt.connect()) {
            std::cerr << "Errore nella connessione al broker MQTT" << std::endl;
            return 1;
        }
        
        // Sottoscrizione ai topic di comando
        for (const auto& topic : MqttTopics::getSubscribedTopics()) {
            if (!mqtt.subscribe(topic)) {
                std::cerr << "Errore nella sottoscrizione al topic: " << topic << std::endl;
            }
        }
        
        // Crea l'interfaccia GPS con i parametri di configurazione
        auto gps = createGPSInterface(
            config->getString("gps_config.uart_device", "/dev/ttyAMA2"),
            config->getInt("gps_config.baudrate", 115200),
            config->getInt("gps_config.uart_timeout_ms", 1000),
            config->getInt("gps_config.max_satellites", 24)
        );
        
        // Configura il logging se specificato nella configurazione
        if (config->getBool("gps_logging.enabled", false)) {
            std::string log_file = config->getString("gps_logging.file", "/opt/smartmower/log/gps_bridge.log");
            // Qui andrebbe implementata la configurazione del logger
            std::cout << "Logging abilitato su: " << log_file << std::endl;
        }
        
        if (!gps || !gps->initialize()) {
            std::cerr << "Errore nell'inizializzazione del GPS" << std::endl;
            return 1;
        }
        
        std::cout << "GPS Bridge avviato con successo" << std::endl;
        
        NMEAParser nmea_parser;
        GPSData gps_data;
        auto last_status_update = std::chrono::steady_clock::now();
        const auto status_interval = std::chrono::seconds(30);
        
        std::cout << "In attesa di dati GPS..." << std::endl;
        
        // Loop principale
        while (g_running) {
            auto now = std::chrono::steady_clock::now();
            
            // Leggi i dati dal GPS
            std::string nmea_data = gps->readData();
            
            if (!nmea_data.empty()) {
                // Elabora ogni riga NMEA ricevuta
                std::istringstream stream(nmea_data);
                std::string line;
                
                while (std::getline(stream, line)) {
                    // Pulisci la riga da eventuali caratteri di ritorno a capo
                    line.erase(std::remove(line.begin(), line.end(), '\r'), line.end());
                    line.erase(std::remove(line.begin(), line.end(), '\n'), line.end());
                    
                    if (line.empty()) continue;
                    
                    try {
                        // Prova a fare il parsing della riga NMEA
                        if (nmea_parser.parse(line, gps_data)) {
                            // Costruisci un oggetto JSON con i dati GPS
                            nlohmann::json j;
                            j["timestamp"] = gps_data.timestamp;
                            j["latitude"] = gps_data.latitude;
                            j["longitude"] = gps_data.longitude;
                            j["altitude"] = gps_data.altitude;
                            j["hdop"] = gps_data.hdop;
                            j["pdop"] = gps_data.pdop;
                            j["vdop"] = gps_data.vdop;
                            j["speed"] = gps_data.speed;
                            j["course"] = gps_data.course;
                            j["satellites"] = gps_data.satellites;
                            j["fix_quality"] = gps_data.fix_quality;
                            j["status"] = gps_data.status;
                            
                            // Pubblica i dati GPS formattati in JSON
                            mqtt.publish(
                                MqttTopics::getTopic(MqttTopics::TOPIC_DATA),
                                j.dump(),
                                config->getInt("mqtt.qos", 1),
                                config->getBool("mqtt.retain", false)
                            );
                            
                            // Se è un fix valido, pubblica anche i dati in un formato più semplice
                            if (gps_data.fix_quality > 0) {
                                std::ostringstream simple_fix;
                                simple_fix << std::fixed << std::setprecision(6)
                                          << gps_data.latitude << "," << gps_data.longitude;
                                
                                mqtt.publish(
                                    MqttTopics::getTopic(MqttTopics::TOPIC_DATA) + "/simple",
                                    simple_fix.str(),
                                    config->getInt("mqtt.qos", 1),
                                    false
                                );
                            }
                        }
                    } catch (const std::exception& e) {
                        std::cerr << "Errore nel parsing NMEA: " << e.what() << std::endl;
                    }
                }
            }
            
            // Invia periodicamente lo stato
            if (now - last_status_update > status_interval) {
                nlohmann::json status;
                status["status"] = "online";
                status["timestamp"] = std::chrono::system_clock::to_time_t(
                    std::chrono::system_clock::now()
                );
                status["gps_connected"] = gps->isConnected();
                status["fix_quality"] = gps_data.fix_quality;
                status["satellites"] = gps_data.satellites;
                status["hdop"] = gps_data.hdop;
                status["pdop"] = gps_data.pdop;
                status["vdop"] = gps_data.vdop;
                
                mqtt.publish(
                    MqttTopics::getTopic(MqttTopics::TOPIC_STATUS),
                    status.dump(),
                    config->getInt("mqtt.qos", 1),
                    false
                );
                
                last_status_update = now;
            }
            
            // Gestisci i messaggi MQTT in arrivo
            mqtt.loop(100);
            
            // Piccola pausa per non sovraccaricare la CPU
            std::this_thread::sleep_for(10ms);
        }
        
        // Pulizia
        gps->shutdown();
        mqtt.disconnect();
        
    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << std::endl;
        return 1;
    }
    
    std::cout << "GPS Bridge terminato correttamente" << std::endl;
    return 0;
}
