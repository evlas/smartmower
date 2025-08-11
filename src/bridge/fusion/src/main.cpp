#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <unistd.h>  // Per getpid()
#include "fusion/sensor_fusion.h"
#include "config/config_manager.h"
#include "mqtt/mqtt_client.h"

using namespace std::chrono_literals;

// Variabile atomica per gestire l'uscita pulita
std::atomic<bool> g_running{true};

// Gestore del segnale di interruzione
void signal_handler(int signal) {
    if (signal == SIGINT || signal == SIGTERM) {
        std::cout << "Ricevuto segnale di terminazione. Uscita..." << std::endl;
        g_running = false;
    }
}

int main(int argc, char* argv[]) {
    // Configura i gestori di segnale
    std::signal(SIGINT, signal_handler);
    std::signal(SIGTERM, signal_handler);
    
    std::cout << "Avvio Fusion Bridge..." << std::endl;
    
    try {
        // Carica la configurazione
        std::shared_ptr<fusion::config::ConfigManager> config(fusion::config::createConfigManager().release());
        const char* config_path = "/opt/smartmower/etc/config/robot_config.json";
        if (!config->loadFromFile(config_path)) {
            std::cerr << "Errore nel caricamento della configurazione da " << config_path << std::endl;
            return 1;
        }
        
        // Crea il client MQTT
        std::string broker = config->getString("mqtt.broker", "localhost");
        int port = config->getInt("mqtt.port", 1883);
        std::string client_id = "fusion_bridge_" + std::to_string(getpid());
        std::string username = config->getString("mqtt.username", "");
        std::string password = config->getString("mqtt.password", "");
        
        auto mqtt_client = std::make_shared<fusion::mqtt::MqttClient>(
            broker, port, client_id, username, password
        );
        
        if (!mqtt_client->connect()) {
            std::cerr << "Impossibile connettersi al broker MQTT" << std::endl;
            return 1;
        }
        
        // Crea e avvia il modulo di fusione
        fusion::SensorFusion fusion(config, mqtt_client);
        if (!fusion.initialize()) {
            std::cerr << "Errore nell'inizializzazione del modulo di fusione" << std::endl;
            return 1;
        }
        
        fusion.start();
        std::cout << "Fusion Bridge avviato con successo" << std::endl;
        
        // Loop principale
        while (g_running) {
            std::this_thread::sleep_for(100ms);
        }
        
        std::cout << "Arresto del Fusion Bridge..." << std::endl;
        fusion.stop();
        
    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
