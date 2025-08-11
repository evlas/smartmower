#include <iostream>
#include <memory>
#include <csignal>
#include <atomic>
#include <thread>
#include <chrono>
#include "pico/pico_interface.h"
#include "config/config_manager.h"

using namespace std::chrono_literals;

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
    
    std::cout << "Avvio Pico Bridge..." << std::endl;
    
    try {
        // Carica la configurazione
        auto config = createConfigManager();
        const char* config_path = "/opt/smartmower/etc/config/robot_config.json";
        if (!config->loadFromFile(config_path)) {
            std::cerr << "Errore nel caricamento della configurazione da " << config_path << std::endl;
            return 1;
        }
        
        // Crea e avvia l'interfaccia Pico
        pico::PicoInterface pico_interface(std::move(config));
        if (!pico_interface.initialize()) {
            std::cerr << "Errore nell'inizializzazione del Pico Bridge" << std::endl;
            return 1;
        }
        
        std::cout << "Pico Bridge avviato con successo" << std::endl;
        
        // Loop principale
        while (g_running) {
            pico_interface.run();
            std::this_thread::sleep_for(100ms);
        }
        
        std::cout << "Spegnimento Pico Bridge..." << std::endl;
        
    } catch (const std::exception& e) {
        std::cerr << "Errore: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}
