#include <csignal>
#include <iostream>
#include <memory>
#include <atomic>
#include <chrono>
#include <vector>
#include <string>
#include <cstdlib>  // for getenv
#include <sys/stat.h>  // for stat
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
        
        // Lista di percorsi di configurazione da provare in ordine di priorità
        std::vector<std::string> config_paths = {
            "/opt/smartmower/etc/config/robot_config.json",  // Percorso di produzione
            std::string(getenv("HOME")) + "/smartmower_config/robot_config.json",  // Percorso di sviluppo
            "./robot_config.json"  // Percorso locale per test
        };
        
        bool config_loaded = false;
        for (const auto& config_path : config_paths) {
            std::cout << "Tentativo di caricamento configurazione da: " << config_path << std::endl;
            if (config->loadFromFile(config_path.c_str())) {
                std::cout << "Configurazione caricata con successo da: " << config_path << std::endl;
                config_loaded = true;
                break;
            } else {
                std::cerr << "Impossibile caricare la configurazione da: " << config_path << std::endl;
                // Mostra i permessi del file
                struct stat file_stat;
                if (stat(config_path.c_str(), &file_stat) == 0) {
                    std::cerr << "Permessi file: " << std::oct << (file_stat.st_mode & 0777) << std::dec << std::endl;
                    std::cerr << "Proprietario: " << file_stat.st_uid << ", Gruppo: " << file_stat.st_gid << std::endl;
                } else {
                    std::cerr << "File non trovato o accesso negato" << std::endl;
                }
            }
        }
        
        if (!config_loaded) {
            std::cerr << "Errore: impossibile caricare la configurazione da nessun percorso noto" << std::endl;
            std::cerr << "Verifica che il file di configurazione esista e abbia i permessi corretti" << std::endl;
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
