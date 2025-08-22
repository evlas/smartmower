#include "slam/mqtt_slam_node.h"
#include <iostream>
#include <csignal>
#include <cstdlib>

std::unique_ptr<slam::MqttSlamNode> slam_node;

void signalHandler(int signum) {
    std::cout << "Interrupt signal (" << signum << ") received.\n";
    if (slam_node) {
        slam_node->stop();
    }
    exit(signum);
}

int main(int argc, char** argv) {
    // Gestisci i segnali di interruzione
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);
    
    try {
        // Percorso del file di configurazione di default
        std::string config_file = "/opt/smartmower/etc/config/robot_config.json";
        
        // Se viene specificato un file di configurazione come argomento
        if (argc > 1) {
            config_file = argv[1];
        } else {
            std::cout << "Using default config file: " << config_file << std::endl;
        }
        
        std::cout << "Starting SLAM node with config: " << config_file << std::endl;
        
        // Crea e avvia il nodo SLAM
        slam_node = std::make_unique<slam::MqttSlamNode>(config_file);
        slam_node->run();
        
        return 0;
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << std::endl;
        return 1;
    }
}
