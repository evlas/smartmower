#ifndef GPS_INTERFACE_H
#define GPS_INTERFACE_H

#include <string>
#include <memory>

class GPSInterface {
public:
    virtual ~GPSInterface() = default;
    
    // Inizializza la connessione GPS
    virtual bool initialize() = 0;
    
    // Legge i dati GPS grezzi
    virtual std::string readData() = 0;
    
    // Verifica se il GPS è connesso e funzionante
    virtual bool isConnected() const = 0;
    
    // Chiude la connessione GPS
    virtual void shutdown() = 0;
};

// Factory function per creare l'istanza corretta del GPS
std::unique_ptr<GPSInterface> createGPSInterface(
    const std::string& device, 
    int baud_rate,
    int timeout_ms = 1000, 
    int max_satellites = 24
);

#endif // GPS_INTERFACE_H
