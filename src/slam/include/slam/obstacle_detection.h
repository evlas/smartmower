#ifndef OBSTACLE_DETECTION_H
#define OBSTACLE_DETECTION_H

#include <vector>
#include <deque>
#include <memory>
#include <Eigen/Dense>
#include <pico/pico_protocol.h>
#include "slam/config.h"

namespace slam {

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;

/**
 * @brief Struttura che rappresenta un ostacolo rilevato
 */
struct DetectedObstacle {
    Vector2d position;      // Posizione dell'ostacolo nel frame del robot (x, y)
    double distance;        // Distanza dall'ostacolo in metri
    double confidence;      // Affidabilità della rilevazione [0-1]
    uint64_t timestamp;     // Timestamp della rilevazione
};

/**
 * @brief Classe per il rilevamento ostacoli con sensori ad ultrasuoni
 */
class ObstacleDetector {
public:
    /**
     * @brief Costruttore con configurazione
     * @param config Configurazione del rilevamento ostacoli
     */
    explicit ObstacleDetector(const std::shared_ptr<SlamConfig>& config);
    
    /**
     * @brief Costruttore con parametri espliciti (solo per test)
     */
    ObstacleDetector(size_t window_size, double min_distance, double max_distance);

    /**
     * @brief Processa i dati dei sensori ad ultrasuoni
     * @param sensor_data Dati dei sensori dal modulo Pico
     * @param robot_pose Posa corrente del robot (x, y, theta)
     * @return Vettore di ostacoli rilevati
     */
    std::vector<DetectedObstacle> processUltrasonicData(
        const pico::SensorData& sensor_data,
        const Vector3d& robot_pose);

    /**
     * @brief Verifica se è presente un ostacolo in una certa direzione
     * @param angle Angolo rispetto alla direzione del robot (radianti)
     * @param max_distance Distanza massima di controllo
     * @return True se viene rilevato un ostacolo, false altrimenti
     */
    bool isObstacleInDirection(double angle, double max_distance) const;

    /**
     * @brief Ottiene la distanza minima rilevata in una certa direzione
     * @param angle Angolo rispetto alla direzione del robot (radianti)
     * @param max_distance Distanza massima di controllo
     * @return Distanza minima rilevata o max_distance se nessun ostacolo
     */
    double getObstacleDistance(double angle, double max_distance) const;

    /**
     * @brief Aggiorna la mappa degli ostacoli
     * @param obstacles Ostacoli rilevati da altri sensori (es. telecamera)
     */
    void updateObstacleMap(const std::vector<DetectedObstacle>& obstacles);

    /**
     * @brief Pulisce la mappa degli ostacoli
     */
    void clearObstacleMap();

private:
    // Struttura per memorizzare le letture filtrate degli ultrasuoni
    struct UltrasonicReading {
        std::deque<double> distances;  // Finestra mobile di letture
        double filtered_distance;      // Distanza filtrata
        double variance;               // Varianza delle letture
    };

    // Posizioni dei sensori rispetto al centro del robot (x, y in metri)
    static constexpr double SENSOR_POSITIONS[3][2] = {
        {0.2, 0.15},   // Sensore sinistro (x, y)
        {0.3, 0.0},    // Sensore centrale
        {0.2, -0.15}   // Sensore destro
    };

    // Angoli di puntamento dei sensori (radianti)
    static constexpr double SENSOR_ANGLES[3] = {
        M_PI / 6.0,    // 30 gradi a sinistra
        0.0,           // Dritto
        -M_PI / 6.0    // 30 gradi a destra
    };

    // Configurazione
    std::shared_ptr<SlamConfig> config_;
    
    // Parametri di configurazione (copia locale per efficienza)
    size_t window_size_;      // Dimensione finestra per filtraggio
    double min_distance_;     // Distanza minima di sicurezza
    double max_distance_;     // Distanza massima di rilevamento
    double obstacle_timeout_; // Timeout per la rimozione degli ostacoli

    // Stato interno
    UltrasonicReading sensors_[3];  // Dati dei tre sensori
    std::vector<DetectedObstacle> obstacle_map_;  // Mappa degli ostacoli

    /**
     * @brief Filtra una nuova lettura con media mobile
     */
    void filterReading(UltrasonicReading& reading, double new_distance);

    /**
     * @brief Converte le letture degli ultrasuoni in ostacoli
     */
    std::vector<DetectedObstacle> convertToObstacles(
        const pico::SensorData& sensor_data,
        const Vector3d& robot_pose) const;
};

} // namespace slam

#endif // OBSTACLE_DETECTION_H
