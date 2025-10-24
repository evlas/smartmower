/**
 * @file bno055.hpp
 * @brief Driver minimale per sensore IMU Bosch BNO055 su RP2040 tramite I2C.
 *
 * @details Fornisce API bloccanti per inizializzazione e lettura burst dei dati
 * quaternion, accelerometro e giroscopio. Le unità sono conformi alla
 * configurazione impostata in `bno055.cpp` (quat unitless; accel m/s^2; gyro rad/s).
 */
#pragma once
#include <cstdint>
#include <array>

/**
 * @class BNO055
 * @brief Driver I2C minimale per BNO055 con lettura burst.
 */
class BNO055 {
public:
    /**
     * @brief Costruisce il driver per un dato indirizzo I2C.
     * @param addr Indirizzo I2C a 7 bit (default 0x28).
     */
    explicit BNO055(uint8_t addr = 0x28);

    /**
     * @brief Inizializza il sensore impostando le modalità operative.
     * @return true se l'inizializzazione ha avuto successo.
     */
    bool begin();

    /**
     * @brief Legge in burst quaternione, accelerazioni e velocità angolari.
     * @param out Array di 10 float in uscita: {qw,qx,qy,qz, ax,ay,az, gx,gy,gz}.
     * @return true se la lettura ha avuto successo, false altrimenti.
     */
    bool read_all(std::array<float,10>& out); // qw,qx,qy,qz, ax,ay,az, gx,gy,gz

private:
    /** @brief Indirizzo I2C a 7 bit. */
    uint8_t addr_;

    /**
     * @brief Scrive un valore a 8 bit in un registro del dispositivo.
     * @param reg Registro di destinazione.
     * @param val Valore da scrivere.
     * @return true se l'operazione ha successo.
     */
    bool write8(uint8_t reg, uint8_t val);

    /**
     * @brief Legge N byte consecutivi a partire da un registro.
     * @param reg Registro iniziale.
     * @param n Numero di byte da leggere.
     * @param dst Buffer di destinazione (dimensione >= n).
     * @return true se l'operazione ha successo.
     */
    bool readN(uint8_t reg, uint8_t n, uint8_t* dst);
};
