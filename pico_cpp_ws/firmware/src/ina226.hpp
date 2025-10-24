/**
 * @file ina226.hpp
 * @brief Interfaccia driver INA226 (monitor corrente/tensione) via I2C.
 *
 * @details Fornisce API per calibrazione semplificata e lettura di tensione del bus (V)
 * e corrente (A) basata su shunt configurato. Pensato per uso blocking su RP2040.
 */
#pragma once
#include <cstdint>
#include "hardware/i2c.h"

/** \brief Driver minimale per INA226 (monitor batteria) su I2C.
 *  Implementa la calibrazione e la lettura di tensione (V) e corrente (A).
 */
class INA226 {
public:
    /** \brief Crea il driver.
     *  \param i2c Istanza I2C (i2c0/i2c1)
     *  \param addr Indirizzo 7-bit (default 0x40)
     *  \param shunt_ohms Valore shunt (ohm)
     *  \param max_current_a Corrente massima attesa (A)
     */
    INA226(i2c_inst_t* i2c, uint8_t addr = 0x40, float shunt_ohms = 0.002f, float max_current_a = 20.0f);
    /** \brief Ritorna true se inizializzazione e calibrazione sono OK. */
    bool ok() const;
    /** \brief Legge tensione (V) e corrente (A). Ritorna true se OK. */
    bool read(float &voltage_V, float &current_A);
private:
    /** \brief Scrive un registro 16-bit big-endian. */
    bool write16(uint8_t reg, uint16_t val);
    /** \brief Legge un registro 16-bit big-endian. */
    bool read16(uint8_t reg, uint16_t &val);
    /** \brief Calibra il sensore in base ai parametri forniti. */
    bool init();

    /** \brief Istanza I2C utilizzata. */
    i2c_inst_t* i2c_;
    /** \brief Indirizzo 7-bit del dispositivo. */
    uint8_t addr_;
    /** \brief Valore dello shunt (ohm). */
    float shunt_ohms_;
    /** \brief Corrente massima attesa (A). */
    float max_current_a_;
    /** \brief Risoluzione LSB della corrente (A/LSB) calcolata. */
    float current_lsb_;
    /** \brief Stato di inizializzazione e calibrazione. */
    bool init_ok_;
};
