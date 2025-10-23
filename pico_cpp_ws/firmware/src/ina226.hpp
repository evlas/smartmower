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
    bool write16(uint8_t reg, uint16_t val);
    bool read16(uint8_t reg, uint16_t &val);
    bool init();
    i2c_inst_t* i2c_;
    uint8_t addr_;
    float shunt_ohms_;
    float max_current_a_;
    float current_lsb_;
    bool init_ok_;
};
