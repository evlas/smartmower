#pragma once
#include <cstdint>
#include "hardware/i2c.h"

/** \brief Driver minimale per PCF8574 (expander I2C 8-bit).
 *  Permette di leggere/scrivere l'intero port (8 bit). Lettura tipicamente
 *  usata per ingressi con pull-up esterne (1=alto, 0=basso).
 */
class PCF8574 {
public:
    /** \brief Crea il driver.
     *  \param i2c     Istanza I2C (i2c0/i2c1)
     *  \param addr    Indirizzo 7-bit (es. 0x20)
     */
    PCF8574(i2c_inst_t* i2c, uint8_t addr = 0x20);
    /** \brief Legge l'intero port (8 bit). */
    bool read_port(uint8_t &val);
    /** \brief Scrive l'intero port (8 bit). */
    bool write_port(uint8_t val);
private:
    i2c_inst_t* i2c_;
    uint8_t addr_;
};
