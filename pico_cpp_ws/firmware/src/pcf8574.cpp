/**
 * @file pcf8574.cpp
 * @brief Implementazione driver minimale PCF8574 (expander I2C 8-bit).
 */
#include "pcf8574.hpp"
#include "hardware/i2c.h"

/**
 * @brief Costruttore: salva istanza I2C e indirizzo.
 * @param i2c Istanza I2C.
 * @param addr Indirizzo 7-bit del dispositivo.
 */
PCF8574::PCF8574(i2c_inst_t* i2c, uint8_t addr) : i2c_(i2c), addr_(addr) {}

/**
 * @brief Legge l'intero port a 8 bit.
 * @param val Riferimento dove scrivere il valore letto.
 * @return true se la lettura I2C ha successo.
 */
bool PCF8574::read_port(uint8_t &val) {
    // Per leggere il port: operazione di read semplice 1 byte
    int n = i2c_read_blocking(i2c_, addr_, &val, 1, false);
    return n == 1;
}

/**
 * @brief Scrive l'intero port a 8 bit.
 * @param v Valore da scrivere sul port.
 * @return true se la scrittura I2C ha successo.
 */
bool PCF8574::write_port(uint8_t v) {
    int n = i2c_write_blocking(i2c_, addr_, &v, 1, false);
    return n == 1;
}
