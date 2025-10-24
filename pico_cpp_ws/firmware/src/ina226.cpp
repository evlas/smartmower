/**
 * @file ina226.cpp
 * @brief Implementazione driver INA226 per lettura tensione e corrente via I2C.
 */
#include "ina226.hpp"

/**
 * @brief Costruttore: salva parametri e avvia la calibrazione.
 * @param i2c Istanza I2C (i2c0/i2c1).
 * @param addr Indirizzo 7-bit del dispositivo (default 0x40).
 * @param shunt_ohms Valore dello shunt in ohm.
 * @param max_current_a Corrente massima attesa in ampere.
 */
INA226::INA226(i2c_inst_t* i2c, uint8_t addr, float shunt_ohms, float max_current_a)
: i2c_(i2c), addr_(addr), shunt_ohms_(shunt_ohms), max_current_a_(max_current_a), current_lsb_(0.0f), init_ok_(false) {
    init_ok_ = init();
}

/**
 * @brief Verifica se il dispositivo è stato inizializzato correttamente.
 * @return true se calibrazione e parametri sono validi.
 */
bool INA226::ok() const { return init_ok_ && current_lsb_ > 0.0f; }

/**
 * @brief Scrive un registro 16-bit (big-endian) sull'INA226.
 * @param reg Indirizzo registro.
 * @param val Valore da scrivere.
 * @return true se il trasferimento ha successo.
 */
bool INA226::write16(uint8_t reg, uint16_t val) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(val & 0xFF);
    int n = i2c_write_blocking(i2c_, addr_, buf, 3, false);
    return n == 3;
}

/**
 * @brief Legge un registro 16-bit (big-endian) dall'INA226.
 * @param reg Indirizzo registro.
 * @param val Output del valore letto.
 * @return true se la lettura ha successo.
 */
bool INA226::read16(uint8_t reg, uint16_t &val) {
    int w = i2c_write_blocking(i2c_, addr_, &reg, 1, true);
    if (w != 1) return false;
    uint8_t b[2] = {0};
    int r = i2c_read_blocking(i2c_, addr_, b, 2, false);
    if (r != 2) return false;
    val = (static_cast<uint16_t>(b[0]) << 8) | b[1];
    return true;
}

/**
 * @brief Calibra il convertitore impostando il registro di calibrazione.
 * @return true se la scrittura di calibrazione ha successo.
 */
bool INA226::init() {
    // Compute calibration based on shunt and max current
    float i_lsb = max_current_a_ / 32768.0f;
    if (i_lsb <= 0.0f) i_lsb = 1e-6f;
    int cal = static_cast<int>(0.00512f / (i_lsb * shunt_ohms_));
    if (cal < 1) cal = 1;
    if (cal > 0xFFFF) cal = 0xFFFF;
    if (!write16(0x05, static_cast<uint16_t>(cal))) return false; // CALIBRATION
    current_lsb_ = i_lsb;
    return true;
}

/**
 * @brief Legge tensione del bus (V) e corrente (A) calcolata dallo shunt.
 * @param voltage_V Output tensione bus in volt.
 * @param current_A Output corrente in ampere.
 * @return true se entrambe le letture hanno successo.
 */
bool INA226::read(float &voltage_V, float &current_A) {
    if (!ok()) return false;
    uint16_t raw_v = 0, raw_i = 0;
    if (!read16(0x02, raw_v)) return false; // BUS VOLTAGE
    voltage_V = static_cast<float>(raw_v) * 1.25e-3f;
    if (!read16(0x04, raw_i)) return false; // CURRENT (signed)
    int16_t sraw = static_cast<int16_t>(raw_i);
    current_A = static_cast<float>(sraw) * current_lsb_;
    return true;
}
