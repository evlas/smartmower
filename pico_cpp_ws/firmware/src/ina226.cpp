#include "ina226.hpp"

INA226::INA226(i2c_inst_t* i2c, uint8_t addr, float shunt_ohms, float max_current_a)
: i2c_(i2c), addr_(addr), shunt_ohms_(shunt_ohms), max_current_a_(max_current_a), current_lsb_(0.0f), init_ok_(false) {
    init_ok_ = init();
}

bool INA226::ok() const { return init_ok_ && current_lsb_ > 0.0f; }

bool INA226::write16(uint8_t reg, uint16_t val) {
    uint8_t buf[3];
    buf[0] = reg;
    buf[1] = static_cast<uint8_t>((val >> 8) & 0xFF);
    buf[2] = static_cast<uint8_t>(val & 0xFF);
    int n = i2c_write_blocking(i2c_, addr_, buf, 3, false);
    return n == 3;
}

bool INA226::read16(uint8_t reg, uint16_t &val) {
    int w = i2c_write_blocking(i2c_, addr_, &reg, 1, true);
    if (w != 1) return false;
    uint8_t b[2] = {0};
    int r = i2c_read_blocking(i2c_, addr_, b, 2, false);
    if (r != 2) return false;
    val = (static_cast<uint16_t>(b[0]) << 8) | b[1];
    return true;
}

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
