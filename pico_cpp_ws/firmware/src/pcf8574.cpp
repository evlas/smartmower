#include "pcf8574.hpp"
#include "hardware/i2c.h"

PCF8574::PCF8574(i2c_inst_t* i2c, uint8_t addr) : i2c_(i2c), addr_(addr) {}

bool PCF8574::read_port(uint8_t &val) {
    // Per leggere il port: operazione di read semplice 1 byte
    int n = i2c_read_blocking(i2c_, addr_, &val, 1, false);
    return n == 1;
}

bool PCF8574::write_port(uint8_t v) {
    int n = i2c_write_blocking(i2c_, addr_, &v, 1, false);
    return n == 1;
}
