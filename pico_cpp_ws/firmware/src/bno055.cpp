#include "bno055.hpp"
#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <cmath>

// Registers
static constexpr uint8_t REG_PAGE_ID = 0x07;
static constexpr uint8_t REG_CHIP_ID = 0x00;
static constexpr uint8_t CHIP_ID = 0xA0;
static constexpr uint8_t REG_SYS_TRIGGER = 0x3F;
static constexpr uint8_t REG_PWR_MODE   = 0x3E;
static constexpr uint8_t REG_OPR_MODE   = 0x3D;
static constexpr uint8_t REG_UNIT_SEL   = 0x3B;
static constexpr uint8_t OPR_CONFIG     = 0x00;
static constexpr uint8_t OPR_NDOF       = 0x0C;

static constexpr uint8_t REG_ACCEL_X_LSB = 0x08; // burst base

BNO055::BNO055(uint8_t addr) : addr_(addr) {}

bool BNO055::write8(uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_blocking(i2c0, addr_, buf, 2, false) == 2;
}

bool BNO055::readN(uint8_t reg, uint8_t n, uint8_t* dst) {
    if (i2c_write_blocking(i2c0, addr_, &reg, 1, true) != 1) return false;
    return i2c_read_blocking(i2c0, addr_, dst, n, false) == n;
}

bool BNO055::begin() {
    sleep_ms(50);
    // Enter CONFIG mode
    if (!write8(REG_OPR_MODE, OPR_CONFIG)) return false;
    sleep_ms(30);
    // Page 0
    write8(REG_PAGE_ID, 0);
    // Check chip ID
    uint8_t id=0;
    if (!readN(REG_CHIP_ID, 1, &id) || id != CHIP_ID) return false;
    // Reset
    write8(REG_SYS_TRIGGER, 0x20);
    sleep_ms(650);
    // Normal power + internal osc + units (m/s^2, dps, degrees)
    write8(REG_PWR_MODE, 0x00);
    write8(REG_SYS_TRIGGER, 0x00);
    write8(REG_UNIT_SEL, 0x00);
    // NDOF
    if (!write8(REG_OPR_MODE, OPR_NDOF)) return false;
    sleep_ms(150);
    return true;
}

bool BNO055::read_all(std::array<float,10>& out) {
    // Burst 0x08..0x27 (32B): accel(6) + mag(6) + gyro(6) + euler(6) + quat(8)
    uint8_t raw[32] = {0};
    if (!readN(REG_ACCEL_X_LSB, 32, raw)) return false;
    auto s16 = [](uint8_t lo, uint8_t hi) {
        uint16_t v = static_cast<uint16_t>(hi) << 8 | lo;
        return static_cast<int16_t>(v);
    };
    // accel
    float ax = s16(raw[0], raw[1]) / 100.0f;
    float ay = s16(raw[2], raw[3]) / 100.0f;
    float az = s16(raw[4], raw[5]) / 100.0f;
    // gyro dps at offsets 12..17, scale 1/900
    float gx_dps = s16(raw[12], raw[13]) / 900.0f;
    float gy_dps = s16(raw[14], raw[15]) / 900.0f;
    float gz_dps = s16(raw[16], raw[17]) / 900.0f;
    const float k = static_cast<float>(M_PI/180.0);
    float gx = gx_dps * k, gy = gy_dps * k, gz = gz_dps * k;
    // quat wxyz at offsets 24..31 with scale 1/2^14
    float scale = 1.0f / (1 << 14);
    float qw = s16(raw[24], raw[25]) * scale;
    float qx = s16(raw[26], raw[27]) * scale;
    float qy = s16(raw[28], raw[29]) * scale;
    float qz = s16(raw[30], raw[31]) * scale;
    out = {qw,qx,qy,qz, ax,ay,az, gx,gy,gz};
    return true;
}
