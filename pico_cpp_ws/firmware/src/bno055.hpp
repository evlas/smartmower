#pragma once
#include <cstdint>
#include <array>

// Minimal BNO055 I2C driver for RP2040 (blocking burst-read)
// Returns quaternion WXYZ (unitless), accel (m/s^2), gyro (rad/s)

class BNO055 {
public:
    explicit BNO055(uint8_t addr = 0x28);
    bool begin();
    bool read_all(std::array<float,10>& out); // qw,qx,qy,qz, ax,ay,az, gx,gy,gz

private:
    uint8_t addr_;
    bool write8(uint8_t reg, uint8_t val);
    bool readN(uint8_t reg, uint8_t n, uint8_t* dst);
};
