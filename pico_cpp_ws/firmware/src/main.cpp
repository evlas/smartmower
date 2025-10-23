#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include <array>
#include <vector>
#include <cstring>
#include <cmath>

#include "protocol.hpp"
#include "bno055.hpp"
#include "encoder.hpp"
#include "pcf8574.hpp"
#include "sonar3.hpp"
#include "ina226.hpp"

// Pins (match MicroPython config)
static constexpr uint UART_BAUD = 230400;
static constexpr uint UART_ID = uart0;
static constexpr uint UART_TX_PIN = 0; // GP0
static constexpr uint UART_RX_PIN = 1; // GP1

static i2c_inst_t* I2C = i2c0;
static constexpr uint I2C_SDA_PIN = 16; // GP16
static constexpr uint I2C_SCL_PIN = 17; // GP17
static constexpr uint I2C_BAUD = 400000;

// Motor pins
static constexpr uint MOTOR_LEFT_PWM_PIN = 8;   // GP8
static constexpr uint MOTOR_LEFT_DIR_PIN = 12;  // GP12
static constexpr bool LEFT_DIR_INVERTED = false;
static constexpr uint MOTOR_RIGHT_PWM_PIN = 9;  // GP9
static constexpr uint MOTOR_RIGHT_DIR_PIN = 13; // GP13
static constexpr bool RIGHT_DIR_INVERTED = true;

// Safety relay pin
static constexpr uint SAFETY_RELAY_PIN = 22;    // GP22

// Odometry rate (placeholder)
static constexpr uint ODOM_RATE_HZ = 10;

// Encoder pins
static constexpr uint MOTOR_LEFT_ENC_PIN = 18;  // GP18
static constexpr uint MOTOR_RIGHT_ENC_PIN = 19; // GP19

// PCF8574 settings/bits
static constexpr uint8_t PCF8574_ADDR = 0x20;
static constexpr uint PCF_POLL_HZ = 10;
static constexpr uint PCF_BIT_RAIN = 0;
static constexpr uint PCF_BIT_LIFT = 1;
static constexpr uint PCF_BIT_BUMPER_LEFT = 2;
static constexpr uint PCF_BIT_BUMPER_RIGHT = 3;
static constexpr uint PCF_BIT_AUX1 = 4;
static constexpr uint PCF_BIT_AUX2 = 5;
static constexpr uint PCF_BIT_AUX3 = 6;
static constexpr uint PCF_BIT_AUX4 = 7;

// Sonar pins and rate
static constexpr uint US_FRONT_LEFT_TRIG_PIN = 2;   // GP2
static constexpr uint US_FRONT_LEFT_ECHO_PIN = 3;   // GP3
static constexpr uint US_FRONT_CENTER_TRIG_PIN = 4; // GP4
static constexpr uint US_FRONT_CENTER_ECHO_PIN = 5; // GP5
static constexpr uint US_FRONT_RIGHT_TRIG_PIN = 6;  // GP6
static constexpr uint US_FRONT_RIGHT_ECHO_PIN = 7;  // GP7
static constexpr uint SONAR_RATE_HZ = 10;
static constexpr uint SONAR_TIMEOUT_US = 30000;

// Battery (INA226) rate
static constexpr uint BATT_RATE_HZ = 2;

// Blades pins and params
static constexpr uint BLADE1_PWM_PIN = 10; // GP10
static constexpr uint BLADE1_DIR_PIN = 14; // GP14
static constexpr uint BLADE2_PWM_PIN = 11; // GP11
static constexpr uint BLADE2_DIR_PIN = 15; // GP15
static constexpr uint BLADE1_ENC_PIN = 20; // GP20
static constexpr uint BLADE2_ENC_PIN = 21; // GP21
static constexpr uint BLADE_RPM_RATE_HZ = 10;
static constexpr float BLADE_PULSES_PER_REV_MOTOR = 6.0f;
static constexpr float BLADE_GEAR_RATIO = 1.0f;

int main() {
    stdio_init_all();

    // UART0 @230400
    uart_init(uart0, UART_BAUD);
    gpio_set_function(UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    // I2C0 @400k
    i2c_init(I2C, I2C_BAUD);
    gpio_set_function(I2C_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SDA_PIN);
    gpio_pull_up(I2C_SCL_PIN);

    // Safety relay output
    gpio_init(SAFETY_RELAY_PIN);
    gpio_set_dir(SAFETY_RELAY_PIN, GPIO_OUT);
    gpio_put(SAFETY_RELAY_PIN, 0); // disabled initially

    // Motor DIR pins
    gpio_init(MOTOR_LEFT_DIR_PIN);
    gpio_set_dir(MOTOR_LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR_LEFT_DIR_PIN, 0);
    gpio_init(MOTOR_RIGHT_DIR_PIN);
    gpio_set_dir(MOTOR_RIGHT_DIR_PIN, GPIO_OUT);
    gpio_put(MOTOR_RIGHT_DIR_PIN, 0);

    // Motor PWM setup at ~20kHz
    auto setup_pwm = [](uint pin){
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        uint chan = pwm_gpio_to_channel(pin);
        // 125MHz / (wrap+1) / clkdiv = freq -> wrap=6249, clkdiv=1 -> 20kHz
        pwm_set_wrap(slice, 6249);
        pwm_set_clkdiv(slice, 1.0f);
        pwm_set_enabled(slice, true);
        pwm_set_chan_level(slice, chan, 0);
    };
    setup_pwm(MOTOR_LEFT_PWM_PIN);
    setup_pwm(MOTOR_RIGHT_PWM_PIN);
    // Blades PWM
    setup_pwm(BLADE1_PWM_PIN);
    setup_pwm(BLADE2_PWM_PIN);

    BNO055 imu(0x28);
    bool ok = imu.begin();

    absolute_time_t last = get_absolute_time();
    uint8_t seq = 0;
    absolute_time_t last_odom = get_absolute_time();

    // Motor control state
    float left_cmd = 0.0f, right_cmd = 0.0f; // [-1..1]
    float max_abs_speed = 1.0f;
    float accel_limit = 5.0f; // units per second
    bool relay_enabled = false;

    proto::COBSStreamDecoder decoder(256);

    // Encoders
    SingleChannelEncoder encL(MOTOR_LEFT_ENC_PIN, 200);
    SingleChannelEncoder encR(MOTOR_RIGHT_ENC_PIN, 200);

    // PCF8574 (inputs)
    PCF8574 pcf(I2C, PCF8574_ADDR);
    bool pcf_ok = true;
    // try initial write to configure all high (inputs with pull-ups)
    pcf_ok = pcf.write_port(0xFF);
    uint8_t last_port = 0xFF; // all high

    // Event last states
    bool last_enabled_state = false;
    bool last_lift = false;
    bool last_bL = false, last_bR = false;
    bool last_rain = false;
    bool last_aux1=false, last_aux2=false, last_aux3=false, last_aux4=false;

    // Sonar and battery
    Sonar3 sonar(US_FRONT_LEFT_TRIG_PIN, US_FRONT_LEFT_ECHO_PIN,
                 US_FRONT_CENTER_TRIG_PIN, US_FRONT_CENTER_ECHO_PIN,
                 US_FRONT_RIGHT_TRIG_PIN, US_FRONT_RIGHT_ECHO_PIN,
                 SONAR_TIMEOUT_US);
    bool sonar_ok = true;
    INA226 batt(I2C, 0x40, 0.002f, 20.0f);
    bool batt_ok = batt.ok();

    // Blade encoders for RPM
    SingleChannelEncoder bladeEnc1(BLADE1_ENC_PIN, 150);
    SingleChannelEncoder bladeEnc2(BLADE2_ENC_PIN, 150);

    // Blades motor command state (persist across frames)
    static float blade1_cmd = 0.0f, blade2_cmd = 0.0f;

    auto apply_motor = [&](float cmd, uint pwm_pin, uint dir_pin, bool inv_dir){
        bool fwd = cmd >= 0.0f;
        if (inv_dir) fwd = !fwd;
        gpio_put(dir_pin, fwd ? 1 : 0);
        float a = cmd;
        if (a < 0) a = -a;
        if (a > 1.0f) a = 1.0f;
        uint slice = pwm_gpio_to_slice_num(pwm_pin);
        uint chan = pwm_gpio_to_channel(pwm_pin);
        uint16_t level = static_cast<uint16_t>(a * 65535.0f);
        pwm_set_chan_level(slice, chan, level);
    };

    while (true) {
        // Loop tick ~1ms
        sleep_until(delayed_by_ms(last, 1));
        last = get_absolute_time();

        // UART RX -> decoder
        uint8_t rxbuf[64];
        size_t n = 0;
        while (uart_is_readable(uart0) && n < sizeof(rxbuf)) {
            rxbuf[n++] = uart_getc(uart0);
        }
        if (n > 0) decoder.feed(rxbuf, n);

        // Handle any full frames
        proto::DecodedFrame f;
        while (decoder.pop_frame(f)) {
            switch (f.msg_id) {
                case proto::MSG_CMD_RELAY: {
                    if (f.payload.size() >= 1) {
                        relay_enabled = f.payload[0] != 0;
                        gpio_put(SAFETY_RELAY_PIN, relay_enabled ? 1 : 0);
                    }
                } break;
                case proto::MSG_CMD_DRIVE: {
                    if (f.payload.size() == 8) {
                        float l, r;
                        std::memcpy(&l, &f.payload[0], 4);
                        std::memcpy(&r, &f.payload[4], 4);
                        // clamp
                        if (l > 1) l = 1; if (l < -1) l = -1;
                        if (r > 1) r = 1; if (r < -1) r = -1;
                        left_cmd = l; right_cmd = r;
                    }
                } break;
                case proto::MSG_CMD_LIMITS: {
                    if (f.payload.size() == 8) {
                        float maxs, acc;
                        std::memcpy(&maxs, &f.payload[0], 4);
                        std::memcpy(&acc, &f.payload[4], 4);
                        if (maxs <= 0) maxs = 1.0f;
                        if (acc < 0) acc = 0.0f;
                        max_abs_speed = maxs;
                        accel_limit = acc;
                    }
                } break;
                case proto::MSG_CMD_BLADES: {
                    if (f.payload.size() == 8) {
                        float b1, b2;
                        std::memcpy(&b1, &f.payload[0], 4);
                        std::memcpy(&b2, &f.payload[4], 4);
                        if (b1 > 1) b1 = 1; if (b1 < -1) b1 = -1;
                        if (b2 > 1) b2 = 1; if (b2 < -1) b2 = -1;
                        blade1_cmd = b1;
                        blade2_cmd = b2;
                    }
                } break;
                default: break;
            }
        }

        // Apply motor outputs if relay enabled
        float l_cmd = left_cmd, r_cmd = right_cmd;
        if (!relay_enabled) { l_cmd = 0; r_cmd = 0; }
        // simple clamp by max_abs_speed
        if (l_cmd > max_abs_speed) l_cmd = max_abs_speed;
        if (l_cmd < -max_abs_speed) l_cmd = -max_abs_speed;
        if (r_cmd > max_abs_speed) r_cmd = max_abs_speed;
        if (r_cmd < -max_abs_speed) r_cmd = -max_abs_speed;
        apply_motor(l_cmd, MOTOR_LEFT_PWM_PIN, MOTOR_LEFT_DIR_PIN, LEFT_DIR_INVERTED);
        apply_motor(r_cmd, MOTOR_RIGHT_PWM_PIN, MOTOR_RIGHT_DIR_PIN, RIGHT_DIR_INVERTED);

        // Blades motor control: track commands (persist across frames)
        static float blade1_cmd = 0.0f, blade2_cmd = 0.0f;
        // Update from last received frame if any was decoded
        // We updated inside MSG_CMD_BLADES case above, but ensure persistence
        // Apply safety gate
        float b1_apply = relay_enabled ? blade1_cmd : 0.0f;
        float b2_apply = relay_enabled ? blade2_cmd : 0.0f;
        // Apply to hardware
        apply_motor(b1_apply, BLADE1_PWM_PIN, BLADE1_DIR_PIN, false);
        apply_motor(b2_apply, BLADE2_PWM_PIN, BLADE2_DIR_PIN, false);

        // IMU @100Hz
        static uint32_t imu_last_ms = 0;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - imu_last_ms >= 10) {
            imu_last_ms = now_ms;
            std::array<float,10> vec{};
            bool have = ok && imu.read_all(vec);
            if (!have) {
                for (auto &v : vec) v = NAN;
            }
            std::vector<uint8_t> payload(10*sizeof(float));
            std::memcpy(payload.data()+0,  &vec[0], 4);
            std::memcpy(payload.data()+4,  &vec[1], 4);
            std::memcpy(payload.data()+8,  &vec[2], 4);
            std::memcpy(payload.data()+12, &vec[3], 4);
            std::memcpy(payload.data()+16, &vec[4], 4);
            std::memcpy(payload.data()+20, &vec[5], 4);
            std::memcpy(payload.data()+24, &vec[6], 4);
            std::memcpy(payload.data()+28, &vec[7], 4);
            std::memcpy(payload.data()+32, &vec[8], 4);
            std::memcpy(payload.data()+36, &vec[9], 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_IMU, payload, seq++, now_ms, enc);
            if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
        }

        // Odom @10Hz reale con encoder single-channel
        static uint32_t odom_last_ms = 0;
        if (now_ms - odom_last_ms >= (1000/ODOM_RATE_HZ)) {
            uint32_t dt_ms = now_ms - odom_last_ms;
            odom_last_ms = now_ms;
            int32_t tL = encL.read_and_reset();
            int32_t tR = encR.read_and_reset();
            // segno da dir pins
            bool dirL_fwd = gpio_get(MOTOR_LEFT_DIR_PIN) != 0;
            bool dirR_fwd = gpio_get(MOTOR_RIGHT_DIR_PIN) != 0;
            if (LEFT_DIR_INVERTED) dirL_fwd = !dirL_fwd;
            if (RIGHT_DIR_INVERTED) dirR_fwd = !dirR_fwd;
            int sL = dirL_fwd ? 1 : -1;
            int sR = dirR_fwd ? 1 : -1;
            tL *= sL;
            tR *= sR;
            float dt = dt_ms / 1000.0f;
            std::vector<uint8_t> payload;
            payload.resize(4+4+4);
            std::memcpy(payload.data()+0, &tL, 4);
            std::memcpy(payload.data()+4, &tR, 4);
            std::memcpy(payload.data()+8, &dt, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_ODOM, payload, seq++, now_ms, enc);
            if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
        }

        // PCF8574 poll @10Hz -> eventi
        static uint32_t pcf_last_ms = 0;
        if (now_ms - pcf_last_ms >= (1000/PCF_POLL_HZ)) {
            pcf_last_ms = now_ms;
            uint8_t val = 0xFF;
            bool okread = pcf.read_port(val);
            if (!okread) {
                pcf_ok = false;
            } else {
                pcf_ok = true;
                // Active-low for rain, bumpers and lift per MicroPython mapping
                bool rain = ((val >> PCF_BIT_RAIN) & 1) == 0;
                bool lift = ((val >> PCF_BIT_LIFT) & 1) == 1; // lift active-high in MP code? they used GPIO helpers; set true if bit=1
                bool bL = ((val >> PCF_BIT_BUMPER_LEFT) & 1) == 1;
                bool bR = ((val >> PCF_BIT_BUMPER_RIGHT) & 1) == 1;
                bool aux1 = ((val >> PCF_BIT_AUX1) & 1) == 1;
                bool aux2 = ((val >> PCF_BIT_AUX2) & 1) == 1;
                bool aux3 = ((val >> PCF_BIT_AUX3) & 1) == 1;
                bool aux4 = ((val >> PCF_BIT_AUX4) & 1) == 1;

                // Build bitfield
                uint16_t bf = 0;
                if (relay_enabled) bf |= (1u<<0); // EVENT_RELAY_ENABLED
                if (bL) bf |= (1u<<1);
                if (bR) bf |= (1u<<2);
                if (lift) bf |= (1u<<3);
                if (rain) bf |= (1u<<4);
                if (aux1) bf |= (1u<<5);
                if (aux2) bf |= (1u<<6);
                if (aux3) bf |= (1u<<7);
                if (aux4) bf |= (1u<<8);

                // Emit only on change of any
                if (rain != last_rain || lift != last_lift || bL != last_bL || bR != last_bR ||
                    aux1 != last_aux1 || aux2 != last_aux2 || aux3 != last_aux3 || aux4 != last_aux4 ||
                    relay_enabled != last_enabled_state) {
                    std::vector<uint8_t> payload(2);
                    payload[0] = static_cast<uint8_t>(bf & 0xFF);
                    payload[1] = static_cast<uint8_t>((bf >> 8) & 0xFF);
                    std::vector<uint8_t> enc;
                    proto::pack_and_encode(proto::MSG_EVENT, payload, seq++, now_ms, enc);
                    if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
                    last_rain = rain; last_lift = lift; last_bL = bL; last_bR = bR;
                    last_aux1 = aux1; last_aux2 = aux2; last_aux3 = aux3; last_aux4 = aux4;
                    last_enabled_state = relay_enabled;
                }
                last_port = val;
            }
        }

        // Sonar @SONAR_RATE_HZ
        static uint32_t sonar_last_ms = 0;
        if (now_ms - sonar_last_ms >= (1000/SONAR_RATE_HZ)) {
            sonar_last_ms = now_ms;
            float dL= -1.0f, dC= -1.0f, dR= -1.0f;
            sonar_ok = sonar.read_all(dL, dC, dR);
            // Convert -1 to max_range = 5.0m
            float max_range = 5.0f;
            if (dL < 0) dL = max_range;
            if (dC < 0) dC = max_range;
            if (dR < 0) dR = max_range;
            std::vector<uint8_t> payload(3*sizeof(float));
            std::memcpy(payload.data()+0, &dL, 4);
            std::memcpy(payload.data()+4, &dC, 4);
            std::memcpy(payload.data()+8, &dR, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_SONAR, payload, seq++, now_ms, enc);
            if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
            if (!sonar_ok) {
                // emit error event bit 15
                uint16_t bf = (1u<<15);
                std::vector<uint8_t> ev(2);
                ev[0] = static_cast<uint8_t>(bf & 0xFF);
                ev[1] = static_cast<uint8_t>((bf>>8)&0xFF);
                std::vector<uint8_t> enc2;
                proto::pack_and_encode(proto::MSG_EVENT, ev, seq++, now_ms, enc2);
                if (!enc2.empty()) uart_write_blocking(uart0, enc2.data(), enc2.size());
            }
        }

        // Battery @BATT_RATE_HZ
        static uint32_t batt_last_ms = 0;
        if (now_ms - batt_last_ms >= (1000/BATT_RATE_HZ)) {
            batt_last_ms = now_ms;
            float v= -1.0f, a= 0.0f;
            if (batt_ok) {
                batt_ok = batt.read(v, a);
            }
            if (!batt_ok) {
                // emit error event bit 14
                uint16_t bf = (1u<<14);
                std::vector<uint8_t> ev(2);
                ev[0] = static_cast<uint8_t>(bf & 0xFF);
                ev[1] = static_cast<uint8_t>((bf>>8)&0xFF);
                std::vector<uint8_t> enc2;
                proto::pack_and_encode(proto::MSG_EVENT, ev, seq++, now_ms, enc2);
                if (!enc2.empty()) uart_write_blocking(uart0, enc2.data(), enc2.size());
            }
            std::vector<uint8_t> payload(2*sizeof(float));
            std::memcpy(payload.data()+0, &v, 4);
            std::memcpy(payload.data()+4, &a, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_BATT, payload, seq++, now_ms, enc);
            if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
        }

        // Blades RPM @BLADE_RPM_RATE_HZ
        static uint32_t blade_last_ms = 0;
        if (now_ms - blade_last_ms >= (1000/BLADE_RPM_RATE_HZ)) {
            uint32_t dt_ms = now_ms - blade_last_ms;
            blade_last_ms = now_ms;
            int32_t c1 = bladeEnc1.read_and_reset();
            int32_t c2 = bladeEnc2.read_and_reset();
            float ppr_blade = BLADE_PULSES_PER_REV_MOTOR * BLADE_GEAR_RATIO;
            if (ppr_blade <= 0.0f) ppr_blade = 1.0f;
            float dtb = dt_ms / 1000.0f;
            float rpm1 = (dtb > 0.0f) ? ((static_cast<float>(c1) / ppr_blade) / dtb * 60.0f) : 0.0f;
            float rpm2 = (dtb > 0.0f) ? ((static_cast<float>(c2) / ppr_blade) / dtb * 60.0f) : 0.0f;
            std::vector<uint8_t> payload(2*sizeof(float));
            std::memcpy(payload.data()+0, &rpm1, 4);
            std::memcpy(payload.data()+4, &rpm2, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_BLADES_RPM, payload, seq++, now_ms, enc);
            if (!enc.empty()) uart_write_blocking(uart0, enc.data(), enc.size());
        }
    }
}

