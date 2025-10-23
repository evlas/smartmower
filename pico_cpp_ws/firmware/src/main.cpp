#include "pico/stdlib.h"
#include "hardware/uart.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/clocks.h"
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
#include "pins_config.hpp"

// UART settings centralizzate in pins_config.hpp

// I2C instance
static i2c_inst_t* I2C = i2c0;

int main() {
    stdio_init_all();

    // UART0 @ pinscfg::UART_BAUD
    uart_init(uart0, pinscfg::UART_BAUD);
    gpio_set_function(pinscfg::UART_TX_PIN, GPIO_FUNC_UART);
    gpio_set_function(pinscfg::UART_RX_PIN, GPIO_FUNC_UART);
    uart_set_format(uart0, 8, 1, UART_PARITY_NONE);
    uart_set_fifo_enabled(uart0, true);

    gpio_init(pinscfg::LED_PIN);
    gpio_set_dir(pinscfg::LED_PIN, GPIO_OUT);
    gpio_put(pinscfg::LED_PIN, 0);

    // I2C0 @ pinscfg::I2C0_FREQ_HZ
    i2c_init(I2C, pinscfg::I2C0_FREQ_HZ);
    gpio_set_function(pinscfg::I2C0_SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(pinscfg::I2C0_SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(pinscfg::I2C0_SDA_PIN);
    gpio_pull_up(pinscfg::I2C0_SCL_PIN);

    // Safety relay output (respect active-low polarity)
    gpio_init(pinscfg::SAFETY_RELAY_PIN);
    gpio_set_dir(pinscfg::SAFETY_RELAY_PIN, GPIO_OUT);
    // Disabled initially: OFF -> level = active_low ? 1 : 0
    gpio_put(pinscfg::SAFETY_RELAY_PIN, pinscfg::RELAY_ACTIVE_LOW ? 1 : 0);

    // Motor DIR pins
    gpio_init(pinscfg::MOTOR_LEFT_DIR_PIN);
    gpio_set_dir(pinscfg::MOTOR_LEFT_DIR_PIN, GPIO_OUT);
    gpio_put(pinscfg::MOTOR_LEFT_DIR_PIN, 0);
    gpio_init(pinscfg::MOTOR_RIGHT_DIR_PIN);
    gpio_set_dir(pinscfg::MOTOR_RIGHT_DIR_PIN, GPIO_OUT);
    gpio_put(pinscfg::MOTOR_RIGHT_DIR_PIN, 0);

    // Motor PWM setup using pinscfg::PWM_FREQ_HZ
    auto setup_pwm = [](uint pin){
        gpio_set_function(pin, GPIO_FUNC_PWM);
        uint slice = pwm_gpio_to_slice_num(pin);
        uint chan = pwm_gpio_to_channel(pin);
        // f_sys / (wrap+1) / clkdiv = PWM_FREQ
        float clkdiv = 1.0f;
        uint32_t f_sys = clock_get_hz(clk_sys);
        uint32_t wrap = (uint32_t)((float)f_sys / (clkdiv * (float)pinscfg::PWM_FREQ_HZ) - 1.0f);
        pwm_set_wrap(slice, wrap);
        pwm_set_clkdiv(slice, clkdiv);
        pwm_set_enabled(slice, true);
        pwm_set_chan_level(slice, chan, 0);
    };
    setup_pwm(pinscfg::MOTOR_LEFT_PWM_PIN);
    setup_pwm(pinscfg::MOTOR_RIGHT_PWM_PIN);
    // Blades PWM
    setup_pwm(pinscfg::BLADE1_PWM_PIN);
    setup_pwm(pinscfg::BLADE2_PWM_PIN);

    BNO055 imu(pinscfg::BNO055_ADDR);
    bool ok = imu.begin();

    absolute_time_t last = get_absolute_time();
    uint8_t seq = 0;
    absolute_time_t last_odom = get_absolute_time();

    // Motor control state
    float left_cmd = 0.0f, right_cmd = 0.0f; // [-1..1]
    float max_abs_speed = pinscfg::DEFAULT_MAX_ABS_SPEED;
    float accel_limit = pinscfg::DEFAULT_ACCEL_LIMIT; // units per second
    bool relay_enabled = false;
    bool uart_led_on = false;
    uint32_t last_uart_activity_ms = 0;

    proto::COBSStreamDecoder decoder(256);

    // Encoders
    SingleChannelEncoder encL(pinscfg::MOTOR_LEFT_ENC_PIN, 200);
    SingleChannelEncoder encR(pinscfg::MOTOR_RIGHT_ENC_PIN, 200);

    // PCF8574 (inputs)
    PCF8574 pcf(I2C, pinscfg::PCF8574_ADDR);
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
    Sonar3 sonar(pinscfg::US_FRONT_LEFT_TRIG_PIN, pinscfg::US_FRONT_LEFT_ECHO_PIN,
                 pinscfg::US_FRONT_CENTER_TRIG_PIN, pinscfg::US_FRONT_CENTER_ECHO_PIN,
                 pinscfg::US_FRONT_RIGHT_TRIG_PIN, pinscfg::US_FRONT_RIGHT_ECHO_PIN,
                 pinscfg::SONAR_TIMEOUT_US);
    bool sonar_ok = true;
    INA226 batt(I2C, pinscfg::INA226_ADDR, pinscfg::INA226_SHUNT_OHMS, pinscfg::INA226_MAX_CURRENT_A);
    bool batt_ok = batt.ok();

    // Blade encoders for RPM
    SingleChannelEncoder bladeEnc1(pinscfg::BLADE1_ENC_PIN, 150);
    SingleChannelEncoder bladeEnc2(pinscfg::BLADE2_ENC_PIN, 150);

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
        uint16_t level = static_cast<uint16_t>(a * pinscfg::PWM_DUTY_MAX);
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

        // UART LED auto-off after inactivity timeout
        if (uart_led_on) {
            uint32_t now_ms2 = to_ms_since_boot(get_absolute_time());
            if (now_ms2 - last_uart_activity_ms > pinscfg::UART_LED_TIMEOUT_MS) {
                gpio_put(pinscfg::LED_PIN, 0);
                uart_led_on = false;
            }
        }
        if (n > 0) {
            decoder.feed(rxbuf, n);
            gpio_put(pinscfg::LED_PIN, 1);
            uart_led_on = true;
            last_uart_activity_ms = to_ms_since_boot(get_absolute_time());
        }

        // Handle any full frames
        proto::DecodedFrame f;
        while (decoder.pop_frame(f)) {
            switch (f.msg_id) {
                case proto::MSG_CMD_RELAY: {
                    if (f.payload.size() >= 1) {
                        relay_enabled = f.payload[0] != 0;
                        // ON -> level = active_low ? 0 : 1; OFF -> active_low ? 1 : 0
                        uint level = 0;
                        if (relay_enabled) level = pinscfg::RELAY_ACTIVE_LOW ? 0u : 1u;
                        else level = pinscfg::RELAY_ACTIVE_LOW ? 1u : 0u;
                        gpio_put(pinscfg::SAFETY_RELAY_PIN, level);
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
        if (fabsf(l_cmd) < pinscfg::DEFAULT_DEADZONE) l_cmd = 0.0f;
        if (fabsf(r_cmd) < pinscfg::DEFAULT_DEADZONE) r_cmd = 0.0f;
        apply_motor(l_cmd, pinscfg::MOTOR_LEFT_PWM_PIN, pinscfg::MOTOR_LEFT_DIR_PIN, pinscfg::LEFT_DIR_INVERTED);
        apply_motor(r_cmd, pinscfg::MOTOR_RIGHT_PWM_PIN, pinscfg::MOTOR_RIGHT_DIR_PIN, pinscfg::RIGHT_DIR_INVERTED);

        // Blades motor control: track commands (persist across frames)
        // Update from last received frame if any was decoded
        // We updated inside MSG_CMD_BLADES case above, but ensure persistence
        // Apply safety gate
        float b1_apply = relay_enabled ? blade1_cmd : 0.0f;
        float b2_apply = relay_enabled ? blade2_cmd : 0.0f;
        if (fabsf(b1_apply) < pinscfg::BLADE_DEFAULT_DEADZONE) b1_apply = 0.0f;
        if (fabsf(b2_apply) < pinscfg::BLADE_DEFAULT_DEADZONE) b2_apply = 0.0f;
        // Apply to hardware
        apply_motor(b1_apply, pinscfg::BLADE1_PWM_PIN, pinscfg::BLADE1_DIR_PIN, false);
        apply_motor(b2_apply, pinscfg::BLADE2_PWM_PIN, pinscfg::BLADE2_DIR_PIN, false);

        // IMU @ pinscfg::IMU_RATE_HZ
        static uint32_t imu_last_ms = 0;
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        if (now_ms - imu_last_ms >= (1000 / pinscfg::IMU_RATE_HZ)) {
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
            if (!enc.empty()) {
                uart_write_blocking(uart0, enc.data(), enc.size());
                gpio_put(pinscfg::LED_PIN, 1);
                uart_led_on = true;
                last_uart_activity_ms = now_ms;
            }
        }

        // Odom @ pinscfg::ODOM_RATE_HZ con encoder single-channel
        static uint32_t odom_last_ms = 0;
        if (now_ms - odom_last_ms >= (1000/pinscfg::ODOM_RATE_HZ)) {
            uint32_t dt_ms = now_ms - odom_last_ms;
            odom_last_ms = now_ms;
            int32_t tL = encL.read_and_reset();
            int32_t tR = encR.read_and_reset();
            // segno da dir pins
            bool dirL_fwd = gpio_get(pinscfg::MOTOR_LEFT_DIR_PIN) != 0;
            bool dirR_fwd = gpio_get(pinscfg::MOTOR_RIGHT_DIR_PIN) != 0;
            if (pinscfg::LEFT_DIR_INVERTED) dirL_fwd = !dirL_fwd;
            if (pinscfg::RIGHT_DIR_INVERTED) dirR_fwd = !dirR_fwd;
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
            if (!enc.empty()) {
                uart_write_blocking(uart0, enc.data(), enc.size());
                gpio_put(pinscfg::LED_PIN, 1);
                uart_led_on = true;
                last_uart_activity_ms = now_ms;
            }
        }

        // PCF8574 poll @ pinscfg::PCF_POLL_HZ -> eventi
        static uint32_t pcf_last_ms = 0;
        if (now_ms - pcf_last_ms >= (1000/pinscfg::PCF_POLL_HZ)) {
            pcf_last_ms = now_ms;
            uint8_t val = 0xFF;
            bool okread = pcf.read_port(val);
            if (!okread) {
                pcf_ok = false;
            } else {
                pcf_ok = true;
                // Active-low per rain, bumpers e lift per MicroPython mapping
                bool rain = ((val >> pinscfg::PCF_BIT_RAIN) & 1) == 0;
                bool lift = ((val >> pinscfg::PCF_BIT_LIFT) & 1) == 1; // lift active-high
                bool bL = ((val >> pinscfg::PCF_BIT_BUMPER_LEFT) & 1) == 1;
                bool bR = ((val >> pinscfg::PCF_BIT_BUMPER_RIGHT) & 1) == 1;
                bool aux1 = ((val >> pinscfg::PCF_BIT_AUX1) & 1) == 1;
                bool aux2 = ((val >> pinscfg::PCF_BIT_AUX2) & 1) == 1;
                bool aux3 = ((val >> pinscfg::PCF_BIT_AUX3) & 1) == 1;
                bool aux4 = ((val >> pinscfg::PCF_BIT_AUX4) & 1) == 1;

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
                    if (!enc.empty()) {
                        uart_write_blocking(uart0, enc.data(), enc.size());
                        gpio_put(pinscfg::LED_PIN, 1);
                        uart_led_on = true;
                        last_uart_activity_ms = now_ms;
                    }
                    last_rain = rain; last_lift = lift; last_bL = bL; last_bR = bR;
                    last_aux1 = aux1; last_aux2 = aux2; last_aux3 = aux3; last_aux4 = aux4;
                    last_enabled_state = relay_enabled;
                }
                last_port = val;
            }
        }

        // Sonar @ pinscfg::SONAR_RATE_HZ
        static uint32_t sonar_last_ms = 0;
        if (now_ms - sonar_last_ms >= (1000/pinscfg::SONAR_RATE_HZ)) {
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
            if (!enc.empty()) {
                uart_write_blocking(uart0, enc.data(), enc.size());
                gpio_put(pinscfg::LED_PIN, 1);
                uart_led_on = true;
                last_uart_activity_ms = now_ms;
            }
            if (!sonar_ok) {
                // emit error event bit 15
                uint16_t bf = (1u<<15);
                std::vector<uint8_t> ev(2);
                ev[0] = static_cast<uint8_t>(bf & 0xFF);
                ev[1] = static_cast<uint8_t>((bf>>8)&0xFF);
                std::vector<uint8_t> enc2;
                proto::pack_and_encode(proto::MSG_EVENT, ev, seq++, now_ms, enc2);
                if (!enc2.empty()) {
                    uart_write_blocking(uart0, enc2.data(), enc2.size());
                    gpio_put(pinscfg::LED_PIN, 1);
                    uart_led_on = true;
                    last_uart_activity_ms = now_ms;
                }
            }
        }

        // Battery @ pinscfg::BATT_RATE_HZ
        static uint32_t batt_last_ms = 0;
        if (now_ms - batt_last_ms >= (1000/pinscfg::BATT_RATE_HZ)) {
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
                if (!enc2.empty()) {
                    uart_write_blocking(uart0, enc2.data(), enc2.size());
                    gpio_put(pinscfg::LED_PIN, 1);
                    uart_led_on = true;
                    last_uart_activity_ms = now_ms;
                }
            }
            std::vector<uint8_t> payload(2*sizeof(float));
            std::memcpy(payload.data()+0, &v, 4);
            std::memcpy(payload.data()+4, &a, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_BATT, payload, seq++, now_ms, enc);
            if (!enc.empty()) {
                uart_write_blocking(uart0, enc.data(), enc.size());
                gpio_put(pinscfg::LED_PIN, 1);
                uart_led_on = true;
                last_uart_activity_ms = now_ms;
            }
        }

        // Blades RPM @ pinscfg::BLADE_RPM_RATE_HZ
        static uint32_t blade_last_ms = 0;
        if (now_ms - blade_last_ms >= (1000/pinscfg::BLADE_RPM_RATE_HZ)) {
            uint32_t dt_ms = now_ms - blade_last_ms;
            blade_last_ms = now_ms;
            int32_t c1 = bladeEnc1.read_and_reset();
            int32_t c2 = bladeEnc2.read_and_reset();
            float ppr_blade = pinscfg::BLADE_PULSES_PER_REV_MOTOR * pinscfg::BLADE_GEAR_RATIO;
            if (ppr_blade <= 0.0f) ppr_blade = 1.0f;
            float dtb = dt_ms / 1000.0f;
            float rpm1 = (dtb > 0.0f) ? ((static_cast<float>(c1) / ppr_blade) / dtb * 60.0f) : 0.0f;
            float rpm2 = (dtb > 0.0f) ? ((static_cast<float>(c2) / ppr_blade) / dtb * 60.0f) : 0.0f;
            std::vector<uint8_t> payload(2*sizeof(float));
            std::memcpy(payload.data()+0, &rpm1, 4);
            std::memcpy(payload.data()+4, &rpm2, 4);
            std::vector<uint8_t> enc;
            proto::pack_and_encode(proto::MSG_BLADES_RPM, payload, seq++, now_ms, enc);
            if (!enc.empty()) {
                uart_write_blocking(uart0, enc.data(), enc.size());
                gpio_put(pinscfg::LED_PIN, 1);
                uart_led_on = true;
                last_uart_activity_ms = now_ms;
            }
        }
    }
}

