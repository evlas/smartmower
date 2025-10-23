#pragma once
#include <cstdint>

/**
 * \file pins_config.hpp
 * \brief Definizioni pin e parametri hardware per RP2040 (Pico) derivate da pins_config.py.
 *
 * Questo header replica i valori della configurazione MicroPython in forma C++ (constexpr)
 * per uso nel firmware C++ in `pico_cpp_ws`. Tutti i valori sono coerenti con
 * `pico_micropython_ws/firmware/app/pins_config.py`.
 */
namespace pinscfg {

// ===================== UART (telemetria) =====================
/** \brief Baud rate UART per telemetria. */
static constexpr uint32_t UART_BAUD = 230400;
/** \brief Pin TX UART (GP0). */
static constexpr uint32_t UART_TX_PIN = 0;
/** \brief Pin RX UART (GP1). */
static constexpr uint32_t UART_RX_PIN = 1;
/** \brief Pin LED onboard (GP25). */
static constexpr uint32_t LED_PIN = 25;
/** \brief Timeout LED attività UART (ms). */
static constexpr uint32_t UART_LED_TIMEOUT_MS = 2000;

// ===================== MOTORS =====================
/** \brief Frequenza PWM per controllo motori (Hz). */
static constexpr uint32_t PWM_FREQ_HZ = 20000; // 20 kHz
/** \brief Valore massimo duty a 16 bit per PWM. */
static constexpr uint16_t PWM_DUTY_MAX = 65535;

// ====================== DRIVE =====================
/** \brief Pin PWM motore sinistro (GP8). */
static constexpr uint32_t MOTOR_LEFT_PWM_PIN = 8;
/** \brief Pin direzione motore sinistro (GP12). */
static constexpr uint32_t MOTOR_LEFT_DIR_PIN = 12;
/** \brief Inversione logica direzione motore sinistro. */
static constexpr bool LEFT_DIR_INVERTED = false;

/** \brief Pin PWM motore destro (GP9). */
static constexpr uint32_t MOTOR_RIGHT_PWM_PIN = 9;
/** \brief Pin direzione motore destro (GP13). */
static constexpr uint32_t MOTOR_RIGHT_DIR_PIN = 13;
/** \brief Inversione logica direzione motore destro. */
static constexpr bool RIGHT_DIR_INVERTED = true;

/** \brief Pin relay di sicurezza (GP22). */
static constexpr uint32_t SAFETY_RELAY_PIN = 22;
/** \brief Polarity relay: true se attivo a livello basso (0=ON, 1=OFF). */
static constexpr bool RELAY_ACTIVE_LOW = true;

/** \brief Limite di velocità assoluta normalizzata di default [-1..1]. */
static constexpr float DEFAULT_MAX_ABS_SPEED = 1.0f;
/** \brief Limite di accelerazione normalizzato di default (unit/s). */
static constexpr float DEFAULT_ACCEL_LIMIT = 5.0f;
/** \brief Deadzone normalizzata di default. */
static constexpr float DEFAULT_DEADZONE = 0.02f;

// ===================== ENCODER / ODOM =====================
/** \brief Pin encoder motore sinistro (GP18). */
static constexpr uint32_t MOTOR_LEFT_ENC_PIN = 18;
/** \brief Pin encoder motore destro (GP19). */
static constexpr uint32_t MOTOR_RIGHT_ENC_PIN = 19;

/** \brief Tick per giro albero motore encoder. */
static constexpr uint32_t TICKS_PER_REV_MOTOR = 12;
/** \brief Rapporto di riduzione motore:ruota. */
static constexpr float GEAR_RATIO = 185.0f;
/** \brief Raggio ruota (m). */
static constexpr float WHEEL_RADIUS_M = 0.30f;
/** \brief Distanza tra ruote (m). */
static constexpr float WHEEL_SEPARATION_M = 0.55f;
/** \brief Sorgente del segno per i tick encoder ("dir" o "command"). */
static constexpr const char* ENCODER_SIGN_SOURCE = "dir";
/** \brief Frequenza pubblicazione odometria (Hz). */
static constexpr uint32_t ODOM_RATE_HZ = 10;

// ===================== BLADES (lame) =====================
/** \brief Pin PWM lama 1 (GP10). */
static constexpr uint32_t BLADE1_PWM_PIN = 10;
/** \brief Pin direzione lama 1 (GP14). */
static constexpr uint32_t BLADE1_DIR_PIN = 14;
/** \brief Pin encoder lama 1 (GP20). */
static constexpr uint32_t BLADE1_ENC_PIN = 20;

/** \brief Pin PWM lama 2 (GP11). */
static constexpr uint32_t BLADE2_PWM_PIN = 11;
/** \brief Pin direzione lama 2 (GP15). */
static constexpr uint32_t BLADE2_DIR_PIN = 15;
/** \brief Pin encoder lama 2 (GP21). */
static constexpr uint32_t BLADE2_ENC_PIN = 21;

/** \brief Impulsi per giro albero motore della lama. */
static constexpr uint32_t BLADE_PULSES_PER_REV_MOTOR = 6;
/** \brief Rapporto di riduzione motore:lama. */
static constexpr float BLADE_GEAR_RATIO = 1.0f;
/** \brief Limite di velocità assoluta normalizzata di default per lame. */
static constexpr float BLADE_DEFAULT_MAX_ABS_SPEED = 1.0f;
/** \brief Limite di accelerazione normalizzato di default per lame. */
static constexpr float BLADE_DEFAULT_ACCEL_LIMIT = 2.0f;
/** \brief Deadzone normalizzata di default per lame. */
static constexpr float BLADE_DEFAULT_DEADZONE = 0.02f;
/** \brief Frequenza pubblicazione RPM lame (Hz). */
static constexpr uint32_t BLADE_RPM_RATE_HZ = 10;

// ===================== SONAR =====================
/** \brief Pin trigger sonar front-left (GP2). */
static constexpr uint32_t US_FRONT_LEFT_TRIG_PIN = 2;
/** \brief Pin echo sonar front-left (GP3). */
static constexpr uint32_t US_FRONT_LEFT_ECHO_PIN = 3;
/** \brief Pin trigger sonar front-center (GP4). */
static constexpr uint32_t US_FRONT_CENTER_TRIG_PIN = 4;
/** \brief Pin echo sonar front-center (GP5). */
static constexpr uint32_t US_FRONT_CENTER_ECHO_PIN = 5;
/** \brief Pin trigger sonar front-right (GP6). */
static constexpr uint32_t US_FRONT_RIGHT_TRIG_PIN = 6;
/** \brief Pin echo sonar front-right (GP7). */
static constexpr uint32_t US_FRONT_RIGHT_ECHO_PIN = 7;
/** \brief Frequenza lettura sonar (Hz). */
static constexpr uint32_t SONAR_RATE_HZ = 10;
/** \brief Timeout lettura sonar (us). */
static constexpr uint32_t SONAR_TIMEOUT_US = 30000;

// ===================== PERIMETER (ADC) =====================
/** \brief Pin ADC perimetro sinistro (ADC0/GP26). */
static constexpr uint32_t PERIMETER_LEFT_ADC_PIN = 26;
/** \brief Pin ADC perimetro destro (ADC1/GP27). */
static constexpr uint32_t PERIMETER_RIGHT_ADC_PIN = 27;
/** \brief Soglia rilevamento perimetro (0..4095). */
static constexpr uint16_t PERIMETER_THRESHOLD = 1800;

// =========================== I2C ===========================
/** \brief Pin I2C0 SDA (GP16). */
static constexpr uint32_t I2C0_SDA_PIN = 16;
/** \brief Pin I2C0 SCL (GP17). */
static constexpr uint32_t I2C0_SCL_PIN = 17;
/** \brief Frequenza I2C0 (Hz). */
static constexpr uint32_t I2C0_FREQ_HZ = 400000;

// ==================== BATTERY (INA226) =====================
/** \brief Indirizzo I2C INA226. */
static constexpr uint8_t INA226_ADDR = 0x40;
/** \brief Valore shunt (ohm) per INA226. */
static constexpr float INA226_SHUNT_OHMS = 0.002f;
/** \brief Corrente massima attesa (A) per INA226. */
static constexpr float INA226_MAX_CURRENT_A = 20.0f;
/** \brief Frequenza pubblicazione batteria (Hz). */
static constexpr uint32_t BATT_RATE_HZ = 2;

// ===================== IMU (BNO055) =====================
/** \brief Indirizzo I2C BNO055. */
static constexpr uint8_t BNO055_ADDR = 0x28;
/** \brief Frequenza pubblicazione IMU (Hz). */
static constexpr uint32_t IMU_RATE_HZ = 50;
/** \brief Soglia tilt (gradi) per evento di sicurezza. */
static constexpr float TILT_LIMIT_DEG = 30.0f;

// ===================== PCF8574 (ingressi) =====================
/** \brief Indirizzo I2C PCF8574. */
static constexpr uint8_t PCF8574_ADDR = 0x20;
/** \brief Frequenza polling PCF8574 (Hz). */
static constexpr uint32_t PCF_POLL_HZ = 10;
/** \brief Bit P0: rain. */
static constexpr uint8_t PCF_BIT_RAIN = 0;
/** \brief Bit P1: lift. */
static constexpr uint8_t PCF_BIT_LIFT = 1;
/** \brief Bit P2: bumper sinistro. */
static constexpr uint8_t PCF_BIT_BUMPER_LEFT = 2;
/** \brief Bit P3: bumper destro. */
static constexpr uint8_t PCF_BIT_BUMPER_RIGHT = 3;
/** \brief Bit P4: AUX1. */
static constexpr uint8_t PCF_BIT_AUX1 = 4;
/** \brief Bit P5: AUX2. */
static constexpr uint8_t PCF_BIT_AUX2 = 5;
/** \brief Bit P6: AUX3. */
static constexpr uint8_t PCF_BIT_AUX3 = 6;
/** \brief Bit P7: AUX4. */
static constexpr uint8_t PCF_BIT_AUX4 = 7;

} // namespace pinscfg
