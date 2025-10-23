# MicroPython pin configuration for Raspberry Pi Pico (RP2040)
# Mirrors the motor-related entries from C header:
#   /home/vito/pico_ws/firmware/pico/include/pin_definitions.h
# Focus: Left/Right drive motors and safety relay.

# ===================== MOTORS =====================
# PWM frequency for motor control (Hz)
PWM_FREQ_HZ = 20000  # 20 kHz to keep switching noise above audible range

# 16-bit duty range used by MicroPython PWM on RP2040
PWM_DUTY_MAX = 65535

# ====================== DRIVE =====================
# Left Motor (PWM + DIR)
MOTOR_LEFT_PWM_PIN = 8    # GP8  - PWM slice 4A
MOTOR_LEFT_DIR_PIN = 12   # GP12 - Direction
LEFT_DIR_INVERTED = False # set True if DIR=1 corresponds to reverse

# Right Motor (PWM + DIR)
MOTOR_RIGHT_PWM_PIN = 9   # GP9  - PWM slice 4B
MOTOR_RIGHT_DIR_PIN = 13  # GP13 - Direction
RIGHT_DIR_INVERTED = True # set True if DIR=1 corresponds to reverse

# Safety relay - cuts motor power when low (active-high assumed)
SAFETY_RELAY_PIN = 22     # GP22

# Optional: default limits
DEFAULT_MAX_ABS_SPEED = 1.0        # normalized [-1.0, 1.0]
DEFAULT_ACCEL_LIMIT = 5.0          # unit/s (normalized duty per second), set 0 to disable
DEFAULT_DEADZONE = 0.02            # small deadzone around zero to avoid jitter

# Encoder pins (single-channel encoders on motor shafts)
# From pin_definitions.h: left=GP18, right=GP19
MOTOR_LEFT_ENC_PIN = 18   # GP18 - Encoder left
MOTOR_RIGHT_ENC_PIN = 19  # GP19 - Encoder right

# Odometry parameters (configurable)
# Sensor on motor shaft: ticks_per_rev_motor, gear_ratio to wheel
TICKS_PER_REV_MOTOR = 12          # ticks per motor revolution
GEAR_RATIO = 185.0                # motor:wheel ratio (185 motor rev per 1 wheel rev)

# Wheel geometry (meters)
WHEEL_RADIUS_M = 0.30
WHEEL_SEPARATION_M = 0.55

# Select sign source for encoder ticks: "dir" (read DIR pin) or "command"
ENCODER_SIGN_SOURCE = "dir"

# Odometry publishing rate (Hz)
ODOM_RATE_HZ = 10

# ===================== BLADES (lame) =====================
# Pins per header C di riferimento
BLADE1_PWM_PIN = 10   # GP10 - PWM slice 5A
BLADE1_DIR_PIN = 14   # GP14
BLADE1_ENC_PIN = 20   # GP20

BLADE2_PWM_PIN = 11   # GP11 - PWM slice 5B
BLADE2_DIR_PIN = 15   # GP15
BLADE2_ENC_PIN = 21   # GP21

# Parametri lame
# Impulsi per giro albero motore (configurabile)
BLADE_PULSES_PER_REV_MOTOR = 6
# Rapporto di riduzione motore:lama (1.0 = 1:1)
BLADE_GEAR_RATIO = 1.0
# Limiti default lame
BLADE_DEFAULT_MAX_ABS_SPEED = 1.0
BLADE_DEFAULT_ACCEL_LIMIT = 2.0

# RPM publishing rate (Hz)
BLADE_RPM_RATE_HZ = 10

# ===================== SONAR =====================
# Front Left
US_FRONT_LEFT_TRIG_PIN = 2   # GP2
US_FRONT_LEFT_ECHO_PIN = 3   # GP3
# Front Center
US_FRONT_CENTER_TRIG_PIN = 4 # GP4
US_FRONT_CENTER_ECHO_PIN = 5 # GP5
# Front Right
US_FRONT_RIGHT_TRIG_PIN = 6  # GP6
US_FRONT_RIGHT_ECHO_PIN = 7  # GP7

# Sonar rate (Hz) and timeout
SONAR_RATE_HZ = 10
SONAR_TIMEOUT_US = 30000  # 30 ms ~ max ~5 m

# ===================== PERIMETER (ADC) =====================
# Sensori perimetro sinistro/destro su ADC0/ADC1
# GP26 = ADC0, GP27 = ADC1
PERIMETER_LEFT_ADC_PIN = 26
PERIMETER_RIGHT_ADC_PIN = 27
# Soglia di rilevamento (0..4095) — da calibrare in campo
PERIMETER_THRESHOLD = 1800

# =========================== I2C =========================== 
# I2C0 pins
I2C0_SDA_PIN = 16   # GP16
I2C0_SCL_PIN = 17   # GP17
I2C0_FREQ_HZ = 400000

# ==================== BATTERY (INA226) =====================
# INA226 configuration
INA226_ADDR = 0x40
# Parametri shunt: configurabili secondo hardware
INA226_SHUNT_OHMS = 0.002   # ohm (esempio)
INA226_MAX_CURRENT_A = 20.0 # corrente max attesa per calcolo Current_LSB

# Publishing rate battery (Hz)
BATT_RATE_HZ = 2

# ===================== IMU (BNO055) =====================
# Indirizzo BNO055 (per datasheet 0x28)
BNO055_ADDR = 0x28
# Frequenza pubblicazione IMU (Hz)
IMU_RATE_HZ = 50

# Tilt threshold (degrees). Se roll o pitch superano questo limite → evento tilt
TILT_LIMIT_DEG = 30.0

# ===================== PCF8574 (ingressi) =====================
# Expander I2C per bumper/lift/rain/aux
PCF8574_ADDR = 0x20
PCF_POLL_HZ = 10

# Bit mapping on PCF8574 port (0..7)
# P0 rain, P1 lift, P2 bumper left, P3 bumper right, P4..P7 AUX1..AUX4
PCF_BIT_RAIN = 0
PCF_BIT_LIFT = 1
PCF_BIT_BUMPER_LEFT = 2
PCF_BIT_BUMPER_RIGHT = 3
PCF_BIT_AUX1 = 4
PCF_BIT_AUX2 = 5
PCF_BIT_AUX3 = 6
PCF_BIT_AUX4 = 7




