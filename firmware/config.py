"""
Configuration file for Smart Mower Pico Firmware
Contains all configurable parameters and constants
"""

from micropython import const

# Hardware Pin Configuration
class PinConfig:
    # Motor pins (12 total)
    MOTOR_LEFT_PWM = const(0)
    MOTOR_LEFT_DIR = const(1)
    MOTOR_LEFT_ENC = const(2)
    MOTOR_RIGHT_PWM = const(3)
    MOTOR_RIGHT_DIR = const(4)
    MOTOR_RIGHT_ENC = const(5)
    BLADE_1_PWM = const(6)
    BLADE_1_DIR = const(7)
    BLADE_1_ENC = const(8)
    BLADE_2_PWM = const(9)
    BLADE_2_DIR = const(10)
    BLADE_2_ENC = const(11)
    
    # I2C pins (2 total)
    I2C_SDA = const(12)
    I2C_SCL = const(13)
    
    # Ultrasonic pins (6 total)
    US_FRONT_LEFT_TRIG = const(14)
    US_FRONT_LEFT_ECHO = const(15)
    US_FRONT_CENTER_TRIG = const(16)
    US_FRONT_CENTER_ECHO = const(17)
    US_FRONT_RIGHT_TRIG = const(18)
    US_FRONT_RIGHT_ECHO = const(19)
    
    # Communication pins (2 total)
    UART_TX = const(20)
    UART_RX = const(21)
    
    # Relay pin (1 total)
    RELAY_PIN = const(22)
    
    # Reserved pins for future expansion
    RESERVED_PINS = [26, 27, 28]

# I2C Device Addresses
class I2CAddresses:
    MPU6050 = const(0x68)      # IMU (accelerometer + gyroscope)
    HMC5883L = const(0x1E)     # Magnetometer
    INA226 = const(0x40)       # Power monitor
    PCF8574 = const(0x20)      # GPIO expander for safety sensors

# Communication Configuration
class CommConfig:
    UART_BAUDRATE = const(115200)
    COMMUNICATION_FREQ_HZ = const(50)
    COMMUNICATION_PERIOD_MS = const(10)
    COMMAND_TIMEOUT_MS = const(5000)
    
    # Message format version
    PROTOCOL_VERSION = "0.1"

# Motor Configuration
class MotorConfig:
    PWM_FREQUENCY = const(20000)  # 20kHz PWM frequency
    MIN_SPEED = const(-100)       # Minimum speed percentage (-100 to +100%)
    MAX_SPEED = const(100)        # Maximum speed percentage (0-100%)
    MAX_PWM_DUTY = const(65535)   # Maximum PWM duty cycle (16-bit)
    ACCELERATION = const(10)      # Acceleration rate (units/cycle)
    DECELERATION = const(15)      # Deceleration rate (units/cycle)
    WHEEL_ENCODER_PPR = const(12)       # Pulses per motor revolution
    BLADE_ENCODER_PPR = const(6)       # Pulses per motor revolution
    WHEEL_REDUCTION_RATIO = const(185)  # Gear reduction ratio
    BLADE_REDUCTION_RATIO = const(1)  # Gear reduction ratio

    # Motor specifications
    WHEEL_DIAMETER_M = 0.3        # 300mm wheels
    BLADE_DIAMETER_M = 0.22       # 240mm blades

# Sensor Configuration
class SensorConfig:
    # I2C bus frequency
    I2C_FREQUENCY = const(400000)  # 400kHz
    
    # IMU configuration (MPU6050)
    IMU_SAMPLE_RATE_HZ = const(100)
    ACCEL_RANGE_G = const(2)       # ±2g
    GYRO_RANGE_DPS = const(250)    # ±250°/s
    
    # Magnetometer configuration (HMC5883L)
    MAG_SAMPLE_RATE_HZ = const(10)
    MAG_RANGE_GAUSS = 1.3          # ±1.3 Gauss
    
    # Power monitor configuration (INA226)
    POWER_SAMPLE_RATE_HZ = const(1)
    SHUNT_RESISTANCE_OHMS = 0.1    # 0.1Ω shunt resistor
    MAX_CURRENT_A = 20             # 20A maximum current
    
    # Ultrasonic configuration
    US_SAMPLE_RATE_HZ = const(10)
    US_MAX_DISTANCE_M = 4.0        # 4m maximum range
    US_MIN_DISTANCE_M = 0.02       # 2cm minimum range
    US_TIMEOUT_US = const(30000)   # 30ms timeout
    
    # Safety sensor configuration
    SAFETY_SAMPLE_RATE_HZ = const(50)  # High frequency for safety

# Safety Configuration
class SafetyConfig:
    # Distance thresholds
    OBSTACLE_DISTANCE_M = 0.3      # 30cm obstacle detection
    EMERGENCY_DISTANCE_M = 0.1     # 10cm emergency stop
    
    # Safety flags (bitmask)
    RAIN_FLAG = const(0x01)        # Bit 0: Rain detected
    BUMPER_FLAG = const(0x02)      # Bit 1: Bumper pressed
    LIFT_FLAG = const(0x04)        # Bit 2: Lift detected
    
    # Timeouts
    COMMUNICATION_TIMEOUT_MS = const(5000)  # 5s communication timeout
    SENSOR_TIMEOUT_MS = const(1000)         # 1s sensor timeout

# Performance Configuration
class PerformanceConfig:
    # Threading configuration
    SENSOR_THREAD_PRIORITY = 1
    MAIN_THREAD_PRIORITY = 0
    
    # Memory management
    GC_COLLECT_INTERVAL = const(1000)  # Run GC every 1000 loops
    MIN_FREE_MEMORY = const(10000)     # Minimum free memory (bytes)
    
    # Performance monitoring
    PERFORMANCE_REPORT_INTERVAL = const(1000)  # Report every 1000 loops
    
    # Loop timing
    MAIN_LOOP_SLEEP_MS = const(1)      # Main loop sleep time
    SENSOR_LOOP_SLEEP_MS = const(1)    # Sensor loop sleep time

# PCF8574 Pin Mapping
class PCF8574Config:
    RAIN_SENSOR_PIN = const(0)     # P0: Rain sensor input
    BUMPER_PIN = const(1)          # P1: Bumper input
    LIFT_SENSOR_PIN = const(2)     # P2: Lift sensor input
    STATUS_LED_PIN = const(3)      # P3: Status LED output
    
    # Unused pins (available for expansion)
    UNUSED_PINS = [4, 5, 6, 7]

# Calibration Constants
class CalibrationConfig:
    # IMU calibration offsets (to be determined during calibration)
    ACCEL_OFFSET_X = 0.0
    ACCEL_OFFSET_Y = 0.0
    ACCEL_OFFSET_Z = 0.0
    
    GYRO_OFFSET_X = 0.0
    GYRO_OFFSET_Y = 0.0
    GYRO_OFFSET_Z = 0.0
    
    # Magnetometer calibration (hard iron correction)
    MAG_OFFSET_X = 0.0
    MAG_OFFSET_Y = 0.0
    MAG_OFFSET_Z = 0.0
    
    # Motor calibration
    MOTOR_LEFT_OFFSET = 0
    MOTOR_RIGHT_OFFSET = 0
    BLADE_1_OFFSET = 0
    BLADE_2_OFFSET = 0

# Debug Configuration
class DebugConfig:
    ENABLE_DEBUG_PRINT = True
    ENABLE_PERFORMANCE_MONITORING = True
    ENABLE_SENSOR_LOGGING = False
    ENABLE_MOTOR_LOGGING = False
    
    # Debug message levels
    DEBUG_LEVEL_ERROR = const(0)
    DEBUG_LEVEL_WARNING = const(1)
    DEBUG_LEVEL_INFO = const(2)
    DEBUG_LEVEL_DEBUG = const(3)
    
    CURRENT_DEBUG_LEVEL = DEBUG_LEVEL_INFO

# System Information
class SystemInfo:
    FIRMWARE_VERSION = "1.0.1"
    BUILD_DATE = "2025-01-22"
    AUTHOR = "Smart Mower Team"
    DESCRIPTION = "Optimized firmware for 100Hz operation"
    
    # Hardware requirements
    MIN_PICO_VERSION = "RP2040"
    REQUIRED_MEMORY_KB = 256
    REQUIRED_FLASH_KB = 2048

# Validation functions
def validate_config():
    """Validate configuration parameters"""
    errors = []
    
    # Check pin conflicts
    used_pins = [
        PinConfig.MOTOR_LEFT_PWM, PinConfig.MOTOR_LEFT_DIR, PinConfig.MOTOR_LEFT_ENC,
        PinConfig.MOTOR_RIGHT_PWM, PinConfig.MOTOR_RIGHT_DIR, PinConfig.MOTOR_RIGHT_ENC,
        PinConfig.BLADE_1_PWM, PinConfig.BLADE_1_DIR, PinConfig.BLADE_1_ENC,
        PinConfig.BLADE_2_PWM, PinConfig.BLADE_2_DIR, PinConfig.BLADE_2_ENC,
        PinConfig.I2C_SDA, PinConfig.I2C_SCL,
        PinConfig.US_FRONT_LEFT_TRIG, PinConfig.US_FRONT_LEFT_ECHO,
        PinConfig.US_FRONT_CENTER_TRIG, PinConfig.US_FRONT_CENTER_ECHO,
        PinConfig.US_FRONT_RIGHT_TRIG, PinConfig.US_FRONT_RIGHT_ECHO,
        PinConfig.UART_TX, PinConfig.UART_RX,
        PinConfig.RELAY_PIN
    ]
    
    if len(used_pins) != len(set(used_pins)):
        errors.append("Pin conflict detected - same pin used multiple times")
    
    # Check communication frequency
    if CommConfig.COMMUNICATION_FREQ_HZ > 1000:
        errors.append("Communication frequency too high (max 1000Hz)")
    
    # Check PWM frequency
    if MotorConfig.PWM_FREQUENCY < 1000 or MotorConfig.PWM_FREQUENCY > 100000:
        errors.append("PWM frequency out of range (1kHz - 100kHz)")
    
    # Check I2C addresses
    i2c_addresses = [
        I2CAddresses.MPU6050, I2CAddresses.HMC5883L,
        I2CAddresses.INA226, I2CAddresses.PCF8574
    ]
    
    if len(i2c_addresses) != len(set(i2c_addresses)):
        errors.append("I2C address conflict detected")
    
    return errors

# Export commonly used configurations
PINS = PinConfig()
I2C_ADDR = I2CAddresses()
COMM = CommConfig()
MOTORS = MotorConfig()
SENSORS = SensorConfig()
SAFETY = SafetyConfig()
PERF = PerformanceConfig()
PCF = PCF8574Config()
CAL = CalibrationConfig()
DEBUG = DebugConfig()
SYS = SystemInfo()

# Run validation on import
if __name__ == "__main__":
    validation_errors = validate_config()
    if validation_errors:
        print("Configuration validation errors:")
        for error in validation_errors:
            print(f"  - {error}")
    else:
        print("Configuration validation passed")
        print(f"Firmware: {SYS.FIRMWARE_VERSION}")
        print(f"Communication: {COMM.COMMUNICATION_FREQ_HZ}Hz")
        print(f"Total pins used: {22}/26")
