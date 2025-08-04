"""Smart Mower - Raspberry Pi Pico Firmware
Optimized for 100Hz communication with Pi 5
Handles motors, sensors, safety systems and relay control"""

import json
import time
import _thread
from machine import Pin, PWM, I2C, UART, Timer
import gc

# Import configuration
from config import (
    PINS, I2C_ADDR, COMM, MOTORS, SENSORS, SAFETY, PERF, PCF, CAL, DEBUG, SYS
)
from binary_protocol import BinaryProtocol, MSG_MOTOR_CMD, MSG_SYSTEM_CMD, SYS_CMD_EMERGENCY_STOP, SYS_CMD_RESET, SYS_CMD_CALIBRATE


class MotorController:
    """High-performance motor controller with encoder support"""
    
    def __init__(self, pwm_pin, dir_pin, enc_pin, encoder_ppr, reduction_ratio):
        self.pwm = PWM(Pin(pwm_pin))
        self.pwm.freq(MOTORS.PWM_FREQUENCY)
        self.dir_pin = Pin(dir_pin, Pin.OUT)
        self.enc_pin = Pin(enc_pin, Pin.IN, Pin.PULL_UP)
        
        self.encoder_ppr = encoder_ppr
        self.reduction_ratio = reduction_ratio
        
        # Encoder state
        self.encoder_count = 0
        self.last_enc_state = self.enc_pin.value()
        
        # Speed control with acceleration/deceleration
        self.current_speed = 0      # Current actual speed
        self.target_speed = 0       # Target speed
        self.last_update_time = time.ticks_ms()
        
        # RPM calculation
        self.last_encoder_count = 0
        self.last_rpm_time = time.ticks_ms()
        self.current_rpm = 0.0
        
        # RPM calculation window (moving average)
        self.rpm_history = []
        self.rpm_window_size = 5
        
        # Setup encoder interrupt
        self.enc_pin.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING, 
                        handler=self._encoder_isr)
    
    def _encoder_isr(self, pin):
        """Encoder interrupt service routine"""
        current_state = pin.value()
        if current_state != self.last_enc_state:
            if self.current_speed >= 0:
                self.encoder_count += 1
            else:
                self.encoder_count -= 1
            self.last_enc_state = current_state
    
    def set_speed(self, target_speed):
        """Set target motor speed with acceleration/deceleration (-100 to +100%)"""
        # Clamp target speed to valid range
        self.target_speed = max(MOTORS.MIN_SPEED, min(MOTORS.MAX_SPEED, target_speed))
    
    def update_speed(self):
        """Update current speed with acceleration/deceleration limits"""
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_update_time)
        
        if dt < 1:  # Avoid division by zero and too frequent updates
            return
        
        self.last_update_time = current_time
        
        # Calculate speed difference
        speed_diff = self.target_speed - self.current_speed
        
        if abs(speed_diff) < 0.1:  # Close enough to target
            self.current_speed = self.target_speed
        else:
            # Apply acceleration/deceleration limits
            if speed_diff > 0:  # Accelerating
                max_change = MOTORS.ACCELERATION * (dt / 10.0)  # Scale by time
                self.current_speed += min(speed_diff, max_change)
            else:  # Decelerating
                max_change = MOTORS.DECELERATION * (dt / 10.0)  # Scale by time
                self.current_speed += max(speed_diff, -max_change)
        
        # Apply PWM output
        self._apply_pwm()
    
    def _apply_pwm(self):
        """Apply current speed to PWM output using configuration parameters"""
        if abs(self.current_speed) < 1:  # Dead zone
            self.pwm.duty_u16(0)
            return
        
        # Set direction
        if self.current_speed >= 0:
            self.dir_pin.value(1)  # Forward
            duty_percent = self.current_speed / float(MOTORS.MAX_SPEED)
        else:
            self.dir_pin.value(0)  # Reverse
            duty_percent = (-self.current_speed) / float(MOTORS.MAX_SPEED)
        
        # Calculate PWM duty using configuration
        duty = int(duty_percent * MOTORS.MAX_PWM_DUTY)
        duty = max(0, min(MOTORS.MAX_PWM_DUTY, duty))  # Clamp to valid range
        
        self.pwm.duty_u16(duty)
    
    def get_encoder_count(self):
        """Get current encoder count"""
        return self.encoder_count
    
    def get_rotations(self):
        """Get number of rotations of the motor shaft"""
        return self.encoder_count / self.encoder_ppr
    
    def get_wheel_rotations(self):
        """Get number of rotations of the wheel/blade (after reduction)"""
        return self.encoder_count / self.encoder_ppr / self.reduction_ratio
    
    def get_distance_m(self, wheel_diameter_m):
        """Get distance traveled in meters"""
        wheel_rotations = self.get_wheel_rotations()  # Use wheel rotations, not motor rotations
        circumference = 3.14159 * wheel_diameter_m
        return wheel_rotations * circumference
    
    def update_rpm(self):
        """Update RPM calculation based on encoder ticks (with differential direction)"""
        current_time = time.ticks_ms()
        dt = time.ticks_diff(current_time, self.last_rpm_time)
        
        if dt >= 100:  # Update RPM every 100ms
            # Calculate ticks delta (preserving sign for direction)
            ticks_delta = self.encoder_count - self.last_encoder_count
            
            # Calculate wheel/blade RPM: (ticks_delta/dt_sec) / PPR / reduction_ratio * 60
            # Positive RPM = forward, Negative RPM = backward
            if self.encoder_ppr > 0 and dt > 0:
                dt_sec = dt / 1000.0
                # Calculate actual wheel/blade RPM (after gear reduction)
                rpm = (ticks_delta / dt_sec) / self.encoder_ppr / self.reduction_ratio * 60.0
                
                # Add to history for smoothing
                self.rpm_history.append(rpm)
                if len(self.rpm_history) > self.rpm_window_size:
                    self.rpm_history.pop(0)
                
                # Calculate smoothed RPM (moving average)
                self.current_rpm = sum(self.rpm_history) / len(self.rpm_history)
            
            # Update for next calculation
            self.last_encoder_count = self.encoder_count
            self.last_rpm_time = current_time
    
    def get_speed_rpm(self):
        """Get current speed in RPM (real measurement with direction)"""
        return self.current_rpm
    
    def reset_encoder(self):
        """Reset encoder count"""
        self.encoder_count = 0
    
    @property
    def speed(self):
        """Get current speed for compatibility"""
        return self.current_speed


class UltrasonicSensor:
    """Optimized ultrasonic sensor driver"""
    
    def __init__(self, trig_pin, echo_pin):
        self.trig = Pin(trig_pin, Pin.OUT)
        self.echo = Pin(echo_pin, Pin.IN)
        self.trig.value(0)
        self.distance = 0.0
    
    def measure(self):
        """Measure distance in meters"""
        try:
            # Send trigger pulse
            self.trig.value(1)
            time.sleep_us(10)
            self.trig.value(0)
            
            # Measure echo pulse
            start_time = time.ticks_us()
            timeout = start_time + SENSORS.US_TIMEOUT_US
            
            # Wait for echo start
            while self.echo.value() == 0 and time.ticks_us() < timeout:
                pass
            
            if time.ticks_us() >= timeout:
                return SENSORS.US_MAX_DISTANCE_M  # Max range on timeout
            
            pulse_start = time.ticks_us()
            
            # Wait for echo end
            while self.echo.value() == 1 and time.ticks_us() < timeout:
                pass
            
            pulse_end = time.ticks_us()
            
            # Calculate distance
            pulse_duration = time.ticks_diff(pulse_end, pulse_start)
            self.distance = (pulse_duration * 0.000343) / 2  # Speed of sound
            
            return min(SENSORS.US_MAX_DISTANCE_M, max(SENSORS.US_MIN_DISTANCE_M, self.distance))
            
        except:
            return SENSORS.US_MAX_DISTANCE_M  # Return max range on error


class I2CSensorManager:
    """Manages all I2C sensors efficiently"""
    
    def __init__(self):
        self.i2c = I2C(0, sda=Pin(PINS.I2C_SDA), scl=Pin(PINS.I2C_SCL), freq=SENSORS.I2C_FREQUENCY)
        self.imu_data = [0.0] * 6  # ax, ay, az, gx, gy, gz
        self.mag_data = [0.0] * 3  # mx, my, mz
        self.power_data = [0.0, 0.0]  # voltage, current
        self.safety_flags = 0
        
        self._init_sensors()
    
    def _init_sensors(self):
        """Initialize all I2C sensors"""
        try:
            # Initialize MPU6050
            self.i2c.writeto_mem(I2C_ADDR.MPU6050, 0x6B, bytes([0x00]))  # Wake up
            self.i2c.writeto_mem(I2C_ADDR.MPU6050, 0x1C, bytes([0x00]))  # ±2g
            self.i2c.writeto_mem(I2C_ADDR.MPU6050, 0x1B, bytes([0x00]))  # ±250°/s
            
            # Initialize HMC5883L
            self.i2c.writeto_mem(I2C_ADDR.HMC5883L, 0x00, bytes([0x70]))  # Config A
            self.i2c.writeto_mem(I2C_ADDR.HMC5883L, 0x01, bytes([0x20]))  # Config B
            self.i2c.writeto_mem(I2C_ADDR.HMC5883L, 0x02, bytes([0x00]))  # Continuous mode
            
            # Initialize INA226
            self.i2c.writeto_mem(I2C_ADDR.INA226, 0x00, bytes([0x41, 0x27]))  # Config
            
            # Initialize PCF8574
            self.i2c.writeto(I2C_ADDR.PCF8574, bytes([0xFF]))  # Set all pins high
            
        except Exception as e:
            print(f"I2C init error: {e}")
    
    def read_imu(self):
        """Read IMU data (accelerometer + gyroscope)"""
        try:
            # Read accelerometer (6 bytes)
            accel_data = self.i2c.readfrom_mem(I2C_ADDR.MPU6050, 0x3B, 6)
            # Read gyroscope (6 bytes)
            gyro_data = self.i2c.readfrom_mem(I2C_ADDR.MPU6050, 0x43, 6)
            
            # Convert to signed values
            ax = self._bytes_to_int16(accel_data[0:2]) / 16384.0 * 9.81
            ay = self._bytes_to_int16(accel_data[2:4]) / 16384.0 * 9.81
            az = self._bytes_to_int16(accel_data[4:6]) / 16384.0 * 9.81
            
            gx = self._bytes_to_int16(gyro_data[0:2]) / 131.0 * 0.017453
            gy = self._bytes_to_int16(gyro_data[2:4]) / 131.0 * 0.017453
            gz = self._bytes_to_int16(gyro_data[4:6]) / 131.0 * 0.017453
            
            self.imu_data = [ax, ay, az, gx, gy, gz]
            
        except:
            pass  # Keep last valid data
    
    def read_magnetometer(self):
        """Read magnetometer data"""
        try:
            mag_data = self.i2c.readfrom_mem(I2C_ADDR.HMC5883L, 0x03, 6)
            
            mx = self._bytes_to_int16(mag_data[0:2])
            mz = self._bytes_to_int16(mag_data[2:4])  # Note: Y and Z swapped
            my = self._bytes_to_int16(mag_data[4:6])
            
            self.mag_data = [mx * 0.92e-6, my * 0.92e-6, mz * 0.92e-6]
            
        except:
            pass
    
    def read_power_monitor(self):
        """Read power monitor (INA226)"""
        try:
            # Read bus voltage
            voltage_data = self.i2c.readfrom_mem(I2C_ADDR.INA226, 0x02, 2)
            voltage = self._bytes_to_uint16(voltage_data) * 1.25e-3
            
            # Read current
            current_data = self.i2c.readfrom_mem(I2C_ADDR.INA226, 0x04, 2)
            current = self._bytes_to_int16(current_data) * 2.5e-3  # 2.5mA per LSB
            
            self.power_data = [voltage, current]
            
        except:
            pass
    
    def read_safety_sensors(self):
        """Read safety sensors via PCF8574"""
        try:
            data = self.i2c.readfrom(I2C_ADDR.PCF8574, 1)[0]
            
            # Extract safety flags (active low)
            rain = not bool(data & 0x01)
            bumper = not bool(data & 0x02)
            lift = not bool(data & 0x04)
            
            self.safety_flags = (rain << 0) | (bumper << 1) | (lift << 2)
            
        except:
            pass
    
    def set_status_led(self, state):
        """Control status LED via PCF8574"""
        try:
            current = self.i2c.readfrom(I2C_ADDR.PCF8574, 1)[0]
            if state:
                new_value = current & 0xF7  # Clear bit 3 (LED on)
            else:
                new_value = current | 0x08  # Set bit 3 (LED off)
            
            self.i2c.writeto(I2C_ADDR.PCF8574, bytes([new_value]))
        except:
            pass
    
    def _bytes_to_int16(self, data):
        """Convert 2 bytes to signed 16-bit integer"""
        value = (data[0] << 8) | data[1]
        return value - 65536 if value > 32767 else value
    
    def _bytes_to_uint16(self, data):
        """Convert 2 bytes to unsigned 16-bit integer"""
        return (data[0] << 8) | data[1]


class SmartMowerPico:
    """Main controller class for Smart Mower Pico"""
    
    def __init__(self):
        # Initialize hardware
        self.motors = [
            MotorController(PINS.MOTOR_LEFT_PWM, PINS.MOTOR_LEFT_DIR, PINS.MOTOR_LEFT_ENC, MOTORS.WHEEL_ENCODER_PPR, MOTORS.WHEEL_REDUCTION_RATIO),
            MotorController(PINS.MOTOR_RIGHT_PWM, PINS.MOTOR_RIGHT_DIR, PINS.MOTOR_RIGHT_ENC, MOTORS.WHEEL_ENCODER_PPR, MOTORS.WHEEL_REDUCTION_RATIO),
            MotorController(PINS.BLADE_1_PWM, PINS.BLADE_1_DIR, PINS.BLADE_1_ENC, MOTORS.BLADE_ENCODER_PPR, MOTORS.BLADE_REDUCTION_RATIO),
            MotorController(PINS.BLADE_2_PWM, PINS.BLADE_2_DIR, PINS.BLADE_2_ENC, MOTORS.BLADE_ENCODER_PPR, MOTORS.BLADE_REDUCTION_RATIO)
        ]
        
        self.ultrasonics = [
            UltrasonicSensor(PINS.US_FRONT_LEFT_TRIG, PINS.US_FRONT_LEFT_ECHO),
            UltrasonicSensor(PINS.US_FRONT_CENTER_TRIG, PINS.US_FRONT_CENTER_ECHO),
            UltrasonicSensor(PINS.US_FRONT_RIGHT_TRIG, PINS.US_FRONT_RIGHT_ECHO)
        ]
        
        self.i2c_sensors = I2CSensorManager()
        
        self.relay = Pin(PINS.RELAY_PIN, Pin.OUT)
        self.relay.value(0)  # Relay off by default
        
        # UART communication
        self.uart = UART(0, baudrate=COMM.UART_BAUDRATE, tx=Pin(PINS.UART_TX), rx=Pin(PINS.UART_RX))
        
        # Binary protocol for optimized communication
        self.binary_protocol = BinaryProtocol(self.uart)
        
        # Control variables
        self.running = True
        self.emergency_stop = False
        self.last_command_time = time.ticks_ms()
        
        # Performance monitoring
        self.loop_count = 0
        self.start_time = time.ticks_ms()
        
        print("Smart Mower Pico initialized")
    
    def emergency_stop_all(self):
        """Emergency stop all motors"""
        self.emergency_stop = True
        for motor in self.motors:
            motor.set_speed(0)
        self.relay.value(0)
        print("EMERGENCY STOP ACTIVATED")
    
    def process_command(self, command_data):
        """Process incoming binary command from bridge"""
        try:
            # Parse binary command
            parsed_cmd = self.binary_protocol.parse_command(command_data)
            
            if parsed_cmd is None:
                return
            
            cmd_type, *args = parsed_cmd
            
            if cmd_type == 'motor':
                if not self.emergency_stop:
                    left_speed, right_speed, blade1_speed, blade2_speed = args
                    
                    # Set motor speeds
                    self.motors[0].set_speed(left_speed)   # Left wheel
                    self.motors[1].set_speed(right_speed)  # Right wheel
                    self.motors[2].set_speed(blade1_speed) # Blade 1
                    self.motors[3].set_speed(blade2_speed) # Blade 2
                    
                    # Control relay (enable if any blade is active)
                    blade_active = blade1_speed > 0 or blade2_speed > 0
                    self.relay.value(1 if blade_active else 0)
                    
                    print(f"Motor cmd: L={left_speed:.1f} R={right_speed:.1f} B1={blade1_speed:.1f} B2={blade2_speed:.1f}")
                
                self.last_command_time = time.ticks_ms()
                
            elif cmd_type == 'system':
                cmd_id, value = args
                
                if cmd_id == SYS_CMD_EMERGENCY_STOP:
                    self.emergency_stop_all()
                    print("Emergency stop activated")
                elif cmd_id == SYS_CMD_RESET:
                    self.emergency_stop = False
                    print("Emergency stop reset")
                elif cmd_id == SYS_CMD_CALIBRATE:
                    for motor in self.motors:
                        motor.reset_encoder()
                    print("Encoders calibrated")
        
        except Exception as e:
            print(f"Binary command processing error: {e}")
    
    def read_sensors(self):
        """Read all sensors efficiently"""
        # Read I2C sensors
        self.i2c_sensors.read_imu()
        self.i2c_sensors.read_magnetometer()
        self.i2c_sensors.read_power_monitor()
        self.i2c_sensors.read_safety_sensors()
        
        # Check safety conditions
        if self.i2c_sensors.safety_flags & 0x06:  # Bumper or lift
            self.emergency_stop_all()
    
    def read_ultrasonics(self):
        """Read ultrasonic sensors (alternate between sensors)"""
        sensor_idx = self.loop_count % 3
        distance = self.ultrasonics[sensor_idx].measure()
        
        # Check for obstacles
        if distance < SAFETY.OBSTACLE_DISTANCE_M:
            self.emergency_stop_all()
    
    def send_sensor_data(self):
        """Send sensor data using optimized binary protocol"""
        try:
            # Get ultrasonic distances
            ultrasonic_data = [us.distance for us in self.ultrasonics]
            
            # Get power data (voltage, current)
            power_data = self.i2c_sensors.power_data[:2]  # Only voltage and current
            
            # Pack safety flags into bytes
            safety_flags = [
                1 if self.emergency_stop else 0,                    # emergency stop
                1 if (self.i2c_sensors.safety_flags & 0x01) else 0, # rain sensor
                1 if (self.i2c_sensors.safety_flags & 0x02) else 0, # bumper
                1 if (self.i2c_sensors.safety_flags & 0x04) else 0  # lift sensor
            ]
            
            # Send binary sensor data (65 bytes vs ~150 bytes JSON)
            success = self.binary_protocol.send_sensor_data(
                self.i2c_sensors.imu_data,      # IMU [ax,ay,az,gx,gy,gz]
                self.i2c_sensors.mag_data,      # Magnetometer [mx,my,mz]
                ultrasonic_data,                # Ultrasonic [left,center,right]
                power_data,                     # Power [voltage,current]
                safety_flags                    # Safety [emergency,rain,bumper,lift]
            )
            
            if not success:
                print("Failed to send sensor data")
                
        except Exception as e:
            print(f"Send sensor data error: {e}")
    
    def send_status_report(self):
        """Send status report using optimized binary protocol"""
        try:
            # Get motor data
            motor_speeds = [motor.current_speed for motor in self.motors]
            motor_rpm = [motor.get_speed_rpm() for motor in self.motors]
            encoder_counts = [motor.get_encoder_count() for motor in self.motors]
            
            # System flags (can be expanded)
            system_flags = 0
            if self.emergency_stop:
                system_flags |= 0x01
            
            # Send binary status report (55 bytes)
            success = self.binary_protocol.send_status_report(
                motor_speeds,     # Current motor speeds [left,right,blade1,blade2]
                motor_rpm,        # Motor RPM [left,right,blade1,blade2]
                encoder_counts,   # Encoder counts [left,right,blade1,blade2]
                system_flags,     # System status flags
                self.relay.value() # Relay state
            )
            
            if not success:
                print("Failed to send status report")
                
        except Exception as e:
            print(f"Send status report error: {e}")
    
    def check_communication_timeout(self):
        """Check for communication timeout with Pi 5"""
        if time.ticks_diff(time.ticks_ms(), self.last_command_time) > COMM.COMMAND_TIMEOUT_MS:
            if not self.emergency_stop:
                print("Communication timeout - emergency stop")
                self.emergency_stop_all()
    
    def sensor_thread(self):
        """Sensor reading thread (Core 1)"""
        sensor_timer = 0
        ultrasonic_timer = 0
        
        while self.running:
            current_time = time.ticks_ms()
            
            # Read I2C sensors every 10ms
            if time.ticks_diff(current_time, sensor_timer) >= 10:
                self.read_sensors()
                sensor_timer = current_time
            
            # Read ultrasonics every 30ms (staggered)
            if time.ticks_diff(current_time, ultrasonic_timer) >= 30:
                self.read_ultrasonics()
                ultrasonic_timer = current_time
            
            time.sleep_ms(PERF.SENSOR_LOOP_SLEEP_MS)
    
    def main_loop(self):
        """Main control loop (Core 0) - 100Hz"""
        print("Starting main loop at 100Hz")
        
        # Start sensor thread on core 1
        _thread.start_new_thread(self.sensor_thread, ())
        
        loop_timer = time.ticks_ms()
        
        while self.running:
            loop_start = time.ticks_ms()
            
            # Update motor speeds with acceleration/deceleration
            for motor in self.motors:
                motor.update_speed()
                motor.update_rpm()  # Calculate real RPM from encoder
            
            # Process incoming binary commands
            if self.uart.any():
                try:
                    # Read binary command data
                    # Try to read motor command first (17 bytes)
                    motor_cmd_size = self.binary_protocol.get_motor_cmd_size()
                    system_cmd_size = self.binary_protocol.get_system_cmd_size()
                    
                    # Read available data
                    available = self.uart.any()
                    if available >= motor_cmd_size:
                        # Try motor command first
                        data = self.uart.read(motor_cmd_size)
                        if data and len(data) == motor_cmd_size:
                            self.process_command(data)
                    elif available >= system_cmd_size:
                        # Try system command
                        data = self.uart.read(system_cmd_size)
                        if data and len(data) == system_cmd_size:
                            self.process_command(data)
                except Exception as e:
                    print(f"UART read error: {e}")
            
            # Send sensor data at configured frequency
            if time.ticks_diff(loop_start, loop_timer) >= COMM.COMMUNICATION_PERIOD_MS:
                self.send_sensor_data()
                
                # Send status report every 10 cycles (1Hz if running at 100Hz)
                if self.loop_count % 10 == 0:
                    self.send_status_report()
                
                self.check_communication_timeout()
                
                # Update status LED (heartbeat)
                led_state = (self.loop_count // 50) % 2
                self.i2c_sensors.set_status_led(led_state)
                
                loop_timer = loop_start
                self.loop_count += 1
                
                # Performance monitoring
                if self.loop_count % PERF.PERFORMANCE_REPORT_INTERVAL == 0:
                    elapsed = time.ticks_diff(loop_start, self.start_time)
                    actual_freq = (self.loop_count * 1000) // elapsed
                    if DEBUG.ENABLE_PERFORMANCE_MONITORING:
                        print(f"Actual frequency: {actual_freq}Hz, Free mem: {gc.mem_free()}")
                    
                    # Garbage collection
                    if self.loop_count % PERF.GC_COLLECT_INTERVAL == 0:
                        gc.collect()
            
            # Small delay to prevent CPU overload
            time.sleep_ms(PERF.MAIN_LOOP_SLEEP_MS)


# Main execution
if __name__ == "__main__":
    try:
        mower = SmartMowerPico()
        mower.main_loop()
    except KeyboardInterrupt:
        print("Shutting down...")
    except Exception as e:
        print(f"Fatal error: {e}")
        # Emergency stop on any fatal error
        for i in range(4):
            try:
                PWM(Pin(i*3)).duty_u16(0)  # Stop all motors
            except:
                pass
