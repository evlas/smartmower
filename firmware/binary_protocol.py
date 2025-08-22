"""
Binary Protocol for Smart Mower Pico Firmware
Optimized binary communication with bridge
"""

import struct
import time

# Message type definitions
MSG_SENSOR_DATA = 0x01
MSG_STATUS_REPORT = 0x02
MSG_MOTOR_CMD = 0x10
MSG_SYSTEM_CMD = 0x11

# System command IDs
SYS_CMD_EMERGENCY_STOP = 0x01
SYS_CMD_RESET = 0x02
SYS_CMD_SET_RELAY = 0x03
SYS_CMD_CALIBRATE = 0x04
SYS_CMD_RESET_ENCODERS = 0x05

# Framing constants
PICO_SOF = 0xAA

# Binary format definitions (Little Endian)
# Sensor data: type(1) + timestamp(4) + imu(24) + mag(12) + ultrasonic(12) + power(8) + flags(4) = 65 bytes
SENSOR_FORMAT = '<BI6f3f3f2f4B'

# Status report: type(1) + timestamp(4) + motor_speeds(16) + motor_rpm(16) + encoders(16) + flags(2) = 55 bytes  
STATUS_FORMAT = '<BI4f4f4I2B'

# Motor command: type(1) + speeds(16) = 17 bytes
MOTOR_CMD_FORMAT = '<B4f'

# System command: type(1) + cmd_id(1) + value(4) = 6 bytes
SYSTEM_CMD_FORMAT = '<BBf'

class BinaryProtocol:
    """Binary protocol handler for Pico firmware"""
    
    def __init__(self, uart):
        self.uart = uart
    
    def _checksum(self, payload: bytes) -> int:
        """XOR di tutti i byte del payload"""
        c = 0
        for b in payload:
            c ^= b
        return c & 0xFF
    
    def _frame(self, payload: bytes) -> bytes:
        """Costruisce il frame: SOF(0xAA) + LEN(2, little) = len(payload)+1 + payload + CHK"""
        chk = self._checksum(payload)
        length = len(payload) + 1  # include checksum
        return bytes([PICO_SOF, length & 0xFF, (length >> 8) & 0xFF]) + payload + bytes([chk])
        
    def send_sensor_data(self, imu_data, mag_data, ultrasonic_data, power_data, flags):
        """Send sensor data in binary format"""
        try:
            payload = struct.pack(SENSOR_FORMAT,
                MSG_SENSOR_DATA,           # message type
                int(time.ticks_ms()),      # timestamp
                *imu_data,                 # 6 float IMU [ax,ay,az,gx,gy,gz]
                *mag_data,                 # 3 float magnetometer [mx,my,mz]
                *ultrasonic_data,          # 3 float ultrasonic [left,center,right]
                *power_data,               # 2 float power [voltage,current]
                *flags                     # 4 byte flags [emergency,rain,bumper,lift]
            )
            frame = self._frame(payload)
            self.uart.write(frame)
            return True
        except Exception as e:
            print(f"Send sensor data error: {e}")
            return False
    
    def send_status_report(self, motor_speeds, motor_rpm, encoder_counts, system_flags, relay_state):
        """Send status report in binary format"""
        try:
            payload = struct.pack(STATUS_FORMAT,
                MSG_STATUS_REPORT,         # message type
                int(time.ticks_ms()),      # timestamp
                *motor_speeds,             # 4 float motor speeds [left,right,blade1,blade2]
                *motor_rpm,                # 4 float motor RPM [left,right,blade1,blade2]
                *encoder_counts,           # 4 uint32 encoder counts
                system_flags,              # system flags byte
                relay_state                # relay state byte
            )
            frame = self._frame(payload)
            self.uart.write(frame)
            return True
        except Exception as e:
            print(f"Send status report error: {e}")
            return False
    
    def parse_command(self, data):
        """Parse binary command from bridge"""
        if len(data) < 1:
            return None
            
        try:
            msg_type = data[0]
            
            if msg_type == MSG_MOTOR_CMD:
                if len(data) >= struct.calcsize(MOTOR_CMD_FORMAT):
                    _, left, right, blade1, blade2 = struct.unpack(MOTOR_CMD_FORMAT, data[:struct.calcsize(MOTOR_CMD_FORMAT)])
                    return ('motor', left, right, blade1, blade2)
            
            elif msg_type == MSG_SYSTEM_CMD:
                if len(data) >= struct.calcsize(SYSTEM_CMD_FORMAT):
                    _, cmd_id, value = struct.unpack(SYSTEM_CMD_FORMAT, data[:struct.calcsize(SYSTEM_CMD_FORMAT)])
                    return ('system', cmd_id, value)
                    
        except Exception as e:
            print(f"Parse command error: {e}")
            
        return None
    
    def get_sensor_packet_size(self):
        """Get sensor packet size in bytes"""
        return struct.calcsize(SENSOR_FORMAT)
    
    def get_status_packet_size(self):
        """Get status packet size in bytes"""
        return struct.calcsize(STATUS_FORMAT)
    
    def get_motor_cmd_size(self):
        """Get motor command size in bytes"""
        return struct.calcsize(MOTOR_CMD_FORMAT)
    
    def get_system_cmd_size(self):
        """Get system command size in bytes"""
        return struct.calcsize(SYSTEM_CMD_FORMAT)

# Utility functions for compatibility
def create_binary_protocol(uart):
    """Create binary protocol instance"""
    return BinaryProtocol(uart)

def pack_sensor_data(imu_data, mag_data, ultrasonic_data, power_data, flags):
    """Pack sensor data into binary format (standalone function)"""
    return struct.pack(SENSOR_FORMAT,
        MSG_SENSOR_DATA,
        int(time.ticks_ms()),
        *imu_data,
        *mag_data,
        *ultrasonic_data,
        *power_data,
        *flags
    )

def pack_status_report(motor_speeds, motor_rpm, encoder_counts, system_flags, relay_state):
    """Pack status report into binary format (standalone function)"""
    return struct.pack(STATUS_FORMAT,
        MSG_STATUS_REPORT,
        int(time.ticks_ms()),
        *motor_speeds,
        *motor_rpm,
        *encoder_counts,
        system_flags,
        relay_state
    )
