# MicroPython main entry-point for Pico firmware (motors + odometry)
# - UART0 @115200 with COBS-framed binary protocol (see protocol.py)
# - Handles commands: CMD_DRIVE, CMD_RELAY, CMD_LIMITS
# - Publishes odometry (MSG_ODOM) at cfg.ODOM_RATE_HZ using single-channel encoders
# - Watchdog safety: disables relay if no drive command within timeout

from machine import UART, Pin, I2C
import time
import sys

# Add the app directory to the Python path (MicroPython compatible)
sys.path.append('/app')  # Assicurati che questo percorso sia corretto per la tua configurazione

import pins_config as cfg
from drive import Drive
from blades import Blades
from safety import SafetyRelay
from bumper_sensors import BumperSensors
from lift_sensors import LiftSensors
from perimeter_sensors import PerimeterSensors
from aux_outputs import AuxOutputs
from pcf8574 import PCF8574
from adafruit_bno055 import BNO055_I2C
from ina226_batt import INA226
from sonar_ultrasonic import Sonar3
from protocol import (
    COBSStreamDecoder,
    pack_frame,
    parse_frame,
    MSG_CMD_DRIVE,
    MSG_CMD_BLADES,
    MSG_CMD_RELAY,
    MSG_CMD_LIMITS,
    MSG_ODOM,
    MSG_SONAR,
    # Optional telemetry IDs for future use
    MSG_EVENT,
    MSG_BLADES_RPM,
    MSG_BATT,
    MSG_IMU,
)
from encoder import SingleChannelEncoder
from odometry import DiffOdometry

# ---- Debug facility ----
DEBUG_INIT = True
DEBUG_STATUS_RATE_MS = 1000  # Log status every 1000ms
try:
    DEBUG_INIT = bool(getattr(cfg, 'DEBUG_INIT', True))
    DEBUG_STATUS_RATE_MS = int(getattr(cfg, 'DEBUG_STATUS_RATE_MS', 1000))
except Exception:
    DEBUG_INIT = True

# Global to track last debug status print
last_debug_status_ms = 0

def _dbg(msg: str):
    if DEBUG_INIT:
        try:
            print(f"[{time.ticks_ms()}] {msg}")
        except Exception:
            pass

def _debug_status():
    """Log current status of all sensors and actuators"""
    if not DEBUG_INIT:
        return
        
    try:
        # Motor states
        left_cmd = drive.left.get_command() if drive and hasattr(drive, 'left') else None
        right_cmd = drive.right.get_command() if drive and hasattr(drive, 'right') else None
        
        # Safety and power
        safety_state = safety.is_enabled() if safety else None
        try:
            batt_voltage = batt.read()[0] if (batt and hasattr(batt, 'read')) else None
        except Exception:
            batt_voltage = None
        
        # Sensor states
        bumper_left = bumpers.read()[0] if bumpers and hasattr(bumpers, 'read') else None
        bumper_right = bumpers.read()[1] if bumpers and hasattr(bumpers, 'read') else None
        lift_state = lifts.read()[0] if lifts and hasattr(lifts, 'read') else None
        
        # Perimeter sensors (if available)
        perim_left = perimeter.left_active() if perimeter and hasattr(perimeter, 'left_active') else None
        perim_right = perimeter.right_active() if perimeter and hasattr(perimeter, 'right_active') else None
        
        # IMU data (if available)
        imu_data = imu.read() if imu and hasattr(imu, 'read') else None
        
        # Sonar distances (if available)
        sonar_dist = sonar.read_all() if sonar and hasattr(sonar, 'read_all') else None
        
        # Build status string
        status = [
            "\n--- SYSTEM STATUS ---",
            f"Safety: {'ENABLED' if safety_state else 'DISABLED'}",
            f"Battery: {batt_voltage:.2f}V" if batt_voltage is not None else "Battery: N/A",
            "",
            "--- MOTORS ---",
            f"Left: {left_cmd:+.2f}" if left_cmd is not None else "Left: N/A",
            f"Right: {right_cmd:+.2f}" if right_cmd is not None else "Right: N/A",
            "",
            "--- SENSORS ---",
            f"Bumpers: L={'ON' if bumper_left else 'OFF'} R={'ON' if bumper_right else 'OFF'}" if None not in [bumper_left, bumper_right] else "Bumpers: N/A",
            f"Lift: {'UP' if lift_state else 'DOWN'}" if lift_state is not None else "Lift: N/A",
            f"Perimeter: L={'ON' if perim_left else 'OFF'} R={'ON' if perim_right else 'OFF'}" if None not in [perim_left, perim_right] else "Perimeter: N/A",
        ]
        
        # Add IMU data: prefer Euler; fallback to quat/accel/gyro; else diagnostics
        try:
            euler = imu.read_euler() if (imu and hasattr(imu, 'read_euler')) else None
        except Exception:
            euler = None
            
        if euler is not None:
            h, r, p = euler
            status.extend([
                "",
                "--- IMU ---",
                f"Euler: H={h:.2f}° R={r:.2f}° P={p:.2f}°",
            ])
        elif imu_data is not None:
            # Fallback to raw quaternion/accel/gyro if Euler not available
            if len(imu_data) >= 10:  # quat + accel + gyro
                qw, qx, qy, qz, ax, ay, az, gx, gy, gz = imu_data
                status.extend([
                    "",
                    "--- IMU ---",
                    f"Quat: W={qw:+.2f} X={qx:+.2f} Y={qy:+.2f} Z={qz:+.2f}",
                    f"Accel: X={ax:+.2f} Y={ay:+.2f} Z={az:+.2f} m/s²",
                    f"Gyro: X={gx:+.1f} Y={gy:+.1f} Z={gz:+.1f} °/s"
                ])
        
        # Add sonar data if available
        if sonar_dist and len(sonar_dist) >= 3:
            status.extend([
                "",
                "--- SONAR ---",
                f"Left: {sonar_dist[0]:.2f}m  Center: {sonar_dist[1]:.2f}m  Right: {sonar_dist[2]:.2f}m"
            ])
            
        _dbg("\n".join(status) + "\n" + "-" * 40)
        
    except Exception as e:
        _dbg(f"[DEBUG_ERR] Status update failed: {e}")
        
def _quat_to_roll_pitch_deg(qw, qx, qy, qz):
    # Compute roll, pitch in degrees from quaternion (same as old helper)
    import math
    sinr_cosp = 2.0 * (qw*qx + qy*qz)
    cosr_cosp = 1.0 - 2.0 * (qx*qx + qy*qy)
    roll = math.degrees(math.atan2(sinr_cosp, cosr_cosp))
    sinp = 2.0 * (qw*qy - qz*qx)
    if sinp >= 1.0:
        pitch = 90.0
    elif sinp <= -1.0:
        pitch = -90.0
    else:
        pitch = math.degrees(math.asin(sinp))
    return roll, pitch

    try:
        # Motor states
        left_cmd = drive.left.get_command() if drive and hasattr(drive, 'left') else None
        right_cmd = drive.right.get_command() if drive and hasattr(drive, 'right') else None
        
        # Safety and power
        safety_state = safety.is_enabled() if safety else None
        try:
            batt_voltage = batt.read()[0] if (batt and hasattr(batt, 'read')) else None
        except Exception:
            batt_voltage = None
        
        # Sensor states
        bumper_left = bumpers.read()[0] if bumpers and hasattr(bumpers, 'read') else None
        bumper_right = bumpers.read()[1] if bumpers and hasattr(bumpers, 'read') else None
        lift_state = lifts.read()[0] if lifts and hasattr(lifts, 'read') else None
        
        # Perimeter sensors (if available)
        perim_left = perimeter.left_active() if perimeter and hasattr(perimeter, 'left_active') else None
        perim_right = perimeter.right_active() if perimeter and hasattr(perimeter, 'right_active') else None
        
        # IMU data (if available)
        imu_data = imu.read() if imu and hasattr(imu, 'read') else None
        
        # Sonar distances (if available)
        sonar_dist = sonar.read_all() if sonar and hasattr(sonar, 'read_all') else None
        
        # Build status string
        status = [
            "\n--- SYSTEM STATUS ---",
            f"Safety: {'ENABLED' if safety_state else 'DISABLED'}",
            f"Battery: {batt_voltage:.2f}V" if batt_voltage is not None else "Battery: N/A",
            "",
            "--- MOTORS ---",
            f"Left: {left_cmd:+.2f}" if left_cmd is not None else "Left: N/A",
            f"Right: {right_cmd:+.2f}" if right_cmd is not None else "Right: N/A",
            "",
            "--- SENSORS ---",
            f"Bumpers: L={'ON' if bumper_left else 'OFF'} R={'ON' if bumper_right else 'OFF'}" if None not in [bumper_left, bumper_right] else "Bumpers: N/A",
            f"Lift: {'UP' if lift_state else 'DOWN'}" if lift_state is not None else "Lift: N/A",
            f"Perimeter: L={'ON' if perim_left else 'OFF'} R={'ON' if perim_right else 'OFF'}" if None not in [perim_left, perim_right] else "Perimeter: N/A",
        ]
        
        # Add IMU data: prefer Euler; fallback to quat/accel/gyro; else diagnostics
        try:
            euler = imu.read_euler() if (imu and hasattr(imu, 'read_euler')) else None
        except Exception:
            euler = None
        if euler is not None:
            h, r, p = euler
            status.extend([
                "",
                "--- IMU ---",
                f"Euler: H={h:.2f}° R={r:.2f}° P={p:.2f}°",
            ])
        else:
            # Try Adafruit-style properties as fallback
            added = False
            try:
                if imu and hasattr(imu, 'quaternion'):
                    q = imu.quaternion
                else:
                    q = None
                if q is not None:
                    qx, qy, qz, qw = q
                    status.extend(["", "--- IMU ---", f"Quat: W={qw:+.2f} X={qx:+.2f} Y={qy:+.2f} Z={qz:+.2f}"])
                    added = True
            except Exception:
                pass
            try:
                if imu and hasattr(imu, 'acceleration'):
                    acc = imu.acceleration
                else:
                    acc = None
                if acc is not None:
                    ax, ay, az = acc
                    status.append(f"Accel: X={ax:+.2f} Y={ay:+.2f} Z={az:+.2f} m/s²")
                    added = True
            except Exception:
                pass
            try:
                if imu and hasattr(imu, 'gyro'):
                    gr = imu.gyro
                else:
                    gr = None
                if gr is not None:
                    gx, gy, gz = gr
                    status.append(f"Gyro: X={gx:+.1f} Y={gy:+.1f} Z={gz:+.1f} °/s")
                    added = True
            except Exception:
                pass
            if not added:
                # Diagnostics when no IMU data available
                diag = []
                try:
                    if imu and hasattr(imu, 'ok'):
                        diag.append(f"ok={imu.ok()}")
                except Exception:
                    pass
                try:
                    if imu and hasattr(imu, 'chip_id'):
                        cid = imu.chip_id()
                        diag.append(f"chip_id=0x{cid:02X}" if isinstance(cid, int) and cid >= 0 else "chip_id=?")
                except Exception:
                    diag.append("chip_id=?")
                try:
                    if imu and hasattr(imu, 'sys_status'):
                        diag.append(f"sys_status={imu.sys_status()}")
                except Exception:
                    diag.append("sys_status=?")
                try:
                    if imu and hasattr(imu, 'sys_error'):
                        diag.append(f"sys_error={imu.sys_error()}")
                except Exception:
                    diag.append("sys_error=?")
                try:
                    if imu and hasattr(imu, 'calibration_status'):
                        diag.append(f"calib={imu.calibration_status()}")
                    elif imu and hasattr(imu, 'calib_status'):
                        diag.append(f"calib={imu.calib_status()}")
                except Exception:
                    diag.append("calib=?")
            suffix = (" [" + " ".join(diag) + "]") if diag else ""
            status.extend(["", "--- IMU ---", "IMU: N/A" + suffix])
            
        # Add sonar data if available
        if sonar_dist and len(sonar_dist) >= 3:
            status.extend([
                "",
                "--- SONAR ---",
                f"Left: {sonar_dist[0]:.2f}m  Center: {sonar_dist[1]:.2f}m  Right: {sonar_dist[2]:.2f}m"
            ])
            
        _dbg("\n".join(status) + "\n" + "-" * 40)
        
    except Exception as e:
        _dbg(f"[DEBUG_ERR] Status update failed: {e}")

# ---- UART configuration ----
# UART0: TX=GP0, RX=GP1, 115200 8N1
try:
    uart = UART(0, baudrate=115200, bits=8, parity=None, stop=1)
    _dbg('[INIT] OK: UART0')
except Exception as e:
    uart = None
    _dbg('[INIT] ERR: UART0: {}'.format(e))

# ---- Onboard LED for UART connection status ----
try:
    # Initialize onboard LED (GP25 on Pico) for UART connection indication
    led = Pin(25, Pin.OUT)
    led.off()  # Start with LED off
    _dbg('[INIT] OK: Onboard LED (GP25)')
except Exception as e:
    led = None
    _dbg('[INIT] ERR: Onboard LED: {}'.format(e))

# UART connection tracking
uart_connected = False
last_uart_activity_ms = 0
UART_LED_TIMEOUT_MS = 2000  # Keep LED on for 2 seconds after UART activity

# ---- Drive system + Blades + shared Safety ----
try:
    safety = SafetyRelay()
    _dbg('[INIT] OK: SafetyRelay')
except Exception as e:
    safety = None
    _dbg('[INIT] ERR: SafetyRelay: {}'.format(e))

try:
    drive = Drive(safety=safety)
    _dbg('[INIT] OK: Drive')
except Exception as e:
    drive = None
    _dbg('[INIT] ERR: Drive: {}'.format(e))

try:
    blades = Blades(safety=safety)
    _dbg('[INIT] OK: Blades')
except Exception as e:
    blades = None
    _dbg('[INIT] ERR: Blades: {}'.format(e))

# ---- IO peripherals (PCF8574 + sensors) ----
_pcf_addr = getattr(cfg, 'PCF8574_ADDR', 0x20)
pcf = None

# Start disabled for safety
# drive.disable() is already called in Drive.__init__

# ---- Protocol runtime ----
decoder = COBSStreamDecoder(max_frame=256)
out_seq = 0

# Safety/watchdog
DRIVE_CMD_TIMEOUT_MS = 200  # disable if no drive cmd within this window
last_drive_cmd_ms = time.ticks_ms()

# Blades RPM computation parameters
BLADE_RPM_PERIOD_MS = int(1000 // max(1, int(getattr(cfg, 'BLADE_RPM_RATE_HZ', 10))))
last_blade_rpm_ms = time.ticks_ms()
blade_rpm_1 = 0.0
blade_rpm_2 = 0.0

# Sonar telemetry parameters
SONAR_PERIOD_MS = int(1000 // max(1, int(getattr(cfg, 'SONAR_RATE_HZ', 10))))
last_sonar_ms = time.ticks_ms()

try:
    _i2c = I2C(0, sda=Pin(cfg.I2C0_SDA_PIN), scl=Pin(cfg.I2C0_SCL_PIN), freq=getattr(cfg, 'I2C0_FREQ_HZ', 400000))
    _dbg('[INIT] OK: I2C0')
except Exception as e:
    _i2c = None
    _dbg('[INIT] ERR: I2C0: {}'.format(e))

try:
    pcf = PCF8574(_i2c, _pcf_addr) if _i2c else None
    if pcf is not None:
        _dbg('[INIT] OK: PCF8574 @0x{:02X}'.format(_pcf_addr))
    else:
        _dbg('[INIT] SKIP: PCF8574 (no I2C)')
except Exception as e:
    pcf = None
    _dbg('[INIT] ERR: PCF8574: {}'.format(e))

# Battery (INA226) telemetry parameters and init
BATT_PERIOD_MS = int(1000 // max(1, int(getattr(cfg, 'BATT_RATE_HZ', 5))))
last_batt_ms = time.ticks_ms()
try:
    batt = INA226(_i2c, addr=getattr(cfg, 'INA226_ADDR', 0x40),
                  shunt_ohms=getattr(cfg, 'INA226_SHUNT_OHMS', 0.002),
                  max_current_a=getattr(cfg, 'INA226_MAX_CURRENT_A', 20.0))
    _dbg('[INIT] OK: INA226')
except Exception as e:
    batt = None
    _dbg('[INIT] ERR: INA226: {}'.format(e))

IMU_PERIOD_MS = int(1000 // max(1, int(getattr(cfg, 'IMU_RATE_HZ', 100))))
last_imu_ms = time.ticks_ms()
try:
    _dbg('[INIT] Initializing BNO055...')
    _dbg(f'[INIT] I2C scan: {_i2c.scan()}')
    imu = BNO055_I2C(_i2c, address=getattr(cfg, 'BNO055_ADDR', 0x28))
    
    if imu.ok():
        _dbg('[INIT] BNO055 initialized successfully')
        # Test read to verify communication: prefer Euler first, then full read
        _dbg('[INIT] Testing BNO055 communication...')
        euler = None
        for _try in range(5):
            euler = imu.read_euler() if hasattr(imu, 'read_euler') else None
            if euler is not None:
                break
            time.sleep_ms(100)
        if euler is not None:
            h, r, p = euler
            _dbg('[INIT] BNO055 Euler OK: H={:.2f} R={:.2f} P={:.2f} deg'.format(h, r, p))
        else:
            test_data = None
            for _try in range(5):
                test_data = imu.read()
                if test_data is not None:
                    break
                time.sleep_ms(100)
            if test_data is not None:
                _dbg('[INIT] BNO055 quat/accel/gyro read OK')
            else:
                _dbg('[INIT] WARN: BNO055 test read failed (no data)')
    else:
        _dbg('[INIT] ERR: BNO055 initialization failed')
        imu = None
except Exception as e:
    imu = None
    _dbg(f'[INIT] ERR: BNO055 initialization failed: {e}')
    import sys
    sys.print_exception(e)

# PCF8574 (inputs) init and polling
PCF_PERIOD_MS = int(1000 // max(1, int(getattr(cfg, 'PCF_POLL_HZ', 50))))
last_pcf_ms = time.ticks_ms()
try:
    # set all pins high to use as inputs with pull-ups
    _i2c.writeto(_pcf_addr, bytes((0xFF,)))
except Exception:
    pass

# Instantiate sensor helpers
bumpers = BumperSensors()
lifts = LiftSensors()
perimeter = PerimeterSensors()
aux = AuxOutputs()

# Optional: simple function to send events (e.g., relay state changes)
def send_event(bitfield: int):
    global out_seq
    # payload: uint16 little-endian
    payload = bytes((bitfield & 0xFF, (bitfield >> 8) & 0xFF))
    frame = pack_frame(MSG_EVENT, payload, out_seq)
    out_seq = (out_seq + 1) & 0xFF
    uart.write(frame)
    # UART transmission - turn on LED
    if led is not None:
        led.on()
    last_uart_activity_ms = time.ticks_ms()

# Event bits for AUX sensors from PCF8574 (draft)
EVENT_RELAY_ENABLED = 1 << 0
EVENT_BUMPER_LEFT   = 1 << 1
EVENT_BUMPER_RIGHT  = 1 << 2
EVENT_LIFT          = 1 << 3
EVENT_RAIN          = 1 << 4
# AUX from PCF8574 (bits 5-8 in bitfield)
EVENT_AUX1 = 1 << 5
EVENT_AUX2 = 1 << 6
EVENT_AUX3 = 1 << 7
EVENT_TILT = 1 << 11  # AUX4 mapped to TILT (bit 11)

# Error/diagnostics bits
EVENT_ERR_PCF    = 1 << 10  # Bit 10 for PCF error
EVENT_ERR_IMU    = 1 << 12
EVENT_ERR_BATT   = 1 << 13
EVENT_ERR_SONAR  = 1 << 14
EVENT_ERR_ODOM   = 1 << 15


# Track last states to emit events on change
last_enabled_state = False
last_tilt_active = False
last_bumper = [False, False]
last_lift = False
last_rain = False
last_perim = [False, False]
last_aux = [False, False, False, False]  # AUX1, AUX2, AUX3, AUX4


def handle_cmd(msg_id: int, payload: bytes):
    global last_drive_cmd_ms, last_enabled_state

    if msg_id == MSG_CMD_DRIVE:
        # payload: 2 * float32 LE (left, right) in [-1..1]
        if len(payload) != 8:
            return
        # Manual unpack without struct to reduce overhead
        import ustruct
        left, right = ustruct.unpack('<ff', payload)
        drive.command(left, right)
        last_drive_cmd_ms = time.ticks_ms()

    elif msg_id == MSG_CMD_RELAY:
        # payload: 1 byte (0/1)
        if not payload:
            return
        en = 1 if payload[0] != 0 else 0
        if en:
            safety.enable()
        else:
            safety.disable()
        # Emit event on change
        enabled = en == 1
        if enabled != last_enabled_state:
            bf = EVENT_RELAY_ENABLED if enabled else 0
            send_event(bf)
            last_enabled_state = enabled

    elif msg_id == MSG_CMD_LIMITS:
        # payload: 2 * float32 LE (max_abs_speed, accel_limit)
        if len(payload) != 8:
            return
        import ustruct
        max_abs_speed, accel_limit = ustruct.unpack('<ff', payload)
        drive.set_limits(max_abs_speed=max_abs_speed, accel_limit=accel_limit)

    elif msg_id == MSG_CMD_BLADES:
        # payload: 2 * float32 LE (blade1, blade2) in [-1..1]
        if len(payload) != 8:
            return
        import ustruct
        b1, b2 = ustruct.unpack('<ff', payload)
        blades.command(b1, b2)


# ---- Main loop ----
READ_CHUNK = 128
UPDATE_PERIOD_MS = 20  # 50 Hz motor update loop
last_update_ms = time.ticks_ms()

# ---- Encoders + Odometry ----
try:
    enc_left = SingleChannelEncoder(cfg.MOTOR_LEFT_ENC_PIN, debounce_us=200)
    enc_right = SingleChannelEncoder(cfg.MOTOR_RIGHT_ENC_PIN, debounce_us=200)
    _dbg('[INIT] OK: Encoders')
except Exception as e:
    enc_left = None
    enc_right = None
    _dbg('[INIT] ERR: Encoders: {}'.format(e))

try:
    odom = DiffOdometry(
        ticks_per_rev_motor=cfg.TICKS_PER_REV_MOTOR,
        gear_ratio=cfg.GEAR_RATIO,
        wheel_radius_m=cfg.WHEEL_RADIUS_M,
        wheel_separation_m=cfg.WHEEL_SEPARATION_M,
    )
    _dbg('[INIT] OK: DiffOdometry')
except Exception as e:
    odom = None
    _dbg('[INIT] ERR: DiffOdometry: {}'.format(e))

# Blade encoders (for RPM)
blade_enc1 = SingleChannelEncoder(cfg.BLADE1_ENC_PIN, debounce_us=150)
blade_enc2 = SingleChannelEncoder(cfg.BLADE2_ENC_PIN, debounce_us=150)

try:
    sonar = Sonar3(
        cfg.US_FRONT_LEFT_TRIG_PIN, cfg.US_FRONT_LEFT_ECHO_PIN,
        cfg.US_FRONT_CENTER_TRIG_PIN, cfg.US_FRONT_CENTER_ECHO_PIN,
        cfg.US_FRONT_RIGHT_TRIG_PIN, cfg.US_FRONT_RIGHT_ECHO_PIN,
        timeout_us=getattr(cfg, 'SONAR_TIMEOUT_US', 30000)
    )
    _dbg('[INIT] OK: Sonar3')
except Exception as e:
    sonar = None
    _dbg('[INIT] ERR: Sonar3: {}'.format(e))

ODOM_PERIOD_MS = int(1000 // max(1, int(cfg.ODOM_RATE_HZ)))
last_odom_ms = time.ticks_ms()

# --- Module availability flags & initial error event ---
imu_ok = (imu is not None)
try:
    if imu_ok and hasattr(imu, 'ok'):
        imu_ok = bool(imu.ok())
except Exception:
    imu_ok = False

batt_ok = (batt is not None)
try:
    if batt_ok and hasattr(batt, 'ok'):
        batt_ok = bool(batt.ok())
except Exception:
    batt_ok = False

sonar_ok = (sonar is not None)

odom_ok = (odom is not None) and (enc_left is not None) and (enc_right is not None)

pcf_ok = (pcf is not None)

# Build initial error bitfield and send once so host knows status
initial_err = 0
if not imu_ok:
    initial_err |= EVENT_ERR_IMU
if not batt_ok:
    initial_err |= EVENT_ERR_BATT
if not sonar_ok:
    initial_err |= EVENT_ERR_SONAR
if not odom_ok:
    initial_err |= EVENT_ERR_ODOM
if not pcf_ok:
    initial_err |= EVENT_ERR_PCF
if initial_err:
    try:
        send_event(initial_err)
    except Exception:
        pass

# Sign source is configured via cfg.ENCODER_SIGN_SOURCE; if "dir", we read direction via drive.dir_forward()

while True:
    # Read and feed UART
    try:
        data = uart.read(READ_CHUNK)
        if data:
            # UART activity detected - turn on LED
            if led is not None:
                led.on()
                uart_connected = True
            last_uart_activity_ms = time.ticks_ms()

            decoder.feed(data)
            # Process any full frames
            for decoded in decoder.frames():
                parsed = parse_frame(decoded)
                if parsed is None:
                    continue
                msg_id, seq, ts_ms, payload = parsed
                handle_cmd(msg_id, payload)
    except Exception:
        # Robustness: ignore UART errors; continue loop
        pass

    # Watchdog: if drive commands timed out, disable for safety
    now = time.ticks_ms()
    if time.ticks_diff(now, last_drive_cmd_ms) > DRIVE_CMD_TIMEOUT_MS:
        safety.disable()

    # Periodic motor update
    if time.ticks_diff(now, last_update_ms) >= UPDATE_PERIOD_MS:
        last_update_ms = now
        try:
            drive.update()
        except Exception:
            # Ensure motors are disabled on unexpected error
            safety.disable()
        try:
            blades.update()
        except Exception:
            # Do not impact drive if blades fail; ensure safety if severe
            pass

    # PCF8574 + perimeter event publishing on change
    if time.ticks_diff(now, last_pcf_ms) >= PCF_PERIOD_MS:
        last_pcf_ms = now
        try:
            # Read rain via PCF bit mapping (active-low)
            rain = False
            if pcf is not None:
                raw = pcf.port
                bit_rain = getattr(cfg, 'PCF_BIT_RAIN', 0)
                v = (raw >> int(bit_rain)) & 1
                rain = (v == 0)  # active-low

            # AUX sensors from PCF8574 (active-high assumed)
            if pcf is not None and 'raw' in locals():
                aux1 = bool((raw >> int(cfg.PCF_BIT_AUX1)) & 1)
                aux2 = bool((raw >> int(cfg.PCF_BIT_AUX2)) & 1)
                aux3 = bool((raw >> int(cfg.PCF_BIT_AUX3)) & 1)
                aux4 = bool((raw >> int(cfg.PCF_BIT_AUX4)) & 1)
            else:
                aux1 = aux2 = aux3 = aux4 = False

            # Lift and bumpers via helper classes
            lift_vals = lifts.read()
            lift = bool(lift_vals[0]) if lift_vals else False
            bL, bR = False, False
            bvr = bumpers.read()
            if len(bvr) >= 2:
                bL, bR = bool(bvr[0]), bool(bvr[1])

            # Perimeter states
            pL = perimeter.left_active()
            pR = perimeter.right_active()

            # Build current event bitfield
            bf = 0
            if safety.is_enabled():
                bf |= EVENT_RELAY_ENABLED
            if bL:
                bf |= EVENT_BUMPER_LEFT
            if bR:
                bf |= EVENT_BUMPER_RIGHT
            if lift:
                bf |= EVENT_LIFT
            if rain:
                bf |= EVENT_RAIN
            if pL:
                bf |= EVENT_PERIMETER_LEFT
            if pR:
                bf |= EVENT_PERIMETER_RIGHT
            if aux1:
                bf |= EVENT_AUX1
            if aux2:
                bf |= EVENT_AUX2
            if aux3:
                bf |= EVENT_AUX3
            if aux4:
                bf |= EVENT_TILT

            # Detect if any change occurred vs last snapshot
            changed = (
                (bL != last_bumper[0]) or
                (bR != last_bumper[1]) or
                (lift != last_lift) or
                (rain != last_rain) or
                (pL != last_perim[0]) or
                (pR != last_perim[1]) or
                (aux1 != last_aux[0]) or
                (aux2 != last_aux[1]) or
                (aux3 != last_aux[2]) or
                (aux4 != last_aux[3])
            )
            if changed:
                send_event(bf)
                last_bumper = [bL, bR]
                last_lift = lift
                last_rain = rain
                last_perim = [pL, pR]
                last_aux = [aux1, aux2, aux3, aux4]
        except Exception:
            pass

    # Odometry publish loop @ cfg.ODOM_RATE_HZ
    if time.ticks_diff(now, last_odom_ms) >= ODOM_PERIOD_MS:
        dt = time.ticks_diff(now, last_odom_ms) / 1000.0
        last_odom_ms = now

        # Read ticks from encoders (if available)
        if odom_ok:
            tL = enc_left.read_and_reset()
            tR = enc_right.read_and_reset()
        else:
            tL = 0
            tR = 0

        # Determine sign per wheel
        if cfg.ENCODER_SIGN_SOURCE == "dir":
            sL = 1 if drive.left.dir_forward() else -1
            sR = 1 if drive.right.dir_forward() else -1
        else:
            # sign from last commanded values
            sL = 1 if drive.left.get_command() >= 0.0 else -1
            sR = 1 if drive.right.get_command() >= 0.0 else -1

        # Apply optional inversion flags
        if getattr(cfg, 'LEFT_DIR_INVERTED', False):
            sL = -sL
        if getattr(cfg, 'RIGHT_DIR_INVERTED', False):
            sR = -sR

        # Apply sign to tick counts
        tL *= sL
        tR *= sR

        # Update odometry and publish
        try:
            import ustruct, math
            if odom_ok:
                odom.update(tL, tR, dt)
                x, y, th, vx, vy, vth = odom.state()
            else:
                # Use NaN sentinels when odom is not available
                nan = float('nan')
                x = y = th = vx = vy = vth = nan
                # Publish sentinel telemetry when odom is not available
                bf = EVENT_ERR_ODOM
                send_event(bf)
            # Pack 6×float32 little-endian
            payload = ustruct.pack('<ffffff', x, y, th, vx, vy, vth)
            frame = pack_frame(MSG_ODOM, payload, out_seq)
            out_seq = (out_seq + 1) & 0xFF
            uart.write(frame)
            if led is not None:
                led.on()
            last_uart_activity_ms = time.ticks_ms()
        except Exception:
            pass

    # Blades RPM computation at fixed period
    if time.ticks_diff(now, last_blade_rpm_ms) >= BLADE_RPM_PERIOD_MS:
        dt_b = time.ticks_diff(now, last_blade_rpm_ms) / 1000.0
        try:
            c1 = blade_enc1.read_and_reset()
            c2 = blade_enc2.read_and_reset()
            # pulses per blade revolution
            ppr_blade = float(getattr(cfg, 'BLADE_PULSES_PER_REV_MOTOR', 6)) * float(getattr(cfg, 'BLADE_GEAR_RATIO', 1.0))
            if ppr_blade <= 0:
                ppr_blade = 1.0
            # RPM = rev/s * 60
            blade_rpm_1 = (float(c1) / ppr_blade) / dt_b * 60.0 if dt_b > 0 else 0.0
            blade_rpm_2 = (float(c2) / ppr_blade) / dt_b * 60.0 if dt_b > 0 else 0.0
            import ustruct
            payload = ustruct.pack('<ff', blade_rpm_1, blade_rpm_2)
            frame = pack_frame(MSG_BLADES_RPM, payload, out_seq)
            out_seq = (out_seq + 1) & 0xFF
            uart.write(frame)
            if led is not None:
                led.on()
            last_uart_activity_ms = time.ticks_ms()
        except Exception:
            # leave last RPM values
            pass

    # Sonar telemetry publish loop @ cfg.SONAR_RATE_HZ
    if time.ticks_diff(now, last_sonar_ms) >= SONAR_PERIOD_MS:
        last_sonar_ms = now
        try:
            if sonar_ok:
                dL, dC, dR = sonar.read_all()
            else:
                dL = dC = dR = -1.0
                # Publish sentinel telemetry when sonar is not available
                bf = EVENT_ERR_SONAR
                send_event(bf)
            # pack 3x float32 (meters); use -1.0 for timeout
            import ustruct
            payload = ustruct.pack('<fff', dL, dC, dR)
            frame = pack_frame(MSG_SONAR, payload, out_seq)
            out_seq = (out_seq + 1) & 0xFF
            uart.write(frame)
            if led is not None:
                led.on()
            last_uart_activity_ms = time.ticks_ms()

        except Exception:
            pass

    # Battery telemetry (INA226) publish loop @ cfg.BATT_RATE_HZ
    if time.ticks_diff(now, last_batt_ms) >= BATT_PERIOD_MS:
        last_batt_ms = now
        try:
            import ustruct
            if batt_ok and batt.ok():
                vb = batt.read()
                if vb is None:
                    raise Exception('Batt read failed')
                voltage_V, current_A = vb
            else:
                voltage_V, current_A = -1.0, 0.0
                # Publish sentinel telemetry when batt is not available
                bf = EVENT_ERR_BATT
                send_event(bf)
            payload = ustruct.pack('<ff', voltage_V, current_A)
            frame = pack_frame(MSG_BATT, payload, out_seq)
            out_seq = (out_seq + 1) & 0xFF
            uart.write(frame)
            # UART transmission - turn on LED
            if led is not None:
                led.on()
            last_uart_activity_ms = time.ticks_ms()
        except Exception:
            pass

    # Periodic debug status
    now = time.ticks_ms()
    if time.ticks_diff(now, last_debug_status_ms) >= DEBUG_STATUS_RATE_MS:
        _debug_status()
        last_debug_status_ms = now

    # UART LED timeout management
    if led is not None and uart_connected:
        if time.ticks_diff(now, last_uart_activity_ms) > UART_LED_TIMEOUT_MS:
            led.off()
            uart_connected = False

    # Small sleep to yield
    time.sleep_ms(1)
