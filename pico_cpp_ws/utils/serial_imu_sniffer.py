#!/usr/bin/env python3
import argparse
import serial
import sys
import time
import json
import re
import struct


def looks_like_text(b: bytes) -> bool:
    # Considera testo se percentuale di ASCII stampabili è alta
    printable = sum(32 <= c <= 126 or c in (9,10,13) for c in b)
    return printable / max(1, len(b)) > 0.8

def highlight_if_imu(line: str) -> str:
    kw = ["imu", "acc", "gyro", "ax", "ay", "az", "gx", "gy", "gz", "yaw", "pitch", "roll", "quat", "q0", "q1", "q2", "q3"]
    if any(k in line.lower() for k in kw):
        return f"[IMU] {line}"
    return line

def try_parse_json(line: str):
    try:
        obj = json.loads(line)
        return obj
    except Exception:
        return None

def hexdump(b: bytes) -> str:
    return " ".join(f"{x:02X}" for x in b)

# ---- Pico protocol helpers (COBS + CRC16-CCITT, little-endian CRC) ----
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for byte in data:
        crc ^= (byte & 0xFF) << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) & 0xFFFF) ^ 0x1021
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

def cobs_decode(inp: bytes) -> bytes | None:
    out = bytearray()
    i = 0
    n = len(inp)
    while i < n:
        code = inp[i]
        i += 1
        if code == 0:
            return None
        for j in range(1, code):
            if i >= n:
                return None
            out.append(inp[i])
            i += 1
        if code < 0xFF and i < n:
            out.append(0)
    return bytes(out)

MSG_TLM_IMU = 0x01
MSG_TLM_ODOM = 0x02
MSG_TLM_SONAR = 0x03
MSG_TLM_BATT = 0x04
MSG_TLM_EVENT = 0x05
MSG_TLM_BLADES_RPM = 0x06

# Event bits (from firmware main.py)
EVENT_RELAY_ENABLED   = 1 << 0
EVENT_BUMPER_LEFT     = 1 << 1
EVENT_BUMPER_RIGHT    = 1 << 2
EVENT_LIFT            = 1 << 3
EVENT_RAIN            = 1 << 4
EVENT_AUX1            = 1 << 5
EVENT_AUX2            = 1 << 6
EVENT_AUX3            = 1 << 7
EVENT_AUX4            = 1 << 8
EVENT_PERIMETER_LEFT  = 1 << 9
EVENT_PERIMETER_RIGHT = 1 << 10
EVENT_TILT            = 1 << 11
EVENT_ERR_PCF         = 1 << 12
EVENT_ERR_IMU         = 1 << 13
EVENT_ERR_BATT        = 1 << 14
EVENT_ERR_SONAR       = 1 << 15

def format_event_bits(ev: int) -> str:
    names = []
    if ev & EVENT_RELAY_ENABLED: names.append('RELAY_ENABLED')
    if ev & EVENT_BUMPER_LEFT: names.append('BUMPER_LEFT')
    if ev & EVENT_BUMPER_RIGHT: names.append('BUMPER_RIGHT')
    if ev & EVENT_LIFT: names.append('LIFT')
    if ev & EVENT_RAIN: names.append('RAIN')
    if ev & EVENT_PERIMETER_LEFT: names.append('PERIMETER_LEFT')
    if ev & EVENT_PERIMETER_RIGHT: names.append('PERIMETER_RIGHT')
    if ev & EVENT_AUX1: names.append('AUX1')
    if ev & EVENT_AUX2: names.append('AUX2')
    if ev & EVENT_AUX3: names.append('AUX3')
    if ev & EVENT_AUX4: names.append('AUX4')
    if ev & EVENT_TILT: names.append('TILT')
    if ev & EVENT_ERR_PCF: names.append('ERR_PCF')
    if ev & EVENT_ERR_IMU: names.append('ERR_IMU')
    if ev & EVENT_ERR_BATT: names.append('ERR_BATT')
    if ev & EVENT_ERR_SONAR: names.append('ERR_SONAR')
    return ','.join(names) if names else 'NONE'

def pico_parse_and_print(decoded: bytes):
    if len(decoded) < 1 + 1 + 1 + 4 + 2:
        return False
    msg_id = decoded[0]
    length = decoded[1]
    seq = decoded[2]
    ts_ms = decoded[3] | (decoded[4] << 8) | (decoded[5] << 16) | (decoded[6] << 24)
    payload_offset = 1 + 1 + 1 + 4
    expected_total = payload_offset + length + 2
    if len(decoded) != expected_total:
        return False
    crc_rx = decoded[-2] | (decoded[-1] << 8)
    crc_calc = crc16_ccitt(decoded[:-2])
    if crc_rx != crc_calc:
        return False

    payload = decoded[payload_offset:-2]
    t = ts_ms / 1000.0
    if msg_id == MSG_TLM_IMU and len(payload) >= 10*4:
        f = struct.unpack('<10f', payload[:40])
        w,x,y,z, ax,ay,az, gx,gy,gz = f
        print(f"{t:.3f} [PICO IMU] seq={seq} quat=[{w:.3f},{x:.3f},{y:.3f},{z:.3f}] acc=[{ax:.3f},{ay:.3f},{az:.3f}] gyro=[{gx:.3f},{gy:.3f},{gz:.3f}]")
        return True
    elif msg_id == MSG_TLM_ODOM and len(payload) >= 12:
        dl_ticks, dr_ticks, dt = struct.unpack('<ii f', payload[:12])
        print(f"{t:.3f} [PICO ODOM] seq={seq} dl_ticks={dl_ticks} dr_ticks={dr_ticks} dt={dt:.4f}")
        return True
    elif msg_id == MSG_TLM_SONAR and len(payload) >= 12:
        s = struct.unpack('<3f', payload[:12])
        print(f"{t:.3f} [PICO SONAR] L={s[0]:.2f} C={s[1]:.2f} R={s[2]:.2f}")
        return True
    elif msg_id == MSG_TLM_BATT and len(payload) >= 8:
        v, i_cur = struct.unpack('<2f', payload[:8])
        print(f"{t:.3f} [PICO BATT] V={v:.2f}V I={i_cur:.2f}A")
        return True
    elif msg_id == MSG_TLM_EVENT and len(payload) >= 2:
        ev = payload[0] | (payload[1] << 8)
        print(f"{t:.3f} [PICO EVENT] code=0x{ev:04X} flags=[{format_event_bits(ev)}]")
        return True
    elif msg_id == MSG_TLM_BLADES_RPM and len(payload) >= 8:
        r0, r1 = struct.unpack('<2f', payload[:8])
        print(f"{t:.3f} [PICO BLADES] rpm0={r0:.1f} rpm1={r1:.1f}")
        return True
    else:
        print(f"{t:.3f} [PICO UNKNOWN] id=0x{msg_id:02X} len={length}")
        return True

def main():
    ap = argparse.ArgumentParser(description="Serial IMU sniffer")
    ap.add_argument("--port", default="/dev/ttyAMA0")
    ap.add_argument("--baud", type=int, default=230400)
    ap.add_argument("--timeout", type=float, default=0.1, help="Serial timeout seconds")
    ap.add_argument("--raw-bytes", action="store_true", help="Mostra hexdump anche se sembra testo")
    ap.add_argument("--rx-bytes", type=int, default=256, help="Dimensione lettura in modalità binaria")
    ap.add_argument("--newline-mode", action="store_true", help="Forza lettura per linee (utile per CSV/JSON)")
    ap.add_argument("--regex", default="", help="Mostra solo linee che combaciano (es. 'imu|acc|gyro')")
    ap.add_argument("--pico", action="store_true", help="Decodifica protocollo Pico (COBS+CRC) e stampa IMU/ODOM/...) ")
    args = ap.parse_args()

    try:
        ser = serial.Serial(args.port, args.baud, timeout=args.timeout)
    except Exception as e:
        print(f"Errore apertura seriale {args.port} @ {args.baud}: {e}", file=sys.stderr)
        sys.exit(1)

    print(f"Aperta {args.port} @ {args.baud}. Ctrl-C per uscire.")
    filt = re.compile(args.regex, re.IGNORECASE) if args.regex else None

    try:
        if args.pico:
            # Pico protocol mode: COBS-framed packets terminated by 0x00
            buf = bytearray()
            while True:
                chunk = ser.read(args.rx_bytes)
                if not chunk:
                    continue
                buf.extend(chunk)
                while True:
                    try:
                        idx = buf.index(0x00)
                    except ValueError:
                        break
                    frame = bytes(buf[:idx])
                    del buf[:idx+1]
                    if not frame:
                        continue
                    decoded = cobs_decode(frame)
                    if decoded is None:
                        continue
                    # If regex provided, apply to HEX of decoded for quick filtering
                    if filt and not filt.search(hexdump(decoded)):
                        continue
                    if not pico_parse_and_print(decoded):
                        # fallback to hex if parse fails
                        print(f"{time.time():.3f} [PICO HEX] {hexdump(decoded)}")
        elif args.newline_mode:
            # Lettura riga per riga
            while True:
                line_b = ser.readline()
                if not line_b:
                    continue
                ts = time.time()
                try:
                    line = line_b.decode(errors="replace").strip()
                except Exception:
                    line = repr(line_b)
                if filt and not filt.search(line):
                    continue
                obj = try_parse_json(line)
                if obj is not None:
                    # Se JSON, evidenzia campi tipici IMU
                    keys = obj.keys()
                    mark = "[JSON/IMU]" if any(k.lower() in ("imu","ax","ay","az","gx","gy","gz","yaw","pitch","roll","q0","q1","q2","q3") for k in keys) else "[JSON]"
                    print(f"{ts:.3f} {mark} {json.dumps(obj)}")
                else:
                    print(f"{ts:.3f} {highlight_if_imu(line)}")
        else:
            # Lettura a blocchi, stampa testo o hexdump
            while True:
                b = ser.read(args.rx_bytes)
                if not b:
                    continue
                ts = time.time()
                if looks_like_text(b) and not args.raw_bytes:
                    # Prova a splittare su newline per migliorare la leggibilità
                    for part in b.splitlines():
                        line = part.decode(errors="replace")
                        if filt and not filt.search(line):
                            continue
                        obj = try_parse_json(line.strip())
                        if obj is not None:
                            mark = "[JSON/IMU]" if any(k.lower() in ("imu","ax","ay","az","gx","gy","gz","yaw","pitch","roll","q0","q1","q2","q3") for k in obj.keys()) else "[JSON]"
                            print(f"{ts:.3f} {mark} {json.dumps(obj)}")
                        else:
                            print(f"{ts:.3f} {highlight_if_imu(line.rstrip())}")
                else:
                    # Binario
                    dump = hexdump(b)
                    if not filt or filt.search(dump):
                        print(f"{ts:.3f} [HEX] {dump}")
    except KeyboardInterrupt:
        pass
    finally:
        ser.close()

if __name__ == "__main__":
    main()

