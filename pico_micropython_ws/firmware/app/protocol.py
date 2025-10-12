# MicroPython-friendly serial protocol utilities: COBS framing + CRC16-CCITT
# Frame format (before COBS):
#   [msg_id(1)][len(1)][seq(1)][ts_ms(4)][payload(len)][crc16(2)]
# After COBS-encoding, a single 0x00 delimiter terminates each frame on the wire.

from micropython import const
# ---- Constants ----
HEADER_SIZE = const(7)   # msg_id(1) + len(1) + seq(1) + ts_ms(4)
CRC_SIZE = const(2)
FRAME_OVERHEAD = const(HEADER_SIZE + CRC_SIZE)

# Message IDs (Pico  Host and Host  Pico)
MSG_IMU = const(0x01)
MSG_ODOM = const(0x02)
MSG_SONAR = const(0x03)
MSG_BATT = const(0x04)
MSG_EVENT = const(0x05)
MSG_BLADES_RPM = const(0x06)

MSG_CMD_DRIVE  = const(0x10)
MSG_CMD_BLADES = const(0x11)
MSG_CMD_RELAY  = const(0x12)
MSG_CMD_LIMITS = const(0x13)
# ---- CRC16-CCITT (poly 0x1021, init 0xFFFF, no xorout) ----
def crc16_ccitt(data: bytes, init: int = 0xFFFF) -> int:
    crc = init
    for b in data:
        crc ^= (b << 8) & 0xFFFF
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc & 0xFFFF

# ---- Minimal COBS encode/decode ----
# Note: We terminate frames with a single zero byte (0x00) after COBS-encoding.
# See: https://en.wikipedia.org/wiki/Consistent_Overhead_Byte_Stuffing

def cobs_encode(data: bytes) -> bytes:
    if not data:
        return b"\x01\x00"
    out = bytearray()
    idx = 0
    while idx < len(data):
        code_idx = len(out)
        out.append(0)  # placeholder for code
        code = 1
        while idx < len(data) and data[idx] != 0 and code < 0xFF:
            out.append(data[idx])
            idx += 1
            code += 1
        out[code_idx] = code
        if idx < len(data) and data[idx] == 0:
            idx += 1  # skip the zero byte
    out.append(0)  # delimiter
    return bytes(out)


def cobs_decode(stream: bytes) -> bytes:
    # stream should be a single COBS-encoded frame WITHOUT the trailing delimiter
    out = bytearray()
    idx = 0
    while idx < len(stream):
        code = stream[idx]
        if code == 0:
            # invalid in COBS body
            return b""
        idx += 1
        end = idx + code - 1
        if end > len(stream):
            # truncated
            return b""
        out.extend(stream[idx:end])
        idx = end
        if code < 0xFF and idx < len(stream):
            out.append(0)
    return bytes(out)

# ---- Packing/unpacking frames ----
import ustruct as _struct
import time


def pack_frame(msg_id: int, payload: bytes, seq: int) -> bytes:
    if payload is None:
        payload = b""
    if len(payload) > 255:
        raise ValueError("payload too large")
    ts_ms = time.ticks_ms() & 0xFFFFFFFF
    header = _struct.pack('<BBBI', msg_id & 0xFF, len(payload) & 0xFF, seq & 0xFF, ts_ms)
    body = header + payload
    crc = crc16_ccitt(body)
    frame = body + _struct.pack('<H', crc)
    return cobs_encode(frame)


class COBSStreamDecoder:
    """Incremental decoder. Feed raw bytes from UART; get full decoded frames (raw body bytes)."""
    def __init__(self, max_frame=512):
        self._buf = bytearray()
        self._frames = []
        self._max = max_frame

    def feed(self, data: bytes):
        if not data:
            return
        for b in data:
            if b == 0:
                # end of frame; decode
                if self._buf:
                    decoded = cobs_decode(self._buf)
                    if decoded:
                        self._frames.append(decoded)
                self._buf = bytearray()
            else:
                if len(self._buf) < self._max:
                    self._buf.append(b)
                else:
                    # overflow: reset buffer
                    self._buf = bytearray()

    def frames(self):
        if not self._frames:
            return []
        out = self._frames
        self._frames = []
        return out


def parse_frame(decoded: bytes):
    """Returns (msg_id, seq, ts_ms, payload) if CRC valid else None."""
    if len(decoded) < FRAME_OVERHEAD:
        return None
    # verify CRC
    body = decoded[:-2]
    rx_crc = (decoded[-2] | (decoded[-1] << 8)) & 0xFFFF
    calc_crc = crc16_ccitt(body)
    if rx_crc != calc_crc:
        return None
    # unpack header
    msg_id, length, seq, ts_ms = _struct.unpack_from('<BBBI', body, 0)
    if HEADER_SIZE + length != len(body):
        return None
    payload = body[HEADER_SIZE:]
    return (msg_id, seq, ts_ms, payload)
