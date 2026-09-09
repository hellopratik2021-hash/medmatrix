"""protocol.py — MedMetrix shared framing/CRC/opcode library, Python port.

Byte-for-byte compatible with mm_protocol.h: a frame built by this module
decodes correctly on the ESP32-S3/Mega C implementation and vice versa.
"""
from __future__ import annotations
from dataclasses import dataclass, field
from enum import IntEnum

MAX_PAYLOAD = 64


class MsgType(IntEnum):
    CMD = 0x01
    RESP = 0x02
    TELEM = 0x03
    ERR = 0x04


class Opcode(IntEnum):
    SET_PUMP1_RATE = 0x10
    SET_PUMP2_RATE = 0x11
    SET_O2_VALVE_POS = 0x12
    HOME_O2_VALVE = 0x13
    QUERY_ESTOP = 0x20
    GET_ECG = 0x21
    GET_IMU = 0x22
    GET_SPO2 = 0x23
    GET_TEMP = 0x24
    GET_NIBP = 0x25  # reserved: hardware not yet decided
    GET_FULL_STATUS = 0x26


class TelemType(IntEnum):
    ECG_SAMPLE = 0x01
    IMU_SAMPLE = 0x02
    SPO2_READING = 0x03
    TEMP_READING = 0x04
    ESTOP_CHANGE = 0x05
    MOTOR_STATUS = 0x06
    PROTOCOL_ERR = 0x07


@dataclass
class Frame:
    msg_type: int
    msg_id: int
    payload: bytes = field(default=b"")


def crc16(data: bytes) -> int:
    """CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF)."""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte << 8
        for _ in range(8):
            if crc & 0x8000:
                crc = ((crc << 1) ^ 0x1021) & 0xFFFF
            else:
                crc = (crc << 1) & 0xFFFF
    return crc


def cobs_encode(data: bytes) -> bytes:
    out = bytearray()
    idx = 0
    while True:
        next_zero = data.find(0, idx)
        chunk_end = next_zero if next_zero != -1 else len(data)
        chunk = data[idx:chunk_end]
        pos = 0
        while len(chunk) - pos >= 254:
            out.append(0xFF)
            out.extend(chunk[pos:pos + 254])
            pos += 254
        remaining = chunk[pos:]
        out.append(len(remaining) + 1)
        out.extend(remaining)
        if next_zero == -1:
            break
        idx = next_zero + 1
    return bytes(out)


def cobs_decode(data: bytes) -> bytes:
    out = bytearray()
    i = 0
    n = len(data)
    while i < n:
        code = data[i]
        if code == 0 or i + code > n + 1:
            raise ValueError("malformed COBS data")
        i += 1
        out.extend(data[i:i + code - 1])
        i += code - 1
        if code != 0xFF and i < n:
            out.append(0)
    return bytes(out)


def build_frame(msg_type: int, msg_id: int, payload: bytes = b"") -> bytes:
    if len(payload) > MAX_PAYLOAD:
        raise ValueError("payload too large")
    raw = bytes([msg_type, msg_id, len(payload)]) + payload
    crc = crc16(raw)
    raw += bytes([(crc >> 8) & 0xFF, crc & 0xFF])
    return cobs_encode(raw) + b"\x00"


class Receiver:
    """Streaming decoder: feed() one byte (or a bytes chunk) at a time off
    the serial port. Yields Frame objects for valid frames; corrupted
    frames are dropped silently (check .last_error after a call)."""

    def __init__(self) -> None:
        self._buf = bytearray()
        self.last_error: str | None = None
        self.corrupt_frame_count = 0

    def feed(self, data: bytes) -> list[Frame]:
        frames: list[Frame] = []
        for byte in data:
            frame = self._feed_one(byte)
            if frame is not None:
                frames.append(frame)
        return frames

    def _feed_one(self, byte: int) -> Frame | None:
        if byte == 0:
            if not self._buf:
                return None
            chunk, self._buf = bytes(self._buf), bytearray()
            try:
                decoded = cobs_decode(chunk)
            except ValueError:
                self.last_error = "cobs_decode_failed"
                self.corrupt_frame_count += 1
                return None

            if len(decoded) < 5:
                self.last_error = "frame_too_short"
                self.corrupt_frame_count += 1
                return None

            payload_len = decoded[2]
            if 3 + payload_len + 2 != len(decoded):
                self.last_error = "length_mismatch"
                self.corrupt_frame_count += 1
                return None

            rx_crc = (decoded[3 + payload_len] << 8) | decoded[3 + payload_len + 1]
            calc_crc = crc16(decoded[: 3 + payload_len])
            if rx_crc != calc_crc:
                self.last_error = "crc_mismatch"
                self.corrupt_frame_count += 1
                return None

            self.last_error = None
            return Frame(
                msg_type=decoded[0],
                msg_id=decoded[1],
                payload=bytes(decoded[3 : 3 + payload_len]),
            )

        self._buf.append(byte)
        return None
