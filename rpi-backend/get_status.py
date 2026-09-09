#!/usr/bin/env python3
"""get_status.py — Raspberry Pi side smoke test: open the serial link to the
ESP32-S3, send "Get full status" (opcode 0x26), print whatever comes back.

Usage: python3 get_status.py [/dev/ttyUSB0]
"""
import sys
import time
import serial
from pathlib import Path
PROJECT_ROOT = Path(__file__).resolve().parent.parent
PROTOCOL_DIR = PROJECT_ROOT / "protocol"
sys.path.insert(0, str(PROTOCOL_DIR))



from protocol import Receiver, build_frame, MsgType, Opcode


PORT = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"

BAUD = 115200

def main() -> int:
    print(f"Opening {PORT} @ {BAUD}...")
    try:
        ser = serial.Serial(PORT, BAUD, timeout=0.2)
    except serial.SerialException as e:
        print(f"Could not open {PORT}: {e}")
        print("Check: is the ESP32-S3 plugged in? Is this the right /dev/tty* port?")
        print("List candidates with:  ls /dev/tty.* /dev/ttyUSB* /dev/ttyACM* 2>/dev/null")
        return 1

    rx = Receiver()
    frame = build_frame(MsgType.CMD, msg_id=1, payload=bytes([Opcode.GET_FULL_STATUS]))
    ser.write(frame)
    print(f"Sent GET_FULL_STATUS ({len(frame)} bytes on the wire)")

    deadline = time.time() + 2.0
    while time.time() < deadline:
        chunk = ser.read(64)
        if chunk:
            for f in rx.feed(chunk):
                print(f"Got reply: type=0x{f.msg_type:02X} id={f.msg_id} payload={f.payload.hex()}")
                return 0
        if rx.last_error:
            print(f"Protocol error while waiting: {rx.last_error}")
            rx.last_error = None

    print("No reply within 2s — expected until the ESP32-S3 firmware in step 4 is flashed and wired up.")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
