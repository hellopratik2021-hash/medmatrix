"""test_protocol.py — Python-side self-test, mirrors test_protocol.c."""
import random
from protocol import Receiver, build_frame, crc16, Opcode, MsgType

fails = 0


def check(cond: bool, msg: str) -> None:
    global fails
    print(f"  [{'OK' if cond else 'FAIL'}] {msg}")
    if not cond:
        fails += 1


def test_crc_known_vector():
    print("Test 1: CRC-16/CCITT-FALSE known test vector")
    crc = crc16(b"123456789")
    print(f"  computed=0x{crc:04X} expected=0x29B1")
    check(crc == 0x29B1, "CRC matches published CRC-16/CCITT-FALSE check value")


def test_roundtrip_every_opcode():
    print("Test 2: build + decode round-trip for every opcode")
    ok = True
    for i, op in enumerate(Opcode):
        payload = bytes([op.value, 0xAB])
        frame_bytes = build_frame(MsgType.CMD, i & 0xFF, payload)
        rx = Receiver()
        frames = rx.feed(frame_bytes)
        if not (
            len(frames) == 1
            and frames[0].msg_type == MsgType.CMD
            and frames[0].msg_id == (i & 0xFF)
            and frames[0].payload == payload
        ):
            ok = False
    check(ok, "all 11 command opcodes survive build->wire->decode unchanged")


def test_corruption_rejected_and_resyncs():
    print("Test 3: corrupted frame dropped, next valid frame still decodes")
    good1 = bytearray(build_frame(MsgType.CMD, 1, bytes([0x2A])))
    good2 = build_frame(MsgType.CMD, 2, bytes([0x2A]))
    good1[1] ^= 0xFF

    rx = Receiver()
    frames = rx.feed(bytes(good1))
    check(len(frames) == 0 and rx.last_error == "crc_mismatch", "corrupted frame reported, not accepted")

    frames = rx.feed(good2)
    check(len(frames) == 1 and frames[0].msg_id == 2, "decoder resyncs on the very next frame")


def test_fuzz_never_false_accepts():
    print("Test 4: fuzz — random corruption never false-accepts")
    random.seed(42)
    payload = bytes([0x11, 0x22, 0x33, 0x44])
    false_accepts = 0
    iterations = 50000
    for i in range(iterations):
        frame = bytearray(build_frame(MsgType.CMD, i & 0xFF, payload))
        for _ in range(random.randint(0, 2)):
            idx = random.randrange(0, max(1, len(frame) - 1))
            frame[idx] ^= random.randint(1, 255)
        rx = Receiver()
        frames = rx.feed(bytes(frame))
        for f in frames:
            if not (f.msg_type == MsgType.CMD and f.msg_id == (i & 0xFF) and f.payload == payload):
                false_accepts += 1
    print(f"  {iterations} iterations, {false_accepts} false-accepts")
    check(false_accepts == 0, "no corrupted frame was ever accepted as valid")


if __name__ == "__main__":
    print("=== MedMetrix protocol.py self-test ===\n")
    test_crc_known_vector()
    test_roundtrip_every_opcode()
    test_corruption_rejected_and_resyncs()
    test_fuzz_never_false_accepts()
    status = "ALL TESTS PASSED" if fails == 0 else "TESTS FAILED"
    print(f"\n=== {status} ({fails} failing check{'s' if fails != 1 else ''}) ===")
    raise SystemExit(0 if fails == 0 else 1)
