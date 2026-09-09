/* test_protocol.c — host-side test harness for mm_protocol.h. */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "mm_protocol.h"

static int fails = 0;
#define CHECK(cond, msg) do { \
    if (!(cond)) { printf("  [FAIL] %s\n", msg); fails++; } \
    else { printf("  [ OK ] %s\n", msg); } \
} while (0)

static void test_crc_known_vector(void) {
    printf("Test 1: CRC-16/CCITT-FALSE known test vector\n");
    const uint8_t v[] = "123456789";
    uint16_t crc = mm_crc16(v, 9);
    printf("  computed=0x%04X expected=0x29B1\n", crc);
    CHECK(crc == 0x29B1, "CRC matches published CRC-16/CCITT-FALSE check value");
}

static void test_roundtrip_every_opcode(void) {
    printf("Test 2: build + decode round-trip for every opcode\n");
    mm_opcode_t opcodes[] = {
        MM_CMD_SET_PUMP1_RATE, MM_CMD_SET_PUMP2_RATE, MM_CMD_SET_O2_VALVE_POS,
        MM_CMD_HOME_O2_VALVE, MM_CMD_QUERY_ESTOP, MM_CMD_GET_ECG, MM_CMD_GET_IMU,
        MM_CMD_GET_SPO2, MM_CMD_GET_TEMP, MM_CMD_GET_NIBP, MM_CMD_GET_FULL_STATUS
    };
    int n = (int)(sizeof(opcodes) / sizeof(opcodes[0]));
    int all_ok = 1;

    for (int i = 0; i < n; i++) {
        uint8_t payload[2] = { (uint8_t)(opcodes[i]), 0xAB };
        uint8_t encoded[MM_MAX_ENCODED];
        size_t enc_len = mm_build_frame(MM_MSG_CMD, (uint8_t)i, payload, 2, encoded);

        mm_receiver_t rx;
        mm_receiver_init(&rx);
        mm_frame_t got;
        int result = 0;
        for (size_t b = 0; b < enc_len; b++) {
            result = mm_receiver_feed(&rx, encoded[b], &got);
        }
        int ok = (result == 1) && got.msg_type == MM_MSG_CMD && got.msg_id == (uint8_t)i
                  && got.payload_len == 2 && got.payload[0] == (uint8_t)opcodes[i] && got.payload[1] == 0xAB;
        if (!ok) all_ok = 0;
    }
    CHECK(all_ok, "all 11 command opcodes survive build->wire->decode unchanged");
}

static void test_corruption_rejected_and_resyncs(void) {
    printf("Test 3: corrupted frame is dropped, next valid frame still decodes\n");
    uint8_t payload[1] = { 0x2A };
    uint8_t good1[MM_MAX_ENCODED], good2[MM_MAX_ENCODED];
    size_t len1 = mm_build_frame(MM_MSG_CMD, 1, payload, 1, good1);
    size_t len2 = mm_build_frame(MM_MSG_CMD, 2, payload, 1, good2);

    uint8_t corrupt1[MM_MAX_ENCODED];
    memcpy(corrupt1, good1, len1);
    corrupt1[1] ^= 0xFF;

    mm_receiver_t rx;
    mm_receiver_init(&rx);
    mm_frame_t got;
    int corrupt_result = 0, resync_result = 0;

    for (size_t b = 0; b < len1; b++) corrupt_result = mm_receiver_feed(&rx, corrupt1[b], &got);
    for (size_t b = 0; b < len2; b++) resync_result = mm_receiver_feed(&rx, good2[b], &got);

    CHECK(corrupt_result == -1, "corrupted frame reported as CRC failure, not silently accepted");
    CHECK(resync_result == 1 && got.msg_id == 2, "decoder resyncs on the very next frame, no restart needed");
}

static void test_fuzz_never_crashes_or_false_accepts(void) {
    printf("Test 4: fuzz — random byte corruption never crashes and never false-accepts\n");
    srand(42);
    uint8_t payload[4] = { 0x11, 0x22, 0x33, 0x44 };
    int iterations = 200000;
    int false_accepts = 0;

    for (int iter = 0; iter < iterations; iter++) {
        uint8_t frame[MM_MAX_ENCODED];
        size_t len = mm_build_frame(MM_MSG_CMD, (uint8_t)(iter & 0xFF), payload, 4, frame);

        int n_flips = rand() % 3;
        for (int f = 0; f < n_flips; f++) {
            size_t idx = (size_t)(rand() % (int)(len > 1 ? len - 1 : 1));
            frame[idx] ^= (uint8_t)(1 + (rand() % 255));
        }

        mm_receiver_t rx;
        mm_receiver_init(&rx);
        mm_frame_t got;
        int result = 0;
        for (size_t b = 0; b < len; b++) {
            result = mm_receiver_feed(&rx, frame[b], &got);
        }

        if (result == 1) {
            int matches = got.msg_type == MM_MSG_CMD && got.msg_id == (uint8_t)(iter & 0xFF)
                          && got.payload_len == 4 && memcmp(got.payload, payload, 4) == 0;
            if (!matches) false_accepts++;
        }
    }
    printf("  %d iterations, %d false-accepts\n", iterations, false_accepts);
    CHECK(false_accepts == 0, "no corrupted frame was ever accepted as if it were valid");
}

int main(void) {
    printf("=== MedMetrix protocol library self-test ===\n\n");
    test_crc_known_vector();
    test_roundtrip_every_opcode();
    test_corruption_rejected_and_resyncs();
    test_fuzz_never_crashes_or_false_accepts();

    printf("\n=== %s (%d failing check%s) ===\n",
           fails == 0 ? "ALL TESTS PASSED" : "TESTS FAILED",
           fails, fails == 1 ? "" : "s");
    return fails == 0 ? 0 : 1;
}
