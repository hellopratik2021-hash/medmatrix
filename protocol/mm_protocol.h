/* mm_protocol.h — MedMetrix shared framing/CRC/opcode library.
 * Portable C (C99), no dynamic allocation, no platform dependencies.
 * Used identically on ESP32-S3 (Arduino/ESP-IDF), Arduino Mega (AVR),
 * and ported 1:1 to Python for the Raspberry Pi backend.
 */
#ifndef MM_PROTOCOL_H
#define MM_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define MM_MAX_PAYLOAD   64
#define MM_MAX_FRAME     (1 + 1 + 1 + MM_MAX_PAYLOAD + 2)  /* type+id+len+payload+crc */
#define MM_MAX_ENCODED   (MM_MAX_FRAME + (MM_MAX_FRAME / 254) + 2)

/* ---- message types ---- */
typedef enum {
    MM_MSG_CMD   = 0x01,
    MM_MSG_RESP  = 0x02,
    MM_MSG_TELEM = 0x03,
    MM_MSG_ERR   = 0x04
} mm_msg_type_t;

/* ---- command opcodes (spec section 3, closed contract) ---- */
typedef enum {
    MM_CMD_SET_PUMP1_RATE   = 0x10,
    MM_CMD_SET_PUMP2_RATE   = 0x11,
    MM_CMD_SET_O2_VALVE_POS = 0x12,
    MM_CMD_HOME_O2_VALVE    = 0x13,
    MM_CMD_QUERY_ESTOP      = 0x20,
    MM_CMD_GET_ECG          = 0x21,
    MM_CMD_GET_IMU          = 0x22,
    MM_CMD_GET_SPO2         = 0x23,
    MM_CMD_GET_TEMP         = 0x24,
    MM_CMD_GET_NIBP         = 0x25,   /* reserved: hardware not yet decided */
    MM_CMD_GET_FULL_STATUS  = 0x26
} mm_opcode_t;

/* ---- telemetry subtypes (carried in payload[0] of an MM_MSG_TELEM frame) ---- */
typedef enum {
    MM_TELEM_ECG_SAMPLE    = 0x01,
    MM_TELEM_IMU_SAMPLE    = 0x02,
    MM_TELEM_SPO2_READING  = 0x03,
    MM_TELEM_TEMP_READING  = 0x04,
    MM_TELEM_ESTOP_CHANGE  = 0x05,
    MM_TELEM_MOTOR_STATUS  = 0x06,
    MM_TELEM_PROTOCOL_ERR  = 0x07
} mm_telem_type_t;

typedef struct {
    uint8_t msg_type;
    uint8_t msg_id;
    uint8_t payload_len;
    uint8_t payload[MM_MAX_PAYLOAD];
} mm_frame_t;

/* ================= CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF) ================= */
static inline uint16_t mm_crc16(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ 0x1021) : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* ================= COBS encode/decode (0x00 delimiter) ================= */
/* out must be at least (len + len/254 + 1) bytes. Returns encoded length. */
static inline size_t mm_cobs_encode(const uint8_t *in, size_t len, uint8_t *out) {
    size_t read_i = 0, write_i = 1, code_i = 0;
    uint8_t code = 1;

    while (read_i < len) {
        if (in[read_i] == 0) {
            out[code_i] = code;
            code = 1;
            code_i = write_i++;
            read_i++;
        } else {
            out[write_i++] = in[read_i++];
            code++;
            if (code == 0xFF) {
                out[code_i] = code;
                code = 1;
                code_i = write_i++;
            }
        }
    }
    out[code_i] = code;
    return write_i;
}

/* out must be at least len bytes. Returns decoded length, or 0 on malformed input. */
static inline size_t mm_cobs_decode(const uint8_t *in, size_t len, uint8_t *out) {
    if (len == 0) return 0;
    size_t read_i = 0, write_i = 0;

    while (read_i < len) {
        uint8_t code = in[read_i];
        if (code == 0 || read_i + code > len + 1) return 0; /* malformed */
        read_i++;
        for (uint8_t i = 1; i < code; i++) {
            if (read_i >= len) return 0;
            out[write_i++] = in[read_i++];
        }
        if (code != 0xFF && read_i < len) {
            out[write_i++] = 0;
        }
    }
    return write_i;
}

/* ================= Frame build (TX side) ================= */
static inline size_t mm_build_frame(uint8_t msg_type, uint8_t msg_id,
                                     const uint8_t *payload, uint8_t payload_len,
                                     uint8_t *out_encoded) {
    if (payload_len > MM_MAX_PAYLOAD) return 0;

    uint8_t raw[MM_MAX_FRAME];
    raw[0] = msg_type;
    raw[1] = msg_id;
    raw[2] = payload_len;
    if (payload_len) memcpy(&raw[3], payload, payload_len);

    uint16_t crc = mm_crc16(raw, (size_t)3 + payload_len);
    raw[3 + payload_len]     = (uint8_t)(crc >> 8);
    raw[3 + payload_len + 1] = (uint8_t)(crc & 0xFF);

    size_t raw_len = (size_t)3 + payload_len + 2;
    size_t enc_len = mm_cobs_encode(raw, raw_len, out_encoded);
    out_encoded[enc_len++] = 0x00;
    return enc_len;
}

/* ================= Streaming receiver (RX side) ================= */
typedef struct {
    uint8_t buf[MM_MAX_ENCODED];
    size_t  len;
} mm_receiver_t;

static inline void mm_receiver_init(mm_receiver_t *rx) {
    rx->len = 0;
}

static inline int mm_receiver_feed(mm_receiver_t *rx, uint8_t byte, mm_frame_t *out_frame) {
    if (byte == 0x00) {
        if (rx->len == 0) return 0; /* stray delimiter, ignore */

        uint8_t decoded[MM_MAX_FRAME];
        size_t dec_len = mm_cobs_decode(rx->buf, rx->len, decoded);
        rx->len = 0; /* always resync — next byte starts a fresh frame */

        if (dec_len < 5) return -1;
        uint8_t payload_len = decoded[2];
        if ((size_t)(3 + payload_len + 2) != dec_len) return -1;

        uint16_t rx_crc = ((uint16_t)decoded[3 + payload_len] << 8) | decoded[3 + payload_len + 1];
        uint16_t calc_crc = mm_crc16(decoded, (size_t)3 + payload_len);
        if (rx_crc != calc_crc) return -1;

        out_frame->msg_type    = decoded[0];
        out_frame->msg_id      = decoded[1];
        out_frame->payload_len = payload_len;
        if (payload_len) memcpy(out_frame->payload, &decoded[3], payload_len);
        return 1;
    }

    if (rx->len < MM_MAX_ENCODED) {
        rx->buf[rx->len++] = byte;
    } else {
        rx->len = 0;
    }
    return 0;
}

#endif /* MM_PROTOCOL_H */
