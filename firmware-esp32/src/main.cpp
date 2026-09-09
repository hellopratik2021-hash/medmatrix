/* MedMetrix — ESP32-S3 firmware skeleton.
 * USB Serial  = link to Raspberry Pi.
 * Serial1 (UART on GPIO17=TX, GPIO18=RX, adjust to your wiring) = link to Mega.
 * E-stop input on GPIO4 (adjust to your wiring).
 *
 * This skeleton wires up the protocol + command dispatch + relay-to-Mega
 * boundary for real, and returns a clean "not implemented yet" for every
 * sensor command — those get filled in one at a time in later steps
 * (IMU driver, SpO2 driver, ECG driver) without touching this dispatcher.
 */
#include <Arduino.h>
#include "mm_protocol.h"

#define ESTOP_PIN     4
#define MEGA_RX_PIN   18
#define MEGA_TX_PIN   17

mm_receiver_t rx_from_rpi;
mm_receiver_t rx_from_mega;

volatile uint8_t estop_state = 0;      /* cached, updated only by the ISR */
volatile uint8_t estop_changed_flag = 0;

void IRAM_ATTR estop_isr() {
  estop_state = digitalRead(ESTOP_PIN);
  estop_changed_flag = 1; /* main loop pushes the telemetry frame, ISR stays minimal */
}

void send_to_rpi(uint8_t msg_type, uint8_t msg_id, const uint8_t *payload, uint8_t len) {
  uint8_t encoded[MM_MAX_ENCODED];
  size_t n = mm_build_frame(msg_type, msg_id, payload, len, encoded);
  Serial.write(encoded, n);
}

void relay_to_mega(uint8_t msg_id, const uint8_t *payload, uint8_t len) {
  /* Re-originates a NEW frame on the Mega link — never forwards raw bytes.
   * This is the code-level enforcement of the chain-of-command boundary. */
  uint8_t encoded[MM_MAX_ENCODED];
  size_t n = mm_build_frame(MM_MSG_CMD, msg_id, payload, len, encoded);
  Serial1.write(encoded, n);
}

void nack(uint8_t msg_id, uint8_t reason) {
  send_to_rpi(MM_MSG_ERR, msg_id, &reason, 1);
}

void handle_command_from_rpi(const mm_frame_t &f) {
  if (f.payload_len < 1) { nack(f.msg_id, 0xFE); return; }
  uint8_t opcode = f.payload[0];

  switch (opcode) {
    /* ---- motor commands: relay to Mega, don't reinterpret ---- */
    case MM_CMD_SET_PUMP1_RATE:
    case MM_CMD_SET_PUMP2_RATE:
    case MM_CMD_SET_O2_VALVE_POS:
    case MM_CMD_HOME_O2_VALVE:
      relay_to_mega(f.msg_id, f.payload, f.payload_len);
      break;

    /* ---- handled locally on the ESP32-S3 ---- */
    case MM_CMD_QUERY_ESTOP: {
      uint8_t state = estop_state;
      send_to_rpi(MM_MSG_RESP, f.msg_id, &state, 1);
      break;
    }
    case MM_CMD_GET_NIBP:
      nack(f.msg_id, 0x01); /* reserved: hardware not yet decided, per spec */
      break;

    case MM_CMD_GET_ECG:
    case MM_CMD_GET_IMU:
    case MM_CMD_GET_SPO2:
    case MM_CMD_GET_TEMP:
    case MM_CMD_GET_FULL_STATUS:
      nack(f.msg_id, 0x02); /* driver not wired up yet — filled in next steps */
      break;

    default:
      nack(f.msg_id, 0xFF); /* unknown opcode: not in the closed contract */
      break;
  }
}

void setup() {
  Serial.begin(115200);           /* USB link to RPi */
  Serial1.begin(115200, SERIAL_8N1, MEGA_RX_PIN, MEGA_TX_PIN); /* link to Mega */

  pinMode(ESTOP_PIN, INPUT_PULLUP);
  estop_state = digitalRead(ESTOP_PIN);
  attachInterrupt(digitalPinToInterrupt(ESTOP_PIN), estop_isr, CHANGE);

  mm_receiver_init(&rx_from_rpi);
  mm_receiver_init(&rx_from_mega);

  Serial.println("MedMetrix ESP32-S3 skeleton up.");
}

void loop() {
  /* RPi -> here */
  while (Serial.available()) {
    mm_frame_t f;
    int r = mm_receiver_feed(&rx_from_rpi, (uint8_t)Serial.read(), &f);
    if (r == 1 && f.msg_type == MM_MSG_CMD) handle_command_from_rpi(f);
  }

  /* Mega -> here -> relay up to RPi (motor status / RESPs) */
  while (Serial1.available()) {
    mm_frame_t f;
    int r = mm_receiver_feed(&rx_from_mega, (uint8_t)Serial1.read(), &f);
    if (r == 1) {
      send_to_rpi(f.msg_type, f.msg_id, f.payload, f.payload_len);
    }
  }

  /* e-stop change: report promptly, not on a polling cycle (debounced) */
  static uint32_t last_estop_ms = 0;
  if (estop_changed_flag && millis() - last_estop_ms > 20) {
    estop_changed_flag = 0;
    last_estop_ms = millis();
    uint8_t state = estop_state;
    send_to_rpi(MM_MSG_TELEM, 0, &state, 1);
  }
}
