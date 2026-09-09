/* MedMetrix — Arduino Mega motor firmware skeleton.
 * One hardware timer per axis, CTC mode, ISR only toggles a STEP pin.
 * No blocking waits anywhere in this file (spec section 5 requirement).
 *
 * Link: Serial1 (pins 18=TX1, 19=RX1) <-> ESP32-S3.
 * Pin numbers below are placeholders — update PUMP1/PUMP2/VALVE pins and
 * VALVE_LIMIT_PIN to match your actual wiring once it's finalized.
 */
#include <Arduino.h>
#include "mm_protocol.h"

#define PUMP1_STEP_PIN   22
#define PUMP1_DIR_PIN    23
#define PUMP2_STEP_PIN   24
#define PUMP2_DIR_PIN    25
#define VALVE_STEP_PIN   26
#define VALVE_DIR_PIN    27
#define VALVE_LIMIT_PIN  2   /* interrupt-capable pin on Mega */

typedef enum { AXIS_IDLE, AXIS_RUNNING, AXIS_HOMING, AXIS_MOVING, AXIS_FAULTED } axis_status_t;

volatile int16_t  pump1_rate = 0;
volatile axis_status_t pump1_status = AXIS_IDLE;

volatile int16_t  pump2_rate = 0;
volatile axis_status_t pump2_status = AXIS_IDLE;

volatile int32_t  valve_current_steps = 0;
volatile int32_t  valve_target_steps = 0;
volatile uint8_t  valve_homed = 0;
volatile axis_status_t valve_status = AXIS_IDLE;

/* ================= Timer1 -> Pump 1 (continuous rate) ================= */
void timer1_set_rate(int16_t steps_per_sec) {
  noInterrupts();
  pump1_rate = steps_per_sec;
  if (steps_per_sec == 0) {
    TCCR1B &= ~((1 << CS12) | (1 << CS11) | (1 << CS10));
    pump1_status = AXIS_IDLE;
  } else {
    digitalWrite(PUMP1_DIR_PIN, steps_per_sec > 0 ? HIGH : LOW);
    uint16_t abs_rate = (uint16_t)abs(steps_per_sec);
    uint32_t ocr = (16000000UL / 8UL / abs_rate / 2UL) - 1UL;
    if (ocr > 0xFFFF) ocr = 0xFFFF;
    OCR1A = (uint16_t)ocr;
    TCCR1B = (1 << WGM12) | (1 << CS11);
    pump1_status = AXIS_RUNNING;
  }
  interrupts();
}

ISR(TIMER1_COMPA_vect) {
  digitalWrite(PUMP1_STEP_PIN, !digitalRead(PUMP1_STEP_PIN));
}

/* ================= Timer3 -> Pump 2 (continuous rate) ================= */
void timer3_set_rate(int16_t steps_per_sec) {
  noInterrupts();
  pump2_rate = steps_per_sec;
  if (steps_per_sec == 0) {
    TCCR3B &= ~((1 << CS32) | (1 << CS31) | (1 << CS30));
    pump2_status = AXIS_IDLE;
  } else {
    digitalWrite(PUMP2_DIR_PIN, steps_per_sec > 0 ? HIGH : LOW);
    uint16_t abs_rate = (uint16_t)abs(steps_per_sec);
    uint32_t ocr = (16000000UL / 8UL / abs_rate / 2UL) - 1UL;
    if (ocr > 0xFFFF) ocr = 0xFFFF;
    OCR3A = (uint16_t)ocr;
    TCCR3B = (1 << WGM32) | (1 << CS31);
    pump2_status = AXIS_RUNNING;
  }
  interrupts();
}

ISR(TIMER3_COMPA_vect) {
  digitalWrite(PUMP2_STEP_PIN, !digitalRead(PUMP2_STEP_PIN));
}

/* ================= Timer4 -> O2 valve (absolute position) ================= */
#define VALVE_HOMING_RATE   200
#define VALVE_MOVE_RATE     800
#define VALVE_MAX_STEPS     20000UL

void timer4_start(uint16_t steps_per_sec, bool forward) {
  digitalWrite(VALVE_DIR_PIN, forward ? HIGH : LOW);
  uint32_t ocr = (16000000UL / 8UL / steps_per_sec / 2UL) - 1UL;
  if (ocr > 0xFFFF) ocr = 0xFFFF;
  OCR4A = (uint16_t)ocr;
  TCCR4B = (1 << WGM42) | (1 << CS41);
}

void timer4_stop(void) {
  TCCR4B &= ~((1 << CS42) | (1 << CS41) | (1 << CS40));
}

ISR(TIMER4_COMPA_vect) {
  digitalWrite(VALVE_STEP_PIN, !digitalRead(VALVE_STEP_PIN));
  static uint8_t edge = 0;
  edge = !edge;
  if (!edge) return;

  if (valve_status == AXIS_HOMING) {
    valve_current_steps++;
    if (valve_current_steps > (int32_t)VALVE_MAX_STEPS) {
      timer4_stop();
      valve_status = AXIS_FAULTED;
    }
  } else if (valve_status == AXIS_MOVING) {
    valve_current_steps += (valve_target_steps >= valve_current_steps) ? 1 : -1;
    if (valve_current_steps == valve_target_steps) {
      timer4_stop();
      valve_status = AXIS_IDLE;
    }
  }
}

void valve_limit_isr(void) {
  if (valve_status == AXIS_HOMING) {
    timer4_stop();
    valve_current_steps = 0;
    valve_homed = 1;
    valve_status = AXIS_IDLE;
  }
}

void start_homing(void) {
  valve_status = AXIS_HOMING;
  valve_current_steps = 0;
  timer4_start(VALVE_HOMING_RATE, false);
}

void start_move(int32_t target) {
  if (!valve_homed) return;
  valve_target_steps = target;
  valve_status = AXIS_MOVING;
  timer4_start(VALVE_MOVE_RATE, target >= valve_current_steps);
}

/* ================= Protocol link to ESP32-S3 (Serial1) ================= */
mm_receiver_t rx_state;

void send_frame(uint8_t msg_type, uint8_t msg_id, const uint8_t *payload, uint8_t len) {
  uint8_t encoded[MM_MAX_ENCODED];
  size_t n = mm_build_frame(msg_type, msg_id, payload, len, encoded);
  Serial1.write(encoded, n);
}

void send_motor_status(void) {
  uint8_t payload[3] = { (uint8_t)pump1_status, (uint8_t)pump2_status, (uint8_t)valve_status };
  send_frame(MM_MSG_TELEM, 0, payload, 3);
}

void handle_command(const mm_frame_t &f) {
  if (f.payload_len < 1) return;
  uint8_t opcode = f.payload[0];

  switch (opcode) {
    case MM_CMD_SET_PUMP1_RATE: {
      int16_t rate = (int16_t)((f.payload[1] << 8) | f.payload[2]);
      timer1_set_rate(rate);
      send_frame(MM_MSG_RESP, f.msg_id, NULL, 0);
      break;
    }
    case MM_CMD_SET_PUMP2_RATE: {
      int16_t rate = (int16_t)((f.payload[1] << 8) | f.payload[2]);
      timer3_set_rate(rate);
      send_frame(MM_MSG_RESP, f.msg_id, NULL, 0);
      break;
    }
    case MM_CMD_SET_O2_VALVE_POS: {
      uint16_t target = (uint16_t)((f.payload[1] << 8) | f.payload[2]);
      if (!valve_homed) {
        uint8_t err = 0x01;
        send_frame(MM_MSG_ERR, f.msg_id, &err, 1);
      } else {
        start_move(target);
        send_frame(MM_MSG_RESP, f.msg_id, NULL, 0);
      }
      break;
    }
    case MM_CMD_HOME_O2_VALVE: {
      start_homing();
      send_frame(MM_MSG_RESP, f.msg_id, NULL, 0);
      break;
    }
    default:
      break;
  }
}

void setup() {
  pinMode(PUMP1_STEP_PIN, OUTPUT);
  pinMode(PUMP1_DIR_PIN, OUTPUT);
  pinMode(PUMP2_STEP_PIN, OUTPUT);
  pinMode(PUMP2_DIR_PIN, OUTPUT);
  pinMode(VALVE_STEP_PIN, OUTPUT);
  pinMode(VALVE_DIR_PIN, OUTPUT);
  pinMode(VALVE_LIMIT_PIN, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(VALVE_LIMIT_PIN), valve_limit_isr, FALLING);

  Serial1.begin(115200);
  mm_receiver_init(&rx_state);
}

uint32_t last_status_ms = 0;

void loop() {
  while (Serial1.available()) {
    mm_frame_t frame;
    int r = mm_receiver_feed(&rx_state, (uint8_t)Serial1.read(), &frame);
    if (r == 1 && frame.msg_type == MM_MSG_CMD) {
      handle_command(frame);
    }
  }

  if (millis() - last_status_ms > 100) {
    send_motor_status();
    last_status_ms = millis();
  }
}
