#include <Arduino.h>

#include "model_settings.h"

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 44;
static constexpr int kUartTxPin = 43;

static constexpr int kP4DetectAdcPin = 8;
static constexpr int kP4DetectThreshold = 2000;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;
static constexpr uint8_t kMsgTypeControl   = 0x02;  // S3 → P4 control messages

// S3 → P4 control commands
static constexpr uint8_t kCtrlAckStop        = 0x01;  // confirmed sign → stop TX
static constexpr uint8_t kCtrlResumeJunction = 0x02;  // done tasks → resume + junction

// Sign confirmation: require N consecutive frames of the same sign class
// with confidence above threshold before sending ACK_STOP.
// Threshold is set low (120/255 ≈ 47%) because int8-quantized models on
// ESP32-P4 often produce subdued confidence scores.  Tune per model.
static constexpr int kSignConfirmFrames = 5;
static constexpr uint8_t kSignConfirmConfidence = 120;  // 120/255 ≈ 47%

#define UartFromP4 Serial0
#define DBG Serial

// The deployed gatekeeper runs standalone (no PC).  USB CDC writes block
// once the host-less buffer fills, which would stall the uart task and
// kill the gate — so periodic / per-frame debug lines print only while a
// host actually has the port open (same HWCDC signal the P4 sketch uses).
static inline bool host_attached() {
  return DBG.isConnected();
}

struct Packet {
  uint8_t msg_type;
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

// ── flags + label_id contract (shared with the P4 sketches) ─────────────
//   flags byte = [high nibble: verdict] [low nibble: sign_ready]
//   low  nibble == 0x2  →  sign_ready (legacy value, unchanged)
//   high nibble == 0x0  →  MATCH (flags 0x02; label_id = matched plate index)
//   high nibble == 0xE  →  MISMATCH (flags 0xE2; label_id = 0xFE)
//   high nibble == 0xF  →  NO SIGN / OUT of distribution (flags 0xF2;
//                          label_id = 0xFF)
//
// Byte 5 (label_id) is what lets the S3 know WHICH allowed plate matched:
// TFLite_Sign sends the matched plate index 0 .. kTargetSignCount-1 on a
// MATCH, so the gate can name the car instead of only saying "a sign".
// Older P4 sketches still send a class index with the same flags; this
// receiver resolves the id through the exported kTargetSignSequences table
// (copied from the sign export) and falls back to kCategoryLabels for a
// legacy build.
//
// Receivers MUST use these helpers — NEVER trust label_id/confidence when
// pkt_is_no_sign() returns true, even if confidence is close to 255.  A
// MISMATCH is deliberately NOT a real sign: it must never trigger ACK_STOP.
static constexpr uint8_t kFlagsMaskSignReady = 0x0F;
static constexpr uint8_t kFlagsMaskOodStatus = 0xF0;
static constexpr uint8_t kFlagsSignReady     = 0x02;
static constexpr uint8_t kFlagsVerdictMatch  = 0x00;
static constexpr uint8_t kFlagsVerdictMismatch = 0xE0;
static constexpr uint8_t kFlagsOodNoSign     = 0xF0;
static constexpr uint8_t kSignMismatchLabel  = 0xFE;
static constexpr uint8_t kSignNoSignLabel    = 0xFF;

// Multi-plate exports put the plate index in label_id.  The table is only
// present when model_settings.h came from a sign export (the S3 folder
// ships with the same copy as TFLite_Sign/); legacy exports keep the
// class-index interpretation.
#ifndef TM_SIGN_MULTI_TARGET_EXPORTED
#define TM_SIGN_MULTI_TARGET_EXPORTED 0
#endif

static inline bool label_is_plate_index(uint8_t label_id, bool real_sign) {
#if TM_SIGN_MULTI_TARGET_EXPORTED
  return real_sign && label_id < (uint8_t)kTargetSignCount;
#else
  (void)label_id; (void)real_sign;
  return false;
#endif
}
static inline const char* plate_text(uint8_t plate_index) {
#if TM_SIGN_MULTI_TARGET_EXPORTED
  if (plate_index < (uint8_t)kTargetSignCount) {
    return kTargetSignSequences[plate_index];
  }
#else
  (void)plate_index;
#endif
  return "?";
}

static inline bool pkt_is_real_sign(uint8_t flags) {
  return (flags & kFlagsMaskSignReady) == kFlagsSignReady &&
         (flags & kFlagsMaskOodStatus) == kFlagsVerdictMatch;
}
static inline bool pkt_is_mismatch(uint8_t flags) {
  return (flags & kFlagsMaskOodStatus) == kFlagsVerdictMismatch;
}
static inline bool pkt_is_no_sign(uint8_t flags) {
  return (flags & kFlagsMaskOodStatus) == kFlagsOodNoSign;
}

static uint32_t s_bytes_rx = 0;
static uint32_t s_packets_ok = 0;
static uint32_t s_packets_bad = 0;
static bool s_p4_connected = false;
static bool s_uart_started = false;

// Sign confirmation state machine
static bool s_sign_confirmed = false;
static uint8_t s_last_sign_label = 0xFF;
static int s_consecutive_sign_frames = 0;
// Arming gate: after a completed cycle the sign must LEAVE the view (at
// least one non-confirmable frame) before the next confirmation can start.
// Without it, frames that kept flowing because the P4 never stopped on
// ACK_STOP would immediately re-confirm the same sign — a double trigger.
static bool s_sign_armed = true;

// After ACK_STOP, S3 "does its tasks" for this many ms before sending RESUME_JUNCTION.
// Adjust to match your real task duration (e.g. display, logging, actuator control).
static constexpr uint32_t kTaskDurationMs = 3000;

static constexpr int kP4DetectSamples = 16;

static uint8_t calc_uart_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static void send_control_packet(uint8_t cmd) {
  uint8_t buf[5];
  buf[0] = kSync0;          // 0xAA
  buf[1] = kSync1;          // 0x55
  buf[2] = kMsgTypeControl; // 0x02
  buf[3] = cmd;
  buf[4] = calc_uart_checksum(buf, 4);  // XOR of bytes 0-3
  UartFromP4.write(buf, sizeof(buf));
  UartFromP4.flush();

  const char *name = (cmd == kCtrlAckStop) ? "ACK_STOP" :
                     (cmd == kCtrlResumeJunction) ? "RESUME_JUNCTION" : "???";
  DBG.printf("S3 → P4: %s (0x%02X)\n", name, cmd);
}

static int read_adc_avg() {
  int32_t sum = 0;
  for (int i = 0; i < kP4DetectSamples; i++) {
    sum += analogRead(kP4DetectAdcPin);
    vTaskDelay(pdMS_TO_TICKS(2));
  }
  return (int)(sum / kP4DetectSamples);
}

static void wait_for_p4_connected_blocking() {
  pinMode(kP4DetectAdcPin, INPUT);

  uint32_t last_log_ms = 0;
  while (!s_p4_connected) {
    const int v = read_adc_avg();
      // DBG.print("adc=");
      // DBG.println(v);
    if (v >= kP4DetectThreshold) {
      s_p4_connected = true;
      DBG.print("P4 detected adc=");
      DBG.println(v);
      break;
    }
    const uint32_t now = millis();
    if (now - last_log_ms > 500) {
      last_log_ms = now;
      DBG.print("Waiting for P4 adc=");
      DBG.println(v);
    }
    vTaskDelay(pdMS_TO_TICKS(10));
  }
}

static bool read_packet(Packet *out) {
  static uint8_t buf[9];
  static uint8_t idx = 0;
  static uint8_t state = 0;

  while (UartFromP4.available() > 0) {
    const uint8_t b = (uint8_t)UartFromP4.read();
    s_bytes_rx++;

    if (state == 0) {
      if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      }
      continue;
    }

    if (state == 1) {
      if (b == kSync1) {
        buf[1] = b;
        idx = 2;
        state = 2;
      } else if (b == kSync0) {
        buf[0] = b;
        idx = 1;
        state = 1;
      } else {
        state = 0;
      }
      continue;
    }

    buf[idx++] = b;
    if (idx < sizeof(buf)) {
      continue;
    }

    state = 0;
    idx = 0;

    if (buf[2] != kMsgTypeInference) {
      s_packets_bad++;
      return false;
    }

    if (calc_uart_checksum(buf, 8) != buf[8]) {
      s_packets_bad++;
      return false;
    }

    out->msg_type = buf[2];
    out->frame_id = (uint16_t)buf[3] | ((uint16_t)buf[4] << 8);
    out->label_id = buf[5];
    out->confidence = buf[6];
    out->flags = buf[7];
    return true;
  }

  return false;
}

static void detect_task(void *arg) {
  (void)arg;
  wait_for_p4_connected_blocking();
  vTaskDelete(nullptr);
}

static void uart_task(void *arg) {
  (void)arg;
  while (!s_p4_connected) {
    vTaskDelay(pdMS_TO_TICKS(50));
  }

  if (!s_uart_started) {
    s_uart_started = true;
    UartFromP4.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
    DBG.printf("UART started rx=%d tx=%d baud=%d\n", kUartRxPin, kUartTxPin, kUartBaud);
    // Boot-time RESUME: if this S3 rebooted while the P4 was still silenced
    // by an earlier ACK_STOP (power glitch, relay kick), the P4 would wait
    // forever and the gate would stay dead until someone resets the P4.
    // Sending RESUME at boot breaks that deadlock; when the P4 was never
    // silenced the enable is a harmless no-op (idempotent).
    send_control_packet(kCtrlResumeJunction);
    DBG.println("S3: RESUME sent at boot (clears any stale ACK_STOP silence)");
  }

  uint32_t last_avail_ms = 0;
  for (;;) {
    Packet p;
    if (read_packet(&p)) {
      s_packets_ok++;
      const bool real_sign = pkt_is_real_sign(p.flags);
      const bool mismatch  = pkt_is_mismatch(p.flags);
      const bool no_sign   = pkt_is_no_sign(p.flags);
      // Multi-plate protocol: on a MATCH, label_id is the matched plate
      // index; MISMATCH/NO SIGN carry 0xFE/0xFF.  label_is_plate_index()
      // also keeps a legacy class-index packet readable — it only accepts an
      // id inside the exported plate table.
      const bool plate_frame = label_is_plate_index(p.label_id, real_sign);
      const char *label = "No Sign";
      if (real_sign) {
        label = plate_frame ? plate_text(p.label_id)
                            : (p.label_id < kCategoryCount
                                   ? kCategoryLabels[p.label_id]
                                   : "MATCH");
      } else if (mismatch) {
        label = "NO MATCH";
      } else if (no_sign) {
        label = "NO SIGN";
      }

      // Per-frame debug lines print ONLY while a host has the port open:
      // standalone (no PC) the USB CDC write() blocks once its buffer
      // fills, which would stall this task and kill the gate entirely.
      if (host_attached()) {
        DBG.print("frame=");
        DBG.print(p.frame_id);
        DBG.print(" label=");
        if (real_sign) {
          if (plate_frame) DBG.printf("%d(%s)", (int)p.label_id, label);
          else             DBG.print((int)p.label_id);
        } else if (mismatch) {
          DBG.print("MISMATCH");
        } else {
          DBG.print("NO_SIGN");
        }
        DBG.print("(");
        DBG.print(label);
        DBG.print(")");
        DBG.print(" conf=");
        DBG.print(real_sign ? (int)p.confidence : 0);
        DBG.print(" flags=0x");
        DBG.print(p.flags, 16);
        DBG.print(no_sign   ? " [NO SIGN (OOD)]" : "");
        DBG.print(mismatch  ? " [NO MATCH]"      : "");
        DBG.print(real_sign ? " [REAL SIGN]"     : "");
      }

      // ── Sign confirmation state machine ──
      // Real sign only: OOD-suppressed ("No Sign") frames never count, even
      // if softmax hallucinated a high-confidence label_id + confidence.
      // This prevents the old bug of 5 consecutive "END" hallucinations on
      // an empty desk being mis-interpreted as a confirmed sign.
      if (!s_sign_confirmed && s_sign_armed && real_sign && p.confidence >= kSignConfirmConfidence) {
        if (p.label_id == s_last_sign_label) {
          s_consecutive_sign_frames++;
        } else {
          s_last_sign_label = p.label_id;
          s_consecutive_sign_frames = 1;
        }

        if (s_consecutive_sign_frames >= kSignConfirmFrames) {
          s_sign_confirmed = true;
          DBG.printf("SIGN CONFIRMED: label=%d(%s)%s conf=%d after %d frames [flags=0x%02X]\n",
                     p.label_id, label, plate_frame ? " [PLATE]" : "",
                     p.confidence, s_consecutive_sign_frames, p.flags);

          // Step 1: Tell P4 to stop transmitting
          bool p4_stopped = false;
          for (int retry = 0; retry < 3 && !p4_stopped; retry++) {
            if (retry > 0) {
              DBG.printf("S3: ACK_STOP retry %d/3\n", retry + 1);
              vTaskDelay(pdMS_TO_TICKS(50));
            }
            send_control_packet(kCtrlAckStop);
            uint32_t wait_start = millis();
            while (millis() - wait_start < 300) {
              vTaskDelay(pdMS_TO_TICKS(30));
              while (UartFromP4.available() > 0) (void)UartFromP4.read();
              vTaskDelay(pdMS_TO_TICKS(50));
              if (UartFromP4.available() == 0) {
                vTaskDelay(pdMS_TO_TICKS(100));
                if (UartFromP4.available() == 0) {
                  p4_stopped = true;
                  DBG.printf("S3: P4 stopped TX after %lu ms (retry=%d)\n",
                             (unsigned long)(millis() - wait_start), retry);
                  break;
                }
              }
            }
          }
          if (!p4_stopped)
            DBG.println("S3: WARNING — P4 did not stop TX after 3 ACK_STOP retries!");

          // Step 2: Do our tasks
          DBG.printf("S3: performing tasks (%lu ms)...\n", (unsigned long)kTaskDurationMs);
          vTaskDelay(pdMS_TO_TICKS(kTaskDurationMs));

          // Step 3: Tell P4 to resume
          bool p4_resumed = false;
          for (int retry = 0; retry < 3 && !p4_resumed; retry++) {
            if (retry > 0) {
              DBG.printf("S3: RESUME_JUNCTION retry %d/3\n", retry + 1);
              vTaskDelay(pdMS_TO_TICKS(100));
            }
            send_control_packet(kCtrlResumeJunction);
            uint32_t wait_start = millis();
            while (millis() - wait_start < 500) {
              vTaskDelay(pdMS_TO_TICKS(20));
              if (UartFromP4.available() > 0) {
                p4_resumed = true;
                DBG.printf("S3: P4 resumed TX after %lu ms (retry=%d)\n",
                           (unsigned long)(millis() - wait_start), retry);
                break;
              }
            }
          }
          if (!p4_resumed)
            DBG.println("S3: WARNING — P4 did not resume TX after 3 retries!");

          // Reset for next cycle.  The sign must LEAVE the view before the
          // next confirmation may arm — otherwise the frames that kept
          // flowing (P4 never stopped on ACK_STOP) would double-trigger.
          s_sign_confirmed = false;
          s_last_sign_label = 0xFF;
          s_consecutive_sign_frames = 0;
          s_sign_armed = false;
          DBG.println("S3: cycle complete, waiting for sign to leave before re-arming");
        }
      } else {
        // No-Sign / MISMATCH frame, confidence too low, or a different label —
        // reset the counter.  A burst of OOD or mismatch frames MUST reset the
        // count so we don't accumulate a partial match across scene cuts.
        s_last_sign_label = 0xFF;
        s_consecutive_sign_frames = 0;
        // A non-confirmable frame (no sign / mismatch / low confidence)
        // means the sign has left or is not readable: re-arm for the next
        // cycle.
        s_sign_armed = true;
      }
    } else {
      const uint32_t now = millis();
      if (host_attached() && now - last_avail_ms >= 1000) {
        last_avail_ms = now;
        DBG.printf("UART rx_avail=%d bytes=%lu ok=%lu bad=%lu\n", UartFromP4.available(),
                   (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok,
                   (unsigned long)s_packets_bad);
      }
      vTaskDelay(pdMS_TO_TICKS(1));
    }
  }
}

static void heartbeat_task(void *arg) {
  (void)arg;
  for (;;) {
    if (host_attached()) {
      DBG.printf("S3 alive connected=%d bytes=%lu ok=%lu bad=%lu\n", (int)s_p4_connected,
                 (unsigned long)s_bytes_rx, (unsigned long)s_packets_ok,
                 (unsigned long)s_packets_bad);
    }
    vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void setup() {
  DBG.begin(kDebugBaud);
  vTaskDelay(pdMS_TO_TICKS(200));
  DBG.println("S3 UART receiver start");

  // xTaskCreate(heartbeat_task, "hb", 4096, nullptr, 1, nullptr);
  xTaskCreate(detect_task, "detect", 4096, nullptr, 2, nullptr);
  
  xTaskCreate(uart_task, "uart", 8192, nullptr, 2, nullptr);
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
