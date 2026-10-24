/* Copyright 2019 The TensorFlow Authors. All Rights Reserved.

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/

// ═══════════════════════════════════════════════════════════════════════
// TFLite_Sign — 3-character road-sign reader
//
// The model is a SINGLE-GLYPH classifier (the same 28x28 int8 model an
// `ocr` project trains).  This sketch turns it into a sign reader:
//   1. split the sign into exactly three glyph slots   (sign_glyph.c)
//   2. run the model once per slot                      (3 x Invoke)
//   3. compare the three class ids with kTargetSignClassIndices[]
//      and report a binary verdict: MATCH / NO MATCH / NO SIGN
//
// The sibling TFLite_OCR/ sketch reads ONE character per frame; TFLite/
// is the legacy 96x96 sign pipeline.  This folder is the `sign_ocr`
// export target, and its model_settings.h carries the target sequence.
// ═══════════════════════════════════════════════════════════════════════
// RESOLUTION LAYERS — stamped into model_settings.h by Export Model
//
//   TM_CAPTURE_FRAME_SIDE      camera / serial-stream side    (e.g. 160)
//   TM_PREPROCESS_FRAME_SIDE   glyph front-end input side     (e.g. 96)
//   TM_MODEL_INPUT_SIDE        one glyph canvas side          (28)
//
// They are deliberately independent.  The camera captures at 160×160; the
// signature front-end runs on the 96×96 box-averaged frame (the same
// resolution AItraining previews and the training captures use); each slot
// is then resampled to the 28×28 model canvas.  g_frame_side is the CAPTURE
// side only — never assume it equals the model input.
//
//   g_flip_180
//     The camera is mounted upside down on the board; the library rotates
//     the image 180° to match the data-collection (training) orientation.
//     Set to false only if your mount differs.
//
//   (constexpr here, unlike the examples, because this sketch sizes
//    compile-time arrays from g_frame_side / g_preprocess_side.)
// ═══════════════════════════════════════════════════════════════════════
// #include <TensorFlowLite.h>

#include "main_functions.h"
#include "model_settings.h"   // before USER CONFIG: it carries the resolution sides
#include "image_provider.h"   // includes frame_resize.h (box-average helper)
// Include the exported model data header.
// Rename this include to match your exported model's header file name,
// e.g. "Square_model_data.h" or "MyModel_model_data.h".
#include "tm_model_data.h"

// Older exports predate the resolution macros: keep their 96×96 capture
// behaviour, where capture and preprocessing were the same frame.
#ifndef TM_CAPTURE_FRAME_SIDE
#define TM_CAPTURE_FRAME_SIDE 96
#endif
#ifndef TM_PREPROCESS_FRAME_SIDE
#define TM_PREPROCESS_FRAME_SIDE 96
#endif

constexpr int  g_frame_side      = 160;
constexpr int  g_preprocess_side = TM_PREPROCESS_FRAME_SIDE;
constexpr bool g_flip_180        = true;


#include "tensorflow/lite/micro/micro_log.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/micro_mutable_op_resolver.h"
#if __has_include("model_resolver.h")
#include "model_resolver.h"
#define TM_HAS_GENERATED_MODEL_RESOLVER 1
#else
#define TM_HAS_GENERATED_MODEL_RESOLVER 0
#endif
// ── 3-character sign pipeline (see TFLite_Sign/README.md) ───────────────
// The model is the same 28x28 single-glyph classifier TFLite_OCR/ runs; what
// this sketch adds is the split into three slots, one Invoke per slot, and the
// sequence verdict.  There is no mode switch, so the export is checked here
// rather than selected by it: dropping the single-glyph `ocr` export into this
// folder would build a sketch with no target sequence, and dropping a legacy
// 96x96 export would build one whose front-end writes 784 bytes into a
// 9216-byte tensor.
#if !defined(TM_SIGN_OCR_MODEL)
#error "TFLite_Sign needs a sign_ocr export (model_settings.h must #define TM_SIGN_OCR_MODEL 1). This model_settings.h does not define it, so it belongs to another project - export the sign_ocr project into TFLite_Sign/."
#elif TM_SIGN_OCR_MODEL != 1
#error "TFLite_Sign needs TM_SIGN_OCR_MODEL == 1 (a sign_ocr export); this model_settings.h says otherwise - export the sign_ocr project into TFLite_Sign/."
#endif
#if defined(TM_OCR_GLYPH_MODEL) && TM_OCR_GLYPH_MODEL == 1
#error "This is the single-glyph OCR export (TM_OCR_GLYPH_MODEL == 1); it belongs in TFLite_OCR/, not TFLite_Sign/."
#endif
#include "sign_glyph.h"   // must sit next to TFLite_Sign.ino (Arduino builds it in place)

// ── Sign front-end tuning ───────────────────────────────────────────────
// These are properties of the TRAINING set, not knobs: AItraining's sign_ocr
// project records them (ocr_preprocess.py: INK_DARK, MIN_GLYPH_RUN,
// NORM_CANON), and the model only recognises glyphs produced the same way.
// Changing one without re-exporting silently moves the device off the
// distribution the model was trained on.
static constexpr SignPolarity   kSignPolarity   = SIGN_INK_DARK;        // dark ink on a bright sign
static constexpr bool           kSignSegment    = true;                 // vertical-projection segmentation
static constexpr int            kSignMinRun     = SIGN_MIN_GLYPH_RUN;   // = 2, matches MIN_GLYPH_RUN
static constexpr SignConvention kSignConvention = SIGN_NORM_CANON;      // the convention the sign mix trains in

// ── Dark Thr / Lum Thr window ───────────────────────────────────────────
// The editor exports the two sliders in model_settings.h.  Both at their
// defaults (0 and 100) mean "untouched": exact Otsu and no luminance floor,
// which is bit-identical to every model trained before the sliders existed.
// Anything else pins the binarisation threshold to lum-1 and drops pixels at
// or below dark+1, exactly like the host's preprocess_sign_sequence() call.
#ifndef TM_THRESHOLD_WINDOW_EXPORTED
#define TM_THRESHOLD_WINDOW_EXPORTED 0
#endif
#if TM_THRESHOLD_WINDOW_EXPORTED
static constexpr int kExportDarkThr = kDarkThr;
static constexpr int kExportLumThr  = kLumThr;
#else
static constexpr int kExportDarkThr = 0;     // pre-slider export: untouched
static constexpr int kExportLumThr  = 100;
#endif
static constexpr bool kSignWindowAdjusted =
    (kExportDarkThr != 0) || (kExportLumThr != 100);
static constexpr int kSignDarkThr = kSignWindowAdjusted
    ? (kExportDarkThr + 1 < 0 ? 0 : (kExportDarkThr + 1 > 255 ? 255 : kExportDarkThr + 1))
    : 0;
static constexpr int kSignPinnedThresh = !kSignWindowAdjusted
    ? -1                                            // -1 = exact integer Otsu
    : (kExportLumThr - 1 < 1 ? 1 : (kExportLumThr - 1 > 254 ? 254 : kExportLumThr - 1));

// Tripwire, not a knob: SIGN_GLYPH_SIZE must equal AItraining's GLYPH_SIZE
// (ocr_preprocess.py).  Editing the header alone would put the device on a
// different canvas than the model was trained on, and every artifact here
// would still pass.
static_assert(SIGN_GLYPH_SIZE == 28, "SIGN_GLYPH_SIZE changed — it must equal "
                                     "AItraining ocr_preprocess.GLYPH_SIZE; "
                                     "re-verify TFLite_Sign/ tools first");
static_assert(SIGN_SEQUENCE_LEN == 3, "TFLite_Sign reads exactly three slots");
static_assert(sizeof(kTargetSignSequence) == SIGN_SEQUENCE_LEN + 1,
              "kTargetSignSequence must be exactly SIGN_SEQUENCE_LEN characters "
              "(the exporter pads with '?')");
static_assert(kSignReadingOrder == 0 || kSignReadingOrder == 1,
              "kSignReadingOrder must be 0 (LTR) or 1 (RTL)");
static_assert(kSignMatchMinProb >= 0.0f && kSignMatchMinProb <= 1.0f,
              "kSignMatchMinProb must be a probability in [0,1]");
// Multi-plate whitelist accessors.  New exports carry the full table; older
// exports keep their single kTargetSignSequence/kTargetSignClassIndices and
// behave as a one-plate whitelist.
#ifndef TM_SIGN_MULTI_TARGET_EXPORTED
#define TM_SIGN_MULTI_TARGET_EXPORTED 0
#endif
#if TM_SIGN_MULTI_TARGET_EXPORTED
static_assert(kTargetSignCount >= 1 && kTargetSignCount <= 16,
              "kTargetSignCount must be 1..16");
#define SIGN_PLATE_COUNT kTargetSignCount
#define SIGN_PLATE_CLASS(p, s) (kTargetSignClassMatrix[(p)][(s)])
#define SIGN_PLATE_TEXT(p) (kTargetSignSequences[(p)])
#else
#define SIGN_PLATE_COUNT 1
#define SIGN_PLATE_CLASS(p, s) ((void)(p), kTargetSignClassIndices[(s)])
#define SIGN_PLATE_TEXT(p) ((void)(p), kTargetSignSequence)
#endif
// Same class of tripwire for the PREPROCESSING frame size.  The front-end
// refuses frames outside sign_glyph.h's envelope (1 <= w <= SIGN_MAX_W,
// w*h <= SIGN_MAX_PIXELS) and our caller passes w == h == g_preprocess_side,
// so the whole envelope collapses to this one square-frame test.  The camera
// capture side may be larger (160) — it is box-averaged down first, so it is
// deliberately NOT the value tested here.
static_assert(g_preprocess_side >= 1 && g_preprocess_side <= SIGN_MAX_W &&
                  (int64_t)g_preprocess_side * g_preprocess_side <= SIGN_MAX_PIXELS,
              "g_preprocess_side is outside sign_glyph.h's frame envelope "
              "(1 <= side <= SIGN_MAX_W and side^2 <= SIGN_MAX_PIXELS) — "
              "the glyph front-end would refuse every frame");
// The camera library accepts 8..512-pixel output frames; the capture side is
// only constrained by that and by RAM, not by the glyph envelope.
static_assert(g_frame_side >= 8 && g_frame_side <= 512,
              "g_frame_side (kCaptureFrameSide) must be 8..512");
static_assert(g_preprocess_side <= g_frame_side,
              "kPreprocessFrameSide must not exceed the capture side — the "
              "device pipeline downscales, it never invents detail");
// Rejection policy, exported by the host with the model:
//   1 = each slot rejects with the model's own "_background" class; the OOD
//       gates are opened instead of tuned, and the per-slot match rule (ink,
//       not background, class target, prob >= s_sign_match_min_prob) mirrors the
//       host preview.
//   0 = no "_background" class was trained.  The standard OOD gates
//       (kOodSignMinPct/MaxPct, kOodMaxProbMin, kOodEntropyRatioMax) compose
//       with the same per-slot match rule, exactly like the host preview.
// `kBackgroundClassIndex` is the exported index of "_background" in
// kCategoryLabels, or -1 when the class is absent.
// Older exports predate the field and always trained a background class, so
// the fallback keeps their previous behaviour; exports from the first
// version of the toggle carry the legacy TM_SIGN_BACKGROUND_IS_THE_ONLY_REJECTOR
// name and are honoured too.
#ifndef TM_SIGN_USE_BACKGROUND_OOD
  #ifdef TM_SIGN_BACKGROUND_IS_THE_ONLY_REJECTOR
    static constexpr bool kSignBackgroundReplacesOod =
        kSignBackgroundIsTheOnlyRejector;
  #else
    static constexpr bool kSignBackgroundReplacesOod = true;
  #endif
  static constexpr int kBackgroundClassIndex = -1;  // scan labels by name
#endif

#include "tensorflow/lite/schema/schema_generated.h"
#include "tensorflow/lite/micro/micro_interpreter.h"
#include "tensorflow/lite/micro/system_setup.h"

// Enable ESP-NN acceleration for ESP32-P4 if available.
// ESP-NN provides optimized Conv2D, DepthwiseConv2D, and FullyConnected
// kernels using RISC-V SIMD instructions, giving 10-50x speedup over
// the portable reference kernels.
#if __has_include("esp_nn.h") && !defined(ESP_NN)
#define ESP_NN 1
#endif
#if __has_include("esp_nn_conv2d.h")
#include "esp_nn_conv2d.h"
#endif

#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <freertos/task.h>
#include "esp_heap_caps.h"
#include "esp_timer.h"
#include <SD_MMC.h>
#include <FFat.h>
#include <driver/gpio.h>
#include <math.h>
#include <new>
#include <errno.h>
#include <dirent.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>

// Globals, used for compatibility with Arduino-style sketches.
namespace {
tflite::ErrorReporter* error_reporter = nullptr;
const tflite::Model* model = nullptr;
tflite::MicroInterpreter* interpreter = nullptr;
TfLiteTensor* input = nullptr;

// Bi-directional control: S3 can request P4 to stop/start UART transmission.
// Inference continues regardless; only the TX path is gated.
static volatile bool s_transmit_enabled = true;

// Per-slot confidence floor — RUNTIME, not kSignMatchMinProb.  The exported
// constant is only the boot default; `set ood ... <mpmin> ...` (text and the
// binary kCmdSetOodThresholds) updates this together with the OOD layer so a
// lowered threshold reaches the slot match rule, not just Layers 2/3.
static volatile float s_sign_match_min_prob = kSignMatchMinProb;
static inline float sign_clamp01(float v) {
  if (v < 0.0f) return 0.0f;
  if (v > 1.0f) return 1.0f;
  return v;
}

// RX diagnostics: count bytes and control packets received from S3.
static volatile uint32_t s_rx_bytes = 0;
static volatile uint32_t s_rx_ack_stop = 0;
static volatile uint32_t s_rx_resume = 0;

// True while the P4 Debug Serial carries a binary frame stream (capture /
// infer+gray modes).  Defined after s_op_mode below; declared here so the
// UART control helpers can use it.
static bool serial_stream_clean();

static void uart_control_enable() {
  s_transmit_enabled = true;
  if (!serial_stream_clean()) Serial.println("UART TX: ENABLED");
}

static void uart_control_disable() {
  s_transmit_enabled = false;
  if (!serial_stream_clean()) Serial.println("UART TX: DISABLED (S3 ack)");
}

static HardwareSerial UartToS3(1);

static constexpr int kDebugBaud = 921600;
static constexpr int kUartBaud = 921600;
static constexpr int kUartRxPin = 10;
static constexpr int kUartTxPin = 11;

static constexpr int kHandshakePwmPin = 9;
static constexpr int kHandshakePwmDuty = 255;

static constexpr uint8_t kSync0 = 0xAA;
static constexpr uint8_t kSync1 = 0x55;
static constexpr uint8_t kMsgTypeInference = 0x01;
static constexpr uint8_t kMsgTypeControl   = 0x02;  // S3 → P4 control messages

// ── UART verdict encoding (flags byte, 9-byte inference packet) ─────────
// Low nibble 0x02 = "sign reader frame" (same as the other sketches).
// High nibble is the verdict:
//   0x00  MATCH      all three slots matched ONE allowed plate (flags=0x02)
//   0xE0  MISMATCH   sign present, but at least one slot did not match
//                    any allowed plate (flags=0xE2)
//   0xF0  NO SIGN    the frame has no ink at all (flags=0xF2)
//
// label_id (byte 5) says WHICH plate matched — this is the multi-plate
// protocol the ESP32-S3 gatekeeper consumes:
//   0 .. kTargetSignCount-1   MATCH, the matched plate index
//   0xFE                      MISMATCH   (no plate matched all 3 slots)
//   0xFF                      NO SIGN    (nothing to read)
// confidence is the MIN per-slot best raw score (0..255), so it means
// "the weakest slot".
static constexpr uint8_t kFlagsSignReady     = 0x02;
static constexpr uint8_t kFlagsVerdictMask   = 0xF0;
static constexpr uint8_t kFlagsVerdictMatch  = 0x00;
static constexpr uint8_t kFlagsVerdictMismatch = 0xE0;
static constexpr uint8_t kFlagsVerdictNoSign = 0xF0;
static constexpr uint8_t kSignMismatchLabel  = 0xFE;
static constexpr uint8_t kSignNoSignLabel    = 0xFF;

// S3 → P4 control commands
static constexpr uint8_t kCtrlAckStop = 0x01;  // S3 confirmed sign → stop TX
static constexpr uint8_t kCtrlResume  = 0x02;  // S3 done → resume TX

static constexpr uint32_t kUartTxTaskStackBytes = 4 * 1024;
static constexpr uint32_t kUartRxTaskStackBytes = 4 * 1024;
// The inference task needs more than 32 KB: the auto search-box chain
// (adaptive band selection -> find_search_box -> BFS) plus ESP-NN kernels
// overflowed a 32 KB stack (observed 2.6 KB past the bound on Core 1).
static constexpr uint32_t kInferenceTaskStackBytes = 64 * 1024;
static constexpr uint32_t kSdTaskStackBytes = 8 * 1024;
static constexpr uint32_t kDebugCmdStackBytes = 6 * 1024;

// ── Debug Serial Command parser (same sync-based layout as UART) ─────
// Frame format on Debug Serial (USB CDC @ 921600):
//   | SYNC0=0xAA | SYNC1=0x55 | SYNC2=0xCC | cmd | len | payload[len..] | xor8(0..4+len) |
// SYNC2=0xCC distinguishes debug commands from:
//   - 0x01 = UART inference, 0x02 = UART control, 0xAA = RGB stream header
// Responses share the SAME layout; len=N on error responses with an ASCII
// null-terminated message payload.  Cmd IDs are listed in kCmd*.
static constexpr uint8_t kDebugSync0 = 0xAA;
static constexpr uint8_t kDebugSync1 = 0x55;
static constexpr uint8_t kDebugSync2 = 0xCC;

// Commands (Host → P4)
static constexpr uint8_t kCmdNop              = 0x00;
static constexpr uint8_t kCmdSetOodThresholds = 0x01;   // 4× float LE (sign_pct_min, sign_pct_max, max_prob_min, entropy_ratio_max)
static constexpr uint8_t kCmdGetOodThresholds = 0x02;   // → 4× float LE response
static constexpr uint8_t kCmdSetMaskThresholds= 0x03;   // 2× uint8 LE (dark, lum)   — Host UI Dark/Lum sliders
static constexpr uint8_t kCmdGetMaskThresholds= 0x04;   // → 2× uint8 LE response
static constexpr uint8_t kCmdSetMode          = 0x05;   // 1× uint8: 0=INFERENCE, 1=CAPTURE_RGB, 2=CAPTURE_GRAY
static constexpr uint8_t kCmdGetMode          = 0x06;   // → 1× uint8 response
static constexpr uint8_t kCmdPing             = 0x7F;   // → ack len=0

// Operation mode (runtime switchable via Debug Serial)
enum class OpMode : uint8_t {
  kInference = 0,   // default: full pipeline → UART / SD / periodic log
  kCaptureRgb= 1,   // PLAIN: AA 55 AA + 96×96×3 RGB, works with the AItraining capture panel
  kCaptureGray= 2,  // PLAIN: AA 55 AA + g_frame_side² GRAY8 (raw, uncropped)
  kInferGray  = 3,  // infer + stream GRAY on Serial (S3 UART still active), no logs
  kExtCaptureRgb =11,// EXTENDED: AA 55 AB + kind/fid/w/h + pixels + xor (scripting tools)
  kExtCaptureGray=12 // EXTENDED: same as above, raw GRAY8
};
static volatile OpMode s_op_mode = OpMode::kInference;

// True while the P4 Debug Serial carries a binary frame stream (capture /
// infer+gray modes).  In these modes any text written to Serial splices
// bytes into the AA 55 AA frame stream and corrupts frames for
// AItraining's SerialFrameReader — so asynchronous text logs (SD task,
// UART control prints) must stay off the wire.  The one-time banners and
// the mode-transition line print before the first frame, which is safe.
static bool serial_stream_clean() {
  switch (s_op_mode) {
    case OpMode::kCaptureRgb:
    case OpMode::kCaptureGray:
    case OpMode::kInferGray:
    case OpMode::kExtCaptureRgb:
    case OpMode::kExtCaptureGray:
      return true;
    default:
      return false;
  }
}
static const char *op_mode_name(OpMode m) {
  switch (m) {
    case OpMode::kInference:     return "INFERENCE";
    case OpMode::kCaptureRgb:    return "CAPTURE_RGB/plain(AItraining)";
    case OpMode::kCaptureGray:   return "CAPTURE_GRAY/plain(AItraining)";
    case OpMode::kInferGray:     return "INFER+GRAY (S3 + gray stream)";
    case OpMode::kExtCaptureRgb: return "CAPTURE_RGB/extended";
    case OpMode::kExtCaptureGray:return "CAPTURE_GRAY/extended";
    default: return "?";
  }
}

// ── Capture-mode payload sync on Debug Serial ────────────────────────
//   We support TWO wire formats, chosen per mode keyword below.
//   Defaults (mode gray / mode rgb) match the AItraining app so the
//   existing SerialFrameReader class works without code changes:
//
//   Format A = PLAIN (AItraining-compatible, default) —
//     GRAY : AA 55 AA  +  frame_side*frame_side*1  bytes GRAY8
//     RGB  : AA 55 AA  +  frame_side*frame_side*3  bytes RGB24 (R-G-B interleave)
//   This exactly matches `HEADER = bytes([0xAA,0x55,0xAA])` plus
//   `frame_size = side² * channels` layout used by AItraining's
//   SerialFrameReader.read_frame() — so live serial preview, data
//   capture panel and source capture of "upper" 96×96 grayscale work
//   out of the box with no app changes.
//
//   Format B = EXTENDED (script tools only, reachable via explicit
//   numeric mode 11/12) —
//     0xAA 0x55 0xAB  kind(1)  fid_LE(2)  w_LE(2)  h_LE(2)  <BYTES>  xor8
//   kind 1=RGB24 / 2=GRAY8.  xor8 covers every byte before checksum.
//   Students should not need this — reserved for offline tools that
//   want a strict checksum + metadata for every captured frame.
static constexpr uint8_t kCapSync0 = 0xAA;
static constexpr uint8_t kCapSync1 = 0x55;
static constexpr uint8_t kCapSync2 = 0xAA;  // PLAIN default: match AItraining
static constexpr uint8_t kCapExtSync2  = 0xAB;
static constexpr uint8_t kCapKindRgb = 0x01;
static constexpr uint8_t kCapKindGray= 0x02;

static constexpr uint32_t kSaveEveryNFrames = 10;
static constexpr int kImageWidth = OUT_WIDTH;
static constexpr int kImageHeight = OUT_WIDTH;
static constexpr size_t kImageBytes = (size_t)kImageWidth * (size_t)kImageHeight;
static constexpr size_t kSdQueueDepth = 2;

// ── Model-input geometry (runtime, not compile-time) ────────────────────
// kImageWidth/kImageHeight are a fallback; this sketch's real input tensor is
// ONE 28x28x1 glyph canvas (784 B), fed three times per frame (once per slot).
// The SD logger and the pre-Invoke snapshot follow the ACTUAL tensor: setup()
// sets these from input->dims / input->bytes once AllocateTensors() has run.
// A PGM header that disagrees with the byte count is an unreadable file, and a
// snapshot memcpy sized for 9216 reads 8432 bytes of someone else's arena (the
// exact moiré-garbage failure the snapshot exists to prevent).
//
// The buffer is sized for the glyph canvas; the copy length is whatever
// input->bytes says (guarded at boot).  The SD PGM is SLOT 0 — a 3-slot
// montage would need a 3x canvas and a different header.
static constexpr size_t kSnapshotBytes =
    (size_t)SIGN_GLYPH_SIZE * (size_t)SIGN_GLYPH_SIZE;
static int8_t s_input_snapshot[kSnapshotBytes];
static size_t   s_input_bytes = kImageBytes;   // := input->bytes in setup()
static uint16_t s_save_w = (uint16_t)kImageWidth;   // := input->dims in setup()
static uint16_t s_save_h = (uint16_t)kImageHeight;
static constexpr bool kEnableSdLogger = true;
static constexpr bool kUseSdMmcCustomPins = false;
static constexpr int kSdClkPin = -1;
static constexpr int kSdCmdPin = -1;
static constexpr int kSdD0Pin = -1;
static constexpr int kSdD1Pin = -1;
static constexpr int kSdD2Pin = -1;
static constexpr int kSdD3Pin = -1;

static constexpr const char *kMountPoint = "/sdcard";
static constexpr const char *kFfatMountPoint = "/ffat";
enum class StorageBackend : uint8_t { Auto = 0, SdMmc = 1, FFat = 2 };
static constexpr StorageBackend kStorageBackend = StorageBackend::SdMmc;
static constexpr bool kFormatFfatOnFail = true;

#ifndef BOARD_SDMMC_POWER_PIN
#define BOARD_SDMMC_POWER_PIN 45
#endif

#ifndef BOARD_SDMMC_POWER_ON_LEVEL
#define BOARD_SDMMC_POWER_ON_LEVEL 0
#endif

static void sdcard_power_cycle(int on_level) {
  gpio_set_direction((gpio_num_t)BOARD_SDMMC_POWER_PIN, GPIO_MODE_OUTPUT);
  int off_level = on_level ? 0 : 1;
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, off_level);
  delay(200);
  gpio_set_level((gpio_num_t)BOARD_SDMMC_POWER_PIN, on_level);
  delay(300);
}

struct UartPacket {
  uint16_t frame_id;
  uint8_t label_id;
  uint8_t confidence;
  uint8_t flags;
};

static uint8_t calc_uart_checksum(const uint8_t *data, size_t len) {
  uint8_t checksum = 0;
  for (size_t i = 0; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

static QueueHandle_t s_uart_queue = nullptr;

struct SdPacket {
  uint16_t frame_id;
  uint8_t  buffer_index;
  uint8_t  label_id;
  uint8_t  confidence;
  uint8_t  flags;
};

static QueueHandle_t s_sd_queue = nullptr;
// Free-list of s_sd_buffers indices.  The inference task takes an index
// from here before filling a buffer; the sd task returns it after the PGM
// write.  Without this, a slow SD write could outlive two queued packets
// and the ring counter would hand the SAME buffer to the producer while
// the sd task is still reading it (torn PGM frames).
static QueueHandle_t s_sd_free_queue = nullptr;
static uint8_t s_sd_buffers[kSdQueueDepth][kImageBytes];
static char s_run_dir[32] = {0};
static bool s_sd_using_sd_mmc = false;
static bool s_sd_using_ffat = false;
static const char *s_sd_mount_point = kMountPoint;
static bool s_sd_full = false;

// In order to use optimized tensorflow lite kernels, a signed int8_t quantized
// model is preferred over the legacy unsigned model format. This means that
// throughout this project, input images must be converted from unisgned to
// signed format. The easiest and quickest way to convert from unsigned to
// signed 8-bit integers is to subtract 128 from the unsigned value to get a
// signed value.

// An area of memory to use for input, output, and intermediate arrays.
// The compact glyph head (conv 32/64/128 -> GAP -> Dense 128) fits in a
// 320 KB arena, which fits the P4's internal SRAM.  SRAM is tried first;
// if AllocateTensors() still asks for more, setup() retries once in PSRAM
// with the legacy 8 MB ceiling so an older bloated export still boots (it
// just runs from PSRAM).  A failed allocation now prints the exact reason
// instead of returning silently from setup().
constexpr size_t kTensorArenaSize = 320 * 1024;
constexpr size_t kTensorArenaFallbackSize = 8 * 1024 * 1024;
static uint8_t *tensor_arena = nullptr;
static size_t s_arena_bytes = 0;

#define SIGN_FATAL(...)                                                        \
  do {                                                                         \
    Serial.print("FATAL: ");                                                   \
    Serial.printf(__VA_ARGS__);                                                \
    Serial.print("\r\n");                                                     \
    Serial.flush();                                                            \
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }                              \
  } while (0)

}  // namespace

static bool mount_sd() {
  s_sd_using_sd_mmc = false;
  s_sd_using_ffat = false;
  s_sd_mount_point = kMountPoint;

  auto mount_ffat = []() -> bool {
    FFat.end();
    if (FFat.begin(false, kFfatMountPoint, 5, "ffat")) {
      s_sd_using_ffat = true;
      s_sd_mount_point = kFfatMountPoint;
      return true;
    }
    if (kFormatFfatOnFail && FFat.begin(true, kFfatMountPoint, 5, "ffat")) {
      s_sd_using_ffat = true;
      s_sd_mount_point = kFfatMountPoint;
      return true;
    }
    return false;
  };

  auto mount_sdmmc = []() -> bool {
    gpio_set_pull_mode((gpio_num_t)44, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)39, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)40, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)41, GPIO_PULLUP_ONLY);
    gpio_set_pull_mode((gpio_num_t)42, GPIO_PULLUP_ONLY);

    #ifdef SOC_SDMMC_IO_POWER_EXTERNAL
    SD_MMC.setPowerChannel(-1);
    #endif

    const int on_levels[] = {BOARD_SDMMC_POWER_ON_LEVEL, BOARD_SDMMC_POWER_ON_LEVEL ? 0 : 1};
    for (int attempt = 0; attempt < 12; attempt++) {
      for (size_t lv = 0; lv < (sizeof(on_levels) / sizeof(on_levels[0])); lv++) {
        sdcard_power_cycle(on_levels[lv]);

        SD_MMC.end();
        if (SD_MMC.begin(kMountPoint, false, false, SDMMC_FREQ_DEFAULT)) {
          s_sd_using_sd_mmc = true;
          s_sd_mount_point = kMountPoint;
          return true;
        }

        SD_MMC.end();
        if (SD_MMC.begin(kMountPoint, false, false, 400)) {
          s_sd_using_sd_mmc = true;
          s_sd_mount_point = kMountPoint;
          return true;
        }
      }
      delay(200);
    }

    SD_MMC.end();
    s_sd_using_sd_mmc = false;
    return false;
  };

  if (kStorageBackend == StorageBackend::FFat) {
    return mount_ffat();
  }
  if (kStorageBackend == StorageBackend::SdMmc) {
    return mount_sdmmc();
  }

  if (mount_sdmmc()) {
    return true;
  }
  return mount_ffat();

}

static bool write_pgm(const char *path, const uint8_t *gray) {
  FILE *f = fopen(path, "wb");
  if (!f) {
    return false;
  }
  // Runtime geometry: s_save_w/h/bytes follow the model's input tensor
  // (set in setup()), so a sign run writes a readable 28x28 PGM instead of a
  // 96x96 header over 784 bytes of data.
  fprintf(f, "P5\n%u %u\n255\n", (unsigned)s_save_w, (unsigned)s_save_h);
  size_t written = fwrite(gray, 1, s_input_bytes, f);
  fclose(f);
  return written == s_input_bytes;
}

static bool make_run_dir() {
  uint32_t max_id = 0;

  DIR *dir = opendir(s_sd_mount_point);
  if (dir) {
    for (;;) {
      struct dirent *ent = readdir(dir);
      if (!ent) {
        break;
      }
      const char *p = ent->d_name;
      if (strncmp(p, "run_", 4) != 0) {
        continue;
      }
      uint32_t id = atoi(p + 4);
      if (id && id <= 9999 && id > max_id) {
        max_id = id;
      }
    }
    closedir(dir);
  }

  for (uint32_t i = 0; i < 1000; i++) {
    uint32_t id = max_id + 1 + i;
    snprintf(s_run_dir, sizeof(s_run_dir), "%s/run_%04d", s_sd_mount_point, id);
    if (mkdir(s_run_dir, 0775) == 0) {
      return true;
    }
  }

  return false;
}

static void sd_task(void *arg) {
  (void)arg;
  uint32_t saved = 0;
  static bool s_labels_csv_header_written = false;
  while (true) {
    SdPacket pkt;
    if (xQueueReceive(s_sd_queue, &pkt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (s_sd_full) {
      // Buffer never used — return it or the free-list drains and the
      // producer stalls on receive.
      if (s_sd_free_queue) xQueueSend(s_sd_free_queue, &pkt.buffer_index, 0);
      continue;
    }

    char path[96];
    snprintf(path, sizeof(path), "%s/frame_%05u.pgm", s_run_dir, (unsigned)pkt.frame_id);
    bool ok = write_pgm(path, s_sd_buffers[pkt.buffer_index]);

    if (ok) {
      char csv_path[96];
      snprintf(csv_path, sizeof(csv_path), "%s/labels.csv", s_run_dir);
      FILE *fc = fopen(csv_path, "a");
      if (fc) {
        if (!s_labels_csv_header_written) {
          fprintf(fc, "frame_id,label_id,confidence,flags\n");
          s_labels_csv_header_written = true;
        }
        fprintf(fc, "%u,%u,%u,%u\n",
                (unsigned)pkt.frame_id,
                (unsigned)pkt.label_id,
                (unsigned)pkt.confidence,
                (unsigned)pkt.flags);
        fclose(fc);
      } else if (errno == ENOSPC) {
        ok = false;
      }
    }

    if (ok) saved++;

    if (!ok && errno == ENOSPC) {
      s_sd_full = true;
      if (!serial_stream_clean()) {
        Serial.printf("Storage full (ENOSPC). Stop saving frames. Last=%s\n", path);
      }
    }

    // PGM write finished — return the buffer to the free-list so the
    // producer can safely reuse it.
    if (s_sd_free_queue) xQueueSend(s_sd_free_queue, &pkt.buffer_index, 0);

    if ((saved % 10) == 0 && !serial_stream_clean()) {
      Serial.printf("SD saved=%lu last=%s %s\n", (unsigned long)saved, path, ok ? "OK" : "FAIL");
    }
  }
}

static void uart_tx_task(void *arg) {
  (void)arg;
  for (;;) {
    UartPacket pkt;
    if (xQueueReceive(s_uart_queue, &pkt, portMAX_DELAY) != pdTRUE) {
      continue;
    }

    if (!s_transmit_enabled) {
      continue;  // silently drop — S3 hasn't asked us to resume yet
    }

    uint8_t buf[9];
    buf[0] = kSync0;
    buf[1] = kSync1;
    buf[2] = kMsgTypeInference;
    buf[3] = (uint8_t)(pkt.frame_id & 0xFF);
    buf[4] = (uint8_t)((pkt.frame_id >> 8) & 0xFF);
    buf[5] = pkt.label_id;
    buf[6] = pkt.confidence;
    buf[7] = pkt.flags;
    buf[8] = calc_uart_checksum(buf, 8);
    UartToS3.write(buf, sizeof(buf));
  }
}

static void uart_rx_task(void *arg) {
  (void)arg;
  // State machine for reading 5-byte control packets from S3:
  //   [0xAA, 0x55, 0x02, cmd, checksum]
  static uint8_t rx_buf[5];
  static uint8_t rx_idx = 0;
  static uint8_t rx_state = 0;  // 0=wait sync0, 1=wait sync1, 2=wait rest

  uint32_t last_rx_log_ms = 0;

  for (;;) {
    while (UartToS3.available() > 0) {
      uint8_t b = (uint8_t)UartToS3.read();
      s_rx_bytes++;

      if (rx_state == 0) {
        if (b == kSync0) {
          rx_buf[0] = b;
          rx_idx = 1;
          rx_state = 1;
        }
        continue;
      }

      if (rx_state == 1) {
        if (b == kSync1) {
          rx_buf[1] = b;
          rx_idx = 2;
          rx_state = 2;
        } else if (b == kSync0) {
          rx_buf[0] = b;
          rx_idx = 1;
          rx_state = 1;
        } else {
          rx_state = 0;
        }
        continue;
      }

      // rx_state == 2: collecting remaining bytes
      rx_buf[rx_idx++] = b;
      if (rx_idx < sizeof(rx_buf)) {
        continue;
      }

      // Full packet received
      rx_state = 0;
      rx_idx = 0;

      if (rx_buf[2] != kMsgTypeControl) {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: unknown msg_type 0x%02X (expected 0x%02X)\n",
                        rx_buf[2], kMsgTypeControl);
        }
        continue;
      }

      // Verify checksum: XOR of bytes 0-3
      uint8_t expected_csum = calc_uart_checksum(rx_buf, 4);
      if (rx_buf[4] != expected_csum) {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: bad checksum (got 0x%02X expected 0x%02X)\n",
                        rx_buf[4], expected_csum);
        }
        continue;
      }

      uint8_t cmd = rx_buf[3];

      if (cmd == kCtrlAckStop) {
        s_rx_ack_stop++;
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: ACK_STOP from S3 (#%lu) — disabling TX\n",
                        (unsigned long)s_rx_ack_stop);
        }
        uart_control_disable();
      } else if (cmd == kCtrlResume) {
        s_rx_resume++;
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: RESUME from S3 (#%lu) — resuming TX\n",
                        (unsigned long)s_rx_resume);
        }
        uart_control_enable();
      } else {
        if (!serial_stream_clean()) {
          Serial.printf("UART RX: unknown control cmd 0x%02X\n", cmd);
        }
      }
    }

    // Periodic heartbeat: show RX stats even when idle.
    // NEVER print while Serial carries a binary frame stream (capture /
    // infer+gray modes) — the text line would be injected between frames
    // and desync the host frame parser.
    uint32_t now = millis();
    if (now - last_rx_log_ms >= 5000) {
      last_rx_log_ms = now;
      if (!serial_stream_clean()) {
        Serial.printf("UART RX: bytes=%lu ack_stop=%lu resume=%lu tx_enabled=%d\n",
                      (unsigned long)s_rx_bytes, (unsigned long)s_rx_ack_stop,
                      (unsigned long)s_rx_resume, (int)s_transmit_enabled);
      }
    }

    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

// ── Debug Serial → Host Response helpers ───────────────────────────────
//  All responses use the SAME sync (0xAA 0x55 0xCC) frame layout as
//  incoming commands.  This matches the UART↔S3 "parser sync" feel you
//  liked.  On error the payload is a null-terminated ASCII string.
static uint8_t debug_xor(const uint8_t *p, size_t n) {
  uint8_t x = 0;
  for (size_t i = 0; i < n; i++) x ^= p[i];
  return x;
}

static void debug_send_ack(uint8_t cmd, const uint8_t *payload, uint8_t len) {
  uint8_t hdr[6] = { kDebugSync0, kDebugSync1, kDebugSync2, cmd, len, 0x00 };
  uint8_t x = debug_xor(hdr, 5);
  for (uint8_t i = 0; i < len; i++) x ^= payload[i];
  Serial.write(hdr, 5);
  if (len > 0 && payload) Serial.write(payload, len);
  Serial.write(x);
  Serial.flush();
}
static void debug_send_ok(uint8_t cmd) {
  debug_send_ack(cmd, nullptr, 0);
}
static void debug_send_error(uint8_t cmd, const char *msg) {
  if (!msg) msg = "err";
  size_t n = strlen(msg) + 1;  // include null terminator
  if (n > 255) n = 255;
  debug_send_ack((uint8_t)(cmd | 0x80), (const uint8_t *)msg, (uint8_t)n);
}

// Read a little-endian float from bytes.  Trivial helper — no unaligned load
// on RISC-V so we byte-by-byte copy to a local aligned.
static float read_f32_le(const uint8_t *p) {
  uint32_t u = (uint32_t)p[0]
             | ((uint32_t)p[1] << 8)
             | ((uint32_t)p[2] << 16)
             | ((uint32_t)p[3] << 24);
  float f;
  memcpy(&f, &u, 4);
  return f;
}
static void write_f32_le(uint8_t *dst, float f) {
  uint32_t u;
  memcpy(&u, &f, 4);
  dst[0] = (uint8_t)(u & 0xFF);
  dst[1] = (uint8_t)((u >> 8) & 0xFF);
  dst[2] = (uint8_t)((u >> 16) & 0xFF);
  dst[3] = (uint8_t)((u >> 24) & 0xFF);
}
static void write_u16_le(uint8_t *dst, uint16_t v) {
  dst[0] = (uint8_t)(v & 0xFF);
  dst[1] = (uint8_t)((v >> 8) & 0xFF);
}

// ── Debug Serial: text + binary dual parser, runtime control + capture ──
//  Two input styles share one serial port so students + scripts coexist:
//
//   Style 1 (students, Arduino Serial Monitor):   plain ASCII lines
//     e.g.  set mask 30 85  →  OK  MASK: dark=30 lum=85
//           mode gray       →  OK  MODE: capture_gray
//           get ood         →  OOD: sp=0.50..70.00 mp=0.700 er=0.650
//           help            →  list of commands
//     Case-insensitive, space/tab separated, line ends with CR or LF.
//
//   Style 2 (Python/scripts): sync-framed binary packets, identical feel
//     to the P4↔S3 UART "flag-based parser" the student asked for:
//        AA 55 CC | cmd | len | payload | xor8
//
//  Runtime mode switch lives at the top of the inference loop; when mode
//  is not kInference we skip Invoke, SD writer and UART TX/RX so nothing
//  clobbers the bus — only capture frames stream out over Debug Serial.
//
// ── ASCII line command helpers (for Arduino Serial Monitor) ─────────────
//   Students type a line (e.g. "set mask 30 85" + Enter) instead of binary.
//   Supported commands (case-insensitive, space/tab separated):
//     help | ?         → print this list
//     ping             → "PONG"
//     get mask         → "MASK: dark=<D> lum=<L>"
//     set mask D L     → Dark/Lum (0..255, auto-clamped, dark<lum enforced)
//     get ood          → "OOD: sp=<min>..<max> mp=<mpmin> er=<ermax>"
//     set ood spmin spmax mpmin ermax
//                      → 4 floats, see image_provider OOD docs
//     get mode         → "MODE: infer | rgb | gray"
//     mode infer       → back to normal inference/SD/UART
//     mode rgb         → dump raw RGB 96×96×3 on Debug Serial (CAPTURE_RGB)
//     mode gray        → dump preprocessed GRAY8 96×96 (what TFLite sees)
static char *skip_ws(char *p) {
  while (*p && (*p==' ' || *p=='\t' || *p=='\r' || *p=='\n')) p++;
  return p;
}
static char *next_tok(char *p, char **out_tok) {
  p = skip_ws(p);
  *out_tok = p;
  while (*p && *p!=' ' && *p!='\t' && *p!='\r' && *p!='\n') {
    *p = (char)tolower((uint8_t)*p);
    p++;
  }
  if (*p) { *p = '\0'; p++; }
  return p;
}
static void ascii_print_help() {
  Serial.println(F(
    "Commands (Serial Monitor, Newline=NL&CR baud 921600):\r\n"
    "  help | ?         → this list\r\n"
    "  ping             → PONG\r\n"
    "  get mask         → MASK dark=?? lum=??  (G channel sign-mask)\r\n"
    "  set mask D L     → D=int L=int e.g.  set mask 30 85\r\n"
    "  get ood          → OOD spmin..spmax mpmin ermax (mpmin = per-slot match floor)\r\n"
    "  set ood A B C D  → 4 floats, e.g. set ood 0.5 70.0 0.4 0.65; C is also the per-slot match floor\r\n"
    "  get mode         → MODE infer|rgb|gray\r\n"
    "  mode infer       → normal inference + UART + SD\r\n"
    "  mode rgb         → pause, dump RAW RGB24 capture frames on serial\r\n"
    "  mode gray        → pause, dump GRAY8 = what TFLite input tensor sees"
  ));
}
static void ascii_dispatch(char *line) {
  line = skip_ws(line);
  if (!*line) return;
  char *save = nullptr, *tok = nullptr;
  (void)save;
  char *p = next_tok(line, &tok);
  if (!*tok) return;
  if (0 == strcmp(tok, "help") || 0 == strcmp(tok, "?")) {
    ascii_print_help(); return;
  }
  if (0 == strcmp(tok, "ping")) {
    Serial.println(F("PONG")); return;
  }
  if (0 == strcmp(tok, "get")) {
    p = next_tok(p, &tok);
    if (0 == strcmp(tok, "mask")) {
      int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
      Serial.printf("MASK: dark=%d lum=%d\r\n", d, l);
    } else if (0 == strcmp(tok, "ood")) {
      float a,b,c,d; ImageProviderGetOodThresholds(&a,&b,&c,&d);
      Serial.printf("OOD: sp=%.2f..%.2f (disarmed) mp=%.3f er=%.3f match=%.3f\r\n",
                    (double)a,(double)b,(double)c,(double)d,
                    (double)s_sign_match_min_prob);
    } else if (0 == strcmp(tok, "mode")) {
      Serial.printf("MODE: %s\r\n", op_mode_name(s_op_mode));
    } else {
      Serial.printf("ERR: unknown get '%s', try 'help'\r\n", tok);
    }
    return;
  }
  if (0 == strcmp(tok, "set")) {
    p = next_tok(p, &tok);
    if (0 == strcmp(tok, "mask")) {
      char *a = nullptr, *b = nullptr;
      p = next_tok(p, &a); p = next_tok(p, &b);
      if (!*a || !*b) { Serial.println(F("ERR: set mask <D> <L>, e.g. set mask 30 85")); return; }
      int d = atoi(a), l = atoi(b);
      ImageProviderSetMaskThresholds(d, l);
      int d2=0,l2=0; ImageProviderGetMaskThresholds(&d2,&l2);
      Serial.printf("OK  MASK: dark=%d lum=%d\r\n", d2, l2);
    } else if (0 == strcmp(tok, "ood")) {
      char *a=nullptr,*b=nullptr,*c=nullptr,*d=nullptr;
      p = next_tok(p, &a); p = next_tok(p, &b); p = next_tok(p, &c); p = next_tok(p, &d);
      if (!*a||!*b||!*c||!*d) { Serial.println(F("ERR: set ood <spmin> <spmax> <mpmin> <ermax>")); return; }
      float fa=(float)atof(a), fb=(float)atof(b), fc=(float)atof(c), fd=(float)atof(d);
      ImageProviderSetOodThresholds(fa,fb,fc,fd);
      // The third argument (mpmin) is the per-slot confidence floor too:
      // the slot match rule must follow the same knob the user just lowered.
      s_sign_match_min_prob = sign_clamp01(fc);
      float a2,b2,c2,d2; ImageProviderGetOodThresholds(&a2,&b2,&c2,&d2);
      Serial.printf("OK  OOD: sp=%.2f..%.2f mp=%.3f er=%.3f match=%.3f\r\n",
                    (double)a2,(double)b2,(double)c2,(double)d2,
                    (double)s_sign_match_min_prob);
    } else {
      Serial.printf("ERR: unknown set '%s', try 'help'\r\n", tok);
    }
    return;
  }
  if (0 == strcmp(tok, "mode")) {
    p = next_tok(p, &tok);
    OpMode m;
    if      (0==strcmp(tok,"infer")||0==strcmp(tok,"inference")||0==strcmp(tok,"0")) m = OpMode::kInference;
    else if (0==strcmp(tok,"rgb") ||0==strcmp(tok,"1"))                                 m = OpMode::kCaptureRgb;
    else if (0==strcmp(tok,"gray")||0==strcmp(tok,"grey")||0==strcmp(tok,"2"))         m = OpMode::kCaptureGray;
    else if (0==strcmp(tok,"infergray")||0==strcmp(tok,"infer_gray")||0==strcmp(tok,"3")) m = OpMode::kInferGray;
    else if (0==strcmp(tok,"ext_rgb") ||0==strcmp(tok,"11")) m = OpMode::kExtCaptureRgb;
    else if (0==strcmp(tok,"ext_gray")||0==strcmp(tok,"12")) m = OpMode::kExtCaptureGray;
    else { Serial.printf("ERR: mode infer|rgb|gray|infergray|ext_rgb|ext_gray, got '%s'\r\n", tok); return; }
    s_op_mode = m;
    Serial.printf("OK  MODE: %s\r\n", op_mode_name(m));
    return;
  }
  Serial.printf("ERR: unknown command '%s', type 'help'\r\n", tok);
}

static void debug_serial_task(void *arg) {
  (void)arg;
  enum { SYNC0, SYNC1, SYNC2, CMD, LEN, PAY, SUM };
  static uint8_t st = SYNC0;
  static uint8_t buf[5 + 255 + 1];   // hdr 5 + payload 255 + checksum 1
  static uint8_t idx = 0;
  static uint8_t pln = 0;

  static char ascii_buf[256];
  static uint8_t ascii_len = 0;

  Serial.printf("DEBUG: task up @ %u baud — type 'help' for Serial Monitor commands; binary sync=AA 55 CC\r\n", (unsigned)kDebugBaud);

  for (;;) {
    while (Serial.available() > 0) {
      const uint8_t b = (uint8_t)Serial.read();

      // ── ASCII line route (Serial Monitor, human typed) ────────────
      if (st == SYNC0 && b != kDebugSync0) {
        if (b == '\r' || b == '\n') {
          if (ascii_len) {
            ascii_buf[ascii_len] = '\0';
            ascii_dispatch(ascii_buf);
            ascii_len = 0;
          }
          continue;
        }
        if (ascii_len < (uint8_t)(sizeof(ascii_buf)-1)) {
          ascii_buf[ascii_len++] = (char)b;
        }
        continue;
      }

      // ── Binary packet route (Python/script, sync-framed) ──────────
      switch (st) {
        case SYNC0: st = (b == kDebugSync0) ? SYNC1 : SYNC0; continue;
        case SYNC1:
          if (b == kDebugSync1) { st = SYNC2; buf[0]=kDebugSync0; buf[1]=kDebugSync1; idx=2; }
          else if (b == kDebugSync0) { st = SYNC1; }
          else st = SYNC0;
          continue;
        case SYNC2:
          if (b == kDebugSync2) { buf[idx++] = b; st = CMD; }
          else if (b == kDebugSync0) { st = SYNC1; }
          else st = SYNC0;
          continue;
        case CMD:  buf[idx++] = b; st = LEN; continue;
        case LEN:  pln = b; buf[idx++] = b; st = (pln == 0) ? SUM : PAY; continue;
        case PAY:  buf[idx++] = b; if (idx - 5 >= pln) st = SUM; continue;
        case SUM:  break;   // handled below
      }

      // ── SUM state reached: validate checksum then dispatch ──────────
      const uint8_t calc = debug_xor(buf, (size_t)(5 + pln));
      uint8_t cmd = buf[3];
      st = SYNC0;
      if (calc != b) {
        debug_send_error(cmd, "checksum");
        continue;
      }
      const uint8_t *pl = buf + 5;
      bool handled = true;
      switch (cmd) {
        case kCmdNop: {
          debug_send_ok(cmd);
        } break;
        case kCmdPing: {
          debug_send_ok(cmd);
        } break;
        case kCmdSetOodThresholds: {
          if (pln < 16) { handled = false; debug_send_error(cmd, "len16"); break; }
          float spmin = read_f32_le(pl + 0);
          float spmax = read_f32_le(pl + 4);
          float mpmin = read_f32_le(pl + 8);
          float ermax = read_f32_le(pl + 12);
          ImageProviderSetOodThresholds(spmin, spmax, mpmin, ermax);
          s_sign_match_min_prob = sign_clamp01(mpmin);
          Serial.printf("DEBUG: OOD thresholds → sp=%.2f..%.2f mp=%.3f er=%.3f match=%.3f\n",
                        (double)spmin, (double)spmax, (double)mpmin, (double)ermax,
                        (double)s_sign_match_min_prob);
          debug_send_ok(cmd);
        } break;
        case kCmdGetOodThresholds: {
          uint8_t obuf[16]; float spmin,spmax,mpmin,ermax;
          ImageProviderGetOodThresholds(&spmin,&spmax,&mpmin,&ermax);
          write_f32_le(obuf+0, spmin);
          write_f32_le(obuf+4, spmax);
          write_f32_le(obuf+8, mpmin);
          write_f32_le(obuf+12, ermax);
          debug_send_ack(cmd, obuf, 16);
        } break;
        case kCmdSetMaskThresholds: {
          if (pln < 2) { handled = false; debug_send_error(cmd, "len2"); break; }
          int dark = (int)pl[0];
          int lum  = (int)pl[1];
          ImageProviderSetMaskThresholds(dark, lum);
          { int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
            Serial.printf("DEBUG: Dark/Lum (mask thresholds) → dark=%d lum=%d\n", d, l); }
          debug_send_ok(cmd);
        } break;
        case kCmdGetMaskThresholds: {
          int d=0,l=0; ImageProviderGetMaskThresholds(&d,&l);
          uint8_t obuf[2] = { (uint8_t)(d&0xFF), (uint8_t)(l&0xFF) };
          debug_send_ack(cmd, obuf, 2);
        } break;
        case kCmdSetMode: {
          if (pln < 1) { handled = false; debug_send_error(cmd, "len1"); break; }
          uint8_t raw = pl[0];
          OpMode new_mode;
          switch (raw) {
            case 0:  new_mode = OpMode::kInference;      break;
            case 1:  new_mode = OpMode::kCaptureRgb;     break;
            case 2:  new_mode = OpMode::kCaptureGray;    break;
            case 3:  new_mode = OpMode::kInferGray;      break;
            case 11: new_mode = OpMode::kExtCaptureRgb;  break;
            case 12: new_mode = OpMode::kExtCaptureGray; break;
            default: handled = false; debug_send_error(cmd, "bad mode"); continue;
          }
          s_op_mode = new_mode;
          Serial.printf("DEBUG: mode → %s (0x%02X)\n", op_mode_name(new_mode), raw);
          debug_send_ok(cmd);
        } break;
        case kCmdGetMode: {
          uint8_t obuf[1] = { (uint8_t)s_op_mode };
          debug_send_ack(cmd, obuf, 1);
        } break;
        default: handled = false; debug_send_error(cmd, "bad cmd"); break;
      }
      (void)handled;
    }
    vTaskDelay(pdMS_TO_TICKS(2));
  }
}

// ── Capture-mode frame dump helpers on Debug Serial ────────────────────
//   Frame format:
//     0xAA 0x55 0xAB kind frame_id_LE w_LE h_LE  <BYTES>  xor8
//   kind = 0x01 RGB24 / 0x02 GRAY8.  xor covers every byte BEFORE checksum.
static void dump_capture_frame_header(uint8_t kind, uint16_t frame_id,
                                       uint16_t w, uint16_t h, size_t bytes) {
  uint8_t hdr[3 + 1 + 2 + 2 + 2];   // 3 sync + kind + fid + w + h
  hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapSync2;
  hdr[3] = kind;
  write_u16_le(hdr + 4, frame_id);
  write_u16_le(hdr + 6, w);
  write_u16_le(hdr + 8, h);
  uint8_t x = debug_xor(hdr, sizeof(hdr));
  (void)bytes;   // implied by w,h,kind; useful for logging
  Serial.write(hdr, sizeof(hdr));
}
static inline void dump_xor_final(uint8_t running_xor, const uint8_t *p, size_t n) {
  for (size_t i = 0; i < n; i++) running_xor ^= p[i];
  Serial.write(p, n);
  Serial.write(running_xor);
  Serial.flush();
}

// ── the trailing "_background" class ────────────────────────────────────
// An OCR/sign export carries N real classes plus a synthetic "_background"
// class meaning "no character in this crop" — a rejection, not a character.
// Each of the three slots rejects with it independently: a blank slot must not
// vote for a real class, or a noise column could complete a spurious MATCH.
//
// New exports carry the index explicitly (kBackgroundClassIndex); older ones
// are detected BY NAME, not by "the last index is always background": projects
// may add classes later, and a positional rule would silently reinterpret the
// exported target indices.  A model without the class leaves this at -1; when
// OOD is the rejector that is normal, when _background is it is a boot error.
static int8_t background_class_id() {
#ifdef TM_SIGN_USE_BACKGROUND_OOD
  /* The exporter recorded the index in the same labels.txt it generated, so
   * this is authoritative — no name scan can drift if a label is renamed. */
  return (int8_t)kBackgroundClassIndex;
#else
  static int8_t cached = -2;   // -2 = not looked up yet, -1 = no such class
  if (cached == -2) {
    cached = -1;
    for (int i = 0; i < kCategoryCount; i++) {
      const char *name = kCategoryLabels[i];
      if (name != nullptr && strcmp(name, "_background") == 0) {
        cached = (int8_t)i;
        break;
      }
    }
  }
  return cached;
#endif
}

static inline bool is_background_id(uint8_t id) {
  const int8_t bg = background_class_id();
  return bg >= 0 && (int)id == (int)bg;
}

// Human-readable label helper.  Returns the kCategoryLabels entry for a real
// class (id < kCategoryCount) and "No Sign" for the synthetic OOD class that
// we emit when the 3-layer gating believes there is no sign in frame.
//
// The "_background" index IS < kCategoryCount, so it must be tested first —
// otherwise the debug print and the SD/CSV log say "_background" instead of
// the "No Sign" every receiver already understands.
static const char* label_name(uint8_t id) {
  if (is_background_id(id)) {
    return "No Sign";
  }
  if (id < (uint8_t)kCategoryCount) {
    const char *name = kCategoryLabels[id];
    return name ? name : "?";
  }
  return "No Sign";
}

// OOD / pick helpers use fixed 32-slot score stacks — models with more
// classes than that would silently overflow them.
static_assert(kCategoryCount <= 32, "kCategoryCount exceeds scores_raw[32]");

// `confidence` is the raw [0,255] best score (the wire convention);
// `prob` is best_raw / sum(raw) — the SAME device-exact probability the host
// preview uses for a slot match (record_controller._preview_predict_sign_ocr),
// NOT a dequantised softmax.  `total_raw_out` is exposed for logging.
static void pick_label_and_confidence(TfLiteTensor *output, uint8_t *label_id,
                                      uint8_t *confidence, float *prob,
                                      int *total_raw_out) {
  uint8_t best_label = 0;
  int best_score = 0;
  int total_raw = 0;
  // Normalise over the raw uint8-scaled scores: this is the same arithmetic
  // the host preview and ood_is_in_distribution() use, so a slot's `prob`
  // means the same thing on the device, on the host, and in the log.
  for (int i = 0; i < kCategoryCount; i++) {
    int score = 0;
    if (output->type == kTfLiteInt8) {
      score = (int)output->data.int8[i] + 128;
    } else if (output->type == kTfLiteUInt8) {
      score = (int)output->data.uint8[i];
    } else if (output->type == kTfLiteFloat32) {
      score = (int)(output->data.f[i] * 255.0f);
    }
    if (score < 0) score = 0;
    if (score > 255) score = 255;
    total_raw += score;
    if (score > best_score) {
      best_score = score;
      best_label = (uint8_t)i;
    }
  }
  *label_id = best_label;
  *confidence = (uint8_t)best_score;
  if (prob) *prob = (float)best_score / (float)(total_raw > 0 ? total_raw : 1);
  if (total_raw_out) *total_raw_out = total_raw;
}

// Returns true iff the slot is "in-distribution".  Layers 2 (top-1 raw
// probability) and 3 (normalised entropy) are evaluated; Layer 1 (the
// frame-level sign_pct percentage) is DISARMED on the 3-slot path — the
// split already gates each slot on ink, and a frame-level window used to
// reject all three correct slots at once.  `out_entropy_only` reports a
// Layer-3-only failure so the caller can exempt a slot whose top class is
// the plate target.  Any decisive rule that fires → OOD = false.
static bool ood_is_in_distribution(float sign_pct, TfLiteTensor *output,
                                   float *out_max_prob, float *out_entropy_ratio,
                                   bool *out_entropy_only = nullptr) {
  if (out_entropy_only) *out_entropy_only = false;
#if OOD_ENABLE != 1
  if (out_max_prob)     *out_max_prob = 1.0f;
  if (out_entropy_ratio) *out_entropy_ratio = 0.0f;
  return true;
#else
  float max_prob_min = 0.0f, entropy_max = 0.0f;
  // Only Layers 2/3 thresholds are read; Layer 1 is disarmed below.
  ImageProviderGetOodThresholds(nullptr, nullptr, &max_prob_min, &entropy_max);

  // ── sign path: Layer 1 (sign_pct) is DISARMED ─────────────────────────
  // sign_pct is a FRAME-level ink percentage.  On the 3-slot sign path the
  // split already gates every slot on ink inside the character-band ROI;
  // a frame-level window (especially one supplied by `set ood`) is too
  // coarse and used to reject all three correct slots at once.  The value
  // stays available for diagnostics, but it never rejects here.
  (void)sign_pct;
  // With kSignBackgroundReplacesOod the model's own "_background" class is
  // the only rejector, so Layers 2/3 are opened as well; the per-slot match
  // rule (ink + not background + class + prob >= match floor) still applies.
  if (kSignBackgroundReplacesOod) {
    max_prob_min = 0.0f;
    entropy_max = 0.0f;
  }

  // Aggregate raw scores for max_prob + entropy.  Use same [0,255] int-score
  // normalisation as pick_label_and_confidence to keep them consistent.
  int raw_total = 0;
  int raw_best = 0;
  int raw_arr[32];
  for (int i = 0; i < kCategoryCount; i++) {
    int v = 0;
    if (output->type == kTfLiteInt8) v = (int)output->data.int8[i] + 128;
    else if (output->type == kTfLiteUInt8) v = (int)output->data.uint8[i];
    else if (output->type == kTfLiteFloat32) v = (int)(output->data.f[i] * 255.0f);
    if (v < 0) v = 0;
    if (v > 255) v = 255;
    raw_arr[i] = v;
    raw_total += v;
    if (v > raw_best) raw_best = v;
  }
  if (raw_total <= 0) raw_total = 1;

  float max_prob = (float)raw_best / (float)raw_total;
  if (out_max_prob) *out_max_prob = max_prob;

  // Layer 2: max_prob minimum confidence
  if (max_prob_min > 0.0f && max_prob < max_prob_min) {
    if (out_entropy_ratio) *out_entropy_ratio = 1.0f;
    return false;
  }

  // Layer 3: normalised entropy (0..1).  Use natural log; normalise by ln(N).
  float entropy = 0.0f;
  for (int i = 0; i < kCategoryCount; i++) {
    float p = (float)raw_arr[i] / (float)raw_total;
    if (p <= 1e-6f) continue;
    entropy -= p * logf(p);
  }
  float max_entropy = (kCategoryCount > 1) ? logf((float)kCategoryCount) : 1.0f;
  if (max_entropy < 1e-6f) max_entropy = 1.0f;
  float ent_ratio = entropy / max_entropy;
  if (ent_ratio > 1.0f) ent_ratio = 1.0f;
  if (out_entropy_ratio) *out_entropy_ratio = ent_ratio;

  if (entropy_max > 0.0f && ent_ratio > entropy_max) {
    if (out_entropy_only) *out_entropy_only = true;
    return false;
  }
  return true;
#endif  // OOD_ENABLE == 1
}

static uint32_t s_inference_task_stack_bytes = 0;  // set by setup() fallback so banner below

// ── Per-mode timing accumulators (INTERNAL SRAM, 8-byte aligned) ────────
// The core dump "cap=312ms → 312023ms" is a classic 32-bit half-store tear
// of a uint64_t that lived in an unaligned / PSRAM-backed location.  Putting
// these counters in file-scope .bss keeps them firmly in internal SRAM and
// the aligned(8) guarantees RISC-V LWU/SWU pairs never straddle a cacheline
// boundary on the P4's split LSU.
static uint64_t s_total_capture_us __attribute__((aligned(8))) = 0;
static uint64_t s_total_invoke_us  __attribute__((aligned(8))) = 0;
static uint64_t s_total_loop_us    __attribute__((aligned(8))) = 0;
static uint32_t s_timed_frames                          = 0;

// Hard ceiling (~1000 seconds) for any accumulator.  If the math ever
// exceeds this (store-tear / wrap / PSRAM-cache garbage) we clamp BEFORE
// dividing for the average, so the "frame=N … cap=312023ms" line never
// gets to the host again with a physically impossible value.
static constexpr uint64_t kTimingUsMax = uint64_t(1000000000ULL);
static inline uint64_t clamp_timing(uint64_t v) {
  if (v > kTimingUsMax) v = kTimingUsMax;
  return v;
}
static inline void reset_accumulators() {
  s_total_capture_us = 0;
  s_total_invoke_us  = 0;
  s_total_loop_us    = 0;
  s_timed_frames     = 0;
}

// ── Sign glyph front-end plumbing ───────────────────────────────────────
// The three model inputs are 28x28 glyph canvases built from the RAW library
// gray frame — the same BT.601 buffer the IMX219_Grayscale_Serial stream
// records for training.  GetImage() must NOT be used on this path: it
// produces the sign pipeline's cropped/stretched 96x96 image (a different
// distribution) and it writes IMG_SIZE² = 9216 bytes into a 784-byte tensor,
// corrupting the arena.
//
// The split is preprocess_sign_sequence_c(): frame-level Otsu, projection
// segmentation, then exactly SIGN_SEQUENCE_LEN boxes (merge/split/equal-thirds
// reconciliation) and one normalised glyph per box.  Reading order is applied
// by reversing the box list; pixels are never mirrored.
//
// RESOLUTION: the camera captures at g_frame_side (160 by default in a new
// export) but the split runs at g_preprocess_side (96) — the host preview
// box-averages the same 160→96, and the split's absolute-pixel heuristics
// (border strip, min run) were tuned at that resolution.  The downscale is
// the exact integer box average in frame_resize.h.
static SignScratch s_sign_scratch;                 // views into s_sign_scratch_block
static uint8_t   *s_sign_scratch_block = nullptr;
static uint8_t    s_preprocess_gray[(size_t)g_preprocess_side * (size_t)g_preprocess_side];
static uint8_t    s_slot_glyphs[SIGN_SEQUENCE_LEN][SIGN_GLYPH_SIZE * SIGN_GLYPH_SIZE];
static SignSlotInfo s_slot_info[SIGN_SEQUENCE_LEN];
static SignGlyphInfo s_sign_info;
static uint32_t   s_sign_frames = 0;
static uint32_t   s_sign_empty = 0;   // frames where the split found no ink at all
// OOD Layer 1's input comes from sign_glyph.c's interior-ROI pass
// (s_sign_info.sign_pct): the share of the tight character-band ROI that is
// ink, after the same border/speck/frame cleanup the host sequence uses.

// One allocation at boot.  sign_scratch_bytes() is a pure function of the
// geometry and a failure here is a build/config fact, not a transient
// condition — so setup() halts rather than limping along producing garbage
// glyphs from a half-initialised scratch.  Sized for the PREPROCESS side,
// which is what the front-end actually reads.
static bool sign_scratch_alloc() {
  const size_t need = sign_scratch_bytes(g_preprocess_side, g_preprocess_side, SIGN_GLYPH_SIZE);
  const char *where = "internal SRAM";
  s_sign_scratch_block = (uint8_t *)heap_caps_malloc(need, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  if (s_sign_scratch_block == nullptr) {
    s_sign_scratch_block = (uint8_t *)heap_caps_malloc(need, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    where = "PSRAM (internal SRAM exhausted)";
  }
  if (s_sign_scratch_block == nullptr) {
    Serial.printf("FATAL: sign scratch %u bytes: out of memory.\r\n", (unsigned)need);
    return false;
  }
  if (!sign_scratch_init(&s_sign_scratch, s_sign_scratch_block, need,
                         g_preprocess_side, g_preprocess_side, SIGN_GLYPH_SIZE)) {
    Serial.printf("FATAL: sign scratch %u bytes is too small for %dx%d -> %dx%d.\r\n",
                  (unsigned)need, g_preprocess_side, g_preprocess_side,
                  SIGN_SEQUENCE_LEN, SIGN_GLYPH_SIZE);
    return false;
  }
  Serial.printf("Sign scratch: %u bytes in %s (%dx%d gray -> %d slots of %d)\r\n",
                (unsigned)need, where, g_preprocess_side, g_preprocess_side,
                SIGN_SEQUENCE_LEN, SIGN_GLYPH_SIZE);
  return true;
}

// Capture one fresh frame and run the integer-exact sign front-end.  Returns
// false when no frame arrived (the caller retries) or the geometry was
// rejected; on success every slot is fully written — a no-ink frame yields
// three all-zero glyphs, which are real inputs the model was trained to
// answer "_background" for, not an error.
static bool sign_feed_slots() {
  if (!ImageProviderEnsureCamera()) {
    return false;
  }
  bool fresh = false;
  for (int tries = 0; tries < 100 && !fresh; tries++) {
    fresh = CameraUpdate();
    if (!fresh) delayMicroseconds(2000);
  }
  if (!fresh) {
    return false;
  }
  const uint8_t *gray = CameraGetGrayImgSized();   // library gray, g_frame_side²
  if (gray == nullptr) {
    return false;
  }
  const uint8_t *frame = gray;
  int frame_side = g_frame_side;
  if (g_frame_side != g_preprocess_side) {
    // Capture 160×160 -> front-end 96×96; same box average as the host.
    FrameResizeBoxAverageGray(gray, g_frame_side,
                              s_preprocess_gray, g_preprocess_side);
    frame = s_preprocess_gray;
    frame_side = g_preprocess_side;
  }

  const bool ok = preprocess_sign_sequence_c(
      frame, frame_side, frame_side, SIGN_GLYPH_SIZE,
      SIGN_SEQUENCE_LEN,
      kSignReadingOrder,
      kSignPolarity,      /* captures: dark ink on a bright sign    */
      kSignSegment,
      kSignMinRun,        /* = 2, matches Python MIN_GLYPH_RUN      */
      kSignPinnedThresh,  /* -1 = exact Otsu (0 is a PINNED 0)      */
      kSignDarkThr,       /* 0 = no luminance floor                 */
      kSignConvention,
      &s_sign_scratch, &s_slot_glyphs[0][0], s_slot_info, &s_sign_info);
  if (!ok) {
    // Only reachable for an invalid geometry, which the static_asserts above
    // rule out at compile time.  Warn once instead of spinning silently.
    static bool warned = false;
    if (!warned) {
      warned = true;
      Serial.println("ERROR: preprocess_sign_sequence_c rejected the frame geometry");
    }
    return false;
  }
  s_sign_frames++;
  if (s_sign_info.empty) s_sign_empty++;
  return true;
}

static void inference_task(void *arg) {
  (void)arg;
  uint32_t stack_sz = s_inference_task_stack_bytes != 0
      ? s_inference_task_stack_bytes
      : (uint32_t)kInferenceTaskStackBytes;
  Serial.printf("[inference_task] START on core=%d prio=%d stack=%u op_mode=%d (%s) PREPROCESS_MODE=%d\r\n",
                xPortGetCoreID(), (int)uxTaskPriorityGet(nullptr),
                (unsigned)stack_sz,
                (int)s_op_mode, op_mode_name(s_op_mode), (int)PREPROCESS_MODE);
  Serial.flush();
  uint16_t frame_id = 0;
  uint32_t sd_dropped = 0;
  static OpMode s_last_mode = OpMode::kInference;


  // Per-mode one-time state so we can re-initialise when the user flips
  // between INFERENCE and a CAPTURE mode, or across CAPTURE_A ↔ CAPTURE_B.
  static bool s_caprgb_ready   = false;
  static bool s_caprgb_banner  = false;
  static bool s_capgray_banner = false;
  static bool s_infergray_banner = false;
  static bool s_extrgb_ready   = false;
  static bool s_extrgb_banner  = false;
  static bool s_extgray_banner = false;

  for (;;) {
    // Snapshot runtime mode once per iteration so behaviour stays consistent
    // even if debug_serial_task writes to the volatile between checks.
    const OpMode mode = s_op_mode;
    if (mode != s_last_mode) {
      // Suppress the transition banner when the NEW mode streams binary
      // frames on Serial — the text would be injected at the stream start.
      if (!serial_stream_clean()) {
        Serial.printf("MODE: %s → %s  [frame=%u]\r\n",
                      op_mode_name(s_last_mode), op_mode_name(mode), (unsigned)frame_id);
        Serial.flush();
      }
      // On any mode transition: reset "consecutive" timing so the first
      // batch of CAPTURE frames doesn't skew averages wrong.
      reset_accumulators();
      // Reset per-mode ready/banner flags so the user can bounce between
      // capture modes (e.g. mode rgb → mode infer → mode rgb) and the
      // camera re-initialises + banner prints correctly every time.
      s_caprgb_ready   = false;
      s_caprgb_banner  = false;
      s_capgray_banner = false;
      s_infergray_banner = false;
      s_extrgb_ready   = false;
      s_extrgb_banner  = false;
      s_extgray_banner = false;
      s_last_mode = mode;
    }

    uint32_t now_ms = millis();

    // ─────────────────────────────────────────────────────────
    // Plain mode kCaptureRgb — AItraining default capture panel format.
    // Wire: AA 55 AA  +  96×96×3 RGB24  (no metadata, no checksum)
    // This matches AItraining's SerialFrameReader (default sync=AA 55 AA,
    // channels=3) exactly — so data capture panel → source capture →
    // training set works without app changes.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kCaptureRgb) {
      if (!s_caprgb_ready) {
        // Begin-once guard: the library begin() is NOT re-entrant — a
        // second call on a runtime mode switch can hang the I2C bus.
        s_caprgb_ready = ImageProviderEnsureCamera();
        if (!s_caprgb_ready) {
          Serial.println("CAPTURE_RGB/plain: CameraBegin failed");
          Serial.flush();
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
        }
      }
      if (!s_caprgb_banner) {
        Serial.printf("CAPTURE_RGB/plain: stream sync=AA 55 AA + %dx%dx3 (AItraining)\r\n",
                      g_frame_side, g_frame_side);
        Serial.flush();
        s_caprgb_banner = true;
      }
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;

      // The library output is already g_frame_side-sized (unified API,
      // ImageProviderConfigureCamera in setup) — stream it directly.
      const uint8_t *rgb = CameraGetRgb();
      const size_t bytes = CameraGetRgbSize();

      static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
      Serial.write(sync, 3);
      Serial.write(rgb, bytes);
      Serial.flush();

      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Plain mode kCaptureGray — AItraining grayscale capture default.
    // Wire: AA 55 AA  +  g_frame_side×g_frame_side×1 GRAY8
    // RAW library gray (BT.601, uncropped, no contrast stretch) — the same
    // source the data-collection stream (training data) uses, so the host
    // runs the full crop+stretch pipeline exactly like training.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kCaptureGray) {
      if (!s_capgray_banner) {
        Serial.printf("CAPTURE_GRAY/plain: stream sync=AA 55 AA + %dx%dx1 GRAY8 (raw library gray, uncropped)\r\n",
                      g_frame_side, g_frame_side);
        Serial.flush();
        s_capgray_banner = true;
      }

      // Capture a FRESH frame — the library buffers only update on
      // esp32_p4_<cam>_update(), and reading without it re-sends the last
      // frame forever (frozen stream).
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;

      const uint8_t *gray_raw = CameraGetGrayImgSized();
      if (gray_raw == nullptr) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

      static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
      Serial.write(sync, 3);
      Serial.write(gray_raw, (size_t)g_frame_side * g_frame_side);
      Serial.flush();
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Extended mode kExtCaptureRgb — for scripting tools, with metadata + checksum.
    // Wire: AA 55 AB  kind(1)  fid_LE(2)  w_LE(2)  h_LE(2)  <RGB bytes>  xor8
    // kind=0x01=RGB24; xor8 covers the header + pixel bytes (checksum byte excluded).
    // Not needed for the default AItraining workflow.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kExtCaptureRgb) {
      if (!s_extrgb_ready) {
        // Begin-once guard: see kCaptureRgb above.
        s_extrgb_ready = ImageProviderEnsureCamera();
        if (!s_extrgb_ready) {
          Serial.println("CAPTURE_RGB/extended: CameraBegin failed");
          Serial.flush();
          vTaskDelay(pdMS_TO_TICKS(500));
          continue;
        }
      }
      if (!s_extrgb_banner) {
        Serial.printf("CAPTURE_RGB/extended: stream sync=AA 55 AB + kind/fid/w/h + pixels + xor8\r\n");
        Serial.flush();
        s_extrgb_banner = true;
      }
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;
      const uint8_t *rgb = CameraGetRgb();  // g_frame_side-sized library output
      const size_t bytes = CameraGetRgbSize();
      uint8_t hdr[3 + 1 + 2 + 2 + 2];
      hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapExtSync2;
      hdr[3] = kCapKindRgb;
      write_u16_le(hdr + 4, frame_id);
      write_u16_le(hdr + 6, (uint16_t)g_frame_side);
      write_u16_le(hdr + 8, (uint16_t)g_frame_side);
      uint8_t x = debug_xor(hdr, sizeof(hdr));
      Serial.write(hdr, sizeof(hdr));
      for (size_t i = 0; i < bytes; i++) x ^= rgb[i];
      Serial.write(rgb, bytes);
      Serial.write(x);
      Serial.flush();
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // Extended mode kExtCaptureGray — same as above, raw GRAY8 + metadata.
    // ─────────────────────────────────────────────────────────
    if (mode == OpMode::kExtCaptureGray) {
      if (!s_extgray_banner) {
        Serial.printf("CAPTURE_GRAY/extended: stream sync=AA 55 AB + kind/fid/w/h + GRAY8 (raw) + xor8\r\n");
        Serial.flush();
        s_extgray_banner = true;
      }
      // Capture a FRESH frame (see kCaptureGray — the library buffers
      // only update on CameraUpdate()).
      bool got_frame = false;
      for (int tries = 0; tries < 100 && !got_frame; tries++) {
        if (CameraUpdate()) got_frame = true;
        else delayMicroseconds(2000);
      }
      if (!got_frame) { vTaskDelay(pdMS_TO_TICKS(2)); continue; }
      frame_id++;

      const uint8_t *gray_raw = CameraGetGrayImgSized();
      if (gray_raw == nullptr) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }
      uint8_t hdr[3 + 1 + 2 + 2 + 2];
      hdr[0] = kCapSync0;  hdr[1] = kCapSync1;  hdr[2] = kCapExtSync2;
      hdr[3] = kCapKindGray;
      write_u16_le(hdr + 4, frame_id);
      write_u16_le(hdr + 6, (uint16_t)g_frame_side);
      write_u16_le(hdr + 8, (uint16_t)g_frame_side);
      uint8_t x = debug_xor(hdr, sizeof(hdr));
      Serial.write(hdr, sizeof(hdr));
      const size_t bytes = (size_t)g_frame_side * g_frame_side;
      for (size_t i = 0; i < bytes; i++) x ^= gray_raw[i];
      Serial.write(gray_raw, bytes);
      Serial.write(x);
      Serial.flush();
      taskYIELD();
      continue;
    }

    // ─────────────────────────────────────────────────────────
    // kInference mode (default).
    // ─────────────────────────────────────────────────────────
    // Both timers are stamped BEFORE the mode switch but consumed AFTER its
    // #endif — t_capture_us and s_total_loop_us below are common code — so
    // they must be declared out here.  Declaring them inside the #else branch
    // left PREPROCESS_MODE == PREPROCESS_MODE_RGB referring to two identifiers
    // that do not exist, and the RGB data-collection build did not compile.
    uint64_t t_loop_start = esp_timer_get_time();

    uint64_t t_capture_start = esp_timer_get_time();
#if PREPROCESS_MODE == PREPROCESS_MODE_RGB
    // ── RGB data-collection mode (compile-time switch) ──
    static bool s_stream_ready = false;
    if (!s_stream_ready) {
      s_stream_ready = CameraBegin();
      if (!s_stream_ready) {
        Serial.println("CameraBegin failed");
        vTaskDelay(pdMS_TO_TICKS(1000));
        continue;
      }
      Serial.printf("RGB stream mode: 0xAA 0x55 0xAA + %dx%dx3\n", g_frame_side, g_frame_side);
    }
    for (int tries = 0; tries < 100; tries++) {
      if (CameraUpdate()) {
        CameraSendRgbToSerialWb(200, 200);  // fixed ×2.0 WB, matches AItraining
        break;
      }
      delayMicroseconds(2000);
    }
    taskYIELD();
    continue;
#else
    if (!input || !interpreter) {
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }

    // Sign path: raw library gray -> 3-slot split -> three glyph canvases.
    // Each canvas is quantised (`glyph - 128`) into the int8 input tensor
    // immediately before its own Invoke.  Note this never calls GetImage(),
    // which is why ImageProviderLastSignPct() stays 0.0f here and why OOD
    // Layer 1 is disarmed in ood_is_in_distribution() (see the banner).
    if (!sign_feed_slots()) {
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }
#endif  // PREPROCESS_MODE == RGB
    uint64_t t_capture_us = esp_timer_get_time() - t_capture_start;
    frame_id++;

    // ── kInferGray: stream the RAW library gray ─────────────────────────
    // Unchanged by the split: this is the same source the
    // IMX219_Grayscale_Serial data-collection stream (training data) uses —
    // uncropped, BT.601, no contrast stretch, library orientation.  It does
    // not touch the model input tensor, so it can run before the per-slot
    // Invokes.
    if (mode == OpMode::kInferGray) {
      if (!s_infergray_banner) {
        Serial.printf("INFER_GRAY: stream sync=AA 55 AA + %dx%dx1 GRAY8 (raw library gray, uncropped) — UART→S3 still active\r\n",
                      g_frame_side, g_frame_side);
        Serial.flush();
        s_infergray_banner = true;
      }
      const uint8_t *gray_raw = CameraGetGrayImgSized();
      if (gray_raw != nullptr) {
        static const uint8_t sync[3] = { kCapSync0, kCapSync1, kCapSync2 };  // AA 55 AA
        Serial.write(sync, 3);
        Serial.write(gray_raw, (size_t)g_frame_side * (size_t)g_frame_side);
        Serial.flush();
      }
    }

    // ── 3 x Invoke: one per sign slot ───────────────────────────────────
    // The arena planner may reuse the input tensor's memory for intermediate
    // tensors during Invoke, so each slot's glyph is copied in immediately
    // before ITS Invoke, and the SD snapshot is taken at that same moment.
    // CRITICAL: input->data.int8 is only guaranteed valid until Invoke is
    // called — reading it afterwards produced deterministic moiré garbage
    // (conv activation scratch data).
    uint8_t slot_label[SIGN_SEQUENCE_LEN] = {0};
    uint8_t slot_conf[SIGN_SEQUENCE_LEN] = {0};
    float   slot_prob[SIGN_SEQUENCE_LEN] = {0.0f};
    bool    slot_accept[SIGN_SEQUENCE_LEN] = {false};
    bool    slot_match[SIGN_SEQUENCE_LEN] = {false};
    int     slot_best_min = 255;   // weakest slot's best raw score → wire confidence
    int     slots_matched = 0;
    bool    invoked = true;

    uint64_t t_invoke_start = esp_timer_get_time();
    for (int slot = 0; slot < SIGN_SEQUENCE_LEN; slot++) {
      sign_glyph_to_int8(s_slot_glyphs[slot],
                         (size_t)SIGN_GLYPH_SIZE * (size_t)SIGN_GLYPH_SIZE,
                         input->data.int8);
      if (slot == 0) {
        // Snapshot slot 0 before its Invoke — that is the glyph the SD logger
        // writes.  The copy length is the ACTUAL tensor size (input->bytes,
        // latched at boot), never sizeof(s_input_snapshot): against a 784-byte
        // tensor a sizeof() copy reads 8432 bytes past it and logs someone
        // else's arena.
        memcpy(s_input_snapshot, input->data.int8, s_input_bytes);
      }

      if (kTfLiteOk != interpreter->Invoke()) {
        invoked = false;
        break;
      }

      TfLiteTensor *output = interpreter->output(0);
      float prob = 0.0f;
      int total_raw = 0;
      pick_label_and_confidence(output, &slot_label[slot], &slot_conf[slot],
                                &prob, &total_raw);
      slot_prob[slot] = prob;
      if ((int)slot_conf[slot] < slot_best_min) slot_best_min = (int)slot_conf[slot];

      // OOD / "_background" rejection, per slot.  s_sign_info.sign_pct is
      // the ink share of the interior ROI measured by sign_glyph.c on this
      // frame; with kSignBackgroundReplacesOod the helper opens all
      // three gates and the model's background class decides, otherwise
      // Layers 1-3 compose with the match rule.
      float max_prob = 0.0f, entropy_ratio = 1.0f;
      bool entropy_only = false;
      const bool in_distribution =
          ood_is_in_distribution(s_sign_info.sign_pct, output, &max_prob,
                                 &entropy_ratio, &entropy_only);
      const bool is_background = is_background_id(slot_label[slot]);

      // A slot is READABLE independent of which plate we compare it to:
      // ink, a real class, OOD in-distribution and the confidence floor.
      // The plate loop below decides which target character it must equal.
      // An entropy-only rejection is NOT decisive: if this slot's top class
      // turns out to be an allowed plate's target character (and prob
      // cleared the floor), the plate loop accepts it.  A correct reading
      // used to be reported as NO because the model's class distribution
      // can push entropy_ratio above 0.70 even when every top class is
      // right.  sign_pct / max_prob still reject unconditionally.
      slot_accept[slot] = s_slot_info[slot].ok &&
                          (in_distribution || entropy_only) &&
                          !is_background &&
                          prob >= s_sign_match_min_prob;
      (void)max_prob;
      (void)entropy_ratio;
      (void)total_raw;
    }

    if (!invoked) {
      Serial.println("Invoke failed");
      vTaskDelay(pdMS_TO_TICKS(1));
      continue;
    }

    // ── compare the three readings against EVERY allowed plate ──────────
    // Exact 3-for-3 wins immediately.  If none is exact, keep the closest
    // plate (most slots hit) for the diagnostic target chars; the verdict is
    // still NO.  Old exports have exactly one plate via the macros.
    int matched_plate = -1;
    int chosen_plate = 0;
    int best_hits = -1;
    for (int p = 0; p < SIGN_PLATE_COUNT; p++) {
      int hits = 0;
      for (int s = 0; s < SIGN_SEQUENCE_LEN; s++) {
        const int t = SIGN_PLATE_CLASS(p, s);
        if (slot_accept[s] && t >= 0 && (int)slot_label[s] == t) hits++;
      }
      if (hits == SIGN_SEQUENCE_LEN) {
        matched_plate = p;
        chosen_plate = p;
        best_hits = hits;
        break;
      }
      if (hits > best_hits) {
        best_hits = hits;
        chosen_plate = p;
      }
    }
    for (int s = 0; s < SIGN_SEQUENCE_LEN; s++) {
      const int t = SIGN_PLATE_CLASS(chosen_plate, s);
      slot_match[s] = slot_accept[s] && t >= 0 &&
                      (int)slot_label[s] == t;
      if (slot_match[s]) slots_matched++;
    }
    const bool all_match = (matched_plate >= 0 && slots_matched == SIGN_SEQUENCE_LEN);
    uint64_t t_invoke_us = esp_timer_get_time() - t_invoke_start;

    // Accumulate timing (clamp in case of store-tear / wrap / cache garbage)
    s_total_capture_us = clamp_timing(s_total_capture_us + t_capture_us);
    s_total_invoke_us  = clamp_timing(s_total_invoke_us  + t_invoke_us);
    s_total_loop_us    = clamp_timing(s_total_loop_us    + (esp_timer_get_time() - t_loop_start));
    s_timed_frames++;

    // ── Binary verdict ──────────────────────────────────────────────────
    // Three outcomes, encoded in the flags high nibble (see the constants at
    // the top of the file):
    //   NO SIGN   no ink anywhere in the frame
    //   MISMATCH  a sign is present, but at least one slot did not match
    //   MATCH     all three slots matched ONE allowed plate
    // A MATCH therefore requires the ink check, the per-slot class check, the
    // "_background" rejection AND the confidence floor; one slot failing any
    // of them is a NO.
    const bool no_sign = s_sign_info.empty;

    uint8_t pkt_flags = kFlagsSignReady;
    if (no_sign) {
      pkt_flags = (uint8_t)((pkt_flags & 0x0F) | kFlagsVerdictNoSign);
    } else if (!all_match) {
      pkt_flags = (uint8_t)((pkt_flags & 0x0F) | kFlagsVerdictMismatch);
    } else {
      pkt_flags = (uint8_t)((pkt_flags & 0x0F) | kFlagsVerdictMatch);
    }

    // label_id carries the verdict detail the S3 needs to know WHICH car
    // matched: the matched plate index on MATCH, 0xFE on MISMATCH, 0xFF on
    // NO SIGN.  confidence is the WEAKEST slot's best raw score so a
    // receiver can gate on the same 0..255 scale as before (S3 uses 120).
    uint8_t label_id = kSignNoSignLabel;
    if (!no_sign) {
      label_id = all_match ? (uint8_t)matched_plate : kSignMismatchLabel;
    }
    const uint8_t confidence = (uint8_t)(slot_best_min < 0 ? 0
                                         : (slot_best_min > 255 ? 255 : slot_best_min));

    if (mode == OpMode::kInference) {
      Serial.print("SIGN: ");
      if (no_sign) {
        Serial.print("NO SIGN (frame has no ink)");
      } else if (all_match) {
        Serial.printf("MATCH (YES) plate#%d=%s", matched_plate + 1,
                      SIGN_PLATE_TEXT(matched_plate));
      } else {
        Serial.print("NO MATCH (NO)");
      }
      for (int slot = 0; slot < SIGN_SEQUENCE_LEN; slot++) {
        Serial.printf("  [%d] '%c'->%s %.2f%s",
                      slot, SIGN_PLATE_TEXT(chosen_plate)[slot],
                      label_name(slot_label[slot]),
                      (double)slot_prob[slot],
                      slot_match[slot] ? "" : " (FAIL)");
      }
      Serial.println();
    }

    // First frames (and every 30th) carry the split diagnostics — a
    // num_boxes != num_slots trend, or a rising sign_empty count, means the
    // frame geometry / polarity is wrong, not that the model is bad.
    if (mode == OpMode::kInference && (frame_id <= 10 || (frame_id % 30) == 0)) {
      Serial.printf("  slots: boxes=%d/%d ink=%d match=%d/%d conf_min=%u flags=0x%02X\r\n",
                    (int)s_sign_info.num_boxes, (int)s_sign_info.num_slots,
                    no_sign ? 0 : 1, slots_matched, (int)SIGN_SEQUENCE_LEN,
                    (unsigned)confidence, (unsigned)pkt_flags);
    }

    if (mode == OpMode::kInference || mode == OpMode::kInferGray) {
      if (kEnableSdLogger && s_sd_queue && !s_sd_full && (frame_id % kSaveEveryNFrames) == 0) {
        // Take a FREE buffer first (see s_sd_free_queue comment) — never
        // overwrite a buffer the sd task may still be writing.
        uint8_t buf_idx = 0;
        if (!s_sd_free_queue || xQueueReceive(s_sd_free_queue, &buf_idx, 0) != pdTRUE) {
          sd_dropped++;
        } else {
          uint8_t *dst = s_sd_buffers[buf_idx];
          // Copy from the pre-Invoke snapshot — input->data.int8 is invalid
          // after Invoke (arena reuse), it would save moiré scratch data.
          // Length = the model input's byte size (784 for a sign slot,
          // 9216 for the legacy sign model); s_sd_buffers are sized for the
          // larger.
          for (size_t i = 0; i < s_input_bytes; i++) {
            dst[i] = (uint8_t)((int)s_input_snapshot[i] + 128);
          }

          SdPacket sp;
          sp.frame_id = frame_id;
          sp.buffer_index = buf_idx;
          sp.flags = pkt_flags;
          sp.label_id = label_id;
          sp.confidence = confidence;
          if (xQueueSend(s_sd_queue, &sp, 0) != pdTRUE) {
            // sd task lagging behind — give the buffer back immediately.
            xQueueSend(s_sd_free_queue, &buf_idx, 0);
            sd_dropped++;
          }
        }
      }

      if (s_uart_queue) {
        UartPacket pkt;
        pkt.frame_id = frame_id;
        pkt.label_id = label_id;
        pkt.confidence = confidence;
        pkt.flags = pkt_flags;
        xQueueOverwrite(s_uart_queue, &pkt);
      }
    }

    // NOTE: kInferGray streaming moved BEFORE interpreter->Invoke() — the
    // arena planner may reuse the input tensor's memory for intermediate
    // tensors during Invoke, so reading input->data.int8 afterwards yields
    // structured garbage (moiré).  See the pre-Invoke block above.

    if (mode == OpMode::kInference && (frame_id % 1) == 0 && s_timed_frames > 0) {
      uint64_t cap = clamp_timing(s_total_capture_us / s_timed_frames);
      uint64_t inv = clamp_timing(s_total_invoke_us  / s_timed_frames);
      uint64_t lop = clamp_timing(s_total_loop_us    / s_timed_frames);
      uint32_t avg_cap  = (uint32_t)cap;
      uint32_t avg_inv  = (uint32_t)inv;
      uint32_t avg_loop = (uint32_t)lop;
      Serial.print("frame=");
      Serial.print(frame_id);
      Serial.print(" verdict=");
      // Same three-way as the per-frame SIGN line, so the log never
      // contradicts the packet this very frame just queued.
      if (no_sign)        Serial.print("NO_SIGN");
      else if (!all_match) Serial.print("NO_MATCH");
      else                 Serial.print("MATCH");
      Serial.print(" conf=");
      Serial.print(confidence);
      Serial.print(" slots=");
      Serial.print(slots_matched);
      Serial.print("/");
      Serial.print(SIGN_SEQUENCE_LEN);
      Serial.print(" flags=0x");
      Serial.print(pkt_flags, 16);
      Serial.print(" | cap=");
      Serial.print(avg_cap / 1000);
      Serial.print("ms inv=");
      Serial.print(avg_inv / 1000);
      Serial.print("ms loop=");
      Serial.print(avg_loop / 1000);
      Serial.print("ms fps≈");
      Serial.print(avg_loop > 0 ? 1000000UL / avg_loop : 0);
      Serial.print(" tx=");
      Serial.print(s_transmit_enabled ? "ON" : "OFF");
      Serial.print(" rx_bytes=");
      Serial.print((unsigned long)s_rx_bytes);
      // How often the split found no ink at all — a steadily rising count
      // means the polarity or the frame geometry is wrong, not that the
      // model is bad.
      Serial.print(" slot_empty=");
      Serial.print(s_sign_empty);
      Serial.print("/");
      Serial.print(s_sign_frames);
      if (kEnableSdLogger) {
        Serial.print(" sd_drop=");
        Serial.print(sd_dropped);
      }
      Serial.println();
      reset_accumulators();
    }

    taskYIELD();
  }
}



// The name of this function is important for Arduino compatibility.
// Names for the operator codes printed before AllocateTensors().  A missing
// resolver kernel is otherwise reported as "AllocateTensors() failed" on
// UART0, which TinyUSB/CDC users never see.
static const char *builtin_op_name(tflite::BuiltinOperator code) {
  switch (code) {
    case tflite::BuiltinOperator_ADD: return "ADD";
    case tflite::BuiltinOperator_AVERAGE_POOL_2D: return "AVERAGE_POOL_2D";
    case tflite::BuiltinOperator_CONCATENATION: return "CONCATENATION";
    case tflite::BuiltinOperator_CONV_2D: return "CONV_2D";
    case tflite::BuiltinOperator_DEPTHWISE_CONV_2D: return "DEPTHWISE_CONV_2D";
    case tflite::BuiltinOperator_DEQUANTIZE: return "DEQUANTIZE";
    case tflite::BuiltinOperator_FULLY_CONNECTED: return "FULLY_CONNECTED";
    case tflite::BuiltinOperator_LEAKY_RELU: return "LEAKY_RELU";
    case tflite::BuiltinOperator_LOGISTIC: return "LOGISTIC";
    case tflite::BuiltinOperator_MAXIMUM: return "MAXIMUM";
    case tflite::BuiltinOperator_MAX_POOL_2D: return "MAX_POOL_2D";
    case tflite::BuiltinOperator_MEAN: return "MEAN";
    case tflite::BuiltinOperator_MINIMUM: return "MINIMUM";
    case tflite::BuiltinOperator_MUL: return "MUL";
    case tflite::BuiltinOperator_PACK: return "PACK";
    case tflite::BuiltinOperator_PAD: return "PAD";
    case tflite::BuiltinOperator_QUANTIZE: return "QUANTIZE";
    case tflite::BuiltinOperator_REDUCE_MAX: return "REDUCE_MAX";
    case tflite::BuiltinOperator_RELU: return "RELU";
    case tflite::BuiltinOperator_RELU6: return "RELU6";
    case tflite::BuiltinOperator_RESHAPE: return "RESHAPE";
    case tflite::BuiltinOperator_RESIZE_BILINEAR: return "RESIZE_BILINEAR";
    case tflite::BuiltinOperator_RSQRT: return "RSQRT";
    case tflite::BuiltinOperator_SHAPE: return "SHAPE";
    case tflite::BuiltinOperator_SOFTMAX: return "SOFTMAX";
    case tflite::BuiltinOperator_STRIDED_SLICE: return "STRIDED_SLICE";
    case tflite::BuiltinOperator_SUB: return "SUB";
    case tflite::BuiltinOperator_TANH: return "TANH";
    case tflite::BuiltinOperator_TRANSPOSE: return "TRANSPOSE";
    case tflite::BuiltinOperator_CUSTOM: return "CUSTOM";
    default: return "UNKNOWN";
  }
}

static void print_model_ops(const tflite::Model *m) {
  if (m == nullptr || m->operator_codes() == nullptr) {
    Serial.println("Model operator codes: (none)");
    return;
  }
  const auto *codes = m->operator_codes();
  Serial.printf("Model operator codes (%u):", (unsigned)codes->size());
  for (unsigned i = 0; i < codes->size(); i++) {
    const auto *oc = codes->Get(i);
    if (oc == nullptr) continue;
    if (oc->builtin_code() == tflite::BuiltinOperator_CUSTOM) {
      const auto *custom = oc->custom_code();
      Serial.printf(" %s", custom ? custom->c_str() : "CUSTOM(?)");
    } else {
      Serial.printf(" %s(%d)", builtin_op_name(oc->builtin_code()),
                    (int)oc->builtin_code());
    }
  }
  Serial.println();
  Serial.flush();
}

void setup() {
  // Set up logging. Google style is to avoid globals or statics because of
  // lifetime uncertainty, but since this has a trivial destructor it's okay.
  // NOLINTNEXTLINE(runtime-global-variables)
  static tflite::MicroErrorReporter micro_error_reporter;
  error_reporter = &micro_error_reporter;

  Serial.begin(kDebugBaud);
  delay(100);
  Serial.println("P4 TFLite start");

  // USER CONFIG (top of this file): push the CAPTURE side + g_flip_180 into
  // the camera library before any capture/inference code runs.
  ImageProviderConfigureCamera(g_frame_side, g_flip_180);
  Serial.printf("Camera config: capture=%d preprocess=%d model=%d flip_180=%d\r\n",
                (int)g_frame_side, (int)g_preprocess_side, (int)kNumCols,
                (int)g_flip_180);
  if (g_frame_side != g_preprocess_side) {
    Serial.printf("Resolution: camera %dx%d -> box-average %dx%d -> glyph front-end\r\n",
                  (int)g_frame_side, (int)g_frame_side,
                  (int)g_preprocess_side, (int)g_preprocess_side);
  }
#if PREPROCESS_MODE == PREPROCESS_MODE_BG
  Serial.printf("Preprocess: B-G  | Camera: %s\n", CameraGetName());
#elif PREPROCESS_MODE == PREPROCESS_MODE_GRAY
  Serial.printf("Preprocess: GRAY (BT.601)  | Camera: %s\n", CameraGetName());
#elif PREPROCESS_MODE == PREPROCESS_MODE_RGB
  Serial.printf("Preprocess: RGB stream (data collection)  | Camera: %s\n", CameraGetName());
#endif

  // ── 3-character sign front-end ────────────────────────────────────────
  // The model input is ONE 28x28 canvas per sign slot, built by sign_glyph.c
  // from the RAW library gray frame; GetImage() is never called on this path.
  Serial.printf("Preprocess: SIGN glyph (%dx%d gray -> %d slots of %d, ink=%s, %s, min_run=%d, thresh=%s)\r\n",
                g_preprocess_side, g_preprocess_side, SIGN_SEQUENCE_LEN, SIGN_GLYPH_SIZE,
                kSignPolarity == SIGN_INK_DARK ? "dark-on-bright" : "bright-on-dark",
                kSignConvention == SIGN_NORM_CANON ? "NORM_CANON" : "NORM_STRETCH",
                kSignMinRun,
                kSignPinnedThresh < 0 ? "otsu" : "pinned");
  Serial.printf("Sign target: \"%s\"  order=%s  match_min_prob=%.2f (set ood adjusts)  window=(dark=%d, lum=%d)%s\r\n",
                kTargetSignSequence,
                kSignReadingOrder ? "RTL" : "LTR",
                (double)s_sign_match_min_prob,
                (int)kExportDarkThr, (int)kExportLumThr,
                kSignWindowAdjusted ? "  [sliders active]" : "  [defaults: exact Otsu]");
  if (g_preprocess_side != 96) {
    Serial.printf("WARNING: the sign training captures are 96x96 — preprocessing at %d "
                  "is a different source resolution than the model was trained on.\r\n",
                  g_preprocess_side);
  }
  if (kSignBackgroundReplacesOod) {
    Serial.println("OOD: bypassed — each slot rejects with the model's \"_background\" class "
                   "(match rule mirrors the host preview).");
  } else {
    Serial.println("OOD: active — Layers 1-3 (sign_pct, max_prob, entropy) compose with "
                   "the per-slot match rule; no _background class in this export.");
  }
  if (!sign_scratch_alloc()) {
    Serial.println("FATAL: sign scratch allocation failed — halting (no glyphs can be built).");
    Serial.flush();
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }

  pinMode(kHandshakePwmPin, OUTPUT);
  analogWrite(kHandshakePwmPin, kHandshakePwmDuty);

  if (kEnableSdLogger) {
    if (kUseSdMmcCustomPins) {
      if (!SD_MMC.setPins(kSdClkPin, kSdCmdPin, kSdD0Pin, kSdD1Pin, kSdD2Pin, kSdD3Pin)) {
        Serial.println("SD_MMC setPins failed");
      }
    }

    if (!mount_sd()) {
      Serial.println("SD mount failed");
    } else if (s_sd_using_ffat) {
      if (!make_run_dir()) {
        Serial.println("Create run dir failed");
      } else {
        Serial.print("Run dir: ");
        Serial.println(s_run_dir);
        s_sd_queue = xQueueCreate(kSdQueueDepth, sizeof(SdPacket));
        s_sd_free_queue = xQueueCreate(kSdQueueDepth, sizeof(uint8_t));
        for (uint8_t i = 0; i < kSdQueueDepth; i++) xQueueSend(s_sd_free_queue, &i, 0);
      }
    } else if (s_sd_using_sd_mmc) {
      if (SD_MMC.cardType() == CARD_NONE) {
        Serial.println("No SD card");
      } else if (!make_run_dir()) {
        Serial.println("Create run dir failed");
      } else {
        Serial.print("Run dir: ");
        Serial.println(s_run_dir);
        s_sd_queue = xQueueCreate(kSdQueueDepth, sizeof(SdPacket));
        s_sd_free_queue = xQueueCreate(kSdQueueDepth, sizeof(uint8_t));
        for (uint8_t i = 0; i < kSdQueueDepth; i++) xQueueSend(s_sd_free_queue, &i, 0);
      }
    } else {
      Serial.println("Storage mount state invalid");
    }
  }

  // GPIO10/GPIO11 crosstalk mitigation (per repo CLAUDE.md): pull the RX
  // pin down BEFORE starting the UART so idle/noise on the S3 line can't
  // be misread as control packets (and vice versa on the S3 side).
  pinMode(kUartRxPin, INPUT_PULLDOWN);
  UartToS3.begin(kUartBaud, SERIAL_8N1, kUartRxPin, kUartTxPin);
  s_uart_queue = xQueueCreate(1, sizeof(UartPacket));

  // Map the model into a usable data structure. This doesn't involve any
  // copying or parsing, it's a very lightweight operation.
  model = tflite::GetModel(g_tm_model_data);
  print_model_ops(model);
  if (model->version() != TFLITE_SCHEMA_VERSION) {
    SIGN_FATAL("model schema version %d != supported %d — re-export the model",
               (int)model->version(), (int)TFLITE_SCHEMA_VERSION);
  }

  #if TM_HAS_GENERATED_MODEL_RESOLVER
  static ModelOpResolver micro_op_resolver;
  static bool resolver_registered = false;
  if (!resolver_registered) {
    if (RegisterModelOps(micro_op_resolver) != kTfLiteOk) {
      SIGN_FATAL("RegisterModelOps() failed — the generated model_resolver.h "
                 "is missing an op listed above; re-export the model");
    }
    resolver_registered = true;
  }
  #else
  static tflite::MicroMutableOpResolver<32> micro_op_resolver;
  static bool resolver_registered = false;
  if (!resolver_registered) {
    // Superset of the kernels used by the image, OCR and sign exports.  Every
    // failure is a named FATAL: a missing kernel otherwise surfaces only as
    // "AllocateTensors() failed" on UART0.
    #define ADD_MICRO_OP(fn)                                                   \
      do {                                                                     \
        if (micro_op_resolver.fn() != kTfLiteOk) {                             \
          SIGN_FATAL("resolver.%s() failed — this TFLM build lacks the kernel", \
                     #fn);                                                     \
        }                                                                      \
      } while (0)
    ADD_MICRO_OP(AddAveragePool2D);
    ADD_MICRO_OP(AddMaxPool2D);
    ADD_MICRO_OP(AddConv2D);
    ADD_MICRO_OP(AddDepthwiseConv2D);
    ADD_MICRO_OP(AddShape);
    ADD_MICRO_OP(AddStridedSlice);
    ADD_MICRO_OP(AddPack);
    ADD_MICRO_OP(AddReshape);
    ADD_MICRO_OP(AddSoftmax);
    ADD_MICRO_OP(AddFullyConnected);
    ADD_MICRO_OP(AddRsqrt);
    ADD_MICRO_OP(AddReduceMax);
    ADD_MICRO_OP(AddQuantize);
    ADD_MICRO_OP(AddDequantize);
    ADD_MICRO_OP(AddAdd);
    ADD_MICRO_OP(AddMul);
    ADD_MICRO_OP(AddMean);
    ADD_MICRO_OP(AddPad);
    ADD_MICRO_OP(AddLeakyRelu);
    ADD_MICRO_OP(AddRelu);
    ADD_MICRO_OP(AddRelu6);
    ADD_MICRO_OP(AddConcatenation);
    ADD_MICRO_OP(AddSub);
    ADD_MICRO_OP(AddMinimum);
    ADD_MICRO_OP(AddMaximum);
    #undef ADD_MICRO_OP
    resolver_registered = true;
  }
  #endif

  // Build an interpreter to run the model with.
  // NOLINTNEXTLINE(runtime-global-variables)
  // Try internal SRAM first (fast, low latency), fall back to PSRAM.
  // Internal SRAM gives ~10x memory bandwidth over PSRAM — critical for
  // inference speed. Reduce model size if it doesn't fit

  uint32_t sram_free   = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint32_t sram_large  = (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  uint32_t spiram_free = (uint32_t)heap_caps_get_free_size(MALLOC_CAP_SPIRAM);
  uint32_t spiram_large= (uint32_t)heap_caps_get_largest_free_block(MALLOC_CAP_SPIRAM);
  Serial.printf("SRAM: free=%u largest=%u | PSRAM: free=%u largest=%u | arena_req=%u\r\n",
                (unsigned)sram_free, (unsigned)sram_large,
                (unsigned)spiram_free, (unsigned)spiram_large,
                (unsigned)kTensorArenaSize);

  auto make_interpreter = [&](uint8_t *arena, size_t bytes) {
    return new (std::nothrow) tflite::MicroInterpreter(
        model, micro_op_resolver, arena, bytes, nullptr, nullptr, false);
  };

  size_t arena_bytes = kTensorArenaSize;
  tensor_arena = (uint8_t *)heap_caps_aligned_alloc(
      16, arena_bytes, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);
  const char *arena_location = "SRAM";

  if (tensor_arena == nullptr) {
    Serial.printf("Tensor arena: SRAM %u too small (largest=%u), trying PSRAM.\r\n",
                  (unsigned)sram_free, (unsigned)sram_large);
    tensor_arena = (uint8_t *)heap_caps_aligned_alloc(
        16, arena_bytes, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    arena_location = "PSRAM (SRAM allocation failed)";
  }
  if (tensor_arena == nullptr) {
    SIGN_FATAL("could not allocate the %u-byte tensor arena in SRAM or PSRAM",
               (unsigned)arena_bytes);
  }

  Serial.printf("Tensor arena: %u bytes in %s @ %p\r\n",
                (unsigned)arena_bytes, arena_location,
                (const void *)tensor_arena);
  tflite::MicroInterpreter *interp = make_interpreter(tensor_arena, arena_bytes);
  if (interp == nullptr) {
    SIGN_FATAL("could not construct the TFLM interpreter");
  }
  TfLiteStatus allocate_status = interp->AllocateTensors();

  // Compact models fit in the 320 KB SRAM arena.  If an older/bloated export
  // still needs more, retry once with the legacy 8 MB PSRAM arena — and say
  // loudly that this happened, because it is slow and should be re-exported.
  if (allocate_status != kTfLiteOk && arena_bytes < kTensorArenaFallbackSize) {
    Serial.printf("FATAL-ish: AllocateTensors() failed with the %u-byte %s arena "
                  "(used=%u); retrying with %u bytes in PSRAM.\r\n",
                  (unsigned)arena_bytes, arena_location,
                  (unsigned)interp->arena_used_bytes(),
                  (unsigned)kTensorArenaFallbackSize);
    delete interp;
    interp = nullptr;
    heap_caps_free(tensor_arena);
    tensor_arena = (uint8_t *)heap_caps_aligned_alloc(
        16, kTensorArenaFallbackSize, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
    arena_bytes = kTensorArenaFallbackSize;
    arena_location = "PSRAM (AllocateTensors fallback)";
    if (tensor_arena == nullptr) {
      SIGN_FATAL("AllocateTensors() needs more than %u bytes and the %u-byte "
                 "PSRAM fallback could not be allocated",
                 (unsigned)kTensorArenaSize, (unsigned)kTensorArenaFallbackSize);
    }
    Serial.printf("Tensor arena retry: %u bytes in %s @ %p\r\n",
                  (unsigned)arena_bytes, arena_location,
                  (const void *)tensor_arena);
    interp = make_interpreter(tensor_arena, arena_bytes);
    if (interp == nullptr) {
      SIGN_FATAL("could not construct the TFLM interpreter on the fallback arena");
    }
    allocate_status = interp->AllocateTensors();
  }
  if (allocate_status != kTfLiteOk) {
    const unsigned used = interp ? (unsigned)interp->arena_used_bytes() : 0;
    delete interp;
    interp = nullptr;
    SIGN_FATAL("AllocateTensors() failed with a %u-byte arena in %s "
               "(arena_used=%u) — the model needs more memory or an op is "
               "missing from the resolver; see the op list above",
               (unsigned)arena_bytes, arena_location, used);
  }
  s_arena_bytes = arena_bytes;
  interpreter = interp;

  // Get information about the memory area to use for the model's input.
  input = interpreter->input(0);

  // ── Fool-proofing: the model input MUST match the selected pipeline ──
  // The model's input dims are baked in at export time.  If the sketch's
  // preprocessing branch and the model disagree, the front-end writes into
  // the wrong-sized tensor: 9216 bytes into a 784-byte sign-slot input corrupts the
  // arena, and a 28x28 glyph into a 96x96 input is silent garbage.  Fail
  // loudly with the one-line fix instead.
  {
    const int expected_side = SIGN_GLYPH_SIZE;
    const bool shape_bad =
        (input->dims->size < 3) ||
        ((int)input->dims->data[1] != expected_side) ||
        ((int)input->dims->data[2] != expected_side);
    if (shape_bad) {
      const int model_h = (input->dims->size >= 3) ? (int)input->dims->data[1] : 0;
      const int model_w = (input->dims->size >= 3) ? (int)input->dims->data[2] : 0;
      Serial.printf(
          "FATAL: model input is %dx%d but this build preprocesses for sign glyphs (%dx%d).\r\n",
          model_h, model_w, expected_side, expected_side);
      if (model_h == SIGN_GLYPH_SIZE) {
        Serial.println("       One side matches SIGN_GLYPH_SIZE but the input is not square — "
                       "re-export the model from AItraining.");
      } else {
        Serial.printf("       Re-export the sign_ocr model in AItraining with img_size=%d and "
                      "reflash.\r\n", expected_side);
      }
      Serial.flush();
      for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
  }

  // ── Input encoder contract ───────────────────────────────────────────
  // The glyph front-end emits `int8 = gray - 128`, which is only the right
  // encoding while the input tensor is exactly scale 1/255 / zero_point -128
  // (what full-int8 PTQ produces from a representative dataset of
  // glyph/255.0).  One earlier export of this same glyph pipeline came out at
  // 235/255 — feeding the model an 8.5% brighter image with no error
  // anywhere.  There is no way to correct for that at runtime, so refuse.
  {
    const bool bad_type  = (input->type != kTfLiteInt8);
    const bool bad_zp    = (input->params.zero_point != -128);
    const bool bad_scale = (fabsf(input->params.scale - (1.0f / 255.0f)) > 1e-6f);
    if (bad_type || bad_zp || bad_scale) {
      Serial.printf("Input encoder mismatch: type=%d scale=%.6f zp=%d (want int8, %.6f, -128)\r\n",
                    (int)input->type, (double)input->params.scale,
                    (int)input->params.zero_point, 1.0 / 255.0);
      Serial.println("FATAL: the glyph front-end feeds (glyph - 128); re-export the sign_ocr "
                     "model with a representative dataset of glyph/255.0.");
      Serial.flush();
      for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
  }

  // ── Runtime save geometry + snapshot bound ───────────────────────────
  // The SD logger writes the model INPUT (that is what the snapshot holds),
  // so its geometry follows the tensor: 28x28x1 = 784 B here.  Guard the
  // snapshot buffer first — a
  // sizeof() copy from a smaller tensor is an arena overread, and from a
  // larger one a stack of zeros.
  if (input->bytes > sizeof(s_input_snapshot)) {
    Serial.printf("FATAL: input tensor is %u bytes but the snapshot buffer is only %u — "
                  "raise kSnapshotBytes in TFLite_Sign.ino.\r\n",
                  (unsigned)input->bytes, (unsigned)sizeof(s_input_snapshot));
    Serial.flush();
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  s_input_bytes = input->bytes;
  s_save_w = (uint16_t)input->dims->data[1];
  s_save_h = (uint16_t)input->dims->data[2];
  Serial.printf("Save/snapshot geometry: %ux%u, %u bytes/frame (buffer %u)\r\n",
                (unsigned)s_save_w, (unsigned)s_save_h,
                (unsigned)s_input_bytes, (unsigned)sizeof(s_input_snapshot));

  // ── Rejection policy ──────────────────────────────────────────────────
  // With kSignBackgroundReplacesOod the model MUST carry the N+1
  // "_background" class: blank slots would otherwise be forced to vote for
  // a real character and a stray line could complete a spurious MATCH.  With
  // the exported policy OFF the model trains on the target characters only,
  // and the OOD gates + the per-slot probability floor are the rejectors.
#ifdef TM_SIGN_USE_BACKGROUND_OOD
  if (kBackgroundClassIndex < -1 || kBackgroundClassIndex >= kCategoryCount) {
    Serial.printf("FATAL: kBackgroundClassIndex=%d is outside [-1, %d) — "
                  "re-export the sign project.\r\n",
                  (int)kBackgroundClassIndex, (int)kCategoryCount);
    Serial.flush();
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  if (kBackgroundClassIndex >= 0) {
    const char *bg_name = kCategoryLabels[kBackgroundClassIndex];
    if (bg_name == nullptr || strcmp(bg_name, "_background") != 0) {
      Serial.printf("FATAL: kBackgroundClassIndex=%d points at \"%s\", not "
                    "\"_background\" — re-export the sign project.\r\n",
                    (int)kBackgroundClassIndex, bg_name ? bg_name : "(null)");
      Serial.flush();
      for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
    }
  }
#endif
  if (kSignBackgroundReplacesOod && background_class_id() < 0) {
    Serial.println("FATAL: the export says _background is the only rejector, but the "
                   "model has no \"_background\" class — re-export the sign project.");
    Serial.flush();
    for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
  }
  if (kSignBackgroundReplacesOod) {
    Serial.printf("Sign labels: %d classes, rejection=_background (id %d) + %.2f floor\r\n",
                  (int)kCategoryCount, (int)background_class_id(),
                  (double)s_sign_match_min_prob);
  } else {
    Serial.printf("Sign labels: %d classes, rejection=OOD mp>=%.2f er<=%.2f "
                  "(sign_pct disarmed) + %.2f floor\r\n",
                  (int)kCategoryCount,
                  (double)kOodMaxProbMin, (double)kOodEntropyRatioMax,
                  (double)s_sign_match_min_prob);
  }

  // ── The exported whitelist must address real, non-background classes ──
  // Every row of kTargetSignClassMatrix is generated from the project's label
  // list; a stale export can point past it or at "_background" (which can
  // never match).  Fail at boot instead of reporting NO MATCH forever.
  Serial.printf("Sign targets: %d allowed plate(s)\r\n", (int)SIGN_PLATE_COUNT);
  for (int p = 0; p < SIGN_PLATE_COUNT; p++) {
    for (int slot = 0; slot < SIGN_SEQUENCE_LEN; slot++) {
      const int idx = SIGN_PLATE_CLASS(p, slot);
      if (idx < -1 || idx >= kCategoryCount) {
        Serial.printf("FATAL: plate '%s' slot %d class index %d is outside [-1, %d) — "
                      "re-export the sign_ocr project.\r\n",
                      SIGN_PLATE_TEXT(p), slot, idx, (int)kCategoryCount);
        Serial.flush();
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
      }
      if (idx >= 0 && is_background_id((uint8_t)idx)) {
        Serial.printf("FATAL: plate '%s' slot %d points at \"_background\" — "
                      "re-export the sign_ocr project.\r\n",
                      SIGN_PLATE_TEXT(p), slot);
        Serial.flush();
        for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
      }
    }
  }

  // --- Diagnostics ---
  const size_t arena_total = s_arena_bytes ? s_arena_bytes : kTensorArenaSize;
  Serial.print("Arena used: ");
  Serial.print(interpreter->arena_used_bytes());
  Serial.print(" / ");
  Serial.print((unsigned)arena_total);
  Serial.print(" (");
  Serial.print((int)(interpreter->arena_used_bytes() * 100LL / arena_total));
  Serial.println("%)");

  Serial.print("Input: ");
  Serial.print(input->dims->data[1]);
  Serial.print("x");
  Serial.print(input->dims->data[2]);
  Serial.print("x");
  Serial.print(input->dims->data[3]);
  Serial.print(" type=");
  Serial.println(input->type == kTfLiteInt8 ? "int8" : input->type == kTfLiteFloat32 ? "float32" : "other");

  {
    TfLiteTensor* out = interpreter->output(0);
    Serial.print("Output: ");
    Serial.print(out->dims->data[1]);
    Serial.print(" type=");
    Serial.print(out->type == kTfLiteInt8 ? "int8" : out->type == kTfLiteUInt8 ? "uint8" : out->type == kTfLiteFloat32 ? "float32" : "other");
    if (out->type == kTfLiteInt8) {
      Serial.print(" q=(");
      Serial.print(out->params.scale);
      Serial.print(",");
      Serial.print(out->params.zero_point);
      Serial.print(")");
    }
    Serial.println();
  }

  Serial.print("Ops: ");
  auto subgraph = model->subgraphs()->Get(0);
  auto opcodes = model->operator_codes();
  for (size_t i = 0; i < subgraph->operators()->size(); i++) {
    auto op = subgraph->operators()->Get(i);
    auto opcode = opcodes->Get(op->opcode_index());
    if (opcode->builtin_code() == tflite::BuiltinOperator_CONV_2D) Serial.print("Conv2D ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_MAX_POOL_2D) Serial.print("MaxPool ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_FULLY_CONNECTED) Serial.print("FC ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_SOFTMAX) Serial.print("Softmax ");
    else if (opcode->builtin_code() == tflite::BuiltinOperator_RESHAPE) Serial.print("Reshape ");
    else Serial.print("op#"); Serial.print(opcode->builtin_code()); Serial.print(" ");
  }
  Serial.println();

#if defined(ESP_NN)
  Serial.println("ESP-NN: ENABLED (optimized kernels)");
#else
  Serial.println("ESP-NN: NOT FOUND (using reference kernels — SLOW)");
  Serial.println("Install esp-nn library for 10-50x inference speedup.");
#endif
  // --- End Diagnostics ---

#if defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1)
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreatePinnedToCore(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr, 0);
  }
  xTaskCreatePinnedToCore(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(uart_rx_task, "uart_rx", kUartRxTaskStackBytes, nullptr, 2, nullptr, 0);
  xTaskCreatePinnedToCore(debug_serial_task, "dbgser", kDebugCmdStackBytes, nullptr, 1, nullptr, 0);

  auto try_create_inference = [](uint32_t stack_bytes, UBaseType_t prio) -> BaseType_t {
    TaskHandle_t h = nullptr;
    BaseType_t r = xTaskCreatePinnedToCore(
        inference_task, "tflm", stack_bytes, nullptr, prio, &h, 1);
    if (r == pdPASS) s_inference_task_stack_bytes = stack_bytes;
    Serial.printf("[setup] xTaskCreatePinnedToCore(inference_task) stack=%u prio=%u core=1 -> rc=%d handle=%p tasks=%u "
                  "heap_internal_8bit=%u heap_spiram=%u free_heap=%u min_free_heap=%u\r\n",
                  (unsigned)stack_bytes, (unsigned)prio, (int)r, (const void*)h,
                  (unsigned)uxTaskGetNumberOfTasks(),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT),
                  (unsigned)heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
                  (unsigned)xPortGetFreeHeapSize(),
                  (unsigned)xPortGetMinimumEverFreeHeapSize());
    Serial.flush();
    return r;
  };

  BaseType_t rc = try_create_inference(kInferenceTaskStackBytes, 3);
  if (rc != pdPASS) {
    static constexpr uint32_t kFallbackStack1 = 48u * 1024u;
    rc = try_create_inference(kFallbackStack1, 3);
  }
  if (rc != pdPASS) {
    static constexpr uint32_t kFallbackStack2 = 48u * 1024u;
    rc = try_create_inference(kFallbackStack2, 2);
  }
  if (rc != pdPASS) {
    static constexpr uint32_t kFallbackStack3 = 32u * 1024u;
    rc = try_create_inference(kFallbackStack3, 2);
  }
  if (rc != pdPASS) {
    Serial.println("[setup] FATAL: inference_task could not be created after 4 fallbacks. "
                   "CameraBegin / GetImage will NEVER run. Reduce static buffers, "
                   "enable PSRAM task stacks, or move tensor_arena to PSRAM.");
    Serial.flush();
  }
#else
  if (kEnableSdLogger && s_sd_queue) {
    xTaskCreate(sd_task, "sd", kSdTaskStackBytes, nullptr, 1, nullptr);
  }
  xTaskCreate(uart_tx_task, "uart_tx", kUartTxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(uart_rx_task, "uart_rx", kUartRxTaskStackBytes, nullptr, 2, nullptr);
  xTaskCreate(debug_serial_task, "dbgser", kDebugCmdStackBytes, nullptr, 1, nullptr);
  TaskHandle_t h = nullptr;
  BaseType_t rc = xTaskCreate(inference_task, "tflm", kInferenceTaskStackBytes, nullptr, 3, &h);
  if (rc == pdPASS) s_inference_task_stack_bytes = kInferenceTaskStackBytes;
  Serial.printf("[setup] xTaskCreate(inference_task) rc=%d stack=%u prio=3 handle=%p tasks=%u free_heap=%u\r\n",
                (int)rc, (unsigned)kInferenceTaskStackBytes, (const void*)h,
                (unsigned)uxTaskGetNumberOfTasks(), (unsigned)xPortGetFreeHeapSize());
  Serial.flush();
  if (rc != pdPASS) {
    static constexpr uint32_t kFallbackStack1 = 48u * 1024u;
    h = nullptr;
    rc = xTaskCreate(inference_task, "tflm", kFallbackStack1, nullptr, 2, &h);
    if (rc == pdPASS) s_inference_task_stack_bytes = kFallbackStack1;
    Serial.printf("[setup] xTaskCreate(inference_task) FB rc=%d stack=%u prio=2 handle=%p\r\n",
                  (int)rc, (unsigned)kFallbackStack1, (const void*)h);
    Serial.flush();
  }
#endif
  Serial.println("[setup] END -> returning to Arduino scheduler");
  Serial.flush();
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(1000));
}
