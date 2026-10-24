# TFLite_Sign — the 3-character road-sign reader

`TFLite_Sign` is the device half of the `sign_ocr` project.  It is a separate,
self-contained Arduino sketch: it shares the camera plumbing with
`TFLite_OCR/` and `TFLite/`, but its front-end and its inference loop are its
own, and its `model_settings.h` is written by the sign project's export.

The model is still a **single-glyph** 28×28 int8 classifier — the same model an
`ocr` project trains.  What makes it a sign reader is the code around the
interpreter:

```
       96x96 library gray
              │
              ▼
   ┌──────────────────────┐   sign_glyph.c — integer-exact mirror of
   │ split into 3 slots   │   ocr_preprocess.split_sign_glyph_boxes
   └──────────┬───────────┘
              │  slot 0, slot 1, slot 2   (LABEL order)
              ▼
   ┌──────────────────────┐   one Invoke per slot; each 28x28 glyph is
   │ 3 × interpreter      │   quantised into the input tensor just before
   │      Invoke()        │   its own Invoke
   └──────────┬───────────┘
              │  label + prob + OOD decision, per slot
              ▼
   ┌──────────────────────┐   all three slots must match
   │ MATCH / MISMATCH /   │   kTargetSignClassIndices[] and clear
   │ NO SIGN              │   kSignMatchMinProb
   └──────────────────────┘
```

Nothing in the split or the glyph transform uses floating point.  The same
pipeline in AItraining (`ocr_preprocess.preprocess_sign_sequence`) is the
training-side reference, and `AItraining/tests/test_sign_glyph_parity.py`
holds the two implementations bit-for-bit.

---

## 1. Files

| File | Role |
|---|---|
| `TFLite_Sign.ino` | Sketch: capture, split, 3× inference, verdict, UART/SD/diagnostics |
| `sign_glyph.c` / `.h` | Integer-exact sign front-end (Otsu, split, per-slot glyph) |
| `image_provider.cpp` / `.h` | Camera library wrapper — **byte-identical** to the other sketches |
| `main_functions.h`, `arduino_main.cpp` | Arduino/TFLM glue — **byte-identical** to the other sketches |
| `model_settings.h` | **Written by the export.** Defines the routing macros, the target sequence, the class indices, the reading order and the luminance window |
| `model_settings.cpp` | Label table — written by the export |
| `model_resolver.h` | Operator resolver — written by the export |
| `tm_model_data.cpp` / `.h` | Model bytes — written by the export |
| `tools/sign_glyph_host.c` | Host harness used by the parity test |

`sign_glyph.c/.h` live in this folder and nowhere else — `sync_check.py`
reports them if they appear anywhere else.  The shared files must stay
byte-identical across all three sketch folders; see §5.

---

## 2. The contract the export stamps

For a `sign_ocr` project, `model_settings.h` carries:

```c
#define TM_OCR_GLYPH_MODEL 0
#define TM_SIGN_OCR_MODEL 1

#define TM_THRESHOLD_WINDOW_EXPORTED 1
#define kDarkThr 0          // the editor's Dark Thr, already nudged host-side
#define kLumThr 100

#define TM_MODEL_INPUT_SIDE 28
#define TM_PREPROCESS_FRAME_SIDE 96
#define TM_CAPTURE_FRAME_SIDE 160

constexpr int kCategoryCount = N;
extern const char* kCategoryLabels[kCategoryCount];

// Primary target (legacy-compatible; always sign_sequences[0]).
constexpr char kTargetSignSequence[] = "A12";          // SIGN_SEQUENCE_LEN chars
constexpr int kTargetSignClassIndices[3] = {0, 1, 2};  // indices into the labels

// Gatekeeper whitelist: one row per allowed plate.
#define TM_SIGN_MULTI_TARGET_EXPORTED 1
constexpr int kTargetSignCount = 2;
constexpr char kTargetSignSequences[kTargetSignCount][4] = { "A12", "B18" };
constexpr int kTargetSignClassMatrix[kTargetSignCount][3] = {
    { 0, 1, 2 },
    { 3, 1, 4 },
};
constexpr int kSignReadingOrder = 0;                   // 0 = LTR, 1 = RTL
constexpr float kSignMatchMinProb = 0.400000f;   // == kOodMaxProbMin
```

`g_frame_side` is initialized from `kCaptureFrameSide` (the macro), and
`g_preprocess_side` from `kPreprocessFrameSide`; older exports without the
macros fall back to 96/96.  The capture frame is box-averaged to
`g_preprocess_side` before the split (`frame_resize.h`), so a 160×160 camera
works with the 96×96-tuned front-end.  A factory-built export from before the
whitelist has `TM_SIGN_MULTI_TARGET_EXPORTED 0` and the sketch uses
`kTargetSignSequence` / `kTargetSignClassIndices` as a one-plate table.

There is **no per-slot mirroring**.  `kSignReadingOrder = 1` reverses the three
crops after the split, because the leftmost glyph is the LAST character of an
RTL label.  Mirroring the frame would hand the model mirrored glyphs — a
different, wrong, input.

A class index of `-1` means the character has no class in this export; the
device treats that slot as never matching.  The sketch validates every index
at boot and refuses to run (`FATAL`) if one is outside `[-1, kCategoryCount)`
or names `_background`.

The luminance window follows the host exactly.  Both defaults are a bit-exact
no-op; any other pair pins the binarisation threshold and adds the dark floor:

```c
adjusted = (kDarkThr != 0) || (kLumThr != 100)
thresh   = adjusted ? clamp(kLumThr - 1, 1, 254) : -1   // -1 = exact Otsu
dark_thr = adjusted ? clamp(kDarkThr + 1, 0, 255) : 0
```

---

## 3. The per-slot match rule

A slot matches when **all** of the following hold, mirroring
`RecordController._preview_predict_sign_ocr`:

1. `s_slot_info[slot].ok` — the slot's crop contained ink;
2. `ood_is_in_distribution(...)` — the raw-int8 OOD layers 2/3 agree.
   **Layer 1 (`sign_pct`) is disarmed unconditionally** on this path: the
   split already gates each slot on ink, and a frame-level percentage (often
   supplied by `set ood`) used to reject all three correct slots at once.
   An **entropy-only** Layer 3 failure is exempt when this slot's top class
   is the target of the plate being tested and `prob` clears the floor;
   `max_prob` still rejects;
3. the winning class is not `_background` (matched by **name**, not index);
4. there is an allowed plate whose row in `kTargetSignClassMatrix` names
   this slot's winning class index (every row is tested; all three slots
   must agree on the SAME row);
5. `prob >= s_sign_match_min_prob` — the **runtime** floor.  It is the same
   single threshold as OOD MAX PROB: the export writes one value into both
   `kOodMaxProbMin` and `kSignMatchMinProb` (default 0.40), and
   `set ood spmin spmax mpmin ermax` updates it to `mpmin` (both the text
   command and the binary `kCmdSetOodThresholds`).  There is no separate
   hidden floor, and values below 0.40 (e.g. 0.25) are honoured.  `get ood`
   prints the effective values.  `prob` is the host's raw ratio
   `best_raw / total_raw` over `int8 + 128` values — **not** a dequantised
   softmax probability.

`setup()` applies the exported policy before the first inference:
`ImageProviderSetOodThresholds(kOodSignMinPct, kOodSignMaxPct,
kOodMaxProbMin, kOodEntropyRatioMax)` and
`ImageProviderSetMaskThresholds(kBgDarkThresh, kBgLumThresh)`.  Without those
calls the provider keeps `image_provider.h`'s 0.40 / 0.85 compile-time
defaults and ignores `model_settings.h` entirely.

The frame verdict is:

| Verdict | Condition |
|---|---|
| `MATCH` (YES) | all three slots matched **one allowed plate** (`kTargetSignSequences`) |
| `MISMATCH` (NO) | ink is present, but no allowed plate matched all three slots |
| `NO SIGN` | the split found no ink anywhere in the frame |

Every row of `kTargetSignClassMatrix` is checked; an exact 3-for-3 wins.
When none is exact, the plate with the most hits supplies the diagnostic
`WANT` characters, but the verdict is still NO.

---

## 4. Wire format

The P4 → S3 packet is unchanged: 9 bytes,
`0xAA 0x55 0x01` + `frame_id` (LE16) + `label_id` + `confidence` + `flags` +
XOR checksum over bytes 0–7.

For a sign verdict:

- `confidence` = the weakest slot's best raw score (0..255);
- `flags` low nibble `0x02` = `sign_ready`; high nibble is the verdict;
- **`label_id` says which plate matched**:

| `flags` | Verdict | `label_id` |
|---|---|---|
| `0x02` | MATCH | matched plate index `0 .. kTargetSignCount-1` |
| `0xE2` | MISMATCH | `0xFE` |
| `0xF2` | NO SIGN | `0xFF` |

`S3_UART_Receiver` resolves the index through its copy of
`kTargetSignSequences` (exported with the model), logs the plate
(`label=1(B18)`), and treats only a MATCH as a real sign, so the existing
ACK_STOP → tasks → RESUME_JUNCTION cycle keeps working.  MISMATCH / NO SIGN
never trigger it.  A receiver flashed before this protocol still reads the
flags; it just cannot name the plate.

The on-device serial line is human-readable:

```
SIGN: MATCH (YES) plate#2=B18  [0] 'B'->B 0.98  [1] '1'->1 0.93  [2] '8'->8 0.97
SIGN: NO MATCH (NO)  [0] 'A'->6 0.41 (FAIL)  ...
```

---

## 5. Verify it

All commands run from the repository root.

```bash
# 1. C vs Python, bit-for-bit (split, Otsu, single glyph, full sequence)
python3 AItraining/tests/test_sign_glyph_parity.py

# 2. is the sketch wired to the same transform the trainer used?
python3 TFLite_OCR/tools/wiring_check.py --sign

# 3. are the sketch folders still in lockstep, each with its own export?
python3 TFLite_OCR/tools/sync_check.py

# 4. do all three sketches build, and does each reject the others' exports?
python3 TFLite_OCR/tools/ino_build_check.py --config <arduino-cli.yaml>
```

`wiring_check.py --sign` resolves every argument of the one
`preprocess_sign_sequence_c()` call, compares it against
`AItraining/ocr_preprocess.py`, cross-checks `kSignReadingOrder` and the
target arrays against the export, and verifies the 3-Invoke loop and the
per-slot match rule by pattern.  `sync_check.py` catches a shared file edited
in only one folder and an export copied into the wrong one.

---

## 6. Build and flash

Arduino IDE:

1. **Export** the `sign_ocr` project from AItraining into this folder.
   The export writes `model_settings.h/.cpp`, `model_resolver.h`,
   `tm_model_data.*` and the `.tflite`/`labels.txt`, overwriting the
   placeholders shipped here.
2. Open `TFLite_Sign/TFLite_Sign.ino`.
3. Board: **ESP32P4 Dev Module**, partition scheme: any scheme whose APP slot
   fits ~1.2 MB (`huge_app` is enough — this sketch does not need the legacy
   sketch's 3.6 MB+ slot).
4. Upload.

If the wrong project's export is in this folder, compilation stops at the
sketch's `#error`; the message names the folder the export actually belongs
to.  This is intentional — a silent build would run the wrong preprocessing.

---

## 7. Measured numbers

| | Flash | Static RAM |
|---|---|---|
| `TFLite_Sign` (3.9 MB model C array) | 1,181,996 B | 74,144 B |

Measured with `ino_build_check.py`, core 3.3.12, `app13M_data7M_32MB`.
The sign sketch adds one scratch block (`sign_scratch_bytes()`) and three
slot canvases over the OCR sketch; both sketches use a single boot-time
allocation and no per-frame heap allocation.

---

## 8. Invariants — do not "fix" these

- **`sign_glyph.c` is a mirror, not an implementation.**  The reference is
  `AItraining/ocr_preprocess.py`; a change on either side without the other
  is a silent device/host mismatch.  The parity test is the arbiter.
- **The split's tie rules are part of the contract** (narrowest run merged by
  first index; widest box split at the deepest valley with `cost` compared
  strictly; equal thirds when no cut exists).  "Cleaner" rewrites flip
  borderline frames.
- **Reading order reverses boxes, never pixels.**  There is no mirror anywhere
  in this pipeline.
- **Ink is `gray <= t` for dark ink and `gray > t` for bright ink,** and the
  `dark_thr` floor is applied ON TOP of that predicate.  Changing either
  comparison invalidates every trained model.
- **`prob` is the raw-int8 ratio**, on both sides.  Do not switch the device to
  a softmax/dequantised probability; the host's 0.40 floor is calibrated on the
  raw scale.
- **Slot canvases are quantised into the tensor immediately before each
  Invoke**, so only one 784-byte slot buffer exists in RAM.  Keeping three
  quantised copies would cost 2,352 B for no behavioural change.
- **`label_id` and the flags high nibble are the verdict contract.**  A
  MATCH sends the matched plate index, MISMATCH sends `0xFE` and NO SIGN
  sends `0xFF`; the S3 receiver keys on both bytes.  Changing one side alone
  breaks the gatekeeper logic.
- **Capture side, preprocessing side and model input are independent.**
  `g_frame_side = kCaptureFrameSide` (e.g. 160), `g_preprocess_side =
  kPreprocessFrameSide` (96), and each glyph is 28×28.  The frame is
  box-averaged 160→96 before `sign_glyph.c` runs; never feed the raw capture
  frame to the split.  `frame_resize.h` is the exact C mirror of the host
  resize.
