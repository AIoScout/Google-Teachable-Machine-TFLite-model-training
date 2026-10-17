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

constexpr int kCategoryCount = N;
extern const char* kCategoryLabels[kCategoryCount];

constexpr char kTargetSignSequence[] = "A1B";          // SIGN_SEQUENCE_LEN chars
constexpr int kTargetSignClassIndices[3] = {1, 3, 2};  // indices into the labels
constexpr int kSignReadingOrder = 0;                   // 0 = LTR, 1 = RTL
constexpr float kSignMatchMinProb = 0.600000f;
```

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
2. `ood_is_in_distribution(...)` — the raw-int8 OOD layers 2/3 agree (Layer 1
   is disarmed on this path, exactly as in the OCR sketch);
3. the winning class is not `_background` (matched by **name**, not index);
4. the winning class index equals `kTargetSignClassIndices[slot]`;
5. `prob >= kSignMatchMinProb`, where `prob` is the host's raw ratio
   `best_raw / total_raw` over `int8 + 128` values — **not** a dequantised
   softmax probability.

The frame verdict is:

| Verdict | Condition |
|---|---|
| `MATCH` (YES) | all three slots matched |
| `MISMATCH` (NO) | ink is present, at least one slot failed |
| `NO SIGN` | the split found no ink anywhere in the frame |

---

## 4. Wire format

The P4 → S3 packet is unchanged: 9 bytes,
`0xAA 0x55 0x01` + `frame_id` (LE16) + `label_id` + `confidence` + `flags` +
XOR checksum over bytes 0–7.

For a sign verdict:

- `label_id = 0xFF` — "read the flags", not a class;
- `confidence` = the weakest slot's best raw score (0..255);
- `flags` low nibble `0x02` = `sign_ready`; high nibble is the verdict:

| High nibble | Verdict |
|---|---|
| `0x00` | MATCH |
| `0xE0` | MISMATCH |
| `0xF0` | NO SIGN |

`S3_UART_Receiver` treats a MATCH as a real sign (so the existing ACK_STOP
logic keeps working) and ignores MISMATCH / NO SIGN.  That is deliberate
backward compatibility: a receiver flashed before the sign work still runs.

The on-device serial line is human-readable:

```
SIGN: MATCH (YES)  [0] 'A'->A 0.98  [1] '1'->1 0.93  [2] 'B'->B 0.97
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
  a softmax/dequantised probability; the host's 0.60 floor is calibrated on the
  raw scale.
- **Slot canvases are quantised into the tensor immediately before each
  Invoke**, so only one 784-byte slot buffer exists in RAM.  Keeping three
  quantised copies would cost 2,352 B for no behavioural change.
- **`label_id = 0xFF` and the flags high nibble are the verdict contract.**
  The S3 receiver keys on them; changing one side alone breaks the junction
  logic.
