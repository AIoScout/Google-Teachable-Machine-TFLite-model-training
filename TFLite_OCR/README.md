# TFLite_OCR — device-side glyph front-end, and what to change in `TFLite/`

This folder is the **C port of the OCR glyph pipeline** (the transform that
turns a camera frame into the 28×28 int8 tensor an OCR model consumes) plus
the cross-language proof that it matches the Python that trained the model.

It is deliberately *not* a copy of `TFLite/`. It is the missing half of the
OCR deployment: the training pipeline preprocesses on the host
(`AItraining/ocr_preprocess.py`), and until now nothing on the device
reproduced it. This folder is that reproduction, integer-exact, so the model
sees on the P4 exactly what it was trained on.

**Status (verified, not asserted):**

| check | result |
|---|---|
| C vs Python golden corpus, every stage | **510 / 510 bit-identical** |
| Mutation testing (16 deliberate bugs, incl. the docstring trap) | **16 / 16 caught** |
| riscv32-esp-elf compile, `-Wall -Wextra -Werror` | **clean** |
| Floating-point instructions in the object | **0** |
| Object size | text 3,828 B, data 0, bss 0 |
| Undefined symbols | `__divdi3`, `__udivdi3`, `memcpy`, `memset` |
| Scratch for 96×96 → 28 | **20,392 B** |

---

## 1. Files

| file | role |
|---|---|
| `ocr_glyph.h` | public API + the two documented traps (binarisation predicate, `thresh` semantics) |
| `ocr_glyph.c` | the pipeline: integer BT.601 → exact integer Otsu → polarity binarise → projection segmentation → normalise (STRETCH/CANON) → int8 |
| `tools/gen_corpus.py` | builds the corpus and the Python golden outputs from the real `ocr_preprocess.py` |
| `tools/ocr_glyph_host.c` | replays the corpus through the C port and writes one output per case |
| `tools/compare.py` | per-stage comparison; exits non-zero on any divergence |
| `tools/mutation_check.py` | applies 16 deliberate bugs one at a time and requires the corpus to catch each |

The corpus itself is a build artifact — regenerate it, don't commit it.

## 2. Verify it

```bash
cd TFLite_OCR
TMP=$(mktemp -d)

# 1. corpus (Python golden outputs, from the SAME ocr_preprocess.py that trains)
WS=~/Library/Application\ Support/TFLiteTraining/workspace   # or a project dir
python3 tools/gen_corpus.py --out "$TMP/corpus" --app-data "$WS" --real 400

# 2. C port
cc -O2 -std=c99 -Wall -Wextra -Werror -I . \
   -o "$TMP/host" tools/ocr_glyph_host.c ocr_glyph.c
"$TMP/host" "$TMP/corpus"

# 3. parity
python3 tools/compare.py "$TMP/corpus"      # RESULT: ALL PASS

# 4. does the corpus have teeth?
python3 tools/mutation_check.py "$TMP/corpus"   # RESULT: all 16 mutations caught
```

`--app-data` takes anything whose `.png` files are workspaces worth replaying;
it recurses, so pointing at the app's workspace root picks up both `tm_dataset`
(the 96×96 captures the device streamed) **and** `.tm_processed_cache` (the
28×28 glyphs a previous run already produced). Feeding cached glyphs back
through the pipeline is legitimate and useful — it exercises the small-frame
and degenerate paths — but they are not captures, so don't read the count as
"frames seen by the camera". Synthetic cases alone are too well-behaved to
find anything. Nothing here needs TensorFlow: the corpus is produced by
importing `ocr_preprocess` directly and the comparison is over golden bytes.

**Read `mutation_check.py`'s output, not just its exit status.** A mutation
reported `caught by 0 cases` is a hole in the corpus, not a passing test —
that is how the kept-union seg-box cases in `gen_corpus.py` were added (a
randomly-shaped glyph is tallest on its leftmost run, so the first version of
the corpus could not tell a first-run union from a full union).

---

## 3. The model contract (measured, not assumed)

From the exported OCR model (`fb`-parsed, no TensorFlow needed):

| | |
|---|---|
| file | 57,592 B (≈ 364 KB as a C array) |
| input | `int8 [1, 28, 28, 1]`, scale `0.0039215688…` = **f32(1/255)**, zero_point **−128** |
| output | `int8 [1, 4]`, scale `1/256`, zero_point −128 |
| classes | `["Digit 1", "Stroke L", "Bar T", "_background"]` — **N + 1, background LAST** |
| builtin ops | `CONV_2D, MAX_POOL_2D, RSQRT, STRIDED_SLICE, REDUCE_MAX, RESHAPE, FULLY_CONNECTED, SOFTMAX` |

Two consequences worth stating explicitly:

* **The feed is a plain `glyph_u8 − 128`.** Training fed `glyph/255.0` float;
  PTQ turned that into scale 1/255, zp −128; so `q = g/255/(1/255) − 128 = g − 128`
  — integer-exact, no clamp, no float. That is what `ocr_glyph_to_int8()` emits.
* **The last index means "no glyph".** Not a label — a *rejection*. Section 4.4.

> **The scale is not structurally guaranteed.** An earlier export of this same
> code (the pre-transfer, 3-class run) came out at scale `0.0036139…` =
> 235/255, because the converter takes the input range from the representative
> dataset and that run's calibration samples never reached 255. A device that
> hard-codes `g − 128` against such a model feeds it input 8.5 % brighter than
> training did — silently. Hence the boot assert in §4.8.

---

## 4. Deploying to `TFLite/` — the changes, in order

Line numbers are `TFLite/TFLite.ino` as of this commit.

### 4.1 Fix the op resolver first — nothing works until you do

`TFLite/model_resolver.h` (checked in, and regenerated by every export)
registers:

```
Conv2D, MaxPool2D, Shape, StridedSlice, Pack, Reshape, FullyConnected, Softmax
```

The model in the same sketch needs `RSQRT` and `REDUCE_MAX` (both exports do —
the sign model and the OCR model). `TM_HAS_GENERATED_MODEL_RESOLVER` is
`__has_include("model_resolver.h")` (line 52) and is therefore **1**, so the
generated header wins and the fallback resolver at lines 1675-1689 (which also
lacks both ops) is dead code. `AllocateTensors()` at line 1736 cannot succeed:
TFLM logs *"Didn't find op for builtin opcode 'REDUCE_MAX'"* and the sketch
stops there.

**Fix — two entries in the generator's op map** (`AItraining/record_controller.py`,
`_TFLITE_OP_RESOLVER_MAP`, ~line 180). The map is a filter: any op missing from
it is dropped from the generated header *without a warning*, which is how this
happened.

```python
    "REDUCE_MAX": "ReduceMax",
    "RSQRT": "Rsqrt",
```

Then re-export. The op count does not change (still 8 slots — `Shape` and
`Pack` are simply not in this graph), and both kernels are already present in
the prebuilt P4 TFLM library, so no library rebuild is needed:

```
$ nm -g --defined-only .../esp32p4.orig/lib/libespressif__esp-tflite-micro.a | grep -E 'REDUCE_MAX|RSQRT'
00000000 T _ZN6tflite14Register_RSQRTEv
00000000 T _ZN6tflite19Register_REDUCE_MAXEv
```

A deeper fix, for the next model that surprises you: make the generator **fail
loudly** when the graph needs an op that `_TFLITE_OP_RESOLVER_MAP` cannot name,
instead of exporting a header that cannot load its own model.

### 4.2 `model_settings.h` — N + 1 classes

Nothing in `TFLite.ino` assumes "classes are all real": `label_name()` bounds
by `kCategoryCount`, `pick_label_and_confidence()` and
`ood_is_in_distribution()` loop to it, and `static_assert(kCategoryCount <= 32)`
(line 991) covers the 4-class OCR model. So the OCR `model_settings.h` can be
dropped in as generated — `kCategoryCount = 4`, `kCategoryLabels` including
`"_background"`.

Two rules for the exporter, both already true of this export:

* `_background` must be **last**. Every wire consumer indexes labels by
  position, so a future export that sorts alphabetically would renumber
  `Digit 1` / `Stroke L` / `Bar T` and break the S3's label table silently.
* Never strip it from `kCategoryLabels` — §4.5 needs the name to find it.

### 4.3 Feed the interpreter: gray → glyph → int8

The sign path (line 1374) calls
`GetImage(..., input->data.int8)` — crop → bilinear → BT.601 → contrast stretch
→ int8, i.e. the *sign* pipeline. **The OCR model must not be fed that**: its
input is a 28×28 glyph canvas, and its 96×96 source frame is the *raw* gray
capture, not a crop.

The device already exposes exactly the right buffer
(`TFLite/image_provider.cpp::CameraGetGrayImgSized()`): *"the LIBRARY's own
BT.601 gray conversion — the exact source the IMX219_Grayscale_Serial
data-collection stream uses for training. No crop, no contrast stretch."*
That is byte-for-byte what the training PNGs contain, so the C front-end runs
on the same bytes the Python did.

Sketch the OCR branch in the `tflm` task (replacing the `GetImage` call):

```cpp
#include "ocr_glyph.h"          // from TFLite_OCR/
// ...
    static uint8_t  s_ocr_scratch[OCR_SCRATCH_BYTES_96];
    static OcrScratch s_scratch;
    static uint8_t  glyph[OCR_GLYPH_SIZE * OCR_GLYPH_SIZE];
    static bool     s_scratch_ok = false;
    if (!s_scratch_ok) {
      s_scratch_ok = ocr_scratch_init(&s_scratch, s_ocr_scratch,
                                      sizeof(s_ocr_scratch), g_frame_side,
                                      g_frame_side, OCR_GLYPH_SIZE);
      if (!s_scratch_ok) { /* log + halt: scratch sizing is a build-time fact */ }
    }

    const uint8_t *gray = CameraGetGrayImgSized();   // raw library gray, 96x96
    if (gray == nullptr) { vTaskDelay(pdMS_TO_TICKS(1)); continue; }

    OcrGlyphInfo ginfo;
    ocr_preprocess_gray(gray, g_frame_side, g_frame_side, OCR_GLYPH_SIZE,
                        OCR_INK_DARK,      /* captures: dark ink on bright page */
                        /*apply_segmentation=*/true,
                        OCR_MIN_GLYPH_RUN, /* = 2, matches Python MIN_GLYPH_RUN  */
                        /*thresh=*/-1,      /* -1 = exact Otsu (0 is a PINNED 0) */
                        OCR_NORM_CANON,    /* the convention the OCR mix trains in */
                        &s_scratch, glyph, &ginfo);

    ocr_glyph_to_int8(glyph, sizeof(glyph), input->data.int8);   // g - 128
```

Sizing (`ocr_scratch_bytes(g_frame_side, g_frame_side, OCR_GLYPH_SIZE)`, so it
follows `g_frame_side` instead of a magic number):

```c
static constexpr size_t OCR_SCRATCH_BYTES_96  = 20392;  /* 96x96  -> 28 */
static constexpr size_t OCR_SCRATCH_BYTES_128 = 34856;  /* 128x128 -> 28 */
```

`CameraGetGrayImgSized()` is declared at `TFLite/image_provider.h:191`. Call it
once per frame and pass the same pointer to both the OCR branch and the
`kInferGray` capture stream — they want the same buffer, and a second call
would return the same pointer anyway.

Use `ocr_preprocess_rgb()` only if the training captures came from the **RGB**
stream; then both sides run the same integer BT.601 expression and the library's
float32 gray is bypassed. Mixing the two (train on library gray, infer on
integer BT.601 of RGB) is a 1-LSB-per-pixel mismatch that shows up as a
handful of flipped glyphs, not as an obvious failure.

Two build details:

* `ocr_glyph.c` and `ocr_glyph.h` must sit **next to `TFLite.ino`** (or be
  installed as an Arduino library) — the Arduino builder compiles `.c`/`.cpp`
  files in the sketch folder, and `ocr_glyph.h` already carries the
  `extern "C"` guards needed for the C++ sketch to include it.
* **The input snapshot at line 1389 must be resized for OCR.** It is
  `static int8_t input_snapshot[IMG_SIZE * IMG_SIZE]` and copied with
  `sizeof(input_snapshot)` — against a 28×28 tensor that is a 9,216-byte read
  from a 784-byte buffer, i.e. arena garbage in the SD log (the same class of
  bug the comment above it was written to fix). Use `kMaxImageSize`, or
  `input->bytes`, or guard the whole snapshot on the sign path.

### 4.4 The N + 1 output tensor — what to do when the model says `_background`

This is the question the deployment hinges on. The model has N real classes
plus `_background` **last**, so:

* **`label_id == kCategoryCount - 1` means "no glyph in this crop"** — not a
  character to send.
* It maps onto the **existing** "No Sign" signalling: the high nibble of
  `flags` (`0xF0`), which the S3 and every receiver already implement. No
  protocol change, no S3 firmware change, no new byte.

Add the background test next to the existing OOD block (line 1447-1452) and OR
it into the same condition:

```cpp
// ── OCR background class -> the same "No Sign" signalling ────────────────
// The OCR export carries N real classes plus a trailing synthetic
// "_background" class, so the LAST index means "no glyph here", not a
// character.  Fold it into the high nibble the OOD gate already uses, and
// (exactly like OOD) leave label_id/confidence raw so the CSV still records
// what the model would have said.
static int8_t background_class_id() {
  static int8_t cached = -2;
  if (cached == -2) {
    cached = -1;
    if (kCategoryCount > 0 && kCategoryLabels[kCategoryCount - 1] != nullptr &&
        strcmp(kCategoryLabels[kCategoryCount - 1], "_background") == 0) {
      cached = (int8_t)(kCategoryCount - 1);
    } else {
      Serial.println("Note: no trailing _background class — OCR rejection disabled.");
    }
  }
  return cached;
}

const int8_t bg_id = background_class_id();
const bool is_background = (bg_id >= 0 && label_id == (uint8_t)bg_id);

if (ood_suppressed || is_background) {
  pkt_flags = (uint8_t)((pkt_flags & 0x0F) | 0xF0);   // high nibble 0xF = No Sign
}
```

**Detect it by name, not by "the last index is always background".** The sign
model in the same sketch has three real classes and no background class —
`["LEFT", "STOP", "RIGHT"]` — so a positional rule would report "No Sign" for
every `RIGHT`. The name is in `kCategoryLabels`, so the check costs one
`strcmp` once, and a model without a `_background` class simply leaves OCR
rejection off.

The `(flags & 0xF0) == 0xF0` contract is documented at lines 1436-1443 and is
what makes this a one-line change:

| flags high nibble | meaning | receivers use |
|---|---|---|
| `0xF0` | No Sign (now: OOD **or** `_background`) | ignore `label_id` / `confidence` |
| `0x00` | real label | `label_id` / `confidence` |

Because the background index is suppressed *by flag*, `label_id` never reaches
a receiver as "3" — downstream label tables stay N entries and still line up,
provided `_background` stays last (§4.2).

### 4.5 `label_name()` — never print the raw `"_background"`

`label_name()` (line 981) returns `kCategoryLabels[id]` for
`id < kCategoryCount`, so the background index stringifies to `"_background"`
in the debug print (line 1487) and in the SD/CSV log (line 1548). Have
`label_name()` answer first:

```cpp
static const char* label_name(uint8_t id) {
  if (background_class_id() >= 0 && id == (uint8_t)background_class_id()) {
    return "No Sign";              // OCR N+1: the extra class is a rejection
  }
  if (id < (uint8_t)kCategoryCount) {
    const char *name = kCategoryLabels[id];
    return name ? name : "?";
  }
  return "No Sign";
}
```

Order matters: the background index *is* `< kCategoryCount`, so it must be
tested before the normal lookup. Leave the `id >= kCategoryCount` branch alone
— that is the pre-existing synthetic OOD label and other sketches rely on it.

### 4.6 OOD gates: disable Layer 1 in OCR mode

`ImageProviderLastSignPct()` (`image_provider.cpp:1594`) is written **only** by
the RGB crop/search path inside `GetImage()`. The OCR branch in §4.3 does not
call `GetImage()`, so `s_last_sign_pct` stays at its boot value `0.0f` — and
the Layer-1 gate (`sign_pct < OOD_SIGN_PCT_MIN`, default `0.3f`) then fires on
**every frame**, reporting "No Sign" forever with no other symptom. Skipping
`GetImage()` is correct; leaving Layer 1 armed is the trap.

In OCR mode, set the sign-pct band open and let the explicit `_background`
class plus Layers 2/3 (max_prob ≥ 0.60, entropy ≤ 0.70) do the gating:

```cpp
ImageProviderSetOodThresholds(0.0f, 100.0f, /*max_prob_min=*/0.60f,
                              /*entropy_ratio_max=*/0.70f);
```

This is not a weakening: with an explicit `_background` class the model *is*
the no-glyph detector, which is strictly better information than the dark-pixel
prior. `_background` and the OOD gates compose with OR, so either can suppress.
Keep both layers' math as is — `pick_label_and_confidence()` and
`ood_is_in_distribution()` already loop to `kCategoryCount`, so the N+1
distribution and the `logf(kCategoryCount)` entropy normaliser match the host's
`_preview_predict` automatically.

### 4.7 Tensor arena

Measured from the flatbuffer (13 constants / 14 activations):

| | bytes |
|---|---|
| largest single activation | 6,272 (`[1,28,28,8]`) |
| **sum of all activations, no reuse** | **16,932** |
| liveness peak (planner lower bound) | 7,840 |
| sign model's current arena (`kTensorArenaSize`, line 324) | 8,388,608 |

`kTensorArenaSize` is `8192 * 1024` — sized for the 2.4 MB sign model. The OCR
model is 57 KB, so size the arena for it (24 KB is comfortable above the
no-reuse bound; 8 KB is not) and keep the boot log at lines 1764-1770, which
prints `arena_used_bytes()`. If `AllocateTensors()` fails it is almost always
the resolver (§4.1), not the arena — the model is 6.1 KB of live activation.

### 4.8 Boot assertions

Two silent-mismatch detectors, both cheap. Add next to the existing
`IMG_SIZE` check (lines 1750-1762), which already refuses to run when the model
and the sketch disagree about input size:

```cpp
// The glyph front-end feeds `glyph_u8 - 128`, which is only the right int8
// encoding while the input tensor is exactly scale 1/255 / zp -128 (what a
// representative dataset of glyph/255.0 float produces).  One earlier export
// of this same pipeline came out at 235/255, which would have fed the model
// an 8.5% brighter image with no error anywhere.
if (input->type != kTfLiteInt8 ||
    input->params.zero_point != -128 ||
    fabsf(input->params.scale - (1.0f / 255.0f)) > 1e-6f) {
  Serial.printf("FATAL: model input is not int8 scale=1/255 zp=-128 "
                "(type=%d scale=%.9f zp=%d) — ocr_glyph_to_int8() assumes it.\r\n",
                (int)input->type, (double)input->params.scale,
                (int)input->params.zero_point);
  Serial.flush();
  for (;;) { vTaskDelay(pdMS_TO_TICKS(1000)); }
}

// The front-end constants must be the ones the training captures used.
static_assert(OCR_GLYPH_SIZE == 28, "OCR model input side changed — re-export "
                                    "and re-verify TFLite_OCR/");
Serial.printf("OCR: glyph %dx%d <- gray %dx%d, scratch %u B, background=%d\r\n",
              OCR_GLYPH_SIZE, OCR_GLYPH_SIZE, g_frame_side, g_frame_side,
              (unsigned)ocr_scratch_bytes(g_frame_side, g_frame_side,
                                          OCR_GLYPH_SIZE),
              (int)background_class_id());
```

---

## 5. The wire contract does not change

The OCR model reuses the existing 9-byte P4 → S3 packet (`0xAA 0x55 0x01` +
`frame_id_LE16` + `label_id` + `confidence` + `flags` + `xor8`) byte for byte.
`_background` is carried as `flags |= 0xF0`, which is exactly what OOD
suppression already emits, so `S3_UART_Receiver/` and the host CSV tooling need
no change at all. `label_id` and `confidence` stay unmodified under the flag
(same rule as OOD) — what the model *would* have said remains visible offline.

## 6. Invariants — do not "fix" these

* **`ocr_binarize_ink()` mirrors the Python CODE, not its docstring.** The
  docstring in `ocr_preprocess.otsu_threshold_u8` claims `v < t` / `v >= t`;
  the code that trained every model uses `<=` / `>`. Changing either side to
  match the other's comment silently invalidates every existing model — the
  mutation check catches it in 5 cases, which is the point.
* **`thresh = -1` means Otsu; `thresh = 0` is a legal *pinned* threshold.**
  They are different code paths (Otsu's `t == 0` means "no split, no ink").
* **Keep all three in lockstep.** Any change here must land in
  `AItraining/ocr_preprocess.py` and `ocr_glyph.c` together, and the corpus
  must be regenerated: `510/510` and `16/16` are the gate.
* **Integer-only.** The device port has zero floating-point instructions and
  the Otsu comparison is exact (a 160-bit fixed bignum; riscv32 has no
  `__int128` and float would flip decisions). Do not "simplify" it to
  `double`.

## 7. Measured numbers, one place

```
corpus                       510 cases (295 frames from the app workspace —
                             96x96 captures AND cached 28x28 glyphs — plus
                             215 adversarial synthetic, incl. 33 RGB)
parity                       510 / 510 bit-identical (thresh, ok, box, boxes,
                             tensor, and the final int8 stage)
mutations                    16 / 16 caught
  of which: binarise <=/</>  5      Otsu tie-break          420
            Otsu bignum->64b 370    resize exact rational   409
            CANON rounding   13     CANON paste offset      14
            CANON CoM sign   423    min_run off-by-one      3
            union y1 / y2 / x2  4 / 1 / 8   BT.601 weights  22
            BT.601 rounding  27     int8 zero point         510
riscv32-esp-elf              text 3,828 B, data 0, bss 0, 0 float insns
scratch                      96x96 -> 28 : 20,392 B     128x128 -> 28 : 34,856 B
arena                        6,272 B largest activation, 16,932 B no-reuse total
model                        57,592 B, 28x28x1 int8 in, 4x int8 out, 8 ops
```
