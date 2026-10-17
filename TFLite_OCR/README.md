# TFLite_OCR — the OCR device sketch, and the proof that it is exact

This folder is a **complete, self-contained Arduino sketch** — the OCR half of
the device firmware — plus the cross-language proof that its glyph front-end
matches the Python that trained the model.

It holds the **C port of the OCR glyph pipeline** (the transform that turns a
camera frame into the 28×28 int8 tensor an OCR model consumes).  That half
mattered because the training pipeline preprocesses on the host
(`AItraining/ocr_preprocess.py`) and nothing on the device reproduced it.  This
is that reproduction, integer-exact, so the model sees on the P4 exactly what it
was trained on.

**The three sketches are separate folders.**  `TFLite/` is the legacy 96×96
sign sketch; this is the single-glyph OCR 28×28 one; `TFLite_Sign/` reads a
3-character road sign by splitting it into three slots and running this same
glyph front-end once per slot.  They used to be one `.ino` behind
`#if TM_OCR_GLYPH_MODEL`; that switch is gone, and each `.ino` now `#error`s if
it is handed another sketch's export (§4.9).  `image_provider.cpp/.h`,
`main_functions.h` and `arduino_main.cpp` are duplicated across all three
folders and must stay byte-identical — `tools/sync_check.py` is what makes that
tolerable.  §4 describes what this sketch does; the numbers throughout are
re-measured after the split, not carried over.

**Status (verified, not asserted):**

| check | result |
|---|---|
| C vs Python golden corpus, every stage | **615 / 615 bit-identical** |
| …restricted to the parameters the sketch actually passes | **514 / 514 identical** (101 of the 615 exercise other callers' parameters) |
| …of which frames that came off the device rather than out of a generator | **400 real captures**, all bit-identical |
| Mutation testing (16 deliberate bugs, incl. the docstring trap) | **16 / 16 caught** |
| Device call vs trainer contract (`tools/wiring_check.py`) | **pass** — 33 assertions: the 13-argument `_window` call, its 10 resolved parameters, the trainer contract, the exported Dark/Lum window, and the 9 N+1 / OOD wiring patterns (`--sign` adds 44 more for the 3-character sketch) |
| The three sketches' shared files, sha256 (`tools/sync_check.py`) | **identical**; each folder holds its own export; no unclassified files |
| The split itself: split output vs the branch it came from | **byte-identical preprocessed TUs**, both directions (§4.9.1) |
| **All three sketches compiled + linked** for `esp32p4` | **all link**: `TFLite/` 3 576 209 B, `TFLite_OCR/` 1 180 100 B, `TFLite_Sign/` 1 181 996 B |
| Each sketch handed every OTHER sketch's export | **all six swaps refused at compile time** by the `#error` guards, messages verified — §4.9.1 |
| Legacy sketch vs the **pre-OCR** baseline | **0 OCR-only identifiers**; the split's 90 added / 21 removed lines in 9 hunks, plus the later 3-line sign-export guard, every one accounted for in §4.9 |
| riscv32-esp-elf compile, `-Wall -Wextra -Werror` | **clean** |
| Floating-point instructions in the object | **0** |
| Object size | text 3,852 B, data 0, bss 0 |
| Undefined symbols | `__divdi3`, `__udivdi3`, `memcpy`, `memset` |
| Scratch for 96×96 → 28 | **20,392 B** |

---

## 1. Files

**The sketch** (open `TFLite_OCR.ino` in the Arduino IDE; every file below sits
beside it and is compiled in place):

| file | role |
|---|---|
| `TFLite_OCR.ino` | the sketch: capture → glyph front-end → int8 → inference → OOD/`_background` → UART + SD |
| `ocr_glyph.h` | public API + the two documented traps (binarisation predicate, `thresh` semantics) |
| `ocr_glyph.c` | the pipeline: integer BT.601 → exact integer Otsu → polarity binarise → projection segmentation → normalise (STRETCH/CANON) → int8 |
| `image_provider.cpp/.h`, `main_functions.h`, `arduino_main.cpp` | **shared with `TFLite/` and `TFLite_Sign/`** — byte-identical copies of one source of truth |
| `model_settings.{h,cpp}`, `model_resolver.h`, `tm_model_data.{h,cpp}` | this sketch's **export** — overwritten by every AItraining export aimed at this folder |

**The checks** (none of these is compiled by the Arduino builder — they only
have to be correct, not small):

| file | role |
|---|---|
| `tools/gen_corpus.py` | builds the corpus and the Python golden outputs from the real `ocr_preprocess.py` |
| `tools/ocr_glyph_host.c` | replays the corpus through the C port and writes one output per case |
| `tools/compare.py` | per-stage comparison; exits non-zero on any divergence |
| `tools/mutation_check.py` | applies 16 deliberate bugs one at a time and requires the corpus to catch each; it mutates **this folder's** `ocr_glyph.c` |
| `tools/wiring_check.py` | reads the real `ocr_preprocess_gray()` call out of `TFLite_OCR.ino`, resolves its `kOcr*` arguments, and compares them with the trainer — plus the N+1/OOD wiring patterns.  `--sign` does the same for `preprocess_sign_sequence_c()` in `TFLite_Sign.ino`, including the exported target sequence/order |
| `tools/sync_check.py` | sha256 drift detector for the files shared by all three sketches, per-sketch export guards, and an inventory that flags an export or exclusive file in the wrong folder |
| `tools/ino_build_check.py` | builds **all three** sketches for `esp32p4`, and proves each one refuses every other sketch's export |

The corpus itself is a build artifact — regenerate it, don't commit it.

## 2. Verify it

```bash
cd TFLite_OCR
TMP=$(mktemp -d)

# 1. corpus (Python golden outputs, from the SAME ocr_preprocess.py that trains)
WS=~/Library/Application\ Support/TFLiteTraining/workspace   # or a project dir
python3 tools/gen_corpus.py --out "$TMP/corpus" --app-data "$WS" --real 400

# 2. parity, through the translation unit the device actually compiles
cc -O2 -std=c99 -Wall -Wextra -Werror -I . \
   -o "$TMP/host" tools/ocr_glyph_host.c ocr_glyph.c
"$TMP/host" "$TMP/corpus"
python3 tools/compare.py "$TMP/corpus"      # RESULT: ALL PASS

# 3. does the corpus have teeth?
python3 tools/mutation_check.py "$TMP/corpus"   # RESULT: all 16 mutations caught

# 4. is the DEVICE wired to the same transform the trainer used?
python3 tools/wiring_check.py --corpus "$TMP/corpus"

# 5. are the three sketches still in lockstep?  (shared files + export guards)
python3 tools/sync_check.py                 # RESULT: shared files identical

# 6. do ALL THREE sketches build, and does each refuse the others' exports?
python3 tools/ino_build_check.py --config <arduino-cli config>
```

Step 4 is the one that closes the loop: steps 1–3 prove the C *can* reproduce
Python when driven with the corpus' parameters, and step 4 proves the sketch
passes those parameters (and not, say, `OCR_INK_BRIGHT`) — reading them out of
the real call rather than assuming them.  For the 3-character sketch the same
question is `python3 tools/wiring_check.py --sign` (see
`TFLite_Sign/README.md`).

There used to be a step here that asked "does the shipped copy still match this
reference".  There is nothing to ask: only one copy of `ocr_glyph.c` exists now,
and it lives in this folder, just as `sign_glyph.c` lives only in
`TFLite_Sign/`.  What `sync_check.py` guards instead is what the split *did*
duplicate (§1) — the files all three sketches share — plus an inventory of
every folder, so a file added to one sketch and forgotten in the others is
reported rather than silently absent.

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

## 4. Deploying to the device — the changes, in order

**This is implemented.**  Sections 4.1-4.8 are kept in the imperative because
each one explains *why* a change was necessary — the reasoning is the part worth
re-reading when something breaks.  What shipped, and where:

| § | what it argues for | what landed |
|---|---|---|
| 4.1 | `ReduceMax` + `Rsqrt` in the resolver | `model_resolver.h` (generated) + the fallback in `TFLite_OCR.ino`; `_TFLITE_OP_RESOLVER_MAP` gained both keys **and** a warning when the map drops an op |
| 4.2 | an N+1 `model_settings.h` needs no sketch change | no change needed — plus a boot print of the `_background` id |
| 4.3 | feed the raw gray frame to `ocr_preprocess_gray()` | `ocr_feed_glyph()` in `TFLite_OCR.ino`; `ocr_glyph.{c,h}` beside the sketch; scratch sized by `ocr_scratch_bytes()` and allocated at boot |
| 4.4 | `_background` ⇒ the existing `0xF0` No-Sign nibble | `is_background_id()` / `label_name()`; matched by **name**, so the export does not have to keep it last |
| 4.5 | never print the raw `"_background"` | `label_name()` returns `"No Sign"` for it |
| 4.6 | OOD Layer 1 is meaningless here | disarmed *after* the threshold getter, so a host command cannot re-arm it; L2/L3 opened with it |
| 4.7 | size the arena for the OCR model | **left to the export** — still `kTensorArenaSize` in the sketch, and the boot log prints `arena_used_bytes()` |
| 4.8 | the mismatch detectors | four of them, plus a `static_assert` tripwire on the canvas size |
| — | the snapshot overread (§4.3's second bullet) | sized from `input->bytes`, bounds-checked before the copy — see §4.9 |

**Every one of these was fixed at the root.**  The op-resolver gap, the
snapshot overread, the OOD-Layer-1 constraint and the buffers sized by a pixel
count rather than by `input->bytes` are defects in the *legacy* path too; they
were repaired there rather than shadowed behind an OCR-only branch, so `TFLite/`
inherits all of them and the OCR sketch adds only the front-end and the
`_background` mapping on top.  §4.9 is the ledger of what that costs the legacy
sketch.

The generic rule that replaced the mode macro: **`TM_OCR_GLYPH_MODEL` is
emitted by the exporter into `model_settings.h` and is a one-way guard now, not
a selector** — `TFLite/` `#error`s if it reads `1`, `TFLite_OCR/` `#error`s if
it is absent or `0`.  A pre-macro export (the macro simply not defined) stays
valid in `TFLite/`, so every model deployed before this work still builds there
untouched.

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

The exported header is now `MicroMutableOpResolver<10>` with exactly ten
`Add*()` calls — at capacity, which is fine (one more op would return
`kTfLiteError` rather than corrupt anything).  Both kernels are already present
in the prebuilt P4 TFLM library, so no library rebuild is needed:

```
$ nm -g --defined-only .../esp32p4.orig/lib/libespressif__esp-tflite-micro.a | grep -E 'REDUCE_MAX|RSQRT'
00000000 T _ZN6tflite14Register_RSQRTEv
00000000 T _ZN6tflite19Register_REDUCE_MAXEv
```

A deeper fix, for the next model that surprises you: make the generator **fail
loudly** when the graph needs an op that `_TFLITE_OP_RESOLVER_MAP` cannot name,
instead of exporting a header that cannot load its own model.

### 4.2 `model_settings.h` — N + 1 classes

Nothing in the sketch assumes "classes are all real": `label_name()` bounds
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

`CameraGetGrayImgSized()` is declared at `image_provider.h:191`. Call it
once per frame and pass the same pointer to both the OCR branch and the
`kInferGray` capture stream — they want the same buffer, and a second call
would return the same pointer anyway.

Use `ocr_preprocess_rgb()` only if the training captures came from the **RGB**
stream; then both sides run the same integer BT.601 expression and the library's
float32 gray is bypassed. Mixing the two (train on library gray, infer on
integer BT.601 of RGB) is a 1-LSB-per-pixel mismatch that shows up as a
handful of flipped glyphs, not as an obvious failure.

Two build details:

* `ocr_glyph.c` and `ocr_glyph.h` must sit **next to `TFLite_OCR.ino`** (or be
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
static int8_t background_class_id() {         // cached; -1 = this model has none
  static int8_t cached = -2;
  if (cached == -2) {
    cached = -1;
    for (int i = 0; i < kCategoryCount; i++) {
      const char *name = kCategoryLabels[i];
      if (name != nullptr && strcmp(name, "_background") == 0) { cached = i; break; }
    }
  }
  return cached;
}

const bool is_background = is_background_id(label_id);

if (ood_suppressed || is_background) {
  pkt_flags = (uint8_t)((pkt_flags & 0x0F) | 0xF0);   // high nibble 0xF = No Sign
}
```

**Detect it by name, not by "the last index is always background".** The sign
model in the same sketch has three real classes and no background class —
`["LEFT", "STOP", "RIGHT"]` — so a positional rule would report "No Sign" for
every `RIGHT`. The shipped `background_class_id()` scans `kCategoryLabels` by
`strcmp` (once; the result is cached), so a model without a `_background` class
simply leaves rejection off and the class can sit anywhere in the list.

The `(flags & 0xF0) == 0xF0` contract is documented at lines 1436-1443 and is
what makes this a one-line change:

| flags high nibble | meaning | receivers use |
|---|---|---|
| `0xF0` | No Sign (now: OOD **or** `_background`) | ignore `label_id` / `confidence` |
| `0x00` | real label | `label_id` / `confidence` |

Because the background index is suppressed *by flag*, `label_id` never reaches
a receiver as "3" — downstream label tables stay N entries and still line up.
The device no longer depends on `_background` being last (it is found by name),
but §4.2's exporter rule still matters for anything that indexes labels by
position on the receiver side.

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

### 4.6 OOD gates: disable Layer 1 on this path

`ImageProviderLastSignPct()` (`image_provider.cpp:1594`) is written **only** by
the RGB crop/search path inside `GetImage()`. The OCR branch in §4.3 does not
call `GetImage()`, so `s_last_sign_pct` stays at its boot value `0.0f` — and
the Layer-1 gate (`sign_pct < OOD_SIGN_PCT_MIN`, default `0.3f`) then fires on
**every frame**, reporting "No Sign" forever with no other symptom. Skipping
`GetImage()` is correct; leaving Layer 1 armed is the trap.

In this sketch, set the sign-pct band open and let the explicit `_background`
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

Four silent-mismatch detectors, all cheap.  They sit together in `setup()`,
right after `input = interpreter->input(0)`:

**1. Input-shape guard.** The pre-existing `IMG_SIZE` check, now written against
the side this sketch actually preprocesses for:

```cpp
// TFLite_OCR.ino:
const int expected_side = OCR_GLYPH_SIZE;      // TFLite.ino: IMG_SIZE
// dims->size < 3, or dims[1] != expected_side, or dims[2] != expected_side
//   -> FATAL, naming the fix for whichever side is wrong — and, when the model
//      turns out to be 28x28, naming the sketch that model belongs to.
```
9216 bytes into a 784-byte tensor corrupts the arena; 784 into 9216 is silent
garbage.  The message distinguishes the two cases and never guesses.  Since the
split there is a *compile-time* half to the same guard as well — each `.ino`
`#error`s on the other folder's export (§4.9.1) — so the usual mistake is caught
before a flash rather than at boot.  The runtime check stays because it still
catches the case the `#error` cannot see: an export that carries the right
marker but the wrong `img_size`.
front-end feeds `glyph_u8 − 128`, which is only the right int8 encoding while
the input tensor is exactly scale 1/255 / zp −128 (what a representative
dataset of `glyph/255.0` float produces).  One earlier export of this same
pipeline came out at 235/255 — feeding the model an 8.5 % brighter image with
no error anywhere.  This sketch refuses to run; `TFLite/` keeps its old
behaviour and says so out loud, because it must stay bit-identical.

**3. Snapshot bound (all three sketches).** `input->bytes > sizeof(s_input_snapshot)`
→ FATAL before the copy, so the runtime-sized snapshot (§4.3) can never overread.

**4. `_background` presence (this sketch only).** A project with the macro set but no
`_background` class has no rejector at all (Layer 1 is inert here, §4.6) — every
frame would be a confident character.  Boot prints the class id, or halts.

The front-end constants carry a compile-time tripwire of their own:

```cpp
static_assert(OCR_GLYPH_SIZE == 28, "OCR_GLYPH_SIZE changed — it must equal "
                                    "AItraining ocr_preprocess.GLYPH_SIZE; "
                                    "re-verify TFLite_OCR/ tools first");
```

and `ocr_scratch_alloc()` prints the one line that makes a support ticket
answerable: glyph size, source gray size, the scratch byte count actually
allocated, and where it landed:

```
OCR scratch: 20392 bytes in internal RAM (96x96 gray -> 28 glyph)
```

### 4.9 What the LEGACY sketch sees — the entire delta

The model sketches are separate folders now, so this question has a structural
answer rather than a linker answer: `ocr_glyph.c` is not in `TFLite/` at all, and
`TFLite/image_provider.cpp` — the file CLAUDE.md pins to the Python training
pipeline — is a byte-identical copy of this folder's.  What is left is the delta
the *root-level* fixes leave behind, and that was measured against a true
pre-OCR baseline: `TFLite/TFLite.ino` at its revision before any of this work,
preprocessed exactly like the shipped file and diffed against it.

```bash
clang -E -P -x c++ -DTM_OCR_GLYPH_MODEL=0 \
      -I <stub headers for the camera library> -I TFLite TFLite/TFLite.ino
# same for the pre-change revision, then diff
```

The split's delta is **90 added / 21 removed lines in 9 hunks** (the later
3-line `TM_SIGN_OCR_MODEL` guard is the only addition since), and contains **no
OCR-only identifier** — 0 hits for each of `kOcrPolarity`, `kOcrSegment`,
`kOcrMinRun`, `kOcrPinnedThresh`, `kOcrConvention`,
`kOcrBackgroundIsTheOnlyRejector`, `ocr_feed_glyph`, `ocr_scratch_alloc`,
`ocr_scratch_bytes`, `s_ocr_glyph`, `s_ocr_scratch`.  The string `ocr_` survives
in exactly one place: a comment naming `ocr_glyph.h` where the 28-vs-96
wrong-export hint is phrased.

| delta | effect on a 96×96 int8 sign model |
|---|---|
| `s_input_snapshot` moves from a function-local static to file scope | none — the same 9216 B of BSS |
| snapshot copy and the SD PGM header/payload sized from `input->bytes` + dims | none — latched to 9216 / 96 / 96 before the tasks start |
| `background_class_id()`, `label_name()`, the flags nibble | inert — no class is named `_background`, so the lookup returns −1 and every branch is false |
| input-shape guard | the same refusal with clearer text; the new "this is an OCR export" hint can only fire for a 28×28 model |
| input-encoder check | new **warning only**, never a halt — a legacy model on the old path keeps running exactly as before |
| snapshot bound guard | `9216 > 9216` is false — unreachable |
| OOD layers, capture branch, UART task, SD copy loop | byte-identical |

Two things do change, deliberately:

1. **A legacy model whose input tensor is larger than 9216 B** (say
   `[1,96,96,3]`) used to boot and feed the classifier a truncated copy — a
   silent garbage input.  It now halts and prints both byte counts.  Nothing
   that worked on the old path is affected; something that never worked now
   says so.
2. **A build with no generated `model_resolver.h`** (the fallback branch) goes
   from `MicroMutableOpResolver<10>` to `<12>` and registers `Rsqrt` +
   `ReduceMax`.  In the shipped build the generated header is present, so the
   fallback is compiled out and this is invisible — but an export that always
   died at `AllocateTensors()` now runs.

The reverse direction is symmetric: in `TFLite_OCR/` the legacy `GetImage()`
crop/resize/stretch pipeline does not exist at all — the sketch feeds the glyph
front-end straight from the raw gray frame.

#### 4.9.1 The split itself, and why it is not merely "no worse"

The split was performed mechanically, and the equivalence is not argued from
reading a diff — it is *proved* by preprocessing.  The pre-split `.ino` was run
through the preprocessor twice, once per setting of the mode macro, and each
result was diffed against the preprocessed translation unit of the sketch that
inherited that branch:

| | OCR branch → `TFLite_OCR/TFLite_OCR.ino` | legacy branch → `TFLite/TFLite.ino` |
|---|---|---|
| preprocessed TU | **byte-identical** | **byte-identical** |
| lines | 1 461 | 1 567 |

Byte-identity after preprocessing is the strong form of the claim: the
preprocessor decides what exists before the compiler runs, so an identical token
stream means identical compiled code, not merely equivalent behaviour.  The two
branches were also confirmed to be genuinely different code rather than one
branch emitted twice — `ocr_scratch_alloc` occurs twice in the OCR TU and zero
times in the legacy one, and `GetImage(` occurs once in OCR (its declaration)
versus twice in legacy (declaration plus call).

This is *better* than the old arrangement, not just equal to it.  In the merged
sketch, an OCR identifier the guard happened to reference still had to be
declared before the `#if`, which is why `ocr_scratch_bytes` showed up in a
preprocessed legacy build.  There is no such file to reference now.  The legacy
sketch also lost the 28-vs-96 ternary on the snapshot size, which had been
`max(96², 28²)`.

The `#error` guards that replace the macro were verified the only way a guard
can be: by building the crossed pairs.  With three sketches that is all six
swaps — `TFLite_OCR/` handed `TFLite/`'s export, `TFLite/` handed
`TFLite_OCR/`'s, and the sign sketch handed either of the other two (and vice
versa) — and every one **fails to compile** with the intended message.
`tools/ino_build_check.py` asserts the failure *reason*, not merely that the
build failed, so a link error cannot pass as a fired guard.

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
  must be regenerated: `615/615` and `16/16` are the gate.
* **Integer-only.** The device port has zero floating-point instructions and
  the Otsu comparison is exact (a 160-bit fixed bignum; riscv32 has no
  `__int128` and float would flip decisions). Do not "simplify" it to
  `double`.

## 7. Measured numbers, one place

```
corpus                       615 cases (400 captures sampled from the app
                             workspace — 96x96 frames AND cached 28x28
                             glyphs — plus 215 adversarial synthetic, incl.
                             33 RGB).  One further workspace PNG was skipped
                             and reported: it is larger than ocr_glyph.h's
                             65536-px envelope, so the C port refuses it by
                             contract and the case could not have carried
                             parity information (§7.1).
parity                       615 / 615 bit-identical (thresh, ok, box, boxes,
                             tensor, and the final int8 stage)
restricted replay            514 / 514 of the cases that use this sketch's
                             parameters; the other 101 exercise bright ink /
                             STRETCH / a pinned threshold and belong to other
                             callers
mutations                    16 / 16 caught
  of which: binarise <=/</>   6      Otsu tie-break          189
            Otsu bignum->64b  536    resize exact rational   557
            CANON rounding     50    CANON paste offset       67
            CANON CoM sign    508    min_run off-by-one        6
            union y1 / y2 / x2  11 / 4 / 69  BT.601 weights   28
            BT.601 rounding    29    int8 zero point         615
riscv32-esp-elf              text 3,852 B, data 0, bss 0, 0 float insns
                             (-O2 -march=rv32imafc -mabi=ilp32f)
scratch                      96x96 -> 28 : 20,392 B     128x128 -> 28 : 34,856 B
arena                        6,272 B largest activation, 16,932 B no-reuse total
model                        57,592 B, 28x28x1 int8 in, 4x int8 out, 8 ops

── deployment (re-measured after the split) ──────────────────────────────
device call, sketch params   514 / 514 bit-identical (of the 615-case corpus)
device call, real captures   the 400 real captures are inside that 615/615;
                             they come out of the app workspace's PNGs, so
                             they are frames the device produced rather than
                             frames a generator drew
wiring_check                 33 assertions pass — the wired call, its resolved
                             parameters, the training-side contract, the
                             exported Dark/Lum window, and the 9 N+1 / OOD
                             wiring patterns.  --sign adds 44 for TFLite_Sign.
arduino build (p4)           ALL THREE sketches compile AND LINK for esp32p4,
                             each against its own real export:
                               TFLite/       3,576,209 B flash / 81,300 B RAM
                               TFLite_OCR/   1,180,100 B flash / 72,456 B RAM
                               TFLite_Sign/  1,181,996 B flash / 74,144 B RAM
                             and each REFUSES every other sketch's export —
                             all six swaps verified by building the crossed
                             pairs, and by asserting the failure is the
                             guard's message and not something else
                             (tools/ino_build_check.py).
                             Only the 11 ESP32_P4_IMX219 library symbols are
                             stubbed (that library is not installed here); the
                             stub is never executed and is not shipped.
                             Caveats, both measured, neither caused by this
                             work:
                             (a) arduino-esp32 3.3.12 does NOT define
                                 ARDUINO_ARCH_ESP32P4, so image_provider.cpp
                                 compiles its camera wrapper out
                                 (TFLITE_P4_HAS_ARDUINO_CAM_LIB 0) and the
                                 link then wants the Camera* symbols the
                                 library would have provided.  Check yours:
                                 `echo | <core>/tools/... -dM -E` or just
                                 build.  The repo's custom core (arduino/,
                                 CORE_REBUILD.md) is presumably what defines
                                 it — not verifiable here, macOS blocks
                                 reads of ~/Documents.
                             (b) because of (a), image_provider.cpp lines
                                 107-350 (the camera wrapper) were NOT
                                 compiled in this test.  That file is
                                 untouched by this work AND is now
                                 byte-identical between all three sketches;
                                 every line that IS part of the front-end —
                                 ocr_feed_glyph, the tensor write, the boot
                                 guards, the log changes — was compiled and
                                 linked.

── the partition requirement (read before flashing TFLite/) ──────────────
The legacy sketch is 3,576,209 B of APP.  `huge_app` provides 3,145,728 B,
and what you get there is `Error during build: text section exceeds available
space in board`.  That is a PARTITION failure, not a code failure — the sketch
links cleanly given room, and no stock 16 MB scheme is large enough.  Almost
all of it is the legacy export's own model data (tm_model_data.cpp is
2,467,824 B) plus ~1.1 MB of code, so this is not new to the split.  For the
record `TFLite_OCR/` is less than a third of that and fits in `huge_app`.
Split correctness            the pre-split .ino, preprocessed once per macro
                             value, diffed against the sketch that inherited
                             that branch: BYTE-IDENTICAL translation units in
                             both directions (1,461 and 1,567 lines) — §4.9.1
Legacy sketch vs the TRUE
pre-OCR baseline             90 added / 21 removed lines in 9 hunks (plus the
                             3-line sign-export guard), with 0 hits for all
                             11 OCR-only identifiers (§4.9).
                             ocr_glyph.{c,h} are not in TFLite/ at all, so
                             there is no front-end code left to
                             garbage-collect.
```

### 7.1 The frame-size envelope, and a harness bug it exposed

`ocr_glyph.h` bounds the frames the C port will accept: `w ≤ OCR_MAX_W` (1024)
and `w·h ≤ OCR_MAX_PIXELS` (65536).  The device never comes close — it presents
96×96 or 128×128, a 7× margin — so the limits are not device constraints; they
are the point past which the library *refuses* a frame rather than overrunning
Otsu's 160-bit accumulator or the per-column projection scratch.

`preprocess_glyph` on the Python side has no such envelope.  That asymmetry is
harmless on the device and fatal to a corpus: an out-of-envelope case is
*guaranteed* to "diverge", because C returns early while Python happily
processes it.  Such a case cannot carry parity information by construction.

It was reachable, because `--app-data` accepts any directory tree of PNGs and a
real workspace root is not a pile of captures — it also holds screenshots and
imported images.  Sampling 400 of them turned up a 648×509 frame, and that
exposed two bugs, both now fixed:

* **`tools/ocr_glyph_host.c` overran its buffer.**  `read_file()` did
  `fread(dst, 1, n, f)` and *then* checked whether it had got `n` bytes — the one
  order that cannot work, since the overrun happens on the way to the check.  A
  329,832-byte frame into a `OCR_MAX_PIXELS * 3` = 196,608-byte buffer wrote
  133 KB past the end before reporting "cannot read".  It now refuses on size
  before opening the file, and the message names the case and both numbers
  instead of reading like a missing file.
* **`tools/gen_corpus.py` emitted those frames anyway.**  It now filters real
  captures to the envelope, counts what it dropped, and prints the count — and
  it draws from the permutation until it has `--real` usable frames, rather than
  taking the first `--real` and letting an unusable one shrink the corpus.

Neither bug could affect the firmware: the device cannot present such a frame.
What they affected was the *trustworthiness of the harness* — the first symptom
was `compare.py` reporting 565 divergences that were an artefact of a truncated
replay, not a preprocessing mismatch.  A verification tool that corrupts memory
while telling you your corpus is broken is worse than one that fails loudly, so
both halves are fixed rather than the filter alone.

