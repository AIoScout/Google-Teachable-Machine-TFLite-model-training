# AIoScout / TFLite Training — Image, OCR, and 🚗 Car License (3-Char) Studio

Desktop trainer for ESP32-P4 tinyML, with an IMX219 camera pipeline and
three export targets.  Train in the app, export int8 TFLite + C sources,
flash the matching Arduino sketch.

Branch: `esp32p4-middle-detect`

---

## 1. Three project types

The Start Page offers three cards.  Each one owns its model geometry, its
training strategy, its Preview verdict, and the firmware folder its export
belongs to.

| Project type | Start card | Model input | Output | Export target |
|---|---|---|---|---|
| **Image** (`image`) | Image Project | 96×96 int8 grayscale by default | one class label + confidence, with a 3-layer OOD gate (`sign_pct` range, max prob, entropy) | `TFLite/` |
| **OCR / Character** (`ocr`) | 🔠 OCR / Character Project | 28×28 int8 glyph | one character `0–9, A–Z`, plus `_background` rejector | `TFLite_OCR/` |
| **🚗 Car License Project (3-Char)** (`sign_ocr`) | 🚗 Car License Project | 3 slots × 28×28 int8 glyphs | binary verdict, `🟢 YES — MATCH · Plate #k (B18)` / `🔴 NO — NOT MATCH`, against **one or more** allowed plates | `TFLite_Sign/` |

### Image Project (`image`)

- 96×96 default model input (48 / 96 / 128 / 192 selectable in the worktable;
  the training input follows the project `Image Size`).
- Legacy 96×96 sign pipeline: crop → bilinear → BT.601 → contrast stretch →
  int8 `gray − 128`.
- OOD gates in Settings → Preview: `sign_pct` min/max, max probability, and
  normalized entropy.  The device uses the same raw-int8 math.
- Export stamps `model_settings.h` with the threshold window and the four OOD
  gates; the firmware is `TFLite/TFLite.ino`.

### OCR / Character Project (`ocr`)

- 28×28 single-glyph model; characters `0–9`, `A–Z`, auto-deduplicated
  case-insensitively.
- Integer-exact glyph front-end: Otsu binarisation + vertical-projection crop
  (the device `ocr_glyph.c` mirrors `ocr_preprocess.py` bit-for-bit).
- `_background` is appended by the OCR trainer as the reject class; the
  device rejects by **name**, not index.
- EMNIST mixing and optional two-stage transfer learning (see §3).
- Export route: `TFLite_OCR/TFLite_OCR.ino` (compile-time `TM_OCR_GLYPH_MODEL 1`).

### 🚗 Car License Project (3-Char) (`sign_ocr`)

- The class list is the **union of every allowed plate's characters**, in
  first-appearance order: `["A12", "B18"]` means exactly the classes
  `A, 1, 2, B, 8` (a character shared by two plates is trained once).  Stale
  `Class 1`/`Class 2` folders are pruned from the dataset, processed cache
  and mixed set on open/sync/train, so they can never become model labels.
- **🪪 License List — the gatekeeper whitelist.**  The `🪪 License List`
  button (`#navLicenseList`, with a plate-count badge) sits next to
  `⚙ Settings`; the same modal opens from the `+ Plates (N)` chip in
  `#signSeqBar`.  Add, edit, remove, reorder (⭐ Set as Primary) and save
  **one or more** 3-character plates; `sign_sequences[0]` is the primary and
  is kept in lock-step with the `#signSeqBar` target.  Saving runs
  `/classes/sync_ocr`, which rewrites the class list to the new union and (with
  EMNIST Auto-Fill on) seeds 24 dark-on-white EMNIST samples for every newly
  introduced character that has zero samples.
- Declare the primary ordered target in `#signSeqBar` (e.g. `A ▸ 1 ▸ 2`).
- **Reading Direction**: `Left to Right (L ➔ R)` or `Right to Left (R ➔ L)`.
  RTL reverses the three split boxes after the split; pixels are never
  mirrored.
- **Duplicate character deduplication**: `A-1-A` keeps two class cards (`A`,
  `1`), with the `A` card badged `Slots 1 & 3`.  The sequence still has three
  slots; the class is trained once.
- Training data is **one character glyph per class** (the same 28×28 glyph
  model an OCR project trains).  **Auto mode in a sign class card is
  slot-aware**: a whole `A 1 2` board is split and only that class's slot is
  kept (`A` → slot 1, `1` → slot 2, `2` → slot 3), so classes are never
  trained on the whole sign.  Manual ROI / Auto single-glyph crops strip a
  small disconnected **corner speck** (e.g. a clipped neighbour foot) before
  the 28×28 centring; the Edit modal's Processed Preview is that exact
  canonical 28×28 glyph.  EMNIST Auto-Fill supplies clean glyphs directly.
- **Resolution auto-scaling.**  Camera captures may be larger than the model
  input.  The export stamps three independent sides into
  `model_settings.h` (`TM_CAPTURE_FRAME_SIDE`, `TM_PREPROCESS_FRAME_SIDE`,
  `TM_MODEL_INPUT_SIDE` / `kNumCols`); with the default 160×160 capture the
  P4 box-averages 160→96 before the sign split and 96→28 for each glyph
  canvas.  The desktop live preview applies the same integer box average to
  the 160×160 serial frame, so host and MCU see the same pixels.  The
  compact **< 150 KB INT8** backbone and its 320 KB arena fit in fast
  internal SRAM (PSRAM fallback only if allocation fails).
- Preview first locates the sign ROI on the **preprocessed** frame (96×96,
  box-averaged from a 160×160 device capture exactly like the MCU; an
  upload/synth image keeps its own size): Manual ROI when pinned; when
  Dark/Lum are tuned, an ink-cluster of the three interior character blobs
  (frame-scale lines and corner debris are dropped, but a character near the
  plate/frame edge is kept); otherwise the bright inner plate.
  The ROI is the cluster union plus one uniform 10 % pad (tight and
  symmetric), clamped to the white plate interior so the dark surround /
  holder border cannot enter the crop.  The split runs on that exact ROI —
  there is **no second border-cleaning pass**, so `'A'`, both strokes of a
  broken `'1'`, and `'2'` survive.  The response carries `roi_box` plus a
  real `sign_pct`.
- **ROI OFF** shows the full frame (thresholded by default, raw with Orig)
  and draws the rectangle; **ROI ON** shows the **exact pixel slice inside
  that same rectangle** (same threshold, no re-clean) and hides the
  rectangle.  **Orig OFF (default)** is the Dark/Lum threshold
  view (ink black, everything else pure white); **Orig ON** is the original
  grayscale.  The Edit modal's Processed Preview is black-on-white cropped
  to the ROI, as are the three `.signmatch-slot` thumbnails; no slot strip
  is glued onto the stage.
- Each slot crop gets an 8 % (min 2 px) plate-context margin before glyph
  normalization, so a stroke-only `1` does not collapse to a nearly-black
  tensor and read as `_background`.  Under `NORM_CANON` a narrow glyph
  (ink bbox `w/h < 0.35`) is also widened to a 3-4 px stroke **before** the
  nearest-neighbour fit, matching the EMNIST `1` training samples; the C
  mirror applies the same margin and widening.  The reject class is built
  from textured whole patches (plus a 10 % all-zero minority) and never
  from centered stroke-like bars.
- The Preview stage is a 1:1 square, so a square device frame fills it
  without side letterboxing; webcam frames for glyph projects are
  center-cropped to 1:1 on ingest.  The Edit modal is centred with 1:1
  Original/Processed stages.
- Inference splits the sign into three boxes, runs the same 28×28 glyph
  classifier three times, and requires **all three** slots to match **one**
  allowed plate and clear the confidence floor before returning `YES`.
  Preview compares the reading against every plate in `sign_sequences` and
  names the winner (`🟢 YES — MATCH · Plate #2 (B18)`); a near miss reports
  the closest plate, and a reading that is no whitelisted plate is `NO`.
  A slot whose top class **is** that plate's target character and whose
  probability clears the match floor is accepted even when `entropy_ratio`
  is the only OOD gate that fired — a correct, merely uncertain reading is
  not a rejection.  Layer 1 (`sign_pct`) is **disarmed on the 3-slot path**
  (it is a diagnostic there); `max_prob` still rejects, and the Preview
  banner names the exact gate.  **`OOD MAX PROB` in Input Settings is the
  single probability threshold** (default **0.40**): it gates `max_prob` and
  is also the per-slot match floor shown in the banner (`each at N% or
  better`).  There is no separate hidden floor.  Export stamps the same
  value into both `kOodMaxProbMin` and `kSignMatchMinProb`; Serial
  `set ood spmin spmax mpmin ermax` updates both at runtime.  The OOD
  entropy default is **0.85**, and stale browser caches of the old
  0.60 / 0.70 defaults migrate once to 0.40 / 0.85 (custom values are
  preserved).
  The `🧪 Synth` pills draw the plate selected in the `Plate #k (…)`
  dropdown next to them (the primary by default), so every allowed car can
  be tested without a photo.
- The MCU mirrors the host search in `sign_glyph.c`: binarise the frame,
  drop the 6 % border margin / huge frame components / specks, select the
  character band, pad it symmetrically to a `roi_box` (and compute
  `sign_pct`), then split.  The same integer narrow-stroke widening and
  border-speck stripping are applied per slot.  `sign_feed_slots()` reads
  **Dark/Lum at runtime** via `ImageProviderGetMaskThresholds()`, so the
  exported model_settings window and Serial `set mask D L` both drive the
  slicer immediately.  `sign_glyph_host.c` + `tests/test_sign_glyph_parity.py`
  pin Python↔C bit-for-bit.
- **Settings → Advanced → Use `_background` noise class to replace OOD
  rejection** defaults **OFF** for Car License: the model trains on the
  target characters only and Preview/device reject blank or unknown slots
  with OOD Layers 2/3 (`max_prob`, entropy) plus the per-slot match floor;
  Layer 1 `sign_pct` is disarmed on this path.  ON trains the extra
  `_background` class.  The export carries `TM_SIGN_USE_BACKGROUND_OOD`,
  `kSignBackgroundReplacesOod` and `kBackgroundClassIndex` (-1 when absent);
  the legacy `kSignBackgroundIsTheOnlyRejector` spelling is emitted too.
- The OCR/sign glyph head is compact (`conv -> 1x1 Conv2D(16) -> Flatten ->
  Dense(128) -> softmax`), so the checked-in `TFLite_Sign/tm.tflite` is
  **< 150 KB** (~125 KB) and the device's first-choice tensor arena is 320 KB
  internal SRAM.  `TFLite_Sign.ino` prints the model's operator codes before
  allocation, names every resolver/arena/allocation failure as `FATAL:` on
  USB Serial, and retries once with an 8 MB PSRAM arena only if the compact
  arena cannot allocate.
- Export route: `TFLite_Sign/TFLite_Sign.ino` (compile-time
  `TM_SIGN_OCR_MODEL 1`, `TM_OCR_GLYPH_MODEL 0`).  The export also carries
  the resolution sides (`TM_CAPTURE_FRAME_SIDE`,
  `TM_PREPROCESS_FRAME_SIDE`, `TM_MODEL_INPUT_SIDE`) and the full whitelist:
  `TM_SIGN_MULTI_TARGET_EXPORTED`, `kTargetSignCount`,
  `kTargetSignSequences[kTargetSignCount][4]`,
  `kTargetSignClassMatrix[kTargetSignCount][3]`, `kSignMatchMinProb`, plus
  `TM_SIGN_USE_BACKGROUND_OOD` / `kSignBackgroundReplacesOod` /
  `kBackgroundClassIndex`.

Each sketch `#error`s if handed another project's export, so a wrong-folder
export is a build error rather than a silent preprocessing mismatch.

---

## 2. Firmware matrix

| Folder | Sketch | Board | Partition need | Reads |
|---|---|---|---|---|
| `TFLite/` | legacy 96×96 inference | ESP32P4 Dev Module | APP slot ≥ ~3.6 MB (32 MB scheme or custom table) | `TM_OCR_GLYPH_MODEL 0`, `TM_SIGN_OCR_MODEL 0` |
| `TFLite_OCR/` | single-glyph reader | ESP32P4 Dev Module | `huge_app` (≈1.2 MB app) | `TM_OCR_GLYPH_MODEL 1`, `TM_SIGN_OCR_MODEL 0` |
| `TFLite_Sign/` | 3-character sign reader | ESP32P4 Dev Module | `huge_app` (≈1.2 MB app) | `TM_SIGN_OCR_MODEL 1` |
| `S3_UART_Receiver/` | ESP32-S3 UART receiver / junction controller | ESP32S3 Dev Module | — | 9-byte P4 packets |

`image_provider.cpp/.h`, `frame_resize.h`, `main_functions.h` and
`arduino_main.cpp` are byte-identical across the three P4 sketch folders;
`TFLite_OCR/tools/sync_check.py` enforces it.

All three sketches read their resolution layers from the export:
`g_frame_side = kCaptureFrameSide` is the **camera/serial** side (e.g. 160)
and is deliberately independent of `kNumCols` (96 for the image project, 28
for a glyph canvas).  `frame_resize.h` provides the exact integer
box-average (`FrameResizeBoxAverageGray/Rgb`) used by the firmware and
mirrored by `AItraining/tests/test_frame_resize_parity.py`.

---

## 3. Features and workflows

### Data retention, ROI and thresholds

- Samples are stored at the full captured resolution; the model input is
  cropped and resized at train/eval time.  Use **Edit** on a class card to
  open the Sample Worktable.
- **Auto (find sign)** uses the exposure-adaptive shadow-search box; in the
  glyph/Car-License Edit worktable it instead finds the tight interior
  character band from the current Dark/Lum window (6 % border margin,
  frame/speck cleanup, 10 % symmetric pad) and redraws box + Processed crop
  live.  **Manual ROI** pins a fixed crop (`ROI X1/Y1/X2/Y2`, 0–1
  normalized).  Both are available per class, and the device mirrors the same
  geometry.
- **Dark Thr** (0–100) and **Lum Thr** (50–255) drive the G-channel mask and
  the Preview/openness statistics.  Defaults 0/100 are a bit-exact Otsu
  no-op on the device.
- The Preview **ROI** checkbox draws the crop rectangle on the full scene;
  **Orig** switches to the full masked frame; both together show the
  processed crop / sign split (see the AItraining README for the matrix).

### Zero-hardware quick start

- **EMNIST Auto-Fill**: `⚙ Settings ➔ Advanced Settings ➔ Developer: EMNIST
  template samples ➔ Auto-fill empty classes with EMNIST template samples
  (24 / class)`.  Every character class with zero on-disk PNGs is filled with
  24 dark-ink-on-light 96×96 template captures; classes with real captures
  are never touched.  The one-shot **Fill Empty Classes with EMNIST Samples
  Now** button does the same without turning the checkbox on.
- **3-Character Composite Sign Synthesizer** (sign projects, Preview →
  Upload source): `🧪 Synth Match`, `🔀 Synth Reversed`, `❌ Synth Mismatch`.
  The backend composes the current target characters into a sign plate and
  runs `_preview_predict_sign_ocr` immediately — no road-sign photo needed.
  The drawn plate is retried until it actually produces the intended verdict.

### Preview input sources

`⚙ Settings ➔ Preview ➔ Preview Image Input Source`:

- **Webcam** — local camera.
- **Device (MCU)** — ESP32-P4 over serial (port + Refresh in
  `#previewContextBar`).
- **Local Image File** — Upload button, drag-and-drop, or the synth pills.

The card's context bar shows only the control the active source needs.

### Project isolation, UI and shortcuts

- `.tmproj` archives carry a project-type header.  A file opened from any
  Start-Page card is restored as the type its manifest records; a mismatch
  is announced rather than silently switching workspaces.  Classes, samples,
  preprocessing, training config and the latest run travel with the archive.
- System dark mode follows `prefers-color-scheme: dark` in both the
  Streamlit shell and the workspace SPA.
- Global keyboard shortcuts: `Esc` priority stack (dropdown → worktable →
  settings → preferences → character/glyph modals → source drawer); `Space`
  taps one capture or holds for burst capture while a Webcam/Device drawer
  is open; `⌘/Ctrl+S` save, `⇧⌘/Ctrl+S` Save As, `⌘/Ctrl+O` open,
  `⌘/Ctrl+N` new, `⌘/Ctrl+E` export, `⌘/Ctrl+Enter` train, `⌘/Ctrl+,`
  settings.
- **📖 Tutorial** in the top toolbar launches a state-aware, per-project
  walkthrough (image, OCR and sign each have their own curriculum).

---

## 4. Model pipelines

### Image project — 96×96

```
Camera RGB at kCaptureFrameSide (e.g. 160×160)
→ box-average down to IMG_SIZE (96×96×3, exact integer area average)
→ crop (auto search box, or center 60 % legacy)
→ bilinear resize (float32, PIL-style center mapping)
→ BT.601 luminance (30/59/11, round half up)
→ contrast stretch (span ≥ 24 → full [0,255])
→ int8 (gray − 128)
```

- The capture side and model side are independent: `g_frame_side` comes from
  `TM_CAPTURE_FRAME_SIDE`, while `GetImage()` always emits `kNumCols`
  (`IMG_SIZE`).  `160 → 96` is automatic; a 96×96 capture is an identity
  resize.
- Training data = `IMX219_Grayscale_Serial` (`BT.601` of raw demosaiced RGB,
  **no white balance**); the training cache is the canonical transform.
- Auto search box: deterministic float32 cumsum blur + float64 geometry,
  bit-identical between `image_preprocess._focus_bbox()` and the device
  `find_search_box()` (verified 65/65 boxes, 20/20 end-to-end frames).
- Legacy `crop_mode="center"` / `BG_ENABLE_FOCUS_SEARCH=0` keeps the center
  60 % crop (`side = floor(96*0.60) = 57`, box `[20,77)`).  Retrain after
  switching crop modes.

### OCR and sign projects — 28×28 glyphs

```
Camera gray at kCaptureFrameSide (e.g. 160×160)
→ box-average down to kPreprocessFrameSide (96×96)
→ ROI crop → integer-exact glyph canvas:
Otsu (integer) → vertical-projection split/segment → 28×28 canvas
→ int8 tensor
```

- Both glyph sketches box-average the captured frame to
  `kPreprocessFrameSide` before running `ocr_glyph.c` / `sign_glyph.c`, so a
  160×160 camera behaves like the 96×96 frames the models/previews were
  tuned on.  The camera serial stream itself still carries the full capture
  resolution for data collection.
- `sign_ocr` first splits the sign into three boxes with the same integer
  Otsu/projection rules, then runs the glyph model once per slot.
- RTL reading order reverses the box list after the split — never the pixels.
- The sign verdict is `YES` only when all three slots name **the same
  allowed plate**'s characters, none is `_background`, each clears the
  single `OOD MAX PROB` threshold (default 0.40; `kSignMatchMinProb` and
  `kOodMaxProbMin` are exported from that one value), and each slot contains
  ink.  The device additionally applies OOD layers 2/3 per slot.  Every row
  of `kTargetSignClassMatrix` is checked, so any whitelisted plate can win.

---

## 5. Training

- Standard knobs (Training → Advanced): Image Size, Batch Size, Epochs,
  Validation Split, Learning Rate, Conv1/Conv2 filters, Dense units; plus
  global Ink Polarity and — for sign projects — Sign Reading Direction.
- `💡 Use recommended` applies dataset-aware Batch Size, Epochs, Learning
  Rate and Validation Split.  The recommendation targets ~800 weight
  updates with at least 4 steps/epoch; it never changes Image Size or the
  architecture.
- **EMNIST mixing** (`ocr`/`sign_ocr`): synthesises per-class EMNIST glyphs
  (default 200/class, split `balanced`), polarity-aligned and isolated like
  camera captures, plus an optional `_background` rejector (random crops).
- **Two-stage transfer learning** (default on): a cached EMNIST-pretrained
  backbone is warmed for `Pre-train epochs` (4) with `Pre-train samples /
  class` (1200); the new head trains with the backbone frozen for `Head
  epochs` (12) at `Head learning rate` (0.0015); then the last
  `Conv blocks to unfreeze` (1) are fine-tuned for `Fine-tune epochs` (25)
  at `Fine-tune learning rate` (0.0002).  Turning transfer learning off
  trains the Conv1/Conv2/Dense stack from scratch.

---

## 6. Serial protocols

**P4 → S3 (9 bytes):**

```
0xAA 0x55 0x01 | frame_id (LE16) | label_id | confidence | flags | XOR(0..7)
```

**S3 → P4 (5 bytes):**

```
0xAA 0x55 0x02 | 0x01 ACK_STOP / 0x02 RESUME_JUNCTION | XOR(0..3)
```

**Sign verdict encoding** (`TFLite_Sign`, multi-plate):

- `confidence` = the weakest slot's best raw score (0–255).
- `flags` low nibble `0x02` = `sign_ready`; high nibble is the verdict:

  | `flags` | Verdict | Byte 5 (`label_id`) |
  |---|---|---|
  | `0x02` | **MATCH** — all three slots matched one allowed plate | matched **plate index** `0 … kTargetSignCount-1` |
  | `0xE2` | **MISMATCH** — a sign is present but no allowed plate matched | `0xFE` |
  | `0xF2` | **NO SIGN** — frame has no ink | `0xFF` |

- On a MATCH the S3 resolves the index through its copy of
  `kTargetSignSequences` (exported with the model) and can log/act on the
  **specific car** (`label=1(B18)`); MISMATCH never triggers ACK_STOP.
  Legacy P4 sketches that still send a class index continue to resolve
  through `kCategoryLabels`.
- The S3 keeps the ACK_STOP → 3 s task → RESUME_JUNCTION cycle unchanged;
  a plate change resets the consecutive-frame counter.

**Data collection:** `0xAA 0x55 0xAA` + `CAPTURE_SIDE²×3` RGB for
`IMX219_RGB_Serial` (native capture side, WB applied per pixel) and
`0xAA 0x55 0xAA` + `CAPTURE_SIDE²` GRAY8 for `IMX219_Grayscale_Serial`.
The host's `SerialFrameReader` must use the same **Image Size** configured in
`⚙ Settings` (e.g. 160×160).

---

## 7. Export files

For every project type the export writes, into the folder you pick:

- `<export name>.tflite`
- `<export name>_model_data.h` / `<export name>_model_data.cpp`
- `model_settings.h` / `model_settings.cpp`
- `model_resolver.h`
- `labels.txt`

The **5 files to copy next to the sketch** are the two `_model_data.*` files,
`model_resolver.h`, and `model_settings.h/.cpp`.  The default export name is
`tm`, so a sign export produces `tm_model_data.*`; the C array symbol defaults
to `g_tm_model_data`.

Every export also stamps the resolution layers:

```c
#define TM_MODEL_INPUT_SIDE 28        // kNumCols (28 glyph, 96 image)
#define TM_PREPROCESS_FRAME_SIDE 96   // working frame before the glyph/ROI crop
#define TM_CAPTURE_FRAME_SIDE 160     // camera / serial side from ⚙ Settings
constexpr int kModelInputSide = TM_MODEL_INPUT_SIDE;
constexpr int kPreprocessFrameSide = TM_PREPROCESS_FRAME_SIDE;
constexpr int kCaptureFrameSide = TM_CAPTURE_FRAME_SIDE;
```

`model_settings.h` for `sign_ocr` additionally contains:

```c
#define TM_OCR_GLYPH_MODEL 0
#define TM_SIGN_OCR_MODEL 1

constexpr char kTargetSignSequence[] = "A12";       // primary, legacy-compatible
constexpr int kTargetSignClassIndices[3] = { 0, 1, 2 };
#define TM_SIGN_MULTI_TARGET_EXPORTED 1
constexpr int kTargetSignCount = 2;
constexpr char kTargetSignSequences[kTargetSignCount][4] = { "A12", "B18" };
constexpr int kTargetSignClassMatrix[kTargetSignCount][3] = {
    { 0, 1, 2 },
    { 3, 1, 4 },
};
constexpr int kSignReadingOrder = 0;      // 0 = LTR, 1 = RTL
constexpr float kSignMatchMinProb = 0.400000f;  // == kOodMaxProbMin
#define TM_SIGN_USE_BACKGROUND_OOD 0
constexpr bool kSignBackgroundReplacesOod = false;
constexpr int kBackgroundClassIndex = -1;  // -1 = no _background class
```

plus `kNumCols/kNumRows` (28), `kDarkThr/kLumThr`, and the four OOD gates.
Copy the same `model_settings.h/.cpp` next to `S3_UART_Receiver.ino` as well
if the S3 should name plates from the UART index; its checked-in copy is the
current sign export.

Every export also copies the run meta to `<workspace>/deployed.json`; Preview
then keeps validating the exported model rather than a later training run.

---

## 8. Build & run

```bash
# AItraining desktop app
cd AItraining
python3.11 -m venv .venv && source .venv/bin/activate
pip install -r requirements.txt
python desktop_launcher.py

# Or run Streamlit directly
streamlit run app.py
```

Firmware (Arduino IDE):

- Image: open `TFLite/TFLite.ino`, board **ESP32P4 Dev Module**, 32 MB
  partition scheme with a ≥3.6 MB APP slot.
- OCR: open `TFLite_OCR/TFLite_OCR.ino`, board **ESP32P4 Dev Module**,
  `huge_app`.
- Sign: open `TFLite_Sign/TFLite_Sign.ino`, board **ESP32P4 Dev Module**,
  `huge_app`.
- Receiver: open `S3_UART_Receiver/S3_UART_Receiver.ino`, board **ESP32S3
  Dev Module**; debug serial 921600 baud (USB CDC), like the P4 sketch.

After exporting, copy the 5 source files into the matching sketch folder and
rebuild; the sketch refuses another project's export at compile time.

---

## 9. Repository layout

```
AItraining/                  Streamlit app + hand-written SPA + trainer
TFLite/                      legacy 96×96 P4 inference sketch
TFLite_OCR/                  single-glyph OCR sketch + parity tooling
TFLite_Sign/                 3-character sign sketch + sign_glyph.c
S3_UART_Receiver/            ESP32-S3 UART receiver / junction controller
arduino/                     ESP32-P4 Arduino core package
ESP32-P4-IMX219-PoC/         camera firmware (git submodule)
```

`AItraining/`, `ESP32-P4-IMX219-PoC/` and `TFLite/` are git submodules;
commit inside them first, then bump the root pointer.

---

## 10. Verification

```bash
# Host/device parity and integration
python3 AItraining/tests/test_sign_glyph_parity.py
python3 AItraining/tests/test_frame_resize_parity.py
python3 TFLite_OCR/tools/wiring_check.py --sign
python3 TFLite_OCR/tools/sync_check.py
python3 TFLite_OCR/tools/ino_build_check.py --config <arduino-cli.yaml>

# App regressions (headless Chrome + backend HTTP)
cd AItraining
.venv/bin/python tests/test_sign_ocr_spa.py
.venv/bin/python tests/test_multi_plate.py
.venv/bin/python tests/test_sign_sequence_autofill.py
.venv/bin/python tests/test_sign_synth.py
.venv/bin/python tests/test_emnist_seeder.py
```

See `CLAUDE.md` for the deep invariants (bit-identical crop/bilinear math,
sign split tie rules, OOD device math, threshold-window contract) and
`TFLite_Sign/README.md` for the firmware contract.
