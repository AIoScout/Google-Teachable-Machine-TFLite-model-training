/* OCR glyph preprocessing — the C mirror of AItraining/ocr_preprocess.py.
 *
 * INTEGER-EXACT.  Every decision this file makes (the Otsu threshold, the
 * ink predicate, the segmentation bounds, the resampling index) is computed
 * with integers only — no float, no double, no __int128.  That is not a
 * style preference: each of those is a *decision* whose outcome flips when
 * the last mantissa bit differs between the host that built the training set
 * and the ESP32-P4 that runs the model, and a flipped decision does not move
 * an output pixel by one count — it changes how many glyphs were found, or
 * which source pixel was sampled, and the two sides diverge from there.
 * Measured on a 386-image adversarial corpus, a hand-mirrored float
 * implementation of this pipeline (numpy float64 Otsu + PIL NEAREST)
 * disagreed with the reference on 228/386 = 59.1 % of tensors; this
 * integer-exact form disagrees on 0/386.
 *
 * The Python module is the reference.  If the two ever disagree, the Python
 * one is right and this one is a bug — EXCEPT for the docstring/code
 * mismatch noted against ocr_binarize_ink() below, where the Python *code*
 * is authoritative and its docstring is stale.
 *
 * Pipeline:
 *   RGB -> integer BT.601 luminance -> exact integer Otsu -> polarity-aware
 *   ink -> (optional) vertical-projection segmentation -> union box ->
 *   normalise (STRETCH or CANON) -> uint8 glyph -> optional int8 quantise.
 *
 * No dynamic allocation and no hidden state: every buffer is caller-supplied
 * (see OcrScratch) so the same code runs on the device, in a host test
 * harness, and under a debugger without a heap.
 */
#ifndef OCR_GLYPH_H_
#define OCR_GLYPH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Geometry ──────────────────────────────────────────────────────────── */
#define OCR_GLYPH_SIZE       28   /* model input side (both axes)          */
#define OCR_GLYPH_CANON_FIT  20   /* MNIST/EMNIST longer-side fit target   */
#define OCR_MIN_GLYPH_RUN     2   /* minimum kept column-run width         */

/* Frame-size bounds.  Two independent limits, because they protect two
 * different things:
 *   OCR_MAX_PIXELS  bounds w*h.  This is what keeps Otsu's cross-multiplied
 *                   comparison inside its 160-bit accumulator (derivation
 *                   in ocr_glyph.c) and the per-pixel scratch bounded.
 *   OCR_MAX_W       bounds w alone, because the projection scratch is one
 *                   int32 PER COLUMN — a 65536x1 frame is legal by pixel
 *                   count but would need a 256 KB proj array.
 * The library frame (96x96) has a 7x margin on the first and 10x on the
 * second, so neither is a constraint on the device; they exist so an
 * out-of-envelope frame is refused instead of overrunning a buffer. */
#define OCR_MAX_PIXELS  65536
#define OCR_MAX_W       1024

/* ── Vocabulary ────────────────────────────────────────────────────────── */
/* Otsu splits the histogram at t: the ink predicate asks WHICH side of that
 * split is the glyph, it never rewrites emitted intensities. */
typedef enum {
  OCR_INK_DARK   = 0,  /* dark glyph on a bright page (camera captures)  */
  OCR_INK_BRIGHT = 1   /* bright glyph on a dark page (EMNIST as shipped) */
} OcrPolarity;

/* STRETCH: bbox stretched onto the full grid, one scale factor per axis.
 * CANON:   MNIST/EMNIST convention — longer side to 20 with the aspect ratio
 *          preserved, pasted centred, then shifted so the intensity centre
 *          of mass sits at ((out-1)/2, (out-1)/2).  This is the convention
 *          the OCR project trains with (see ocr_trainer._project_convention)
 *          and the only one that can be applied to EMNIST as distributed. */
typedef enum {
  OCR_NORM_STRETCH = 0,
  OCR_NORM_CANON   = 1
} OcrConvention;

/* Exclusive-ended box, the same convention as ocr_preprocess's tuples. */
typedef struct {
  int x1, y1, x2, y2;
} OcrBox;

typedef struct {
  int     thresh;     /* threshold used; 0 = no valid split existed       */
  int     num_boxes;  /* column runs kept by the segmenter                */
  bool    has_box;    /* box is meaningful                                */
  bool    ok;         /* a glyph tensor was produced (Python's `ok`)      */
  bool    empty;      /* Python's `empty`; equals !ok on every path       */
  OcrBox  box;        /* union of the kept runs (Python's segment box)    */
} OcrGlyphInfo;

/* Caller-owned scratch.  Sizes are bytes except `proj`, which is ints. */
typedef struct {
  uint8_t *gray;    /* >= w*h          luminance / input grayscale       */
  uint8_t *ink;     /* >= w*h          0/1 ink mask                      */
  int32_t *proj;    /* >= w            per-column ink count              */
  uint8_t *canvas;  /* >= out_size^2   normalisation canvas              */
  uint8_t *canvas2; /* >= out_size^2   translate target for CANON        */
} OcrScratch;

/* Bytes needed for one contiguous scratch block serving w x h frames at
 * out_size, so a caller can do a single malloc/heap_caps_alloc. */
size_t ocr_scratch_bytes(int w, int h, int out_size);

/* Split one contiguous scratch block (from ocr_scratch_bytes) into its
 * parts.  Returns false when the block is too small. */
bool ocr_scratch_init(OcrScratch *scratch, void *block, size_t block_bytes,
                      int w, int h, int out_size);

/* ── Individual stages (exposed for tests and for reuse) ───────────────── */

/* lum = (r*30 + g*59 + b*11 + 50) / 100, clipped to [0,255].  Integer only.
 * `rgb` holds n_pixels*3 bytes; `gray` receives n_pixels. */
void ocr_rgb_to_lum_u8(const uint8_t *rgb, size_t n_pixels, uint8_t *gray);

/* Exact integer Otsu.  t in [0,255]; 0 means "no valid split existed"
 * (a flat frame) — every candidate split had an empty class.
 * Tie rule: strictly-greater score, so the FIRST maximiser wins. */
int ocr_otsu_threshold_u8(const uint8_t *gray, size_t n);

/* ink[x] = (gray[x] <= t) for OCR_INK_DARK, (gray[x] > t) for INK_BRIGHT.
 * NOTE the asymmetry: Python's otsu_threshold_u8 docstring claims the
 * predicate is the exclusive `v < t` / `v >= t`, but the code that actually
 * built every trained model is ocr_preprocess.binarize_ink, which uses
 * `<=` / `>`.  This function mirrors the CODE.  Do not "fix" either side to
 * match the other's comment: changing this predicate would silently
 * invalidate every existing model. */
void ocr_binarize_ink(const uint8_t *gray, size_t n, int thresh,
                      OcrPolarity polarity, uint8_t *ink);

/* Vertical-projection segmentation over a w*h 0/1 ink mask.
 * Fills *num_boxes with the count of kept column runs and *box with their
 * union (Python's segment_glyph_box).  Returns false when no run survives —
 * *box is then untouched.  `proj` is w int32 of scratch. */
bool ocr_segment_glyph_box(const uint8_t *ink, int w, int h, int min_run,
                           int32_t *proj, int *num_boxes, OcrBox *box);

/* Normalise an already-isolated grayscale crop (w*h, row-major) into an
 * out_size x out_size canvas.  Both conventions; exact rational NN resize. */
bool ocr_normalise_glyph(const uint8_t *crop, int w, int h, int out_size,
                         OcrConvention convention, uint8_t *canvas,
                         uint8_t *canvas2);

/* ── The pipeline ──────────────────────────────────────────────────────── */

/* Grayscale input.  `thresh` < 0 means "use Otsu"; >= 0 pins the threshold,
 * and 0 is a legal pinned value (it is NOT the same as -1: the Otsu path
 * treats t == 0 as "no split, no ink", while a pinned 0 runs the normal
 * predicate against 0).
 *
 * Never raises and always leaves glyph_out fully written (zeros on failure),
 * mirroring preprocess_glyph's contract.  Returns info->ok. */
bool ocr_preprocess_gray(const uint8_t *gray_in, int w, int h, int out_size,
                         OcrPolarity polarity, bool apply_segmentation,
                         int min_run, int thresh, OcrConvention convention,
                         OcrScratch *scratch, uint8_t *glyph_out,
                         OcrGlyphInfo *info);

/* RGB input: identical, with `ocr_rgb_to_lum_u8` as the first stage.  Use
 * this — not the library's float32 grayscale — when the training captures
 * came from the RGB stream, so both sides run the same integer expression. */
bool ocr_preprocess_rgb(const uint8_t *rgb, int w, int h, int out_size,
                        OcrPolarity polarity, bool apply_segmentation,
                        int min_run, int thresh, OcrConvention convention,
                        OcrScratch *scratch, uint8_t *glyph_out,
                        OcrGlyphInfo *info);

/* Model input: out[i] = (int8_t)(glyph[i] - 128).
 * 128 is the training pipeline's zero point (the representative dataset is
 * glyph/255.0 float, and full-int8 PTQ turns that into scale 1/255,
 * zero_point -128), so a uint8 glyph in [0,255] maps to [-128,127] with no
 * clamp needed. */
void ocr_glyph_to_int8(const uint8_t *glyph, size_t n, int8_t *out);

#ifdef __cplusplus
}
#endif

#endif /* OCR_GLYPH_H_ */
