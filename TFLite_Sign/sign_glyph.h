/* Sign-sequence glyph front-end — C mirror of AItraining/ocr_preprocess.py.
 *
 * This is the TFLite_Sign sibling of TFLite_OCR/ocr_glyph.{c,h}.  The
 * single-glyph pipeline is the SAME transform (one shared implementation of
 * Otsu, the ink predicate, the projection segmenter and NORM_CANON), because
 * a road sign is read by feeding the single-character model one crop per
 * character.  What is new here is the stage between the frame and the crops:
 *
 *     split_sign_glyph_boxes_c()      Python: split_sign_glyph_boxes()
 *     preprocess_sign_sequence_c()    Python: preprocess_sign_sequence()
 *
 * Both must be bit-identical to the Python side, not merely similar: the
 * model was trained on glyphs the Python transform produced, so a one-pixel
 * difference in a box or a one-LSB difference in a resampled value moves the
 * device off the training distribution.  Every decision below is integer
 * arithmetic for exactly that reason — there is no float anywhere in the
 * pipeline, and the Otsu score uses an explicit 160-bit accumulator because
 * the cross-multiplied comparison does not fit in 64 bits and riscv32-esp-elf
 * has no __int128.
 *
 * THRESHOLD WINDOW (the editor's Dark Thr / Lum Thr sliders)
 * ----------------------------------------------------------
 * The Python editor expresses the sign's luminance window as TWO numbers
 * passed into the transform:
 *
 *     thresh   = lum - 1     (pin the binarisation threshold)
 *     dark_thr = dark + 1    (drop every pixel at or below the floor)
 *
 * with both sliders at their defaults (0 and 100) meaning "untouched": the
 * caller then passes thresh = -1 here (exact Otsu) and dark_thr = 0 (no
 * floor), which is BIT-IDENTICAL to the pre-existing pipeline.  That is a
 * hard compatibility rule, not a convenience: every model already in the
 * field was trained through the default path.
 *
 * CONVENTIONS
 * -----------
 *   polarity      SIGN_INK_DARK for camera captures, SIGN_INK_BRIGHT for the
 *                 EMNIST-side samples.  The comparison operator is part of the
 *                 contract (see sign_binarize_ink).
 *   convention    SIGN_NORM_CANON is what an OCR/sign project trains in.
 *                 SIGN_NORM_STRETCH exists because the corpus records both.
 *   reading order 0 = left-to-right, 1 = right-to-left.  Applied by REVERSING
 *                 the box list AFTER the split, never by mirroring pixels: a
 *                 mirrored glyph is a different glyph, not a different order.
 *
 * ZERO-HEAP CONTRACT
 * ------------------
 * The device allocates one scratch block at boot (sign_scratch_bytes) and
 * every function below works inside views of it.  Nothing here calls malloc,
 * and nothing here uses a static mutable buffer, so two frames can be run
 * from two tasks without sharing hidden state.
 */
#ifndef SIGN_GLYPH_H_
#define SIGN_GLYPH_H_

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/* sign_glyph.c is C (the .ino includes this header from C++), so the
 * definitions need C linkage or every call from the sketch is an undefined
 * reference to a mangled name. */
#ifdef __cplusplus
extern "C" {
#endif

/* Canvas the model was trained on (AItraining ocr_preprocess.GLYPH_SIZE). */
#define SIGN_GLYPH_SIZE 28
/* MNIST/EMNIST convention: the longer side is scaled to this, then centred. */
#define SIGN_GLYPH_CANON_FIT 20
/* Projection runs narrower than this are noise (MIN_GLYPH_RUN). */
#define SIGN_MIN_GLYPH_RUN 2
/* Characters on a road-sign label.  kTargetSignSequence is this long + NUL. */
#define SIGN_SEQUENCE_LEN 3

/* Frame envelope.  The 160-bit Otsu proof is sized for OCR_MAX_PIXELS, so a
 * frame outside these bounds is refused rather than risking an overflow. */
#define SIGN_MAX_PIXELS 65536
#define SIGN_MAX_W 1024

typedef enum {
  SIGN_INK_DARK = 0,    /* low class is ink (camera captures)   */
  SIGN_INK_BRIGHT = 1   /* high class is ink (EMNIST-style)     */
} SignPolarity;

typedef enum {
  SIGN_NORM_STRETCH = 0, /* bbox stretched onto the full grid    */
  SIGN_NORM_CANON = 1    /* longer side -> 20, centred, CoM shift */
} SignConvention;

/* Exclusive-ended box: x2/y2 are one past the last pixel. */
typedef struct {
  int x1, y1, x2, y2;
} SignBox;

/* Frame-level result, matching the keys Python's preprocess_sign_sequence
 * returns at the top level.  `box` is the union of the final slot boxes. */
typedef struct {
  int thresh;       /* threshold actually used for the split (0 = no split) */
  int num_boxes;    /* raw column runs found, BEFORE reconciliation        */
  int num_slots;    /* boxes the split produced (== count on success)      */
  bool has_box;     /* box is valid                                       */
  bool ok;          /* at least one slot found ink                        */
  bool empty;       /* !ok                                                */
  SignBox box;
} SignGlyphInfo;

/* One slot, in LABEL order (after any RTL reversal).  `box`, `ok` and `empty`
 * are the per-slot fields Python returns; `thresh` is the threshold the C
 * slot transform used (Python recomputes it internally and does not surface
 * it, but the device logs it). */
typedef struct {
  SignBox box;
  int thresh;
  bool ok;
  bool empty;
} SignSlotInfo;

/* Views into one caller-owned block.  `boxes` has capacity
 * w + SIGN_SEQUENCE_LEN: a column run is at least one column wide, so a frame
 * yields at most w runs, and the sequence split needs at least `count`
 * entries even when the frame is narrower than the label (a 1-px-wide frame
 * still owes the caller three boxes).  sign_scratch_init allocates that. */
typedef struct {
  uint8_t *gray;    /* w*h             */
  uint8_t *ink;     /* w*h             */
  int32_t *proj;    /* w               */
  uint8_t *canvas;  /* out*out         */
  uint8_t *canvas2; /* out*out         */
  SignBox *boxes;   /* w + SEQUENCE_LEN */
} SignScratch;

/* Bytes required by sign_scratch_init for the given geometry. */
size_t sign_scratch_bytes(int w, int h, int out_size);

/* Slice `block` into `scratch`.  False (and scratch zeroed) when the block is
 * too small or the geometry is negative. */
bool sign_scratch_init(SignScratch *scratch, void *block, size_t block_bytes,
                       int w, int h, int out_size);

/* ── stage 1: luminance (ocr_preprocess.rgb_to_lum_u8) ─────────────────── */
void sign_rgb_to_lum_u8(const uint8_t *rgb, size_t n_pixels, uint8_t *gray);

/* ── stage 2: exact integer Otsu (ocr_preprocess.otsu_threshold_u8) ────── */
int sign_otsu_threshold_u8(const uint8_t *gray, size_t n);

/* Strided variant for a slot crop inside the frame; same threshold as
 * sign_otsu_threshold_u8 on a compacted copy.  Exposed because the per-slot
 * transform re-runs Otsu ON THE CROP (Python calls preprocess_glyph on the
 * crop with thresh=None), and the crop is a view, not a copy. */
int sign_otsu_threshold_strided(const uint8_t *gray, int stride, int w, int h);

/* ── stage 3: ink predicate (ocr_preprocess.binarize_ink) ──────────────── */
void sign_binarize_ink(const uint8_t *gray, size_t n, int thresh,
                       SignPolarity polarity, uint8_t *ink);

/* ── stage 4: vertical-projection segmentation ─────────────────────────── */
/* All kept runs, one tight box each, left-to-right (segment_glyph_boxes).
 * Returns the number of boxes written; `proj` is filled with the column
 * counts (w entries) for the callers that need them again. */
int sign_segment_glyph_boxes_c(const uint8_t *ink, int w, int h, int min_run,
                               int32_t *proj, SignBox *boxes, int max_boxes);

/* Union over the KEPT runs (segment_glyph_box) — the single-glyph case. */
bool sign_segment_glyph_box_c(const uint8_t *ink, int w, int h, int min_run,
                              int32_t *proj, SignGlyphInfo *info);

/* EXACTLY `count` boxes, in physical left-to-right order
 * (split_sign_glyph_boxes).  See the .c file for the three reconciliation
 * moves and why they are ordered the way they are. */
int split_sign_glyph_boxes_c(const uint8_t *ink, int w, int h, int count,
                             int min_run, int32_t *proj, SignBox *boxes,
                             int max_boxes);

/* ── stage 5: normalisation ────────────────────────────────────────────── */
bool sign_normalise_glyph(const uint8_t *crop, int w, int h, int out_size,
                          SignConvention convention, uint8_t *canvas,
                          uint8_t *canvas2);

/* ── stage 6: one glyph (ocr_preprocess.preprocess_glyph) ──────────────── */
/* `thresh` < 0 means exact Otsu (the C spelling of Python's thresh=None);
 * `dark_thr` > 0 is the luminance floor and 0 is a no-op. */
bool sign_preprocess_glyph(const uint8_t *gray_in, int w, int h, int out_size,
                           SignPolarity polarity, bool apply_segmentation,
                           int min_run, int thresh, int dark_thr,
                           SignConvention convention, SignScratch *scratch,
                           uint8_t *glyph_out, SignGlyphInfo *info);

/* ── stage 7: the whole sign (ocr_preprocess.preprocess_sign_sequence) ─── */
/* `glyphs_out` is count * out_size * out_size packed slot 0 first; `slots`
 * is count entries in LABEL order.  `info` describes the frame.  Returns
 * false only for invalid arguments or an out-of-envelope frame; a frame with
 * no ink is a SUCCESSFUL call with info->empty == true and all-zero glyphs,
 * exactly as the Python side reports it. */
bool preprocess_sign_sequence_c(const uint8_t *gray_in, int w, int h,
                                int out_size, int count, int reading_order,
                                SignPolarity polarity, bool apply_segmentation,
                                int min_run, int thresh, int dark_thr,
                                SignConvention convention, SignScratch *scratch,
                                uint8_t *glyphs_out, SignSlotInfo *slots,
                                SignGlyphInfo *info);

/* Quantise a uint8 glyph into the int8 model tensor: glyph - 128. */
void sign_glyph_to_int8(const uint8_t *glyph, size_t n, int8_t *out);

#ifdef __cplusplus
}  /* extern "C" */
#endif

#endif /* SIGN_GLYPH_H_ */
