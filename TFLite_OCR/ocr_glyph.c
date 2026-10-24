/* OCR glyph preprocessing — C mirror of AItraining/ocr_preprocess.py.
 * See ocr_glyph.h for the contract; each stage below quotes the Python rule
 * it reproduces so a future change can be checked against the same text.
 *
 * Why 160-bit Otsu instead of the obvious float (or `__int128`) form:
 *
 *   The score to maximise between-class variance is  num^2 / D  with
 *       num = N*s0 - total_s*d0        D = d0*d1
 *   and every candidate is compared by CROSS multiplication,
 *   num_c^2 * D_b  vs  num_b^2 * D_c, so no division ever happens and the
 *   comparison is exact.  The products exceed 64 bits, which leaves three
 *   options: float128/double (a decision that flips on the last mantissa
 *   bit — the whole failure mode this module exists to avoid), __int128
 *   (absent on riscv32-esp-elf-gcc, so it does not build for the P4), or an
 *   explicit small bignum.  This is the bignum.
 *
 *   Bounds, for the frame-size limit OCR_MAX_PIXELS = 65536:
 *       d0, d1          <= 65536          < 2^16
 *       s0, total_s     <= 255*65536      < 2^24
 *       |num|           < 2^41
 *       num^2                             < 2^82
 *       D = d0*d1       <= (N/2)^2        < 2^30   (fits uint32)
 *       num^2 * D                         < 2^112  < 2^160
 *   so a 5xuint32 accumulator is sufficient with 48 bits of headroom; the
 *   carry out of the top limb is provably 0 and is checked under
 *   OCR_GLYPH_DEBUG rather than assumed in silence.
 */

#include "ocr_glyph.h"

#include <string.h>

/* Fail the build if the frame bound the proof relies on is ever raised past
 * what 5 limbs still cover.  Negative array size = portable compile error. */
typedef char ocr_frame_bound_holds[(OCR_MAX_PIXELS <= 262144) ? 1 : -1];

#if defined(OCR_GLYPH_DEBUG)
#include <stdio.h>
#include <stdlib.h>
#define OCR_BN_CHECK(cond)                                                    \
  do {                                                                        \
    if (!(cond)) {                                                            \
      fprintf(stderr, "ocr_glyph: bignum overflow at %s:%d\n", __FILE__,      \
              __LINE__);                                                      \
      abort();                                                                \
    }                                                                         \
  } while (0)
#else
#define OCR_BN_CHECK(cond) ((void)0)
#endif

/* ════════════════════════════════════════════════════════════════════════
 * 0. Fixed 160-bit accumulator (5 x uint32, little-endian limbs)
 * ════════════════════════════════════════════════════════════════════════ */
typedef struct {
  uint32_t l[5];
} OcrBn;

static void bn_set_u128(OcrBn *r, uint64_t hi, uint64_t lo) {
  r->l[0] = (uint32_t)lo;
  r->l[1] = (uint32_t)(lo >> 32);
  r->l[2] = (uint32_t)hi;
  r->l[3] = (uint32_t)(hi >> 32);
  r->l[4] = 0;
}

/* a *= m; returns the carry out of the top limb (0 for every in-range use). */
static uint32_t bn_mul_u32(OcrBn *a, uint32_t m) {
  uint64_t carry = 0;
  for (int i = 0; i < 5; i++) {
    const uint64_t cur = (uint64_t)a->l[i] * (uint64_t)m + carry;
    a->l[i] = (uint32_t)cur;
    carry = cur >> 32;
  }
  return (uint32_t)carry;
}

static int bn_cmp(const OcrBn *a, const OcrBn *b) {
  for (int i = 4; i >= 0; i--) {
    if (a->l[i] != b->l[i]) return (a->l[i] < b->l[i]) ? -1 : 1;
  }
  return 0;
}

/* Full 64x64 -> 128 product from 32-bit halves: no __int128, no compiler
 * builtin, correct on any target with a 32x32->64 multiply. */
static void mul_u64_u64(uint64_t a, uint64_t b, uint64_t *hi, uint64_t *lo) {
  const uint64_t a0 = a & 0xFFFFFFFFu, a1 = a >> 32;
  const uint64_t b0 = b & 0xFFFFFFFFu, b1 = b >> 32;
  const uint64_t p00 = a0 * b0;
  const uint64_t p01 = a0 * b1;
  const uint64_t p10 = a1 * b0;
  const uint64_t p11 = a1 * b1;
  const uint64_t mid = (p00 >> 32) + (p01 & 0xFFFFFFFFu) + (p10 & 0xFFFFFFFFu);
  *lo = (mid << 32) | (p00 & 0xFFFFFFFFu);
  *hi = p11 + (p01 >> 32) + (p10 >> 32) + (mid >> 32);
}

/* r = num^2 * d — ONE SIDE of the cross-multiplied comparison.  Never
 * num^2 * own_D, which is not the ratio being compared. */
static void otsu_score(OcrBn *r, uint64_t num, uint32_t d) {
  uint64_t hi = 0, lo = 0;
  mul_u64_u64(num, num, &hi, &lo);
  bn_set_u128(r, hi, lo);
  const uint32_t carry = bn_mul_u32(r, d);
  (void)carry;                     /* the check below is debug-only */
  OCR_BN_CHECK(carry == 0);
}

/* ════════════════════════════════════════════════════════════════════════
 * 1. Luminance — ocr_preprocess.rgb_to_lum_u8
 * ════════════════════════════════════════════════════════════════════════ */
void ocr_rgb_to_lum_u8(const uint8_t *rgb, size_t n_pixels, uint8_t *gray) {
  for (size_t i = 0; i < n_pixels; i++) {
    const uint32_t r = rgb[3 * i + 0];
    const uint32_t g = rgb[3 * i + 1];
    const uint32_t b = rgb[3 * i + 2];
    /* Python: lum = (r*30 + g*59 + b*11 + 50) // 100, then clip(0,255).
     * Max sum 25500, so the clip is arithmetically unreachable — kept
     * because the reference keeps it. */
    uint32_t lum = (r * 30u + g * 59u + b * 11u + 50u) / 100u;
    if (lum > 255u) lum = 255u;
    gray[i] = (uint8_t)lum;
  }
}

/* ════════════════════════════════════════════════════════════════════════
 * 2. Otsu — ocr_preprocess.otsu_threshold_u8
 *
 *   for t in 1..255:
 *       d0 = count(levels < t)          (accumulated: d0 += hist[t-1])
 *       s0 = sum of levels < t * counts (accumulated likewise)
 *       d1 = total - d0
 *       if d0 == 0 or d1 == 0: skip          <- empty class, NOT scored 0
 *       num = total*s0 - total_s*d0
 *       update when num^2 * best_D > best_num^2 * D   (STRICT: first wins)
 *   no valid split at all -> 0
 * ════════════════════════════════════════════════════════════════════════ */
int ocr_otsu_threshold_u8(const uint8_t *gray, size_t n) {
  uint32_t hist[256];
  memset(hist, 0, sizeof(hist));
  for (size_t i = 0; i < n; i++) hist[gray[i]]++;

  const uint64_t total = (uint64_t)n;
  uint64_t total_s = 0;
  for (int v = 0; v < 256; v++) total_s += (uint64_t)v * (uint64_t)hist[v];

  uint64_t d0 = 0, s0 = 0;
  uint64_t best_num = 0;
  uint32_t best_d = 0;
  int best_t = 0;
  bool have = false;

  for (int t = 1; t < 256; t++) {
    const uint32_t h = hist[t - 1];
    d0 += (uint64_t)h;
    s0 += (uint64_t)(t - 1) * (uint64_t)h;
    if (d0 == 0) continue;
    const uint64_t d1 = total - d0;
    if (d1 == 0) continue;

    /* m0 <= m1 always (class 0 holds the smaller levels), so this signed
     * numerator is <= 0 in exact arithmetic; the score squares it, so the
     * magnitude is all that matters.  Computed signed first so nothing
     * wraps, then made unsigned explicitly. */
    const int64_t num_signed =
        (int64_t)(total * s0) - (int64_t)(total_s * d0);
    const uint64_t num =
        (uint64_t)(num_signed < 0 ? -num_signed : num_signed);
    const uint32_t d = (uint32_t)(d0 * d1);

    if (!have) {
      best_t = t;
      best_num = num;
      best_d = d;
      have = true;
      continue;
    }
    OcrBn lhs, rhs;
    otsu_score(&lhs, num, best_d);   /* candidate num^2 * best_D */
    otsu_score(&rhs, best_num, d);   /* best_num^2 * candidate_D */
    if (bn_cmp(&lhs, &rhs) > 0) {    /* strict: ties keep the FIRST t */
      best_t = t;
      best_num = num;
      best_d = d;
    }
  }
  return best_t;
}

/* ════════════════════════════════════════════════════════════════════════
 * 3. Ink mask — ocr_preprocess.binarize_ink
 *
 * The comparison operator IS the contract: at the t == 0 fallback the two
 * spellings differ by 100 % of the frame (`v <= 0` inks an all-black frame,
 * `v < 0` inks nothing), which is why the Python and C sides must use the
 * same one.  See the header note about the stale docstring.
 * ════════════════════════════════════════════════════════════════════════ */
void ocr_binarize_ink(const uint8_t *gray, size_t n, int thresh,
                      OcrPolarity polarity, uint8_t *ink) {
  const uint8_t t = (uint8_t)thresh;
  if (polarity == OCR_INK_DARK) {
    for (size_t i = 0; i < n; i++) ink[i] = (gray[i] <= t) ? 1u : 0u;
  } else {
    for (size_t i = 0; i < n; i++) ink[i] = (gray[i] > t) ? 1u : 0u;
  }
}

/* The editor's Dark Thr floor, applied ON TOP of the polarity split, exactly
 * as ocr_preprocess.preprocess_glyph does:
 *     ink = ink & (gray >= min(255, dark_thr))
 * Guarded on > 0 so the default path never touches the mask (bit-identical to
 * the pre-slider pipeline, and to every model trained before the slider
 * existed).  `dark_thr` is the already-nudged bound (bg_dark + 1), see
 * model_settings.h's kDarkThr. */
static void apply_dark_floor(const uint8_t *gray, size_t n, int dark_thr,
                             uint8_t *ink) {
  if (dark_thr <= 0) return;
  const int floor = dark_thr > 255 ? 255 : dark_thr;
  for (size_t i = 0; i < n; i++) {
    if (ink[i] && (int)gray[i] < floor) ink[i] = 0;
  }
}

/* ════════════════════════════════════════════════════════════════════════
 * 4. Segmentation — projection_runs + _column_span_bbox + segment_glyph_box
 *
 * Deliberately NO connected components and NO "keep the largest blob":
 * the dots of i/j and thin serif strokes are disconnected from the stem but
 * share its projection columns, so a column run keeps them (measured on the
 * reference: a largest-component filter drops the dot in 4/124 samples and
 * thin strokes in 16/124 more).
 *
 * The union is taken over the KEPT runs only — a run dropped by min_run can
 * still hold ink on rows no kept run reaches, and re-scanning the whole
 * span would silently widen the box.
 * ════════════════════════════════════════════════════════════════════════ */
bool ocr_segment_glyph_box(const uint8_t *ink, int w, int h, int min_run,
                           int32_t *proj, int *num_boxes, OcrBox *box) {
  if (w <= 0 || h <= 0) {
    if (num_boxes) *num_boxes = 0;
    return false;
  }
  for (int x = 0; x < w; x++) {
    int32_t c = 0;
    for (int y = 0; y < h; y++) c += ink[(size_t)y * (size_t)w + x] ? 1 : 0;
    proj[x] = c;
  }

  int kept = 0;
  bool have = false;
  OcrBox acc = {0, 0, 0, 0};

  int x = 0;
  while (x < w) {
    if (proj[x] == 0) {
      x++;
      continue;
    }
    const int s = x;
    while (x < w && proj[x] > 0) x++;
    const int e = x;
    if (e - s < min_run) continue;   /* noise, not a stroke */

    int y0 = -1, y1 = -1;
    for (int y = 0; y < h; y++) {
      const uint8_t *row = ink + (size_t)y * (size_t)w;
      int hit = 0;
      for (int xx = s; xx < e; xx++) {
        if (row[xx]) { hit = 1; break; }
      }
      if (hit) {
        if (y0 < 0) y0 = y;
        y1 = y;
      }
    }
    if (y0 < 0) continue;            /* unreachable for a kept run */
    kept++;
    if (!have) {
      acc.x1 = s;
      acc.y1 = y0;
      acc.x2 = e;
      acc.y2 = y1 + 1;
      have = true;
    } else {
      if (y0 < acc.y1) acc.y1 = y0;
      if (y1 + 1 > acc.y2) acc.y2 = y1 + 1;
      acc.x2 = e;                    /* rightmost KEPT run wins */
    }
  }

  if (num_boxes) *num_boxes = kept;
  if (have && box) *box = acc;
  return have;
}

/* ════════════════════════════════════════════════════════════════════════
 * 5. Nearest-neighbour resize — nn_source_indices
 *
 *   src(o) = floor((2o + 1) * in_len / (2 * out_len))
 *
 * the exact-integer form of the box-centre map; (2o+1) is odd so the
 * numerator is never an exact multiple of the even denominator, which makes
 * the tie behaviour deterministic rather than rounding-mode dependent.  The
 * result is always in range (src(0) = 0, src(out-1) <= in_len-1), so there
 * is deliberately NO clamp — a clamp would be a second code path that could
 * differ between the two languages.
 * ════════════════════════════════════════════════════════════════════════ */
static int nn_index(int o, int in_len, int out_len) {
  return (int)(((int64_t)(2 * o + 1) * (int64_t)in_len) /
               (int64_t)(2 * out_len));
}

/* ── STRETCH: bbox onto the full grid, one factor per axis ─────────────── */
static void normalise_stretch(const uint8_t *src, int stride, int w, int h,
                              int out_size, uint8_t *dst) {
  for (int oy = 0; oy < out_size; oy++) {
    const int sy = nn_index(oy, h, out_size);
    const uint8_t *row = src + (size_t)sy * (size_t)stride;
    uint8_t *drow = dst + (size_t)oy * (size_t)out_size;
    for (int ox = 0; ox < out_size; ox++) {
      drow[ox] = row[nn_index(ox, w, out_size)];
    }
  }
}

/* Integer round-half-up on the real line: floor(num/den + 0.5).
 * Python: _round_half_up(num, den) — Python's floor division makes
 * (2*num + den) // (2*den) correct for negatives too; C division truncates
 * toward zero, so negative quotients need an explicit floor. */
static int round_half_up_i64(int64_t num, int64_t den) {
  const int64_t a = 2 * num + den;
  const int64_t b = 2 * den;
  if (a >= 0) return (int)(a / b);
  return (int)(-(((-a) + b - 1) / b));
}

/* ── CANON: MNIST/EMNIST convention ──────────────────────────────────────
 * _scale_to_fit (longer side -> fit, aspect preserved, integer round-half-up
 * on the new length) -> _paste_centred -> integer centre-of-mass shift.
 * The resize and the paste are fused: nn_resize is a pure gather and paste
 * is a copy, so addressing the source directly is the same image. */
static void normalise_canon(const uint8_t *src, int stride, int w, int h,
                            int out_size, uint8_t *canvas, uint8_t *canvas2) {
  int fit = (out_size == OCR_GLYPH_SIZE)
                ? OCR_GLYPH_CANON_FIT
                : (out_size * OCR_GLYPH_CANON_FIT) / OCR_GLYPH_SIZE;
  if (fit < 1) fit = 1;

  const int long_side = (h > w) ? h : w;
  memset(canvas, 0, (size_t)out_size * (size_t)out_size);
  if (long_side <= 0) return;

  /* new_len = round_half_up(len * fit / long_side) in integers:
   * (2*len*fit + long_side) / (2*long_side), then clamped to [1, fit]. */
  int nh = (2 * h * fit + long_side) / (2 * long_side);
  int nw = (2 * w * fit + long_side) / (2 * long_side);
  if (nh < 1) nh = 1;
  if (nw < 1) nw = 1;
  if (nh > fit) nh = fit;
  if (nw > fit) nw = fit;

  const int dst_y0 = (out_size - nh) / 2;
  const int dst_x0 = (out_size - nw) / 2;

  for (int oy = 0; oy < nh; oy++) {
    const int sy = nn_index(oy, h, nh);
    const uint8_t *row = src + (size_t)sy * (size_t)stride;
    uint8_t *drow = canvas + (size_t)(dst_y0 + oy) * (size_t)out_size + dst_x0;
    for (int ox = 0; ox < nw; ox++) {
      drow[ox] = row[nn_index(ox, w, nw)];
    }
  }

  /* Integer centre of mass; the target is (out-1)/2, which is a half
   * integer for even out_size, so the comparison stays on the doubled
   * lattice and never leaves the integers. */
  int64_t mass = 0, sum_y = 0, sum_x = 0;
  const size_t n = (size_t)out_size * (size_t)out_size;
  for (int y = 0; y < out_size; y++) {
    for (int x = 0; x < out_size; x++) {
      const uint8_t v = canvas[(size_t)y * (size_t)out_size + x];
      mass += v;
      sum_y += (int64_t)y * (int64_t)v;
      sum_x += (int64_t)x * (int64_t)v;
    }
  }
  if (mass <= 0) return;

  const int64_t cy = sum_y / mass;   /* both non-negative: / == Python // */
  const int64_t cx = sum_x / mass;
  const int dy = round_half_up_i64(2 * cy - (out_size - 1), 2);
  const int dx = round_half_up_i64(2 * cx - (out_size - 1), 2);
  if (dy == 0 && dx == 0) return;

  memset(canvas2, 0, n);
  const int ys0 = (dy > 0) ? dy : 0;
  const int ys1 = (out_size + dy < out_size) ? out_size + dy : out_size;
  const int xs0 = (dx > 0) ? dx : 0;
  const int xs1 = (out_size + dx < out_size) ? out_size + dx : out_size;
  for (int y = ys0; y < ys1; y++) {
    for (int x = xs0; x < xs1; x++) {
      canvas2[(size_t)y * (size_t)out_size + x] =
          canvas[(size_t)(y - dy) * (size_t)out_size + (x - dx)];
    }
  }
  memcpy(canvas, canvas2, n);
}

bool ocr_normalise_glyph(const uint8_t *crop, int w, int h, int out_size,
                         OcrConvention convention, uint8_t *canvas,
                         uint8_t *canvas2) {
  if (!crop || !canvas || w <= 0 || h <= 0 || out_size <= 0) return false;
  if (convention == OCR_NORM_STRETCH) {
    normalise_stretch(crop, w, w, h, out_size, canvas);
    return true;
  }
  if (convention != OCR_NORM_CANON || !canvas2) return false;
  normalise_canon(crop, w, w, h, out_size, canvas, canvas2);
  return true;
}

/* ════════════════════════════════════════════════════════════════════════
 * 6. Pipeline — preprocess_glyph
 * ════════════════════════════════════════════════════════════════════════ */
/* Frame bounds MUST be checked before anything writes into the caller's
 * scratch: the scratch is sized for OCR_MAX_PIXELS and a larger frame would
 * overrun it inside the luminance copy, long before the pipeline's own
 * check. */
static bool frame_in_bounds(int w, int h) {
  if (w < 1 || h < 1 || w > OCR_MAX_W) return false;
  return (int64_t)w * (int64_t)h <= (int64_t)OCR_MAX_PIXELS;
}

static void info_reset(OcrGlyphInfo *info) {
  if (!info) return;
  info->thresh = 0;
  info->num_boxes = 0;
  info->has_box = false;
  info->ok = false;
  info->empty = true;
  info->box.x1 = info->box.y1 = info->box.x2 = info->box.y2 = 0;
}

/* Zero the caller's canvas.  `out_size <= 0` or a null pointer is a caller
 * error, not a buffer we may write to, so both are silently skipped. */
static void glyph_zero(uint8_t *glyph_out, int out_size) {
  if (glyph_out && out_size > 0) {
    memset(glyph_out, 0, (size_t)out_size * (size_t)out_size);
  }
}

static bool preprocess_common(const uint8_t *gray, int w, int h, int out_size,
                              OcrPolarity polarity, bool apply_segmentation,
                              int min_run, int thresh, int dark_thr,
                              OcrConvention convention, OcrScratch *s,
                              uint8_t *glyph_out, OcrGlyphInfo *info) {
  glyph_zero(glyph_out, out_size);
  info_reset(info);
  const size_t out_px = (size_t)(out_size > 0 ? out_size : 0) *
                        (size_t)(out_size > 0 ? out_size : 0);
  if (!gray || !s || !s->gray || !s->ink || !s->proj || !s->canvas ||
      !glyph_out || out_size <= 0 || !frame_in_bounds(w, h)) {
    return false;
  }
  const size_t n = (size_t)w * (size_t)h;

  int t;
  if (thresh < 0) {
    t = ocr_otsu_threshold_u8(gray, n);
    if (info) info->thresh = t;
    /* No valid split: a flat/empty frame.  There is no ink to find, so say
     * so rather than inventing one (the all-zero glyph is exactly what the
     * training set's FLAT reject samples are made of). */
    if (t == 0) return false;
  } else {
    t = thresh;
    if (info) info->thresh = t;
  }

  ocr_binarize_ink(gray, n, t, polarity, s->ink);
  apply_dark_floor(gray, n, dark_thr, s->ink);
  bool any = false;
  for (size_t i = 0; i < n; i++) {
    if (s->ink[i]) { any = true; break; }
  }
  if (!any) return false;

  int x1, y1, x2, y2;
  if (apply_segmentation) {
    int nb = 0;
    OcrBox box = {0, 0, 0, 0};
    if (!ocr_segment_glyph_box(s->ink, w, h, min_run, s->proj, &nb, &box)) {
      return false;
    }
    if (info) {
      info->num_boxes = nb;
      info->box = box;
      info->has_box = true;
    }
    x1 = box.x1; y1 = box.y1; x2 = box.x2; y2 = box.y2;
  } else {
    /* Already isolated and centred (EMNIST): the projection scan is
     * provably useless here — 1 column run on 124/124 measured samples. */
    x1 = 0; y1 = 0; x2 = w; y2 = h;
    if (info) {
      info->num_boxes = 1;
      info->box.x1 = 0; info->box.y1 = 0;
      info->box.x2 = w; info->box.y2 = h;
      info->has_box = true;
    }
  }
  if (x2 <= x1 || y2 <= y1) return false;

  /* Normalise the GRAYSCALE crop, never the binary mask: the mask only ever
   * FINDS the box (resizing it discards every anti-aliased edge pixel and
   * measurably fragments thin strokes). */
  const uint8_t *crop = gray + (size_t)y1 * (size_t)w + (size_t)x1;
  const int cw = x2 - x1, ch = y2 - y1;

  if (convention == OCR_NORM_STRETCH) {
    normalise_stretch(crop, w, cw, ch, out_size, s->canvas);
  } else {
    if (!s->canvas2) return false;
    normalise_canon(crop, w, cw, ch, out_size, s->canvas, s->canvas2);
  }
  memcpy(glyph_out, s->canvas, out_px);

  if (info) {
    info->ok = true;
    info->empty = false;
  }
  return true;
}

bool ocr_preprocess_gray_window(const uint8_t *gray_in, int w, int h,
                                int out_size, OcrPolarity polarity,
                                bool apply_segmentation, int min_run,
                                int thresh, int dark_thr,
                                OcrConvention convention, OcrScratch *scratch,
                                uint8_t *glyph_out, OcrGlyphInfo *info) {
  if (!scratch || !scratch->gray || !frame_in_bounds(w, h)) {
    /* The early guard runs BEFORE preprocess_common's zero-fill, so a caller
     * that (correctly, per the header contract) ignores this bool would
     * otherwise quantise whatever the buffer held last — a stale or boot-zero
     * glyph that looks exactly like a real frame.  Honour the contract here
     * too: "never raises and always leaves glyph_out fully written". */
    glyph_zero(glyph_out, out_size);
    info_reset(info);
    return false;
  }
  if (gray_in && gray_in != scratch->gray) {
    memcpy(scratch->gray, gray_in, (size_t)w * (size_t)h);
  }
  return preprocess_common(scratch->gray, w, h, out_size, polarity,
                           apply_segmentation, min_run, thresh, dark_thr,
                           convention, scratch, glyph_out, info);
}

bool ocr_preprocess_gray(const uint8_t *gray_in, int w, int h, int out_size,
                         OcrPolarity polarity, bool apply_segmentation,
                         int min_run, int thresh, OcrConvention convention,
                         OcrScratch *scratch, uint8_t *glyph_out,
                         OcrGlyphInfo *info) {
  return ocr_preprocess_gray_window(gray_in, w, h, out_size, polarity,
                                    apply_segmentation, min_run, thresh,
                                    /* dark_thr = */ 0, convention, scratch,
                                    glyph_out, info);
}

bool ocr_preprocess_rgb_window(const uint8_t *rgb, int w, int h, int out_size,
                               OcrPolarity polarity, bool apply_segmentation,
                               int min_run, int thresh, int dark_thr,
                               OcrConvention convention, OcrScratch *scratch,
                               uint8_t *glyph_out, OcrGlyphInfo *info) {
  if (!scratch || !scratch->gray || !rgb || !frame_in_bounds(w, h)) {
    glyph_zero(glyph_out, out_size);   /* see ocr_preprocess_gray_window */
    info_reset(info);
    return false;
  }
  ocr_rgb_to_lum_u8(rgb, (size_t)w * (size_t)h, scratch->gray);
  return preprocess_common(scratch->gray, w, h, out_size, polarity,
                           apply_segmentation, min_run, thresh, dark_thr,
                           convention, scratch, glyph_out, info);
}

bool ocr_preprocess_rgb(const uint8_t *rgb, int w, int h, int out_size,
                        OcrPolarity polarity, bool apply_segmentation,
                        int min_run, int thresh, OcrConvention convention,
                        OcrScratch *scratch, uint8_t *glyph_out,
                        OcrGlyphInfo *info) {
  return ocr_preprocess_rgb_window(rgb, w, h, out_size, polarity,
                                   apply_segmentation, min_run, thresh,
                                   /* dark_thr = */ 0, convention, scratch,
                                   glyph_out, info);
}

void ocr_glyph_to_int8(const uint8_t *glyph, size_t n, int8_t *out) {
  for (size_t i = 0; i < n; i++) out[i] = (int8_t)((int)glyph[i] - 128);
}

/* ════════════════════════════════════════════════════════════════════════
 * 7. Scratch management
 * ════════════════════════════════════════════════════════════════════════ */
static size_t align_up(size_t v, size_t a) { return (v + (a - 1)) & ~(a - 1); }

size_t ocr_scratch_bytes(int w, int h, int out_size) {
  if (w < 0) w = 0;
  if (h < 0) h = 0;
  if (out_size < 0) out_size = 0;
  const size_t frame = (size_t)w * (size_t)h;
  const size_t out_px = (size_t)out_size * (size_t)out_size;
  return align_up(frame, 4)                       /* gray    */
       + align_up(frame, 4)                       /* ink     */
       + align_up(out_px, 4)                      /* canvas  */
       + align_up(out_px, 4)                      /* canvas2 */
       + align_up((size_t)w * sizeof(int32_t), 4) /* proj    */
       + 8;
}

bool ocr_scratch_init(OcrScratch *scratch, void *block, size_t block_bytes,
                      int w, int h, int out_size) {
  if (!scratch) return false;
  scratch->gray = scratch->ink = scratch->canvas = scratch->canvas2 = NULL;
  scratch->proj = NULL;
  if (!block || w < 0 || h < 0 || out_size < 0) return false;
  if (block_bytes < ocr_scratch_bytes(w, h, out_size)) return false;

  uint8_t *p = (uint8_t *)block;
  const size_t frame = (size_t)w * (size_t)h;
  const size_t out_px = (size_t)out_size * (size_t)out_size;

  scratch->gray = p;    p += align_up(frame, 4);
  scratch->ink = p;     p += align_up(frame, 4);
  scratch->canvas = p;  p += align_up(out_px, 4);
  scratch->canvas2 = p; p += align_up(out_px, 4);
  scratch->proj = (int32_t *)p;
  return true;
}
