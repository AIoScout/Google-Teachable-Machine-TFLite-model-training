/* Sign-sequence glyph front-end — C mirror of AItraining/ocr_preprocess.py.
 *
 * The single-glyph stages (luminance, Otsu, ink predicate, segmentation,
 * NORM_CANON) are the SAME integer algorithms as TFLite_OCR/ocr_glyph.c; the
 * comments in that file carry the full derivations and are not repeated here.
 * This file adds the sign-specific layer on top and keeps the same rule:
 * every decision is an integer expression, so the ESP32-P4 and CPython agree
 * bit-for-bit rather than approximately.
 *
 * THE THREE RECONCILIATION MOVES of split_sign_glyph_boxes_c, quoted from the
 * Python docstring because the order is the contract:
 *
 *   1. the projection already yielded exactly `count` runs -> use verbatim;
 *   2. too many runs -> repeatedly MERGE THE NARROWEST RUN into its closer
 *      neighbour (narrowest, not smallest gap: a noise speck and a broken
 *      stroke are both narrower than the glyphs beside them, while the gaps
 *      between real glyphs are often the smallest gaps in the frame);
 *   3. too few runs -> repeatedly SPLIT THE WIDEST BOX at its deepest valley
 *      (widest first: the widest box is the one most likely to hold more than
 *      one glyph).  If a box cannot be cut, the whole span is re-divided into
 *      equal thirds — the caller's contract is `count` crops;
 *   4. no ink at all -> equal thirds of the frame.
 *
 * The result is always exactly `count` boxes (except for invalid input), in
 * physical left-to-right order.  RTL is applied by the caller reversing the
 * list AFTER the split; this file never mirrors pixels.
 */

#include "sign_glyph.h"

#include <string.h>

typedef char sign_frame_bound_holds[(SIGN_MAX_PIXELS <= 262144) ? 1 : -1];

#if defined(SIGN_GLYPH_DEBUG)
#include <stdio.h>
#include <stdlib.h>
#define SIGN_BN_CHECK(cond)                                                   \
  do {                                                                        \
    if (!(cond)) {                                                            \
      fprintf(stderr, "sign_glyph: bignum overflow at %s:%d\n", __FILE__,     \
              __LINE__);                                                      \
      abort();                                                                \
    }                                                                         \
  } while (0)
#else
#define SIGN_BN_CHECK(cond) ((void)0)
#endif

/* ════════════════════════════════════════════════════════════════════════
 * 0. Fixed 160-bit accumulator (5 x uint32, little-endian limbs)
 *
 * The cross-multiplied Otsu comparison needs num^2 * D with num < 2^41 and
 * D < 2^30: < 2^112.  riscv32-esp-elf-gcc has no __int128 and a float would
 * flip the winner on a last-bit tie, so the accumulator is explicit.
 * ════════════════════════════════════════════════════════════════════════ */
typedef struct {
  uint32_t l[5];
} SignBn;

static void bn_set_u128(SignBn *r, uint64_t hi, uint64_t lo) {
  r->l[0] = (uint32_t)lo;
  r->l[1] = (uint32_t)(lo >> 32);
  r->l[2] = (uint32_t)hi;
  r->l[3] = (uint32_t)(hi >> 32);
  r->l[4] = 0;
}

static uint32_t bn_mul_u32(SignBn *a, uint32_t m) {
  uint64_t carry = 0;
  for (int i = 0; i < 5; i++) {
    const uint64_t cur = (uint64_t)a->l[i] * (uint64_t)m + carry;
    a->l[i] = (uint32_t)cur;
    carry = cur >> 32;
  }
  return (uint32_t)carry;
}

static int bn_cmp(const SignBn *a, const SignBn *b) {
  for (int i = 4; i >= 0; i--) {
    if (a->l[i] != b->l[i]) return (a->l[i] < b->l[i]) ? -1 : 1;
  }
  return 0;
}

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

static void otsu_score(SignBn *r, uint64_t num, uint32_t d) {
  uint64_t hi = 0, lo = 0;
  mul_u64_u64(num, num, &hi, &lo);
  bn_set_u128(r, hi, lo);
  const uint32_t carry = bn_mul_u32(r, d);
  (void)carry;
  SIGN_BN_CHECK(carry == 0);
}

/* ════════════════════════════════════════════════════════════════════════
 * 1. Luminance — ocr_preprocess.rgb_to_lum_u8
 * ════════════════════════════════════════════════════════════════════════ */
void sign_rgb_to_lum_u8(const uint8_t *rgb, size_t n_pixels, uint8_t *gray) {
  for (size_t i = 0; i < n_pixels; i++) {
    const uint32_t r = rgb[3 * i + 0];
    const uint32_t g = rgb[3 * i + 1];
    const uint32_t b = rgb[3 * i + 2];
    /* Python: (r*30 + g*59 + b*11 + 50) // 100, then clip(0,255).
     * Max 25500, so the clip is unreachable; kept because the reference
     * keeps it. */
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
static int otsu_from_hist(const uint32_t hist[256], uint64_t total) {
  if (total == 0) return 0;

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
    SignBn lhs, rhs;
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

int sign_otsu_threshold_u8(const uint8_t *gray, size_t n) {
  uint32_t hist[256];
  memset(hist, 0, sizeof(hist));
  for (size_t i = 0; i < n; i++) hist[gray[i]]++;
  return otsu_from_hist(hist, (uint64_t)n);
}

int sign_otsu_threshold_strided(const uint8_t *gray, int stride, int w, int h) {
  if (!gray || stride < w || w <= 0 || h <= 0) return 0;
  uint32_t hist[256];
  memset(hist, 0, sizeof(hist));
  for (int y = 0; y < h; y++) {
    const uint8_t *row = gray + (size_t)y * (size_t)stride;
    for (int x = 0; x < w; x++) hist[row[x]]++;
  }
  return otsu_from_hist(hist, (uint64_t)w * (uint64_t)h);
}

/* ════════════════════════════════════════════════════════════════════════
 * 3. Ink mask — ocr_preprocess.binarize_ink
 *
 * The comparison operator IS the contract: at the t == 0 fallback the two
 * spellings differ by 100 % of the frame.  Do not "simplify" this.
 * ════════════════════════════════════════════════════════════════════════ */
void sign_binarize_ink(const uint8_t *gray, size_t n, int thresh,
                       SignPolarity polarity, uint8_t *ink) {
  const uint8_t t = (uint8_t)thresh;
  if (polarity == SIGN_INK_DARK) {
    for (size_t i = 0; i < n; i++) ink[i] = (gray[i] <= t) ? 1u : 0u;
  } else {
    for (size_t i = 0; i < n; i++) ink[i] = (gray[i] > t) ? 1u : 0u;
  }
}

/* The editor's Dark Thr floor, applied ON TOP of the polarity split:
 *     ink = ink & (gray >= min(255, dark_thr))
 * Guarded on > 0 so the default path never touches the mask (bit-identical
 * to the pre-slider pipeline). */
static void apply_dark_floor(const uint8_t *gray, size_t n, int dark_thr,
                             uint8_t *ink) {
  if (dark_thr <= 0) return;
  const int floor = dark_thr > 255 ? 255 : dark_thr;
  for (size_t i = 0; i < n; i++) {
    if (ink[i] && (int)gray[i] < floor) ink[i] = 0;
  }
}

/* ════════════════════════════════════════════════════════════════════════
 * 4. Segmentation — projection_runs + _column_span_bbox
 * ════════════════════════════════════════════════════════════════════════ */
static void projection(const uint8_t *ink, int w, int h, int32_t *proj) {
  for (int x = 0; x < w; x++) {
    int32_t c = 0;
    for (int y = 0; y < h; y++) {
      c += ink[(size_t)y * (size_t)w + (size_t)x] ? 1 : 0;
    }
    proj[x] = c;
  }
}

/* Tight exclusive-ended bbox of the ink inside columns [x1, x2).
 * Python: rows = flatnonzero(ink[:, x1:x2].any(axis=1)). */
static bool column_span_bbox(const uint8_t *ink, int w, int h, int x1, int x2,
                             SignBox *out) {
  if (x2 <= x1 || x1 < 0 || x2 > w) return false;
  int y0 = -1, y1 = -1;
  for (int y = 0; y < h; y++) {
    const uint8_t *row = ink + (size_t)y * (size_t)w;
    int hit = 0;
    for (int x = x1; x < x2; x++) {
      if (row[x]) { hit = 1; break; }
    }
    if (hit) {
      if (y0 < 0) y0 = y;
      y1 = y;
    }
  }
  if (y0 < 0) return false;
  out->x1 = x1;
  out->y1 = y0;
  out->x2 = x2;
  out->y2 = y1 + 1;
  return true;
}

int sign_segment_glyph_boxes_c(const uint8_t *ink, int w, int h, int min_run,
                               int32_t *proj, SignBox *boxes, int max_boxes) {
  if (!ink || !proj || !boxes || w <= 0 || h <= 0) return 0;
  projection(ink, w, h, proj);

  int nb = 0;
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

    SignBox b;
    if (!column_span_bbox(ink, w, h, s, e, &b)) continue;  /* unreachable */
    if (nb >= max_boxes) return -1;  /* capacity is w >= w/2: a caller bug */
    boxes[nb++] = b;
  }
  return nb;
}

bool sign_segment_glyph_box_c(const uint8_t *ink, int w, int h, int min_run,
                              int32_t *proj, SignGlyphInfo *info) {
  if (info) {
    info->num_boxes = 0;
    info->has_box = false;
    info->box.x1 = info->box.y1 = info->box.x2 = info->box.y2 = 0;
  }
  if (!ink || !proj || w <= 0 || h <= 0) return false;

  /* The union is over the KEPT runs only.  A run dropped by min_run can still
   * hold ink on rows no kept run reaches; re-scanning the first-to-last column
   * span would silently widen the box (measured: 76 vs 75 rows).  Deriving
   * from the per-run boxes keeps this one code path. */
  int nb = 0;
  projection(ink, w, h, proj);
  int x = 0;
  bool have = false;
  SignBox acc = {0, 0, 0, 0};
  while (x < w) {
    if (proj[x] == 0) {
      x++;
      continue;
    }
    const int s = x;
    while (x < w && proj[x] > 0) x++;
    const int e = x;
    if (e - s < min_run) continue;
    SignBox b;
    if (!column_span_bbox(ink, w, h, s, e, &b)) continue;
    nb++;
    if (!have) {
      acc = b;
      have = true;
    } else {
      if (b.y1 < acc.y1) acc.y1 = b.y1;
      if (b.y2 > acc.y2) acc.y2 = b.y2;
      acc.x2 = b.x2;               /* rightmost KEPT run wins */
    }
  }
  if (info) {
    info->num_boxes = nb;
    info->has_box = have;
    if (have) info->box = acc;
  }
  return have;
}

/* `count` equal-width column bands spanning [x1, x2), all at [y1, y2).
 * The last band ends exactly on x2 so the bands tile the span with no gap
 * however the integer division falls (ocr_preprocess._equal_split_boxes). */
static void equal_split(int x1, int x2, int y1, int y2, int count,
                        SignBox *boxes) {
  const int width = (x2 > x1) ? (x2 - x1) : 0;
  for (int i = 0; i < count; i++) {
    const int bx1 = x1 + (width * i) / count;
    const int bx2 = (i == count - 1) ? x2 : x1 + (width * (i + 1)) / count;
    boxes[i].x1 = bx1;
    boxes[i].y1 = y1;
    boxes[i].x2 = bx2;
    boxes[i].y2 = y2;
  }
}

static SignBox union_box(SignBox a, SignBox b) {
  SignBox r;
  r.x1 = a.x1 < b.x1 ? a.x1 : b.x1;
  r.y1 = a.y1 < b.y1 ? a.y1 : b.y1;
  r.x2 = a.x2 > b.x2 ? a.x2 : b.x2;
  r.y2 = a.y2 > b.y2 ? a.y2 : b.y2;
  return r;
}

/* The boundary inside (x1, x2) that cuts through the least ink
 * (ocr_preprocess._deepest_valley).  Cost is the ink of the two columns the
 * cut separates; ties go to the boundary nearest the midpoint.  Returns -1
 * when the span is too narrow to hold a cut. */
static int deepest_valley(const int32_t *proj, int x1, int x2) {
  if (x2 - x1 < 2) return -1;
  const int mid2 = x1 + x2;
  int best_p = -1;
  int best_cost = 0;
  int best_dist = 0;
  for (int p = x1 + 1; p < x2; p++) {
    const int cost = proj[p - 1] + proj[p];
    const int dist = (2 * p > mid2) ? (2 * p - mid2) : (mid2 - 2 * p);
    if (best_p < 0 || cost < best_cost ||
        (cost == best_cost && dist < best_dist)) {
      best_p = p;
      best_cost = cost;
      best_dist = dist;
    }
  }
  return best_p;
}

/* The reconciliation itself, given the runs already segmented into `boxes`
 * (`len` of them).  split_sign_glyph_boxes_c is the public wrapper that
 * segments first; preprocess_sign_sequence_c reuses this entry point so the
 * frame is scanned for runs exactly once. */
static int split_from_boxes(const uint8_t *ink, int w, int h, int count,
                            const int32_t *proj, SignBox *boxes,
                            int max_boxes, int len) {
  if (!ink || !proj || !boxes || w <= 0 || h <= 0) return 0;
  const int n = (count < 1) ? 1 : count;
  if (max_boxes < n) return -1;

  if (len == 0) {
    equal_split(0, w, 0, h, n, boxes);
    return n;
  }

  /* ── too many runs: merge the narrowest into its closer neighbour ────── */
  while (len > n) {
    int k = 0;
    int narrow = boxes[0].x2 - boxes[0].x1;
    for (int i = 1; i < len; i++) {
      const int wd = boxes[i].x2 - boxes[i].x1;
      if (wd < narrow) { narrow = wd; k = i; }   /* strict: first wins */
    }
    int j;
    if (k == 0) {
      j = 1;
    } else if (k == len - 1) {
      j = k - 1;
    } else {
      const int gap_l = boxes[k].x1 - boxes[k - 1].x2;
      const int gap_r = boxes[k + 1].x1 - boxes[k].x2;
      j = (gap_l <= gap_r) ? (k - 1) : (k + 1);
    }
    const int lo = (j < k) ? j : k;
    const int hi = (j < k) ? k : j;
    boxes[lo] = union_box(boxes[lo], boxes[hi]);
    for (int t = hi + 1; t < len; t++) boxes[t - (hi - lo)] = boxes[t];
    len -= (hi - lo);
  }

  /* ── too few runs: split the widest at its deepest valley ────────────── */
  while (len < n) {
    int i = 0;
    int widest = boxes[0].x2 - boxes[0].x1;
    for (int k = 1; k < len; k++) {
      const int wd = boxes[k].x2 - boxes[k].x1;
      if (wd > widest) { widest = wd; i = k; }   /* strict: first wins */
    }
    const SignBox b = boxes[i];
    const int p = deepest_valley(proj, b.x1, b.x2);
    if (p < 0) {
      /* This box cannot be cut.  Give up on reconciliation and answer
       * geometrically over the whole span, so the caller still gets exactly
       * `n` boxes hugging the ink that was found. */
      const SignBox whole = union_box(boxes[0], boxes[len - 1]);
      equal_split(whole.x1, whole.x2, whole.y1, whole.y2, n, boxes);
      return n;
    }
    SignBox left, right;
    if (!column_span_bbox(ink, w, h, b.x1, p, &left)) {
      left.x1 = b.x1; left.y1 = b.y1; left.x2 = p; left.y2 = b.y2;
    }
    if (!column_span_bbox(ink, w, h, p, b.x2, &right)) {
      right.x1 = p; right.y1 = b.y1; right.x2 = b.x2; right.y2 = b.y2;
    }
    for (int t = len; t > i + 1; t--) boxes[t] = boxes[t - 1];
    boxes[i] = left;
    boxes[i + 1] = right;
    len += 1;
  }
  return len;
}

int split_sign_glyph_boxes_c(const uint8_t *ink, int w, int h, int count,
                             int min_run, int32_t *proj, SignBox *boxes,
                             int max_boxes) {
  const int len = sign_segment_glyph_boxes_c(ink, w, h, min_run, proj, boxes,
                                             max_boxes);
  if (len < 0) return -1;
  return split_from_boxes(ink, w, h, count, proj, boxes, max_boxes, len);
}

/* ════════════════════════════════════════════════════════════════════════
 * 5. Nearest-neighbour resize — nn_source_indices
 *
 *   src(o) = floor((2o + 1) * in_len / (2 * out_len))
 *
 * (2o+1) is odd, so the numerator is never an exact multiple of the even
 * denominator; the tie behaviour is deterministic rather than rounding-mode
 * dependent.  The result is always in range, so there is deliberately NO
 * clamp — a clamp would be a second code path that could drift.
 * ════════════════════════════════════════════════════════════════════════ */
static int nn_index(int o, int in_len, int out_len) {
  return (int)(((int64_t)(2 * o + 1) * (int64_t)in_len) /
               (int64_t)(2 * out_len));
}

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

/* Python ocr_preprocess._narrow_stroke_radius: how many pixels to widen a
 * thin single-stroke glyph on each side before the CANON paste.  A camera
 * '1' is a 1-2 px bar; EMNIST '1' samples are 3-4 px, and the mismatch made
 * the stroke read as _background.  Same integer arithmetic as Python:
 * ink bbox aspect < 35 %, stroke = round_half_up(total_ink / bbox_height),
 * projected through the same round-half-up resize, target >= 3 px. */
static int narrow_stroke_radius(const uint8_t *ink, int w, int x1, int y1,
                                int x2, int y2, int out_size) {
  int min_x = x2, max_x = x1 - 1, min_y = y2, max_y = y1 - 1;
  for (int y = y1; y < y2; y++) {
    const uint8_t *row = ink + (size_t)y * (size_t)w;
    for (int x = x1; x < x2; x++) {
      if (!row[x]) continue;
      if (x < min_x) min_x = x;
      if (x > max_x) max_x = x;
      if (y < min_y) min_y = y;
      if (y > max_y) max_y = y;
    }
  }
  if (max_x < min_x || max_y < min_y) return 0;
  const int bw = max_x - min_x + 1;
  const int bh = max_y - min_y + 1;
  if (bw <= 0 || bh <= 0) return 0;
  if (bw * 100 > 35 * bh) return 0;
  int64_t total = 0;
  for (int y = min_y; y <= max_y; y++) {
    const uint8_t *row = ink + (size_t)y * (size_t)w;
    for (int x = min_x; x <= max_x; x++) total += row[x] ? 1 : 0;
  }
  if (total <= 0) return 0;
  int stroke = (int)((2 * total + bh) / (2 * (int64_t)bh));
  if (stroke < 1) stroke = 1;
  const int cw = x2 - x1;
  const int ch = y2 - y1;
  const int long_side = (cw > ch) ? cw : ch;
  if (long_side <= 0) return 0;
  int fit = (out_size == SIGN_GLYPH_SIZE)
                ? SIGN_GLYPH_CANON_FIT
                : (out_size * SIGN_GLYPH_CANON_FIT) / SIGN_GLYPH_SIZE;
  if (fit < 1) fit = 1;
  const int target_src = (3 * long_side + fit - 1) / fit;
  if (stroke >= target_src) return 0;
  const int r = (target_src - stroke + 1) / 2;
  return (r > 4) ? 4 : r;
}

/* CANON: _scale_to_fit (longer side -> 20, aspect preserved, integer
 * round-half-up) -> optional horizontal stroke widening -> _paste_centred
 * -> integer centre-of-mass shift.  The resize and the paste are fused:
 * nn_resize is a pure gather and paste is a copy, so addressing the source
 * directly is the same image. */
static void normalise_canon(const uint8_t *src, int stride, int w, int h,
                            int out_size, uint8_t *canvas, uint8_t *canvas2,
                            int narrow_radius, SignPolarity polarity) {
  int fit = (out_size == SIGN_GLYPH_SIZE)
                ? SIGN_GLYPH_CANON_FIT
                : (out_size * SIGN_GLYPH_CANON_FIT) / SIGN_GLYPH_SIZE;
  if (fit < 1) fit = 1;

  /* Python normalise_glyph(narrow_radius=...): widen the SOURCE crop
   * horizontally by `pad` on each side BEFORE the fit.  Doing it after
   * the resize would be too late — nearest-neighbour downsampling samples
   * every ~long/fit-th source column and can skip a 1 px stroke.  The
   * added source columns take the plate tone (255 for dark ink, 0 for
   * bright ink); the glyph stays centred because the padding is symmetric. */
  const int pad = (narrow_radius > 0) ? narrow_radius : 0;
  const int w_pad = w + 2 * pad;
  const int long_side = (h > w_pad) ? h : w_pad;
  memset(canvas, 0, (size_t)out_size * (size_t)out_size);
  if (long_side <= 0) return;

  int nh = (2 * h * fit + long_side) / (2 * long_side);
  int nw = (2 * w_pad * fit + long_side) / (2 * long_side);
  if (nh < 1) nh = 1;
  if (nw < 1) nw = 1;
  if (nh > fit) nh = fit;
  if (nw > fit) nw = fit;

  const int dst_y0 = (out_size - nh) / 2;
  const int dst_x0 = (out_size - nw) / 2;
  const int fill = (polarity == SIGN_INK_DARK) ? 255 : 0;

  for (int oy = 0; oy < nh; oy++) {
    const int sy = nn_index(oy, h, nh);
    const uint8_t *row = src + (size_t)sy * (size_t)stride;
    uint8_t *drow = canvas + (size_t)(dst_y0 + oy) * (size_t)out_size + dst_x0;
    if (pad == 0) {
      for (int ox = 0; ox < nw; ox++) {
        drow[ox] = row[nn_index(ox, w, nw)];
      }
      continue;
    }
    for (int ox = 0; ox < nw; ox++) {
      const int sp = nn_index(ox, w_pad, nw);
      int best = fill;
      for (int k = sp - pad; k <= sp + pad; k++) {
        const int sx = k - pad;
        if (sx < 0 || sx >= w) continue;
        const int v = row[sx];
        if (polarity == SIGN_INK_DARK) {
          if (v < best) best = v;
        } else if (v > best) {
          best = v;
        }
      }
      drow[ox] = (uint8_t)best;
    }
  }

  int64_t mass = 0, sum_y = 0, sum_x = 0;
  const size_t n = (size_t)out_size * (size_t)out_size;
  for (int y = 0; y < out_size; y++) {
    for (int x = 0; x < out_size; x++) {
      const uint8_t v = canvas[(size_t)y * (size_t)out_size + (size_t)x];
      mass += v;
      sum_y += (int64_t)y * (int64_t)v;
      sum_x += (int64_t)x * (int64_t)v;
    }
  }
  if (mass <= 0) return;

  const int64_t cy = sum_y / mass;
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
      canvas2[(size_t)y * (size_t)out_size + (size_t)x] =
          canvas[(size_t)(y - dy) * (size_t)out_size + (size_t)(x - dx)];
    }
  }
  memcpy(canvas, canvas2, n);
}

bool sign_normalise_glyph(const uint8_t *crop, int w, int h, int out_size,
                          SignConvention convention, uint8_t *canvas,
                          uint8_t *canvas2) {
  if (!crop || !canvas || w <= 0 || h <= 0 || out_size <= 0) return false;
  if (convention == SIGN_NORM_STRETCH) {
    normalise_stretch(crop, w, w, h, out_size, canvas);
    return true;
  }
  if (convention != SIGN_NORM_CANON || !canvas2) return false;
  /* The standalone normaliser has no ink mask, so it cannot measure a
   * narrow stroke; preprocess_glyph (the caller that has one) passes the
   * radius itself. */
  normalise_canon(crop, w, w, h, out_size, canvas, canvas2, 0, SIGN_INK_DARK);
  return true;
}

/* ════════════════════════════════════════════════════════════════════════
 * 6. One glyph — ocr_preprocess.preprocess_glyph
 * ════════════════════════════════════════════════════════════════════════ */
static bool frame_in_bounds(int w, int h) {
  if (w < 1 || h < 1 || w > SIGN_MAX_W) return false;
  return (int64_t)w * (int64_t)h <= (int64_t)SIGN_MAX_PIXELS;
}

static void info_reset(SignGlyphInfo *info) {
  if (!info) return;
  info->thresh = 0;
  info->num_boxes = 0;
  info->num_slots = 0;
  info->has_box = false;
  info->ok = false;
  info->empty = true;
  info->has_roi = false;
  info->box.x1 = info->box.y1 = info->box.x2 = info->box.y2 = 0;
  info->roi_box.x1 = info->roi_box.y1 = info->roi_box.x2 = info->roi_box.y2 = 0;
  info->sign_pct = 0.0f;
}

static void glyph_zero(uint8_t *glyph_out, int out_size) {
  if (glyph_out && out_size > 0) {
    memset(glyph_out, 0, (size_t)out_size * (size_t)out_size);
  }
}

/* Flood-fill one 8-connected ink component.  `visited` must be zero at the
 * seed; every pixel of the component is set to `visit_value`.  Returns the
 * seed index (raster-first pixel), which identifies the component. */
static int32_t flood_component(uint8_t *ink, int w, int h, int sx, int sy,
                               SignScratch *s, uint8_t visit_value,
                               int *x1_out, int *y1_out,
                               int *x2_out, int *y2_out, int *area_out) {
  const size_t start = (size_t)sy * (size_t)w + (size_t)sx;
  int top = 0;
  int x1 = sx, y1 = sy, x2 = sx + 1, y2 = sy + 1, area = 0;
  s->visited[start] = visit_value;
  s->stack[top++] = (int32_t)start;
  while (top > 0) {
    const int32_t idx = s->stack[--top];
    const int cy = (int)(idx / w);
    const int cx = (int)(idx % w);
    if (cx < x1) x1 = cx;
    if (cx + 1 > x2) x2 = cx + 1;
    if (cy < y1) y1 = cy;
    if (cy + 1 > y2) y2 = cy + 1;
    area++;
    for (int dy = -1; dy <= 1; dy++) {
      const int ny = cy + dy;
      if (ny < 0 || ny >= h) continue;
      for (int dx = -1; dx <= 1; dx++) {
        if (dx == 0 && dy == 0) continue;
        const int nx = cx + dx;
        if (nx < 0 || nx >= w) continue;
        const size_t nidx = (size_t)ny * (size_t)w + (size_t)nx;
        if (!ink[nidx] || s->visited[nidx]) continue;
        s->visited[nidx] = visit_value;
        s->stack[top++] = (int32_t)nidx;
      }
    }
  }
  if (x1_out) *x1_out = x1;
  if (y1_out) *y1_out = y1;
  if (x2_out) *x2_out = x2;
  if (y2_out) *y2_out = y2;
  if (area_out) *area_out = area;
  return (int32_t)start;
}

/* Zero a component already marked visited==1, and set its gray pixels to the
 * background so a removed speck cannot bias the intensity centring either. */
static void clear_component(uint8_t *ink, uint8_t *gray, int w, int h,
                            int32_t seed, SignScratch *s, uint8_t bg) {
  if (!ink[seed]) return;
  int top = 0;
  s->visited[seed] = 3;
  ink[seed] = 0;
  gray[seed] = bg;
  s->stack[top++] = seed;
  while (top > 0) {
    const int32_t idx = s->stack[--top];
    const int cy = (int)(idx / w);
    const int cx = (int)(idx % w);
    for (int dy = -1; dy <= 1; dy++) {
      const int ny = cy + dy;
      if (ny < 0 || ny >= h) continue;
      for (int dx = -1; dx <= 1; dx++) {
        if (dx == 0 && dy == 0) continue;
        const int nx = cx + dx;
        if (nx < 0 || nx >= w) continue;
        const size_t nidx = (size_t)ny * (size_t)w + (size_t)nx;
        if (!ink[nidx] || s->visited[nidx] != 1) continue;
        s->visited[nidx] = 3;
        ink[nidx] = 0;
        gray[nidx] = bg;
        s->stack[top++] = (int32_t)nidx;
      }
    }
  }
}

/* Remove small disconnected specks touching the crop boundary.  Bit-for-bit
 * mirror of ocr_preprocess.strip_border_specks():
 *   dominant = largest 8-connected component;
 *   keep any other component larger than 20 % of it, not touching the crop
 *   boundary, or overlapping the dominant bbox in BOTH axes (a detached
 *   fragment of a broken '1' shares the stem's columns);
 *   drop the rest (the stray foot of a neighbouring glyph clipped by a
 *   Manual ROI).  The gray pixels of dropped specks become background so
 *   they cannot shift NORM_CANON's intensity centring. */
static void strip_border_specks_c(uint8_t *ink, uint8_t *gray, int w, int h,
                                  SignScratch *s, uint8_t bg) {
  if (!ink || !gray || !s || !s->visited || !s->stack || w <= 0 || h <= 0) {
    return;
  }
  const size_t frame = (size_t)w * (size_t)h;
  memset(s->visited, 0, frame);

  int32_t dom_seed = -1;
  int dom_area = -1;
  int dx1 = 0, dy1 = 0, dx2 = 0, dy2 = 0;
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const size_t start = (size_t)y * (size_t)w + (size_t)x;
      if (!ink[start] || s->visited[start]) continue;
      int x1, y1, x2, y2, area;
      flood_component(ink, w, h, x, y, s, 1, &x1, &y1, &x2, &y2, &area);
      if (area > dom_area) {
        dom_area = area;
        dom_seed = (int32_t)start;
        dx1 = x1; dy1 = y1; dx2 = x2; dy2 = y2;
      }
    }
  }
  if (dom_seed < 0 || dom_area <= 0) return;

  memset(s->visited, 0, frame);
  for (int y = 0; y < h; y++) {
    for (int x = 0; x < w; x++) {
      const size_t start = (size_t)y * (size_t)w + (size_t)x;
      if (!ink[start] || s->visited[start]) continue;
      int x1, y1, x2, y2, area;
      const int32_t seed =
          flood_component(ink, w, h, x, y, s, 1, &x1, &y1, &x2, &y2, &area);
      if (seed == dom_seed) continue;
      int keep = 1;
      if (area * 100 <= 20 * dom_area) {
        const int touches = (x1 == 0 || y1 == 0 || x2 == w || y2 == h);
        if (touches) {
          const int x_overlap = !(x2 <= dx1 || x1 >= dx2);
          const int y_overlap = !(y2 <= dy1 || y1 >= dy2);
          if (!(x_overlap && y_overlap)) keep = 0;
        }
      }
      if (!keep) clear_component(ink, gray, w, h, seed, s, bg);
    }
  }
}

static bool preprocess_common(uint8_t *gray, int w, int h, int out_size,
                              SignPolarity polarity, bool apply_segmentation,
                              int min_run, int thresh, int dark_thr,
                              SignConvention convention, SignScratch *s,
                              uint8_t *glyph_out, SignGlyphInfo *info) {
  glyph_zero(glyph_out, out_size);
  info_reset(info);
  const size_t out_px = (size_t)(out_size > 0 ? out_size : 0) *
                        (size_t)(out_size > 0 ? out_size : 0);
  if (!gray || !s || !s->gray || !s->ink || !s->proj || !s->canvas ||
      !s->boxes || !glyph_out || out_size <= 0 ||
      !frame_in_bounds(w, h)) {
    return false;
  }
  const size_t n = (size_t)w * (size_t)h;

  int t;
  if (thresh < 0) {
    t = sign_otsu_threshold_u8(gray, n);
    if (info) info->thresh = t;
    /* No valid split: a flat/empty frame.  There is no ink to find, so say
     * so rather than inventing one (the all-zero glyph is what the training
     * set's FLAT reject samples are made of). */
    if (t == 0) return false;
  } else {
    t = thresh;
    if (info) info->thresh = t;
  }

  sign_binarize_ink(gray, n, t, polarity, s->ink);
  apply_dark_floor(gray, n, dark_thr, s->ink);
  /* Single-glyph and per-slot crops: drop a neighbouring glyph's clipped
   * corner speck before the bbox/metrics see it. */
  strip_border_specks_c(s->ink, gray, w, h, s,
                        (polarity == SIGN_INK_DARK) ? 255 : 0);

  bool any = false;
  for (size_t i = 0; i < n; i++) {
    if (s->ink[i]) { any = true; break; }
  }
  if (!any) return false;

  int x1, y1, x2, y2;
  if (apply_segmentation) {
    int nb = sign_segment_glyph_boxes_c(s->ink, w, h, min_run, s->proj,
                                        s->boxes, w);
    if (nb <= 0) return false;
    SignBox box = s->boxes[0];
    for (int i = 1; i < nb; i++) box = union_box(box, s->boxes[i]);
    if (info) {
      info->num_boxes = nb;
      info->box = box;
      info->has_box = true;
    }
    x1 = box.x1; y1 = box.y1; x2 = box.x2; y2 = box.y2;
  } else {
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

  if (convention == SIGN_NORM_STRETCH) {
    normalise_stretch(crop, w, cw, ch, out_size, s->canvas);
  } else {
    if (!s->canvas2) return false;
    const int radius = narrow_stroke_radius(s->ink, w, x1, y1, x2, y2,
                                            out_size);
    normalise_canon(crop, w, cw, ch, out_size, s->canvas, s->canvas2,
                    radius, polarity);
  }
  memcpy(glyph_out, s->canvas, out_px);

  if (info) {
    info->ok = true;
    info->empty = false;
  }
  return true;
}

bool sign_preprocess_glyph(const uint8_t *gray_in, int w, int h, int out_size,
                           SignPolarity polarity, bool apply_segmentation,
                           int min_run, int thresh, int dark_thr,
                           SignConvention convention, SignScratch *scratch,
                           uint8_t *glyph_out, SignGlyphInfo *info) {
  if (!scratch || !scratch->gray || !frame_in_bounds(w, h)) {
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

/* ════════════════════════════════════════════════════════════════════════
 * 7. The whole sign — ocr_preprocess.preprocess_sign_sequence
 *
 * The frame is binarised ONCE for the split, but every slot is then handed
 * to preprocess_common as its own image with the caller's `thresh` — which
 * means a slot re-runs OTSU ON ITS CROP when the caller passed -1, exactly as
 * Python's preprocess_glyph(crop, thresh=None) does.  That is not an
 * optimisation we can take: it is the training transform, and it is why the
 * slot path is implemented by calling the same preprocess_common rather than
 * by slicing the frame mask.
 * ════════════════════════════════════════════════════════════════════════ */
/* ════════════════════════════════════════════════════════════════════════
 * 7b. Interior ink ROI (ocr_preprocess._interior_ink_components +
 *     _choose_interior_band)
 *
 * The Python side uses OpenCV for speed, but the components are the same
 * 8-connected sets a raster-scan flood fill finds, and the filters/selection
 * are integer-identical.  The result is the tight band of characters (or the
 * single character in an Edit sample) with a uniform 10 % pad; border
 * clutter, frame lines and specks never reach the split.
 * ════════════════════════════════════════════════════════════════════════ */
#define INTERIOR_MAX_COMPONENTS 256

typedef struct {
  SignBox box;
  int area;
} SignCand;

static int iabs(int v) { return (v < 0) ? -v : v; }

/* True only for frame-scale / outer-surround components.  Mirrors
 * ocr_preprocess._is_outer_frame_debris bit-for-bit.
 *
 * A character may legitimately sit within a few pixels of the frame edge
 * (a tight plate capture), so the old unconditional outer-margin drop is
 * gone: it erased 'A' while leaving the plate.  Still removed: the plastic
 * frame / holder border (thin in one axis, spanning >= 70 % of the other)
 * and frame-thickness corner debris at the outer edge.  A thin '1' stroke
 * (tall but < 70 % of the frame) and a detached bottom fragment survive. */
static bool is_outer_frame_debris(int x, int y, int cw, int ch,
                                  int w, int h, int margin) {
  if (w <= 0 || h <= 0) return false;
  int thin_w = (w * 6 + 50) / 100;
  if (thin_w < 4) thin_w = 4;
  int thin_h = (h * 6 + 50) / 100;
  if (thin_h < 4) thin_h = 4;
  if (ch <= thin_h && cw * 100 >= 70 * w) return true;
  if (cw <= thin_w && ch * 100 >= 70 * h) return true;
  const int touches_outer = (x <= margin || y <= margin ||
                             x + cw >= w - margin || y + ch >= h - margin);
  if (touches_outer && cw <= thin_w && ch <= thin_h) return true;
  return false;
}

/* Returns true and fills roi/pct when a band was found.  Ink outside the ROI
 * is zeroed so the projection split only sees the characters. */
static bool interior_roi_c(uint8_t *ink, int w, int h, SignScratch *s,
                           SignBox *roi_out, float *pct_out) {
  if (!ink || !s || !s->visited || !s->stack || w <= 0 || h <= 0) return false;
  const size_t frame = (size_t)w * (size_t)h;
  memset(s->visited, 0, frame);

  const int shorter = (w < h) ? w : h;
  int margin = (shorter * 6 + 50) / 100;
  if (margin < 2) margin = 2;
  int min_area = (int)((frame * 4 + 5000) / 10000);
  if (min_area < 6) min_area = 6;

  SignCand cand[INTERIOR_MAX_COMPONENTS];
  int ncand = 0;

  for (int y = 0; y < h && ncand < INTERIOR_MAX_COMPONENTS; y++) {
    for (int x = 0; x < w && ncand < INTERIOR_MAX_COMPONENTS; x++) {
      const size_t start = (size_t)y * (size_t)w + (size_t)x;
      if (!ink[start] || s->visited[start]) continue;

      int top = 0;
      s->visited[start] = 1;
      s->stack[top++] = (int32_t)start;
      int x1 = x, y1 = y, x2 = x + 1, y2 = y + 1;
      int area = 0;
      while (top > 0) {
        const int32_t idx = s->stack[--top];
        const int cy = (int)(idx / w);
        const int cx = (int)(idx % w);
        if (cx < x1) x1 = cx;
        if (cx + 1 > x2) x2 = cx + 1;
        if (cy < y1) y1 = cy;
        if (cy + 1 > y2) y2 = cy + 1;
        area++;
        for (int dy = -1; dy <= 1; dy++) {
          const int ny = cy + dy;
          if (ny < 0 || ny >= h) continue;
          for (int dx = -1; dx <= 1; dx++) {
            if (dx == 0 && dy == 0) continue;
            const int nx = cx + dx;
            if (nx < 0 || nx >= w) continue;
            const size_t nidx = (size_t)ny * (size_t)w + (size_t)nx;
            if (!ink[nidx] || s->visited[nidx]) continue;
            s->visited[nidx] = 1;
            s->stack[top++] = (int32_t)nidx;
          }
        }
      }

      const int cw = x2 - x1;
      const int ch = y2 - y1;
      if (area < min_area) continue;
      if (is_outer_frame_debris(x1, y1, cw, ch, w, h, margin)) continue;
      if (cw * 100 > 85 * w || ch * 100 > 60 * h) continue;
      if (area * 100 > 40 * (int)frame) continue;
      cand[ncand].box.x1 = x1;
      cand[ncand].box.y1 = y1;
      cand[ncand].box.x2 = x2;
      cand[ncand].box.y2 = y2;
      cand[ncand].area = area;
      ncand++;
    }
  }

  if (ncand < 2) {
    /* A sign needs at least two character components.  One component means
     * a tight capture (or touching characters); cropping to it could discard
     * edge glyphs, so leave the frame intact and let the split reconcile. */
    long count = 0;
    for (size_t i = 0; i < frame; i++) count += ink[i] ? 1 : 0;
    if (pct_out) *pct_out = (float)(100.0 * (double)count / (double)frame);
    return false;
  }

  int best = 0;
  int best_count = -1;
  int best_total = -1;
  for (int i = 0; i < ncand; i++) {
    const int ah = (cand[i].box.y2 - cand[i].box.y1 > 0)
                       ? cand[i].box.y2 - cand[i].box.y1 : 1;
    const int acy2 = 2 * cand[i].box.y1 + ah;
    const int band2 = 2 * ah;
    int count = 0;
    int total = 0;
    for (int j = 0; j < ncand; j++) {
      const int ch = cand[j].box.y2 - cand[j].box.y1;
      const int cy2 = 2 * cand[j].box.y1 + ch;
      if (iabs(cy2 - acy2) > band2) continue;
      count++;
      total += cand[j].area;
    }
    if (count > best_count || (count == best_count && total > best_total)) {
      best_count = count;
      best_total = total;
      best = i;
    }
  }

  const int ah = (cand[best].box.y2 - cand[best].box.y1 > 0)
                     ? cand[best].box.y2 - cand[best].box.y1 : 1;
  const int acy2 = 2 * cand[best].box.y1 + ah;
  const int band2 = 2 * ah;
  int bx1 = w, by1 = h, bx2 = 0, by2 = 0;
  for (int j = 0; j < ncand; j++) {
    const int ch = cand[j].box.y2 - cand[j].box.y1;
    const int cy2 = 2 * cand[j].box.y1 + ch;
    if (iabs(cy2 - acy2) > band2) continue;
    if (cand[j].box.x1 < bx1) bx1 = cand[j].box.x1;
    if (cand[j].box.y1 < by1) by1 = cand[j].box.y1;
    if (cand[j].box.x2 > bx2) bx2 = cand[j].box.x2;
    if (cand[j].box.y2 > by2) by2 = cand[j].box.y2;
  }
  if (bx2 <= bx1 || by2 <= by1) return false;

  const int cw = bx2 - bx1;
  const int ch = by2 - by1;
  int pad = ((cw > ch ? cw : ch) * 10 + 50) / 100;
  if (pad < 2) pad = 2;
  int rx1 = bx1 - pad; if (rx1 < 0) rx1 = 0;
  int ry1 = by1 - pad; if (ry1 < 0) ry1 = 0;
  int rx2 = bx2 + pad; if (rx2 > w) rx2 = w;
  int ry2 = by2 + pad; if (ry2 > h) ry2 = h;
  if (rx2 <= rx1 || ry2 <= ry1) return false;

  /* Only the selected character band survives to the split.  The padded
   * ROI stays white around it, so a frame line crossing the pad cannot
   * become a fourth projection run (Python zeroes the same tight box). */
  for (int y = 0; y < h; y++) {
    uint8_t *row = ink + (size_t)y * (size_t)w;
    for (int x = 0; x < w; x++) {
      if (x < bx1 || x >= bx2 || y < by1 || y >= by2) row[x] = 0;
    }
  }

  long count = 0;
  for (int y = ry1; y < ry2; y++) {
    const uint8_t *row = ink + (size_t)y * (size_t)w;
    for (int x = rx1; x < rx2; x++) count += row[x] ? 1 : 0;
  }
  const long total = (long)(rx2 - rx1) * (long)(ry2 - ry1);
  if (pct_out) {
    *pct_out = (total > 0) ? (float)(100.0 * (double)count / (double)total) : 0.0f;
  }
  if (roi_out) {
    roi_out->x1 = rx1; roi_out->y1 = ry1;
    roi_out->x2 = rx2; roi_out->y2 = ry2;
  }
  return true;
}

bool preprocess_sign_sequence_c(const uint8_t *gray_in, int w, int h,
                                int out_size, int count, int reading_order,
                                SignPolarity polarity, bool apply_segmentation,
                                int min_run, int thresh, int dark_thr,
                                SignConvention convention, SignScratch *scratch,
                                uint8_t *glyphs_out, SignSlotInfo *slots,
                                SignGlyphInfo *info) {
  const int n = (count < 1) ? 1 : count;
  const int safe_out = (out_size > 0) ? out_size : 0;
  const size_t out_px = (size_t)safe_out * (size_t)safe_out;

  /* Every slot is zeroed and every result is "empty" before anything else:
   * the caller may read the buffers even when this returns false, and a stale
   * glyph looks exactly like a real frame. */
  for (int i = 0; i < n; i++) {
    if (glyphs_out) memset(glyphs_out + (size_t)i * out_px, 0, out_px);
    if (slots) {
      slots[i].box.x1 = slots[i].box.y1 = 0;
      slots[i].box.x2 = slots[i].box.y2 = 0;
      slots[i].thresh = 0;
      slots[i].ok = false;
      slots[i].empty = true;
    }
  }
  info_reset(info);

  if (!gray_in || !scratch || !scratch->gray || !scratch->ink ||
      !scratch->proj || !scratch->boxes || !glyphs_out || !slots ||
      out_size <= 0 || !frame_in_bounds(w, h)) {
    return false;
  }
  const size_t frame_px = (size_t)w * (size_t)h;

  /* ── frame-level binarisation, used ONLY to find the boxes ────────────── */
  int t;
  if (thresh < 0) {
    t = sign_otsu_threshold_u8(gray_in, frame_px);
  } else {
    t = thresh;
  }
  if (info) info->thresh = t;

  if (t == 0 && thresh < 0) {
    /* Python: no valid split -> an all-zero ink mask.  The geometric split
     * below still runs, so the caller gets its `count` crops. */
    memset(scratch->ink, 0, frame_px);
  } else {
    sign_binarize_ink(gray_in, frame_px, t, polarity, scratch->ink);
    apply_dark_floor(gray_in, frame_px, dark_thr, scratch->ink);
  }

  /* Interior character-band ROI: the same border/speck/frame cleanup and
   * tight padded box as ocr_preprocess.find_interior_ink_roi /
   * preprocess_sign_sequence(find_roi=True).  With segmentation off the
   * caller already handed us the tight frame, so there is nothing to find. */
  if (apply_segmentation) {
    SignBox roi;
    float pct = 0.0f;
    if (interior_roi_c(scratch->ink, w, h, scratch, &roi, &pct)) {
      if (info) {
        info->has_roi = true;
        info->roi_box = roi;
        info->sign_pct = pct;
      }
    } else if (info) {
      info->roi_box.x1 = 0; info->roi_box.y1 = 0;
      info->roi_box.x2 = w; info->roi_box.y2 = h;
      info->sign_pct = pct;
    }
  } else if (info) {
    long count = 0;
    for (size_t i = 0; i < frame_px; i++) count += scratch->ink[i] ? 1 : 0;
    info->sign_pct = (frame_px > 0)
                         ? (float)(100.0 * (double)count / (double)frame_px)
                         : 0.0f;
  }

  /* ── the split (physical left-to-right) ───────────────────────────────── */
  const int box_cap = w + SIGN_SEQUENCE_LEN;
  int num_boxes;
  if (apply_segmentation) {
    const int raw = sign_segment_glyph_boxes_c(scratch->ink, w, h, min_run,
                                               scratch->proj, scratch->boxes,
                                               box_cap);
    if (raw < 0) return false;
    if (info) info->num_boxes = raw;
    num_boxes = split_from_boxes(scratch->ink, w, h, n, scratch->proj,
                                 scratch->boxes, box_cap, raw);
    if (num_boxes < 0) return false;
  } else {
    /* The frame is already the sign (a tight capture): equal bands. */
    equal_split(0, w, 0, h, n, scratch->boxes);
    num_boxes = n;
    if (info) info->num_boxes = n;
  }
  if (num_boxes != n) return false;

  /* RTL reverses the LIST, never the pixels: the split is physical, and
   * "rtl" means the leftmost glyph is the LAST character of the label. */
  if (reading_order == 1) {
    for (int i = 0; i < n / 2; i++) {
      const SignBox tmp = scratch->boxes[i];
      scratch->boxes[i] = scratch->boxes[n - 1 - i];
      scratch->boxes[n - 1 - i] = tmp;
    }
  }

  /* ── one preprocess_glyph per slot, on that slot's own crop ───────────── */
  bool any_ok = false;
  for (int idx = 0; idx < n; idx++) {
    const SignBox b = scratch->boxes[idx];
    SignSlotInfo *sl = &slots[idx];
    sl->thresh = 0;
    sl->ok = false;
    sl->empty = true;

    /* Python preprocess_sign_sequence pads every slot by max(2, 8 % of
     * the smaller side) so a stroke-only '1' keeps enough bright plate
     * context not to read as _background.  Mirror it exactly, including
     * the integer rounding and clamping. */
    int bw = b.x2 - b.x1;
    int bh = b.y2 - b.y1;
    if (bw < 1) bw = 1;   /* Python: bw = max(1, x2 - x1) */
    if (bh < 1) bh = 1;
    const int smaller = (bw < bh) ? bw : bh;
    int pad = (smaller * 8 + 50) / 100;
    if (pad < 2) pad = 2;
    SignBox pb;
    pb.x1 = (b.x1 - pad > 0) ? (b.x1 - pad) : 0;
    pb.y1 = (b.y1 - pad > 0) ? (b.y1 - pad) : 0;
    pb.x2 = (b.x2 + pad < w) ? (b.x2 + pad) : w;
    pb.y2 = (b.y2 + pad < h) ? (b.y2 + pad) : h;
    sl->box = pb;

    const int cw = pb.x2 - pb.x1;
    const int ch = pb.y2 - pb.y1;
    if (cw <= 0 || ch <= 0) continue;   /* Python: crop.size == 0 -> blank */

    /* Copy the strided crop into the scratch gray buffer.  A copy is safe:
     * Otsu, the mask and the normaliser all read values, not addresses. */
    for (int y = 0; y < ch; y++) {
      memcpy(scratch->gray + (size_t)y * (size_t)cw,
             gray_in + (size_t)(pb.y1 + y) * (size_t)w + (size_t)pb.x1,
             (size_t)cw);
    }

    SignGlyphInfo slot_res;
    preprocess_common(scratch->gray, cw, ch, out_size, polarity,
                      /* apply_segmentation = */ false, min_run, thresh,
                      dark_thr, convention, scratch,
                      glyphs_out + (size_t)idx * out_px, &slot_res);
    sl->thresh = slot_res.thresh;
    sl->ok = slot_res.ok;
    sl->empty = slot_res.empty;
    any_ok = any_ok || slot_res.ok;
  }

  if (info) {
    info->num_slots = n;
    info->box = union_box(scratch->boxes[0], scratch->boxes[n - 1]);
    info->has_box = true;
    info->ok = any_ok;
    info->empty = !any_ok;
  }
  return true;
}

void sign_glyph_to_int8(const uint8_t *glyph, size_t n, int8_t *out) {
  for (size_t i = 0; i < n; i++) out[i] = (int8_t)((int)glyph[i] - 128);
}

/* ════════════════════════════════════════════════════════════════════════
 * 8. Scratch management
 * ════════════════════════════════════════════════════════════════════════ */
static size_t align_up(size_t v, size_t a) { return (v + (a - 1)) & ~(a - 1); }

size_t sign_scratch_bytes(int w, int h, int out_size) {
  if (w < 0) w = 0;
  if (h < 0) h = 0;
  if (out_size < 0) out_size = 0;
  const size_t frame = (size_t)w * (size_t)h;
  const size_t out_px = (size_t)out_size * (size_t)out_size;
  return align_up(frame, 4)                          /* gray    */
       + align_up(frame, 4)                          /* ink     */
       + align_up(frame, 4)                          /* visited */
       + align_up(frame * sizeof(int32_t), 4)        /* stack   */
       + align_up(out_px, 4)                         /* canvas  */
       + align_up(out_px, 4)                         /* canvas2 */
       + align_up((size_t)w * sizeof(int32_t), 4)    /* proj    */
       + align_up((size_t)(w + SIGN_SEQUENCE_LEN) * sizeof(SignBox), 4) /* boxes */
       + 8;
}

bool sign_scratch_init(SignScratch *scratch, void *block, size_t block_bytes,
                       int w, int h, int out_size) {
  if (!scratch) return false;
  scratch->gray = scratch->ink = scratch->canvas = scratch->canvas2 = NULL;
  scratch->visited = NULL;
  scratch->stack = NULL;
  scratch->proj = NULL;
  scratch->boxes = NULL;
  if (!block || w < 0 || h < 0 || out_size < 0) return false;
  if (block_bytes < sign_scratch_bytes(w, h, out_size)) return false;

  uint8_t *p = (uint8_t *)block;
  const size_t frame = (size_t)w * (size_t)h;
  const size_t out_px = (size_t)out_size * (size_t)out_size;

  scratch->gray = p;    p += align_up(frame, 4);
  scratch->ink = p;     p += align_up(frame, 4);
  scratch->visited = p; p += align_up(frame, 4);
  scratch->stack = (int32_t *)p;
  p += align_up(frame * sizeof(int32_t), 4);
  scratch->canvas = p;  p += align_up(out_px, 4);
  scratch->canvas2 = p; p += align_up(out_px, 4);
  scratch->proj = (int32_t *)p;
  p += align_up((size_t)w * sizeof(int32_t), 4);
  scratch->boxes = (SignBox *)p;
  return true;
  /* `p` now points at the start of the (w + SIGN_SEQUENCE_LEN)-entry box
   * region; the byte count above reserved it. */
}
