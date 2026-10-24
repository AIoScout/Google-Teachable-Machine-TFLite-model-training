/* Host harness for the sign-sequence front-end: replays a manifest through
 * sign_glyph.c and writes one output set per case for the Python parity
 * test (AItraining/tests/test_sign_glyph_parity.py).
 *
 *   cc -O2 -std=c99 -Wall -Wextra -o sign_glyph_host \
 *       sign_glyph_host.c ../sign_glyph.c
 *   ./sign_glyph_host <corpus_dir>
 *
 * manifest.tsv, one case per line after a header:
 *
 *   name mode w h out_size count order polarity seg min_run thresh dark_thr conv x0 y0
 *
 *   mode 0  sequence: <name>.raw is w*h grayscale bytes -> preprocess_sign_sequence_c
 *   mode 1  split:    <name>.raw is w*h ink bytes (0 = background, else ink)
 *                     -> split_sign_glyph_boxes_c
 *   mode 2  strided Otsu: threshold of the crop (x0,y0,count,order) inside the
 *                     w*h frame -> sign_otsu_threshold_strided
 *   mode 3  single glyph: <name>.raw is w*h grayscale -> sign_preprocess_glyph
 *   mode 4  luminance: <name>.raw is w*h*3 RGB -> sign_rgb_to_lum_u8
 *
 * mode 0 writes <name>.cmeta (threshold, sign box, per-slot boxes/flags) and
 * <name>.cglyphs (count packed out_size*out_size glyphs).  mode 1 writes
 * <name>.cboxes (one `x1,y1,x2,y2` line per box).  Mode 2 writes
 * <name>.cot (`thresh=N`).  Mode 3 writes <name>.cmeta and one
 * out_size*out_size glyph to <name>.cglyphs.  Mode 4 writes <name>.cgray.
 *
 * The harness also cross-checks sign_scratch_bytes against sign_scratch_init,
 * so a mis-sized device scratch buffer fails here rather than on the ESP32.
 *
 * The parity corpus includes a white plate whose 'A' sits inside the old 6 %
 * frame margin and whose '1' is broken into two strokes.  roi_box and every
 * 28x28 glyph are compared with Python, so an edge-glyph erasure or a
 * border-line bleed in sign_glyph.c fails here before it reaches the P4.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../sign_glyph.h"

#define MAX_NAME 128

/* Read exactly n bytes; refuse (without writing dst) when n > cap.  Same rule
 * as the OCR harness: an oversized case is a broken corpus, and the check
 * must happen before the read or it cannot prevent the overrun. */
static int read_file(const char *path, uint8_t *dst, size_t cap, size_t n) {
  if (n > cap) return 0;
  FILE *f = fopen(path, "rb");
  if (!f) return 0;
  const size_t got = fread(dst, 1, n, f);
  const int extra = fgetc(f) != EOF;
  fclose(f);
  return got == n && !extra;
}

static void write_file(const char *path, const uint8_t *src, size_t n) {
  FILE *f = fopen(path, "wb");
  if (!f) { perror(path); exit(1); }
  if (fwrite(src, 1, n, f) != n) { perror(path); exit(1); }
  fclose(f);
}

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "usage: %s <corpus_dir>\n", argv[0]);
    return 2;
  }
  char path[4096];
  snprintf(path, sizeof(path), "%s/manifest.tsv", argv[1]);
  FILE *mf = fopen(path, "r");
  if (!mf) { perror(path); return 1; }

  /* The envelope is SIGN_MAX_PIXELS = 64K; the corpus is a handful of small
   * frames, so a couple of static buffers keep this harness malloc-free. */
  static uint8_t frame[SIGN_MAX_PIXELS * 3];
  static uint8_t ink[SIGN_MAX_PIXELS];
  static uint8_t gray_buf[SIGN_MAX_PIXELS];
  static uint8_t glyphs[(size_t)SIGN_SEQUENCE_LEN * 64 * 64];
  static SignBox boxes[SIGN_MAX_W];
  static SignSlotInfo slots[SIGN_SEQUENCE_LEN];
  static int32_t proj[SIGN_MAX_W];

  uint8_t *scratch_block = NULL;
  size_t scratch_bytes = 0;

  char line[512];
  int cases = 0;
  if (!fgets(line, sizeof(line), mf)) { fclose(mf); return 1; }  /* header */

  while (fgets(line, sizeof(line), mf)) {
    char name[MAX_NAME];
    int mode, w, h, out_size, count, order, polarity, seg, min_run, thresh,
        dark_thr, convention, x0, y0;
    if (sscanf(line, "%127s %d %d %d %d %d %d %d %d %d %d %d %d %d %d", name,
               &mode, &w, &h, &out_size, &count, &order, &polarity, &seg,
               &min_run, &thresh, &dark_thr, &convention, &x0,
               &y0) != 15) {
      fprintf(stderr, "bad manifest line: %s", line);
      return 1;
    }
    if (w <= 0 || h <= 0 || (int64_t)w * (int64_t)h > SIGN_MAX_PIXELS) {
      fprintf(stderr, "case %s is outside SIGN_MAX_PIXELS\n", name);
      return 1;
    }
    const size_t px = (size_t)w * (size_t)h;
    snprintf(path, sizeof(path), "%s/%s.raw", argv[1], name);
    uint8_t *src = (mode == 1) ? ink : frame;
    const size_t want = (mode == 4) ? px * 3u : px;
    if (!read_file(path, src, sizeof(frame), want)) {
      fprintf(stderr, "cannot read %s (%zu bytes)\n", path, want);
      return 1;
    }

    if (mode == 2) {
      if (x0 < 0 || y0 < 0 || count <= 0 || order <= 0 ||
          x0 + count > w || y0 + order > h) {
        fprintf(stderr, "case %s has an out-of-frame strided crop\n", name);
        return 1;
      }
      const int t = sign_otsu_threshold_strided(
          frame + (size_t)y0 * (size_t)w + (size_t)x0, w, count, order);
      snprintf(path, sizeof(path), "%s/%s.cot", argv[1], name);
      FILE *f = fopen(path, "w");
      if (!f) { perror(path); return 1; }
      fprintf(f, "thresh=%d\n", t);
      fclose(f);
      cases++;
      continue;
    }

    if (mode == 4) {
      sign_rgb_to_lum_u8(frame, px, gray_buf);
      snprintf(path, sizeof(path), "%s/%s.cgray", argv[1], name);
      write_file(path, gray_buf, px);
      cases++;
      continue;
    }

    if (mode == 3) {
      if (out_size <= 0 || out_size > 64) {
        fprintf(stderr, "case %s has out_size %d\n", name, out_size);
        return 1;
      }
      const size_t need = sign_scratch_bytes(w, h, out_size);
      if (need > scratch_bytes) {
        free(scratch_block);
        scratch_block = (uint8_t *)malloc(need);
        if (!scratch_block) { fprintf(stderr, "oom\n"); return 1; }
        scratch_bytes = need;
      }
      SignScratch s;
      if (!sign_scratch_init(&s, scratch_block, scratch_bytes, w, h,
                             out_size)) {
        fprintf(stderr, "sign_scratch_init refused %s\n", name);
        return 1;
      }
      SignGlyphInfo info;
      const bool ok = sign_preprocess_glyph(
          frame, w, h, out_size, (SignPolarity)polarity, seg != 0, min_run,
          thresh, dark_thr, (SignConvention)convention, &s, glyphs, &info);
      snprintf(path, sizeof(path), "%s/%s.cmeta", argv[1], name);
      FILE *f = fopen(path, "w");
      if (!f) { perror(path); return 1; }
      fprintf(f, "thresh=%d nb=%d ns=%d has_box=%d ok=%d empty=%d has_roi=%d "
                 "pct=%.6f box=",
              info.thresh, info.num_boxes, info.num_slots, info.has_box ? 1 : 0,
              info.ok ? 1 : 0, info.empty ? 1 : 0, info.has_roi ? 1 : 0,
              (double)info.sign_pct);
      if (info.has_box) {
        fprintf(f, "%d,%d,%d,%d\n", info.box.x1, info.box.y1, info.box.x2,
                info.box.y2);
      } else {
        fprintf(f, "-\n");
      }
      fclose(f);
      snprintf(path, sizeof(path), "%s/%s.cglyphs", argv[1], name);
      write_file(path, glyphs,
                 (size_t)out_size * (size_t)out_size);
      cases++;
      (void)ok;
      continue;
    }

    if (mode == 1) {
      const int nb = split_sign_glyph_boxes_c(src, w, h, count, min_run, proj,
                                              boxes, (int)SIGN_MAX_W);
      snprintf(path, sizeof(path), "%s/%s.cboxes", argv[1], name);
      FILE *f = fopen(path, "w");
      if (!f) { perror(path); return 1; }
      fprintf(f, "n=%d\n", nb);
      for (int i = 0; i < nb; i++) {
        fprintf(f, "%d,%d,%d,%d\n", boxes[i].x1, boxes[i].y1, boxes[i].x2,
                boxes[i].y2);
      }
      fclose(f);
      cases++;
      continue;
    }

    if (out_size <= 0 || out_size > 64) {
      fprintf(stderr, "case %s has out_size %d\n", name, out_size);
      return 1;
    }
    if (count < 1 || count > SIGN_SEQUENCE_LEN) {
      fprintf(stderr, "case %s has count %d\n", name, count);
      return 1;
    }
    const size_t need = sign_scratch_bytes(w, h, out_size);
    if (need > scratch_bytes) {
      free(scratch_block);
      scratch_block = (uint8_t *)malloc(need);
      if (!scratch_block) { fprintf(stderr, "oom\n"); return 1; }
      scratch_bytes = need;
    }
    SignScratch s;
    if (!sign_scratch_init(&s, scratch_block, scratch_bytes, w, h, out_size)) {
      fprintf(stderr, "sign_scratch_init refused %s\n", name);
      return 1;
    }

    SignGlyphInfo info;
    const bool ok = preprocess_sign_sequence_c(
        frame, w, h, out_size, count, order, (SignPolarity)polarity,
        seg != 0, min_run, thresh, dark_thr, (SignConvention)convention, &s,
        glyphs, slots, &info);
    if (!ok) {
      fprintf(stderr, "preprocess_sign_sequence_c refused %s\n", name);
      return 1;
    }

    snprintf(path, sizeof(path), "%s/%s.cmeta", argv[1], name);
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return 1; }
    fprintf(f, "thresh=%d nb=%d ns=%d has_box=%d ok=%d empty=%d has_roi=%d "
               "pct=%.6f roi=",
            info.thresh, info.num_boxes, info.num_slots, info.has_box ? 1 : 0,
            info.ok ? 1 : 0, info.empty ? 1 : 0, info.has_roi ? 1 : 0,
            (double)info.sign_pct);
    if (info.roi_box.x2 > info.roi_box.x1 &&
        info.roi_box.y2 > info.roi_box.y1) {
      fprintf(f, "%d,%d,%d,%d box=", info.roi_box.x1, info.roi_box.y1,
              info.roi_box.x2, info.roi_box.y2);
    } else {
      fprintf(f, "- box=");
    }
    if (info.has_box) {
      fprintf(f, "%d,%d,%d,%d\n", info.box.x1, info.box.y1, info.box.x2,
              info.box.y2);
    } else {
      fprintf(f, "-\n");
    }
    for (int i = 0; i < count; i++) {
      fprintf(f, "slot%d thresh=%d ok=%d empty=%d box=%d,%d,%d,%d\n", i,
              slots[i].thresh, slots[i].ok ? 1 : 0, slots[i].empty ? 1 : 0,
              slots[i].box.x1, slots[i].box.y1, slots[i].box.x2,
              slots[i].box.y2);
    }
    fclose(f);

    snprintf(path, sizeof(path), "%s/%s.cglyphs", argv[1], name);
    write_file(path, glyphs, (size_t)count * (size_t)out_size *
                                (size_t)out_size);
    cases++;
  }
  fclose(mf);
  free(scratch_block);
  printf("sign_glyph_host: %d cases\n", cases);
  return 0;
}
