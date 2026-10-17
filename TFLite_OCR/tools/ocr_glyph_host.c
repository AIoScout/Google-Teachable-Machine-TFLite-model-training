/* Host harness for the C glyph pipeline: replays a corpus manifest through
 * ocr_glyph.c and writes one output per case for tools/compare.py.
 *
 *   cc -O2 -std=c99 -Wall -Wextra -o ocr_glyph_host ocr_glyph_host.c ../ocr_glyph.c
 *   ./ocr_glyph_host <corpus_dir>
 *
 * It also cross-checks ocr_scratch_init against ocr_scratch_bytes, so a
 * mis-sized device scratch buffer fails here rather than on the ESP32.
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "../ocr_glyph.h"

#define MAX_NAME 128

/* Reads exactly n bytes into dst, which holds cap.  Refuses — without writing
 * a single byte to dst — when the manifest asks for more than cap: a corpus
 * case outside ocr_glyph.h's envelope is a broken corpus, and this harness
 * exists to report that, not to run off the end of its own buffer reporting
 * it.  (It used to fread first and check the count afterwards, which is the
 * one order that cannot work: the overrun happens on the way to the check.) */
static int read_file(const char *path, uint8_t *dst, size_t cap, size_t n) {
  if (n > cap) return 0;
  FILE *f = fopen(path, "rb");
  if (!f) return 0;
  const size_t got = fread(dst, 1, n, f);
  /* Trailing bytes would mean the corpus and the manifest disagree. */
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

  /* Big enough for the corpus' largest frame (128x128 = 16K px, x3 RGB). */
  static uint8_t frame[OCR_MAX_PIXELS * 3];
  static uint8_t glyph[64 * 64];
  static int8_t q8[64 * 64];
  uint8_t *scratch_block = NULL;
  size_t scratch_bytes = 0;

  char line[512];
  int cases = 0, ok_count = 0, inline_ok = 0;
  if (!fgets(line, sizeof(line), mf)) { fclose(mf); return 1; }  /* header */

  while (fgets(line, sizeof(line), mf)) {
    char name[MAX_NAME];
    int kind, w, h, out_size, polarity, seg, min_run, thresh, convention;
    if (sscanf(line, "%127s %d %d %d %d %d %d %d %d %d", name, &kind, &w, &h,
               &out_size, &polarity, &seg, &min_run, &thresh,
               &convention) != 10) {
      fprintf(stderr, "bad manifest line: %s", line);
      return 1;
    }
    const size_t px = (size_t)w * (size_t)h;
    const size_t want = px * (kind == 1 ? 3u : 1u);
    snprintf(path, sizeof(path), "%s/%s.raw", argv[1], name);
    if (!read_file(path, frame, sizeof(frame), want)) {
      if (want > sizeof(frame)) {
        /* Say which case and both numbers: "cannot read" for an oversized
         * frame reads like a missing file and sends you looking in the wrong
         * place.  Reaching this means the manifest was not written by the
         * current gen_corpus.py. */
        fprintf(stderr,
                "case %s wants %zu bytes but this harness' frame buffer is "
                "only %zu (OCR_MAX_PIXELS * 3).\n"
                "The manifest describes a frame outside ocr_glyph.h's "
                "envelope; regenerate the corpus "
                "(python3 tools/gen_corpus.py --app-data <workspace>).\n",
                name, want, sizeof(frame));
      } else {
        fprintf(stderr, "cannot read %s (%zu bytes)\n", path, want);
      }
      return 1;
    }

    /* Rebuild the scratch when the frame size grows. */
    const size_t need = ocr_scratch_bytes(w, h, out_size);
    if (need > scratch_bytes) {
      free(scratch_block);
      scratch_block = (uint8_t *)malloc(need);
      if (!scratch_block) { fprintf(stderr, "oom\n"); return 1; }
      scratch_bytes = need;
    }
    OcrScratch s;
    if (!ocr_scratch_init(&s, scratch_block, scratch_bytes, w, h, out_size)) {
      fprintf(stderr, "ocr_scratch_init refused %s\n", name);
      return 1;
    }

    OcrGlyphInfo info;
    int ok;
    if (kind == 1) {
      ok = ocr_preprocess_rgb(frame, w, h, out_size, (OcrPolarity)polarity,
                              seg != 0, min_run, thresh, (OcrConvention)convention,
                              &s, glyph, &info);
    } else {
      ok = ocr_preprocess_gray(frame, w, h, out_size, (OcrPolarity)polarity,
                               seg != 0, min_run, thresh,
                               (OcrConvention)convention, &s, glyph, &info);
    }

    /* The same frame through the two-entry-point pair must agree: the
     * grayscale path is a memcpy of the caller's buffer, so a divergence
     * would mean the RGB luminance stage wrote somewhere it should not. */
    if (kind == 1) {
      uint8_t glyph2[64 * 64];
      OcrGlyphInfo info2;
      const int ok2 = ocr_preprocess_gray(s.gray, w, h, out_size,
                                          (OcrPolarity)polarity, seg != 0,
                                          min_run, thresh,
                                          (OcrConvention)convention, &s,
                                          glyph2, &info2);
      if (ok2 != ok || info2.thresh != info.thresh ||
          memcmp(glyph2, glyph, (size_t)out_size * (size_t)out_size) != 0) {
        fprintf(stderr, "entry-point divergence on %s\n", name);
        return 1;
      }
      inline_ok++;
    }

    snprintf(path, sizeof(path), "%s/%s.cout", argv[1], name);
    write_file(path, glyph, (size_t)out_size * (size_t)out_size);

    snprintf(path, sizeof(path), "%s/%s.cmeta", argv[1], name);
    FILE *f = fopen(path, "w");
    if (!f) { perror(path); return 1; }
    fprintf(f, "thresh=%d ok=%d boxes=%d box=", info.thresh, ok ? 1 : 0,
            info.num_boxes);
    if (info.has_box) {
      fprintf(f, "%d,%d,%d,%d\n", info.box.x1, info.box.y1, info.box.x2,
              info.box.y2);
    } else {
      fprintf(f, "-\n");
    }
    fclose(f);

    /* The int8 tensor the model actually consumes: glyph - 128, no clamp
     * needed because a uint8 glyph already spans exactly [-128, 127].
     * Written out so the LAST stage is compared too, not just assumed. */
    ocr_glyph_to_int8(glyph, (size_t)out_size * (size_t)out_size, q8);
    snprintf(path, sizeof(path), "%s/%s.cq8", argv[1], name);
    write_file(path, (const uint8_t *)q8, (size_t)out_size * (size_t)out_size);

    cases++;
    ok_count += ok ? 1 : 0;
  }
  fclose(mf);
  free(scratch_block);
  printf("ocr_glyph_host: %d cases (%d ok, %d rgb entry-point cross-checks)\n",
         cases, ok_count, inline_ok);
  return 0;
}
