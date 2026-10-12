#!/usr/bin/env python3
"""Build the C-port parity corpus and its Python golden outputs.

Every case is written as
    <out>/<name>.raw      the input frame (gray: w*h, rgb: w*h*3)
    <out>/<name>.golden   preprocess_glyph's `glyph` (out_size^2 uint8)
    <out>/<name>.gmeta    thresh / ok / box count / union box, one line
and listed in <out>/manifest.tsv, which the C harness reads.

The corpus is mostly adversarial on purpose.  The pipeline's failure modes
are not "off by one pixel" — they are *decisions* that flip:

  * the Otsu winner landing on an OCCUPIED histogram level (exactly where
    `v <= t` and `v < t` disagree, i.e. where an "equivalent" implementation
    silently diverges),
  * a flat frame (no valid split at all -> t = 0),
  * runs dropped by min_run, and ink outside every kept run's rows (the
    kept-union vs span-union box difference),
  * an empty class, zero-mass canvases, a crop with one row or one column,
  * aspect ratios far from square, where STRETCH and CANON disagree most.

Real captures are included because synthetic images are too well behaved:
their histograms are smooth and their boxes are unambiguous.  Point
--app-data at the app's workspace to mix in as many as you like.
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "AItraining"))

from ocr_preprocess import (  # noqa: E402
    GLYPH_SIZE,
    INK_BRIGHT,
    INK_DARK,
    NORM_CANON,
    NORM_STRETCH,
    preprocess_glyph,
)

CONV = {NORM_STRETCH: 0, NORM_CANON: 1}
MANIFEST_HEADER = "name\tkind\tw\th\tout_size\tpolarity\tseg\tmin_run\tthresh\tconvention\n"


# ── deterministic drawing helpers (numpy only, no PIL filters) ───────────
def _blur3(a: np.ndarray) -> np.ndarray:
    """3x3 box blur, integer, edges clamped — soft (anti-aliased) glyph edges."""
    p = np.pad(a.astype(np.int32), 1, mode="edge")
    out = np.zeros_like(a, dtype=np.int32)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            out += p[dy:dy + a.shape[0], dx:dx + a.shape[1]]
    return (out // 9).astype(np.uint8)


def _paper(h: int, w: int, rng: np.random.Generator, bright: int = 235,
           grain: int = 12) -> np.ndarray:
    base = np.full((h, w), bright, dtype=np.int32)
    base += rng.integers(-grain, grain + 1, size=(h, w))
    g = rng.integers(-grain, grain + 1, size=(h, w))
    return np.clip(base + g // 2, 0, 255).astype(np.uint8)


def _glyph_mask(h: int, w: int, rng: np.random.Generator, kind: str) -> np.ndarray:
    m = np.zeros((h, w), dtype=np.uint8)
    y0 = int(rng.integers(0, max(1, h // 3)))
    x0 = int(rng.integers(0, max(1, w // 3)))
    y1 = int(rng.integers(max(y0 + 1, h - h // 3), h + 1))
    x1 = int(rng.integers(max(x0 + 1, w - w // 3), w + 1))
    if kind == "block":
        m[y0:y1, x0:x1] = 1
    elif kind == "bar":
        cx = (x0 + x1) // 2
        m[y0:y1, max(0, cx - 1):cx + 2] = 1
    elif kind == "hline":
        cy = (y0 + y1) // 2
        m[max(0, cy - 1):cy + 2, x0:x1] = 1
    elif kind == "ellipse":
        yy, xx = np.mgrid[0:h, 0:w]
        cy, cx = (y0 + y1) / 2.0, (x0 + x1) / 2.0
        ry, rx = max(1.0, (y1 - y0) / 2.0), max(1.0, (x1 - x0) / 2.0)
        m[(((yy - cy) / ry) ** 2 + ((xx - cx) / rx) ** 2) <= 1.0] = 1
    elif kind == "thin":  # 1-px strokes: dropped as runs when min_run > 1
        m[y0:y1, x0] = 1
        m[y0, x0:x1] = 1
        m[y0:y1, max(0, x1 - 1)] = 1
    elif kind == "cross":
        cy, cx = (y0 + y1) // 2, (x0 + x1) // 2
        m[y0:y1, max(0, cx - 1):cx + 2] = 1
        m[max(0, cy - 1):cy + 2, x0:x1] = 1
    return m


def synth_frame(rng: np.random.Generator, h: int, w: int, kind: str,
                polarity: int, antialias: bool, ink: int | None = None) -> np.ndarray:
    if kind == "noise":
        return rng.integers(0, 256, size=(h, w), dtype=np.uint8)
    if kind == "gradient":
        yy, xx = np.mgrid[0:h, 0:w]
        g = (xx * 255 // max(1, w - 1) + yy * 255 // max(1, h - 1)) // 2
        g = np.clip(g.astype(np.int32) + rng.integers(-8, 9, size=(h, w)), 0, 255)
        return g.astype(np.uint8)

    frame = _paper(h, w, rng) if polarity == INK_DARK else _paper(h, w, rng, bright=20, grain=10)
    mask = _glyph_mask(h, w, rng, kind)
    dark = int(ink if ink is not None else rng.integers(10, 90))
    bright = int(rng.integers(170, 250))
    ink_v, bg_v = (dark, bright) if polarity == INK_DARK else (bright, dark)
    frame = np.where(mask.astype(bool), np.uint8(ink_v), frame).astype(np.uint8)
    if antialias:
        soft = _blur3(frame)
        # blend only near the mask edge so the glyph stays crisp inside
        edge = (_blur3(mask.astype(np.uint8) * 255) > 0) & (_blur3(mask.astype(np.uint8) * 255) < 255)
        frame = np.where(edge, soft, frame)
    return frame.astype(np.uint8)


def levels_frame(rng: np.random.Generator, h: int, w: int, levels, counts) -> np.ndarray:
    """A frame occupying few levels, with the weights chosen so Otsu's winner
    is the level itself — the `<=` vs `<` trap."""
    vals = np.repeat(np.array(levels, dtype=np.uint8), np.array(counts, dtype=np.int64))
    if vals.size < h * w:
        vals = np.concatenate([vals, np.full(h * w - vals.size, levels[-1], dtype=np.uint8)])
    vals = vals[: h * w]
    rng.shuffle(vals)
    return vals.reshape(h, w)


def build_cases(app_data: Path | None, n_real: int, seed: int):
    """→ list of (name, kind, frame, kwargs)"""
    rng = np.random.default_rng(seed)
    cases = []

    def add(name, frame, *, kind="gray", out_size=GLYPH_SIZE, polarity=INK_DARK,
            seg=True, min_run=2, thresh=None, convention=NORM_CANON):
        cases.append((name, kind, np.ascontiguousarray(frame),
                      dict(out_size=out_size, polarity=polarity, seg=seg,
                           min_run=min_run, thresh=thresh, convention=convention)))

    # ── real captures ───────────────────────────────────────────────────
    if app_data and n_real > 0:
        pngs = sorted(app_data.rglob("*.png"))
        if pngs:
            picks = rng.permutation(len(pngs))[:n_real]
            from PIL import Image
            for i, idx in enumerate(picks):
                try:
                    g = np.asarray(Image.open(pngs[int(idx)]).convert("L"), dtype=np.uint8)
                except Exception:
                    continue
                if g.ndim != 2 or g.size == 0:
                    continue
                add(f"real{i:04d}", g)
            print(f"  real captures: {sum(1 for c in cases if c[0].startswith('real'))}")

    # ── degenerate frames ───────────────────────────────────────────────
    for level in range(0, 256, 17):
        add(f"flat{level:03d}", np.full((96, 96), level, dtype=np.uint8))
    add("black1x1", np.zeros((1, 1), dtype=np.uint8))
    add("white1x1", np.full((1, 1), 255, dtype=np.uint8))
    add("row1x64", np.arange(64, dtype=np.uint8).reshape(1, 64))
    add("col64x1", np.arange(64, dtype=np.uint8).reshape(64, 1))
    add("tiny2x2", np.array([[0, 255], [255, 0]], dtype=np.uint8))
    add("tiny3x3", np.array([[0, 0, 0], [0, 255, 0], [0, 0, 0]], dtype=np.uint8))
    add("one_pixel_ink", np.full((32, 32), 250, dtype=np.uint8))  # then poke
    cases[-1][2][16, 16] = 0

    # ── Otsu-tie / occupied-level traps ────────────────────────────────
    add("lev_0_100_200", levels_frame(rng, 96, 96, [0, 100, 200], [1000, 1, 1]))
    add("lev_0_1_255", levels_frame(rng, 96, 96, [0, 1, 255], [1000, 1, 1]))
    add("lev_0_128", levels_frame(rng, 96, 96, [0, 128], [1, 1]))
    add("lev_10_20_25_240", levels_frame(rng, 96, 96, [10, 20, 25, 240], [5, 4000, 300, 20]))
    add("lev_uneven", levels_frame(rng, 96, 96, [7, 8, 9, 200, 201], [1, 5000, 1, 1, 4000]))

    # ── random noise at many sizes (the workhorse) ──────────────────────
    for i in range(24):
        side = int(rng.choice([8, 16, 24, 32, 48, 64, 96, 128]))
        add(f"noise{i:03d}_{side}", rng.integers(0, 256, size=(side, side), dtype=np.uint8))
    for i in range(6):
        h = int(rng.integers(1, 100)); w = int(rng.integers(1, 100))
        add(f"noise_rect{i:02d}_{h}x{w}", rng.integers(0, 256, size=(h, w), dtype=np.uint8))

    # ── glyph-ish frames: both polarities, both conventions ─────────────
    kinds = ["block", "bar", "hline", "ellipse", "thin", "cross"]
    for i in range(18):
        kind = kinds[i % len(kinds)]
        pol = INK_DARK if (i % 2 == 0) else INK_BRIGHT
        aa = (i % 3 != 0)
        side = int(rng.choice([48, 64, 96, 96, 128]))
        frame = synth_frame(rng, side, side, kind, pol, aa)
        add(f"glyph{i:02d}_{kind}_{side}", frame, polarity=pol)
        add(f"glyph{i:02d}_{kind}_{side}_stretch", frame, polarity=pol,
            convention=NORM_STRETCH)
        add(f"glyph{i:02d}_{kind}_{side}_nosplit", frame, polarity=pol,
            convention=NORM_STRETCH, min_run=1)

    # ── union box: every KEPT run contributes, not just the first ───────
    #    The glyph box is the UNION of the per-run bboxes.  An implementation
    #    that keeps the first kept run's top row (or bottom row, or right
    #    column) passes every randomly-shaped frame above, because a random
    #    glyph is tallest on its leftmost run — which is exactly the hole
    #    mutation_check found.  These frames put the extreme ink in a LATER
    #    run, so the first-run answer and the union answer differ.
    def two_runs(blocks, hgt=96, wid=96, paper=235, ink=20):
        f = np.full((hgt, wid), paper, dtype=np.uint8)
        for (ya, yb, xa, xb) in blocks:
            f[ya:yb, xa:xb] = ink
        return f

    add("union_top_late", two_runs([(40, 70, 10, 16), (5, 70, 30, 36)]))
    add("union_top_first", two_runs([(5, 70, 10, 16), (40, 70, 30, 36)]))
    add("union_bot_late", two_runs([(5, 40, 10, 16), (5, 70, 30, 36)]))
    add("union_bot_first", two_runs([(5, 70, 10, 16), (5, 40, 30, 36)]))
    add("union_right_late", two_runs([(5, 70, 10, 16), (5, 70, 30, 36)]))
    # A 1-px full-height sliver between the runs, dropped by min_run=2: a
    # union over the kept runs is unaffected, a union over the whole column
    # SPAN (first to last run) would stretch the box to row 0.
    add("union_sliver_span", two_runs([(40, 70, 10, 16), (5, 70, 30, 36),
                                       (0, 96, 20, 21)]))
    # Same shapes in the bright polarity (the EMNIST side of the mix).
    add("union_top_late_bright", two_runs([(40, 70, 10, 16), (5, 70, 30, 36)],
                                          paper=20, ink=235),
        polarity=INK_BRIGHT)
    add("union_top_late_stretch", two_runs([(40, 70, 10, 16), (5, 70, 30, 36)]),
        convention=NORM_STRETCH)

    # ── EMNIST-shaped: bright ink, already isolated, no segmentation ────
    for i in range(6):
        frame = synth_frame(rng, 28, 28, kinds[i % len(kinds)], INK_BRIGHT, True)
        add(f"emnist{i:02d}", frame, polarity=INK_BRIGHT, seg=False)
        add(f"emnist{i:02d}_stretch", frame, polarity=INK_BRIGHT, seg=False,
            convention=NORM_STRETCH)

    # ── non-standard out_size (exercises the CANON fit scaling branch) ──
    for osz in (8, 16, 20, 28, 32):
        frame = synth_frame(rng, 96, 96, "cross", INK_DARK, True)
        add(f"outsize{osz}", frame, out_size=osz)

    # ── min_run sweep: the run filter is an off-by-one away from a
    #    different box, so the same frame is swept across every width ────
    for i, kind in enumerate(("thin", "bar", "cross")):
        frame = synth_frame(rng, 96, 96, kind, INK_DARK, False)
        for mr in (1, 2, 3, 4, 5):
            add(f"minrun{kind}{mr}", frame, min_run=mr)

    # ── bright ink WITH segmentation: the `v > t` predicate only differs
    #    from `v >= t` at a pixel exactly equal to t, so force one ──────
    for i in range(6):
        frame = synth_frame(rng, 96, 96, kinds[i % len(kinds)], INK_BRIGHT, False)
        frame = frame.astype(np.int32)
        frame[8 + i, 8:60] = 128          # occupied level -> a candidate t
        frame = np.clip(frame, 0, 255).astype(np.uint8)
        add(f"bright_occupied{i}", frame, polarity=INK_BRIGHT, thresh=128)
        add(f"bright_occupied{i}_otsu", frame, polarity=INK_BRIGHT)

    # ── pinned thresholds (isolates the stages from Otsu) ───────────────
    frame = synth_frame(rng, 96, 96, "block", INK_DARK, False)
    for t in (0, 1, 100, 127, 128, 200, 254, 255):
        add(f"thresh{t:03d}_dark", frame, thresh=t)
        add(f"thresh{t:03d}_bright", frame, polarity=INK_BRIGHT, thresh=t, seg=False)
    allblack = np.zeros((32, 32), dtype=np.uint8)
    add("thresh000_allblack", allblack, thresh=0)      # v <= 0 inks everything
    add("thresh000_allblack_bright", allblack, thresh=0, polarity=INK_BRIGHT)

    # ── RGB frames (the device's integer BT.601 input) ──────────────────
    for i in range(8):
        side = int(rng.choice([32, 96]))
        gray = synth_frame(rng, side, side, kinds[i % len(kinds)], INK_DARK, True)
        tint = np.array([
            rng.integers(20, 90), rng.integers(20, 90), rng.integers(120, 255)
        ]) if i % 2 else np.array([255, 255, 255])
        rgb = np.clip(gray[..., None].astype(np.int32) * tint // 255, 0, 255).astype(np.uint8)
        add(f"rgb{i:02d}_{side}", rgb, kind="rgb")

    # Real captures again, as tinted RGB: real paper grain, real glyph
    # edges, but now the luminance stage has to be exactly BT.601.
    real_names = [c[0] for c in cases if c[0].startswith("real")]
    for i, rn in enumerate(real_names[:24]):
        g = dict((c[0], c[2]) for c in cases)[rn]
        if g.ndim != 2:
            continue
        tint = np.array([40 + (i * 7) % 200, 60 + (i * 11) % 180, 200 - (i * 5) % 150])
        rgb = np.clip(g[..., None].astype(np.int32) * tint // 255, 0, 255).astype(np.uint8)
        add(f"rgbreal{i:02d}", rgb, kind="rgb")
    blue = np.zeros((96, 96, 3), dtype=np.uint8)
    blue[..., 2] = 255
    blue[30:60, 40:44] = (255, 0, 0)
    add("rgb_blue_page", blue, kind="rgb")
    return cases


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True, type=Path)
    ap.add_argument("--app-data", type=Path, default=None,
                    help="app workspace root to sample real captures from")
    ap.add_argument("--real", type=int, default=0,
                    help="how many real captures to include")
    ap.add_argument("--seed", type=int, default=20261012)
    args = ap.parse_args()

    out = args.out
    out.mkdir(parents=True, exist_ok=True)
    cases = build_cases(args.app_data, args.real, args.seed)

    manifest = [MANIFEST_HEADER]
    stats = {"ok": 0, "empty": 0, "thresh0": 0, "multi_box": 0, "tensors": 0}
    for name, kind, frame, kw in cases:
        if kind == "rgb":
            assert frame.ndim == 3 and frame.shape[2] == 3, name
        else:
            assert frame.ndim == 2, name
        h, w = frame.shape[:2]
        res = preprocess_glyph(
            frame, out_size=int(kw["out_size"]), polarity=int(kw["polarity"]),
            apply_segmentation=bool(kw["seg"]), min_run=int(kw["min_run"]),
            thresh=kw["thresh"], convention=str(kw["convention"]),
        )
        glyph = np.asarray(res["glyph"], dtype=np.uint8)
        assert glyph.shape == (kw["out_size"], kw["out_size"]), (name, glyph.shape)
        # `empty` is the exact negation of `ok` on every return path in
        # preprocess_glyph; the C port relies on that equivalence.
        assert bool(res["empty"]) == (not bool(res["ok"])), name

        (out / f"{name}.raw").write_bytes(frame.tobytes())
        (out / f"{name}.golden").write_bytes(glyph.tobytes())
        box = res["box"]
        (out / f"{name}.gmeta").write_text(
            "thresh=%d ok=%d boxes=%d box=%s\n"
            % (int(res["thresh"]), 1 if res["ok"] else 0, len(res["boxes"]),
               "-" if box is None else "%d,%d,%d,%d" % tuple(int(v) for v in box))
        )
        manifest.append("%s\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\t%d\n" % (
            name, 1 if kind == "rgb" else 0, w, h, int(kw["out_size"]),
            int(kw["polarity"]), 1 if kw["seg"] else 0, int(kw["min_run"]),
            -1 if kw["thresh"] is None else int(kw["thresh"]),
            CONV[str(kw["convention"])]))

        stats["ok"] += int(bool(res["ok"]))
        stats["empty"] += int(not bool(res["ok"]))
        stats["thresh0"] += int(int(res["thresh"]) == 0)
        stats["multi_box"] += int(len(res["boxes"]) > 1)
        stats["tensors"] += 1

    (out / "manifest.tsv").write_text("".join(manifest))
    (out / "stats.json").write_text(json.dumps(stats, indent=2) + "\n")
    print(f"corpus: {stats['tensors']} cases -> {out}")
    print(f"  ok={stats['ok']} empty={stats['empty']} "
          f"(t==0: {stats['thresh0']}, multi-run boxes: {stats['multi_box']})")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
