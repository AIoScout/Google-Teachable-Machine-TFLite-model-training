#!/usr/bin/env python3
"""Is the DEVICE's wired call the TRAINING-side call?

The corpus proves that ocr_glyph.c reproduces ocr_preprocess.py *when driven
with the corpus' parameters*.  It does not prove the sketch passes those
parameters.  TFLite_OCR.ino could call

    ocr_preprocess_gray(gray, ..., OCR_INK_BRIGHT, ...)

and every corpus test would still pass while the device produced inverted
glyphs on every frame — the one class of bug that survives all the bit-exact
work upstream.

So this check reads the real call out of TFLite_OCR/TFLite_OCR.ino, resolves the
kOcr* constants it names, and compares them against the values AItraining's
OCR project actually trains with
(record_controller._preprocess_image_png_ocr -> ocr_preprocess.preprocess_glyph).

`--sign` runs the same discipline over TFLite_Sign/TFLite_Sign.ino: one
preprocess_sign_sequence_c() call, the three target slots, the per-slot match
rule, and the threshold-window wiring.  It additionally cross-checks the
exported target sequence/order against model_settings.h.

Anchors are matched EXACTLY (same discipline as mutation_check.py): a
refactor that moves the call reports "anchor not found" here instead of
silently checking nothing.

    python3 tools/wiring_check.py [--sketch PATH] [--corpus DIR]
    python3 tools/wiring_check.py --sign [--sketch PATH]

--corpus additionally replays the OCR parity corpus restricted to the sketch's
parameters (needs tools/gen_corpus.py output); without it, the structural
comparison still runs.  The corpus format is OCR-only, so --sign has no replay.
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
REPO = ROOT.parent
DEFAULT_SKETCH = ROOT / "TFLite_OCR.ino"
HEADER = ROOT / "ocr_glyph.h"
SETTINGS = ROOT / "model_settings.h"
RECORD_CONTROLLER = REPO / "AItraining" / "record_controller.py"

SIGN_ROOT = REPO / "TFLite_Sign"
SIGN_SKETCH = SIGN_ROOT / "TFLite_Sign.ino"
SIGN_HEADER = SIGN_ROOT / "sign_glyph.h"
SIGN_SETTINGS = SIGN_ROOT / "model_settings.h"

failures: list[str] = []
notes: list[str] = []


def fail(msg: str) -> None:
    failures.append(msg)
    print(f"  FAIL  {msg}")


def ok(msg: str) -> None:
    print(f"  ok    {msg}")


def strip_comments(text: str) -> str:
    text = re.sub(r"/\*.*?\*/", " ", text, flags=re.S)
    return re.sub(r"//[^\n]*", " ", text)


def split_args(arglist: str) -> list[str]:
    """Split on top-level commas only (the call has no nested parens today,
    but a future one would otherwise desynchronise every index below)."""
    out, depth, cur = [], 0, []
    for ch in arglist:
        if ch in "([{":
            depth += 1
        elif ch in ")]}":
            depth -= 1
        if ch == "," and depth == 0:
            out.append("".join(cur).strip())
            cur = []
        else:
            cur.append(ch)
    if "".join(cur).strip():
        out.append("".join(cur).strip())
    return out


def extract_call(text: str, fname: str) -> list[str]:
    """The one ocr_preprocess_gray( call, as a list of argument expressions."""
    needle = f"{fname}("
    idx = text.find(needle)
    if idx < 0:
        fail(f"no `{fname}(` call found in the sketch")
        return []
    if text.find(needle, idx + 1) >= 0:
        fail(f"more than one `{fname}(` call in the sketch — this checker "
             "resolves exactly one")
        return []
    start = idx + len(needle)
    depth = 1
    i = start
    while i < len(text) and depth:
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                break
        i += 1
    if depth:
        fail(f"unbalanced parentheses in the `{fname}(` call")
        return []
    return split_args(strip_comments(text[start:i]))


def resolve_constexpr(sketch: str, name: str) -> str | None:
    m = re.search(
        rf"(?:static\s+|inline\s+)*constexpr\s+[\w:]+\s+{re.escape(name)}\s*=\s*([^;]+);",
        sketch,
    )
    return m.group(1).strip() if m else None


def resolve_define(header: str, name: str) -> str | None:
    m = re.search(rf"^#define\s+{re.escape(name)}\s+(-?\d+)", header, re.M)
    return m.group(1) if m else None


def check_sign(sketch_path: Path) -> int:
    """The same discipline for the 3-character sign sketch."""
    sketch_raw = sketch_path.read_text()
    sketch = strip_comments(sketch_raw)
    header = SIGN_HEADER.read_text()
    settings = SIGN_SETTINGS.read_text()

    print(f"sketch: {sketch_path}")
    print(f"header: {SIGN_HEADER}")
    print(f"export: {SIGN_SETTINGS}")

    # ── 1. the call itself ───────────────────────────────────────────────
    print("\n[1] the wired call")
    call_args = extract_call(sketch, "preprocess_sign_sequence_c")
    if not call_args:
        return 1
    if len(call_args) != 16:
        fail(f"expected 16 arguments, found {len(call_args)}: {call_args}")
        return 1
    ok(f"16 arguments: {', '.join(call_args)}")
    if "CameraGetGrayImgSized()" not in sketch:
        fail("the sketch never calls CameraGetGrayImgSized() — the sign "
             "front-end must run on the library's raw gray (what the "
             "training PNGs hold)")
    else:
        ok("fed from CameraGetGrayImgSized()")

    # ── 2. resolve the enumerating arguments ─────────────────────────────
    print("\n[2] resolved device parameters")
    enums = {
        "SIGN_INK_DARK": 0, "SIGN_INK_BRIGHT": 1,
        "SIGN_NORM_STRETCH": 0, "SIGN_NORM_CANON": 1,
    }

    def const_int(expr: str) -> int | None:
        seen: set[str] = set()
        cur = expr.strip()
        while cur not in seen:
            seen.add(cur)
            cur = re.sub(r"^\((?:int|unsigned|long|size_t)\)\s*", "", cur).strip()
            if re.fullmatch(r"-?\d+", cur):
                return int(cur)
            if cur in ("true", "false"):
                return int(cur == "true")
            if cur in enums:
                return enums[cur]
            nxt = resolve_constexpr(sketch, cur)
            if nxt is None:
                nxt = resolve_constexpr(settings, cur)
            if nxt is None:
                nxt = resolve_define(header, cur)
            if nxt is None:
                nxt = resolve_define(settings, cur)
            if nxt is None:
                return None
            cur = nxt
        return None

    (buf_arg, w_arg, h_arg, out_arg, count_arg, order_arg, pol_arg,
     seg_arg, min_run_arg, thresh_arg, dark_arg, conv_arg,
     scratch_arg, glyphs_arg, slots_arg, info_arg) = call_args
    if "scratch" not in scratch_arg or "glyph" not in glyphs_arg \
            or "slot" not in slots_arg or "info" not in info_arg:
        fail(f"trailing arguments look wrong: {scratch_arg}, {glyphs_arg}, "
             f"{slots_arg}, {info_arg}")

    device = {}
    for label, expr, want_kind in (
        ("gray buffer", buf_arg, "ptr"),
        ("w", w_arg, "int"), ("h", h_arg, "int"),
        ("out_size", out_arg, "int"), ("count", count_arg, "int"),
        ("reading_order", order_arg, "int"),
        ("polarity", pol_arg, "int"), ("apply_segmentation", seg_arg, "int"),
        ("min_run", min_run_arg, "int"),
        ("thresh", thresh_arg, "expr"),   # computed constexpr — text-checked below
        ("dark_thr", dark_arg, "expr"),
        ("convention", conv_arg, "int"),
    ):
        if want_kind == "ptr":
            device[label] = expr
            ok(f"{label:18s} = {expr}")
            continue
        if want_kind == "expr":
            device[label] = expr.strip()
            ok(f"{label:18s} = {expr.strip()}   (formula checked below)")
            continue
        val = const_int(expr)
        if val is None:
            fail(f"{label} ({expr}) does not resolve to an integer constant")
        device[label] = val
        ok(f"{label:18s} = {val}   (from `{expr}`)")

    if w_arg != h_arg:
        fail(f"w != h ({w_arg} vs {h_arg}) — the gray buffer is square")
    if w_arg != "g_frame_side":
        fail(f"w/h are `{w_arg}` — must be g_frame_side (the camera/library "
             "frame size the training captures were recorded at)")

    # ── 3. compare against the TRAINING side ─────────────────────────────
    print("\n[3] training-side contract (AItraining)")
    sys.path.insert(0, str(REPO / "AItraining"))
    try:
        import numpy  # noqa: F401
        import ocr_preprocess as op
    except Exception as exc:  # pragma: no cover - environment dependent
        fail(f"cannot import AItraining/ocr_preprocess.py: {exc}")
        return 1

    if device["out_size"] != op.GLYPH_SIZE:
        fail(f"out_size {device['out_size']} != ocr_preprocess.GLYPH_SIZE "
             f"{op.GLYPH_SIZE}")
    else:
        ok(f"out_size {device['out_size']} == GLYPH_SIZE")
    if device["count"] != op.SIGN_SEQUENCE_LEN:
        fail(f"count {device['count']} != ocr_preprocess.SIGN_SEQUENCE_LEN "
             f"{op.SIGN_SEQUENCE_LEN}")
    else:
        ok(f"count {device['count']} == SIGN_SEQUENCE_LEN")
    if device["polarity"] != op.INK_DARK:
        fail(f"polarity {device['polarity']} != INK_DARK {op.INK_DARK}")
    else:
        ok(f"polarity {device['polarity']} == INK_DARK (camera captures)")
    if device["min_run"] != op.MIN_GLYPH_RUN:
        fail(f"min_run {device['min_run']} != MIN_GLYPH_RUN {op.MIN_GLYPH_RUN}")
    else:
        ok(f"min_run {device['min_run']} == MIN_GLYPH_RUN")
    canon_want = 1 if op.NORM_CANON == "canon" else 0
    if device["convention"] != canon_want:
        fail(f"convention {device['convention']} != NORM_CANON ({canon_want})")
    else:
        ok(f"convention {device['convention']} == NORM_CANON")
    if device["apply_segmentation"] != 1:
        fail(f"apply_segmentation {device['apply_segmentation']} != True")
    else:
        ok("apply_segmentation == True")
    if device["reading_order"] not in (0, 1):
        fail(f"reading_order {device['reading_order']} is neither 0 (LTR) nor 1 (RTL)")
    else:
        ok(f"reading_order {device['reading_order']} (0=LTR, 1=RTL)")

    # ── 4. the exported target the sketch must read ──────────────────────
    print("\n[4] exported target")
    order_export = resolve_constexpr(settings, "kSignReadingOrder")
    if order_export is None:
        fail("model_settings.h does not define kSignReadingOrder")
    else:
        try:
            order_val = int(order_export)
        except ValueError:
            order_val = None
        if order_val != device["reading_order"]:
            fail(f"the sketch passes `{order_arg}`, which resolves to "
                 f"{device['reading_order']}, but model_settings.h exports "
                 f"kSignReadingOrder = {order_export}")
        else:
            ok(f"reading order {order_val} matches the export")
    if not re.search(r"kTargetSignSequence\s*\[\s*\]\s*=\s*\"[^\"]{3}\"", settings):
        fail("model_settings.h has no 3-character kTargetSignSequence")
    else:
        ok("3-character kTargetSignSequence exported")
    if not re.search(r"kTargetSignClassIndices\s*\[\s*3\s*\]", settings):
        fail("model_settings.h has no kTargetSignClassIndices[3]")
    else:
        ok("kTargetSignClassIndices[3] exported")
    if not re.search(r"kSignMatchMinProb\s*=\s*[\d.]+f?\s*;", settings):
        fail("model_settings.h has no kSignMatchMinProb")
    else:
        ok("kSignMatchMinProb exported")

    # ── 5. threshold window: formula + exported sliders ──────────────────
    print("\n[5] Dark/Lum threshold window")
    exported = resolve_define(settings, "TM_THRESHOLD_WINDOW_EXPORTED")
    if exported is None:
        fail("model_settings.h does not define TM_THRESHOLD_WINDOW_EXPORTED "
             "(older exports are accepted by the sketch, but the export "
             "should now carry it)")
    else:
        ok(f"TM_THRESHOLD_WINDOW_EXPORTED = {exported}")
    dark = resolve_define(settings, "kDarkThr")
    lum = resolve_define(settings, "kLumThr")
    if dark is None or lum is None:
        fail("model_settings.h does not export kDarkThr / kLumThr")
    else:
        dark_i, lum_i = int(dark), int(lum)
        adjusted = (dark_i != 0) or (lum_i != 100)
        want_pinned = max(1, min(254, lum_i - 1)) if adjusted else -1
        want_floor = max(0, min(255, dark_i + 1)) if adjusted else 0
        ok(f"exported dark={dark_i} lum={lum_i} -> adjusted={adjusted}, "
           f"expect pinned={want_pinned}, floor={want_floor}")

        if device["thresh"] != "kSignPinnedThresh":
            fail(f"thresh argument is `{device['thresh']}` — must be "
                 "kSignPinnedThresh (the sketch's resolved window)")
        if device["dark_thr"] != "kSignDarkThr":
            fail(f"dark_thr argument is `{device['dark_thr']}` — must be "
                 "kSignDarkThr")
        for name, needed in (
            ("kSignWindowAdjusted",
             [r"kExportDarkThr\s*!=\s*0", r"kExportLumThr\s*!=\s*100"]),
            ("kSignDarkThr",
             [r"kExportDarkThr\s*\+\s*1", r"255", r":\s*0"]),
            ("kSignPinnedThresh",
             [r"kExportLumThr\s*-\s*1", r"254", r"\?\s*-1"]),
        ):
            definition = resolve_constexpr(sketch, name)
            if definition is None:
                fail(f"the sketch has no `static constexpr ... {name} = ...`")
                continue
            missing = [p for p in needed if not re.search(p, definition)]
            if missing:
                fail(f"{name} does not implement the host clamp ({missing}): "
                     f"{definition}")
            else:
                ok(f"{name} clamps exactly like the host")

    # ── 6. the per-slot match / verdict wiring ───────────────────────────
    print("\n[6] match / verdict wiring")
    for name, pattern in (
        ("three per-slot Invokes",
         r"for\s*\(\s*int\s+slot\s*=\s*0\s*;\s*slot\s*<\s*SIGN_SEQUENCE_LEN\s*;\s*slot\+\+\s*\)"),
        ("each slot is quantised before its Invoke",
         r"sign_glyph_to_int8\(\s*s_slot_glyphs\[slot\]"),
        ("the probability is the host's raw ratio",
         r"prob\s*>=\s*kSignMatchMinProb"),
        ("each slot must have ink",
         r"s_slot_info\[slot\]\.ok"),
        ("each slot must not be _background",
         r"is_background_id\(\s*slot_label\[slot\]\s*\)"),
        ("each slot must equal the exported target",
         r"\(int\)slot_label\[slot\]\s*==\s*target"),
        ("target index comes from kTargetSignClassIndices",
         r"kTargetSignClassIndices\[slot\]"),
        ("_background is matched by NAME",
         r"strcmp\(\s*name\s*,\s*\"_background\"\s*\)"),
        ("MISMATCH flag is wired", r"kFlagsVerdictMismatch"),
        ("NO SIGN flag is wired", r"kFlagsVerdictNoSign"),
        ("verdict sentinel label is wired", r"kSignVerdictLabel"),
        ("OOD Layer 1 is disarmed on the sign path",
         r"kSignBackgroundIsTheOnlyRejector"),
        ("the resolver registers Rsqrt", r"AddRsqrt\(\)"),
        ("the resolver registers ReduceMax", r"AddReduceMax\(\)"),
    ):
        if re.search(pattern, sketch):
            ok(name)
        else:
            fail(f"missing/changed: {name}")

    if (re.search(r"#if\s+!defined\(\s*TM_SIGN_OCR_MODEL\s*\)\s*\n\s*#error", sketch)
            and re.search(r"#elif\s+TM_SIGN_OCR_MODEL\s*!=\s*1\s*\n\s*#error", sketch)
            and re.search(r"TM_OCR_GLYPH_MODEL\s*==\s*1", sketch)):
        ok("an OCR/legacy export is rejected at compile time")
    else:
        fail("no #error guard for a non-sign export: dropping TFLite_OCR/'s "
             "model_settings.h into TFLite_Sign/ would build silently")

    print()
    for n in notes:
        print(f"note: {n}")
    if failures:
        print(f"RESULT: {len(failures)} problem(s):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("RESULT: sign sketch wires the training contract")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--sketch", type=Path, default=None,
                    help=f"sketch to inspect (default: {DEFAULT_SKETCH.name}, "
                         f"or TFLite_Sign.ino with --sign)")
    ap.add_argument("--sign", action="store_true",
                    help="check the 3-character sign sketch instead of the "
                         "single-glyph OCR sketch")
    ap.add_argument("--corpus", type=Path, default=None,
                    help="optional gen_corpus.py output, for the restricted replay")
    args = ap.parse_args()

    if args.sign:
        return check_sign(args.sketch or SIGN_SKETCH)
    if args.sketch is None:
        args.sketch = DEFAULT_SKETCH

    sketch_raw = args.sketch.read_text()
    sketch = strip_comments(sketch_raw)
    header = HEADER.read_text()

    print(f"sketch: {args.sketch}")
    print(f"header: {HEADER}")

    # ── 1. the call itself ───────────────────────────────────────────────
    print("\n[1] the wired call")
    # The window entry point adds one argument (dark_thr).  Accept either
    # spelling, but hold each to its own arity: a stale 12-argument call
    # cannot pass by being renamed.
    window_call = "ocr_preprocess_gray_window(" in sketch
    entry = "ocr_preprocess_gray_window" if window_call else "ocr_preprocess_gray"
    want_args = 13 if window_call else 12
    call_args = extract_call(sketch, entry)
    if not call_args:
        return 1
    if len(call_args) != want_args:
        fail(f"expected {want_args} arguments for `{entry}`, found "
             f"{len(call_args)}: {call_args}")
        return 1
    ok(f"{want_args} arguments ({entry}): {', '.join(call_args)}")

    # The gray buffer must come from the camera library, not from GetImage's
    # sign pipeline: `const uint8_t *gray = CameraGetGrayImgSized();`.
    if "CameraGetGrayImgSized()" not in sketch:
        fail("the sketch never calls CameraGetGrayImgSized() — the OCR front-end "
             "must run on the library's raw gray (what the training PNGs hold)")
    else:
        ok("fed from CameraGetGrayImgSized()")

    # ── 2. resolve the enumerating arguments ─────────────────────────────
    print("\n[2] resolved device parameters")
    enums = {
        "OCR_INK_DARK": 0, "OCR_INK_BRIGHT": 1,
        "OCR_NORM_STRETCH": 0, "OCR_NORM_CANON": 1,
    }

    def enum_of(expr: str) -> int | None:
        return enums.get(expr.strip())

    def const_int(expr: str) -> int | None:
        """Follow the identifier chain one hop at a time.

        `kOcrMinRun` is `OCR_MIN_GLYPH_RUN` (a header #define) and
        `kOcrSegment` is `true` (a bool) — a single-hop resolver reports both
        as "does not resolve", which reads as a sketch bug when it is a
        checker bug.  Each hop re-enters the same ladder, so a value is only
        accepted once it is genuinely an integer.
        """
        seen: set[str] = set()
        cur = expr.strip()
        while cur not in seen:
            seen.add(cur)
            cur = re.sub(r"^\((?:int|unsigned|long|size_t)\)\s*", "", cur).strip()
            if re.fullmatch(r"-?\d+", cur):
                return int(cur)
            if cur in ("true", "false"):
                return int(cur == "true")
            if cur in enums:                      # OCR_INK_DARK / OCR_NORM_CANON
                return enums[cur]
            nxt = resolve_constexpr(sketch, cur)  # a kOcr* const in the sketch
            if nxt is None:
                nxt = resolve_define(header, cur)  # OCR_GLYPH_SIZE, MIN_GLYPH_RUN
            if nxt is None:
                return None
            cur = nxt
        return None                               # cycle (self-referential macro)

    if window_call:
        (buffer_arg, w_arg, h_arg, out_arg, pol_arg, seg_arg, min_run_arg,
         thresh_arg, dark_arg, conv_arg, scratch_arg, out_arg2,
         info_arg) = call_args
    else:
        (buffer_arg, w_arg, h_arg, out_arg, pol_arg, seg_arg, min_run_arg,
         thresh_arg, conv_arg, scratch_arg, out_arg2, info_arg) = call_args
        dark_arg = "0"
    if "scratch" not in scratch_arg or "glyph" not in out_arg2 or "info" not in info_arg:
        fail(f"trailing arguments look wrong: {scratch_arg}, {out_arg2}, {info_arg}")

    device = {}
    for label, expr, want_kind in (
        ("gray buffer", buffer_arg, "ptr"),
        ("w", w_arg, "int"), ("h", h_arg, "int"),
        ("out_size", out_arg, "int"), ("polarity", pol_arg, "int"),
        ("apply_segmentation", seg_arg, "int"), ("min_run", min_run_arg, "int"),
        ("thresh", thresh_arg, "int"), ("dark_thr", dark_arg, "int"),
        ("convention", conv_arg, "int"),
    ):
        if want_kind == "ptr":
            device[label] = expr
            ok(f"{label:18s} = {expr}")
            continue
        # kOcrPinnedThresh / kOcrDarkThr are ternaries derived from the
        # export, not literals; the window block below resolves them against
        # model_settings.h and fills in the numeric values.
        if window_call and label in ("thresh", "dark_thr"):
            if label == "thresh" and expr != "kOcrPinnedThresh":
                fail(f"thresh argument is `{expr}` — must be kOcrPinnedThresh")
            if label == "dark_thr" and expr != "kOcrDarkThr":
                fail(f"dark_thr argument is `{expr}` — must be kOcrDarkThr")
            device[label] = None
            ok(f"{label:18s} = {expr}   (resolved from the export below)")
            continue
        val = const_int(expr)
        if val is None:
            fail(f"{label} ({expr}) does not resolve to an integer constant")
        device[label] = val
        ok(f"{label:18s} = {val}   (from `{expr}`)")

    # The frame geometry must be the CAMERA's frame size, not the model's:
    # the gray buffer is g_frame_side² and the training captures are that same
    # size.  `96` hard-coded here would break a 128-wide rig silently.
    if w_arg != h_arg:
        fail(f"w != h ({w_arg} vs {h_arg}) — the gray buffer is square")
    if w_arg != "g_frame_side":
        fail(f"w/h are `{w_arg}` — must be g_frame_side (the camera/library "
             "frame size the training captures were recorded at)")

    # ── 3. compare against the TRAINING side ─────────────────────────────
    print("\n[3] training-side contract (AItraining)")
    sys.path.insert(0, str(REPO / "AItraining"))
    try:
        import numpy  # noqa: F401
        import ocr_preprocess as op
    except Exception as exc:  # pragma: no cover - environment dependent
        fail(f"cannot import AItraining/ocr_preprocess.py: {exc}")
        return 1

    py = {
        "out_size": op.GLYPH_SIZE,
        "polarity": op.INK_DARK,
        "min_run": op.MIN_GLYPH_RUN,
        "convention": op.NORM_CANON,
    }
    if device["out_size"] != py["out_size"]:
        fail(f"out_size {device['out_size']} != ocr_preprocess.GLYPH_SIZE {py['out_size']}")
    else:
        ok(f"out_size {device['out_size']} == GLYPH_SIZE")
    if device["polarity"] != py["polarity"]:
        fail(f"polarity {device['polarity']} != INK_DARK {py['polarity']}")
    else:
        ok(f"polarity {device['polarity']} == INK_DARK (camera captures)")
    if device["min_run"] != py["min_run"]:
        fail(f"min_run {device['min_run']} != MIN_GLYPH_RUN {py['min_run']}")
    else:
        ok(f"min_run {device['min_run']} == MIN_GLYPH_RUN")

    # preprocess_glyph() takes the convention as the "canon" STRING; the C
    # side carries the enum.  Compare the two spellings, not the numbers.
    canon_want = 1 if op.NORM_CANON == "canon" else 0
    if device["convention"] != canon_want:
        fail(f"convention {device['convention']} != NORM_CANON ({canon_want})")
    else:
        ok(f"convention {device['convention']} == NORM_CANON")

    # The Dark/Lum window.  At the defaults (0/100) the trainer passes no
    # thresh (Otsu) and no floor, which is the bit-exact no-op; any other
    # exported pair pins the threshold and adds the dark floor on the device
    # exactly as the host does.  Resolve the pair here, both to verify the
    # sketch's derived constants and to give the corpus replay a number.
    settings = SETTINGS.read_text() if SETTINGS.exists() else ""
    exported = resolve_define(settings, "TM_THRESHOLD_WINDOW_EXPORTED")
    dark_s = resolve_define(settings, "kDarkThr")
    lum_s = resolve_define(settings, "kLumThr")
    if exported is None or dark_s is None or lum_s is None:
        fail("model_settings.h does not export the threshold window "
             "(TM_THRESHOLD_WINDOW_EXPORTED / kDarkThr / kLumThr)")
        device["thresh"] = -1
        device["dark_thr"] = 0
    else:
        dark_i, lum_i = int(dark_s), int(lum_s)
        adjusted = (dark_i != 0) or (lum_i != 100)
        want_pinned = max(1, min(254, lum_i - 1)) if adjusted else -1
        want_floor = max(0, min(255, dark_i + 1)) if adjusted else 0
        ok(f"exported window: dark={dark_i} lum={lum_i} -> adjusted="
           f"{adjusted}, pinned={want_pinned}, floor={want_floor}")
        if window_call:
            # The definition regexes below are what make `want_pinned` and
            # `want_floor` the sketch's actual values rather than merely the
            # host's: they prove each derived constant implements the clamp.
            for name, needed in (
                ("kOcrWindowAdjusted",
                 [r"kExportDarkThr\s*!=\s*0", r"kExportLumThr\s*!=\s*100"]),
                ("kOcrDarkThr",
                 [r"kExportDarkThr\s*\+\s*1", r"255", r":\s*0"]),
                ("kOcrPinnedThresh",
                 [r"kExportLumThr\s*-\s*1", r"254", r"\?\s*-1"]),
            ):
                definition = resolve_constexpr(sketch, name)
                if definition is None:
                    fail(f"the sketch has no `static constexpr ... {name} = ...`")
                    continue
                missing = [p for p in needed if not re.search(p, definition)]
                if missing:
                    fail(f"{name} does not implement the host clamp "
                         f"({missing}): {definition}")
                else:
                    ok(f"{name} clamps exactly like the host")
            device["thresh"] = want_pinned
            device["dark_thr"] = want_floor
        else:
            # The 12-argument entry point has no floor argument at all.
            if adjusted:
                fail("the export pins a Dark/Lum window but the sketch calls "
                     f"the 12-argument `{entry}(`, which applies no dark floor "
                     "— call the `_window` entry point")
            if device["thresh"] != -1:
                fail(f"thresh {device['thresh']} — the trainer passes no thresh "
                     "(Otsu); the C sentinel for that is -1, and 0 is a PINNED 0")
            else:
                ok("thresh -1 == Python's thresh=None (exact Otsu)")
            device["dark_thr"] = 0

    if device["apply_segmentation"] != 1:
        fail(f"apply_segmentation {device['apply_segmentation']} != True")
    else:
        ok("apply_segmentation == True (default)")

    # The environment the TRAINER runs in.  These are the defaults
    # _preprocess_image_png_ocr reads from the per-class config; the device
    # hard-codes the same values, so a project that overrides them in the UI
    # would train for one transform and deploy another.
    rc = record_controller_scan(RECORD_CONTROLLER)
    if rc is None:
        fail("could not read the OCR branch defaults out of record_controller.py")
    else:
        for key, expr, want in rc:
            if expr != want:
                fail(f"record_controller default {key}={expr}, expected {want} "
                     "(the values TFLite_OCR.ino hard-codes)")
            else:
                ok(f"record_controller default {key}={want}")

    # ── 4. the N+1 and OOD contracts the wiring depends on ───────────────
    print("\n[4] N+1 / OOD wiring")
    checks = [
        ("the glyph is quantised into the model tensor",
         r"ocr_glyph_to_int8\(\s*s_ocr_glyph\s*,\s*sizeof\(s_ocr_glyph\)\s*,\s*\w+->data\.int8\s*\)"),
        ("the snapshot copy is sized by input->bytes",
         r"memcpy\(\s*s_input_snapshot\s*,\s*input->data\.int8\s*,\s*s_input_bytes\s*\)"),
        ("the snapshot buffer is bounds-checked at boot",
         r"input->bytes\s*>\s*sizeof\(s_input_snapshot\)"),
        ("_background is matched by NAME",
         r"strcmp\(\s*name\s*,\s*\"_background\"\s*\)"),
        ("_background folds into the 0xF0 No-Sign flag",
         r"ood_suppressed\s*\|\|\s*is_background"),
        ("OOD Layer 1 is disarmed on the OCR path",
         r"kOcrBackgroundIsTheOnlyRejector"),
        ("the resolver registers Rsqrt",
         r"AddRsqrt\(\)"),
        ("the resolver registers ReduceMax",
         r"AddReduceMax\(\)"),
    ]
    for label, pattern in checks:
        if re.search(pattern, sketch):
            ok(label)
        else:
            fail(f"missing/changed: {label}")

    # The sketch is OCR-only now, so the export is CHECKED rather than
    # SELECTED: without the guard, dropping a legacy model_settings.h into
    # TFLite_OCR/ builds a sketch whose 28x28 front-end is handed a 96x96
    # tensor, and nothing says a word.
    if (re.search(r"#if\s+!defined\(\s*TM_OCR_GLYPH_MODEL\s*\)\s*\n\s*#error", sketch)
            and re.search(r"#elif\s+TM_OCR_GLYPH_MODEL\s*!=\s*1\s*\n\s*#error", sketch)
            and re.search(r"TM_SIGN_OCR_MODEL\s*==\s*1", sketch)):
        ok("a legacy or sign export is rejected at compile time")
    else:
        fail("no #error guard for a non-OCR export: dropping a legacy or "
             "sign model_settings.h into TFLite_OCR/ would build silently")

    # ── 5. optional: replay the corpus with the sketch's parameters ──────
    if args.corpus is not None:
        print("\n[5] corpus replay restricted to the sketch's parameters")
        rc2 = replay(args.corpus, device)
        if rc2 != 0:
            failures.append("corpus replay (sketch parameters) failed")

    print()
    for n in notes:
        print(f"note: {n}")
    if failures:
        print(f"RESULT: {len(failures)} problem(s):")
        for f in failures:
            print(f"  - {f}")
        return 1
    print("RESULT: device call matches the training contract")
    return 0


def record_controller_scan(path: Path) -> list[tuple[str, str, str]] | None:
    """The OCR branch's per-class config defaults, as (key, found, expected)."""
    if not path.exists():
        return None
    text = path.read_text()
    out = []
    for key, want in (("ink_polarity", "INK_DARK"),
                      ("glyph_convention", "NORM_CANON"),
                      ("glyph_segment", "True")):
        m = re.search(rf'config\.get\(\s*"{key}"\s*,\s*([A-Za-z_][\w.]*|True|False)\s*\)', text)
        if not m:
            return None
        out.append((key, m.group(1), want))
    return out


def replay(corpus: Path, device: dict) -> int:
    """Run the corpus cases whose parameters ARE the sketch's, and require
    bit-identical output.  Cases recorded with other parameters (bright ink,
    STRETCH, a pinned threshold) are excluded by construction."""
    import subprocess
    import tempfile

    manifest = corpus / "manifest.tsv"
    if not manifest.exists():
        fail(f"no manifest.tsv in {corpus}")
        return 1
    want = (device["polarity"], device["apply_segmentation"],
            device["min_run"], device["thresh"], device["convention"])
    lines = manifest.read_text().splitlines()
    header, rows = lines[0], [l for l in lines[1:] if l.strip()]
    keep = []
    for line in rows:
        f = line.split("\t")
        if len(f) < 10:
            continue
        got = (int(f[5]), int(f[6]), int(f[7]), int(f[8]), int(f[9]))
        if got == want:
            keep.append((line, f[0]))
    if not keep:
        fail("no corpus case uses the sketch's parameters — the corpus cannot "
             "test this wiring")
        return 1

    with tempfile.TemporaryDirectory() as td:
        sub = Path(td) / "canon"
        sub.mkdir()
        (sub / "manifest.tsv").write_text("\n".join([header] + [k[0] for k in keep]) + "\n")
        # INPUTS only.  `.cout`, `.cmeta` and `.cq8` are written by the host
        # into this directory; symlinking them back to the real corpus would
        # make the host write *through* the link and clobber the fixtures.
        for _, name in keep:
            for ext in ("raw", "golden", "gmeta"):
                src = corpus / f"{name}.{ext}"
                if src.exists():
                    (sub / f"{name}.{ext}").symlink_to(src.resolve())
        host = Path(td) / "host"
        build = subprocess.run(
            ["cc", "-O2", "-std=c99", "-w", "-I", str(ROOT),
             "-o", str(host), str(ROOT / "tools" / "ocr_glyph_host.c"),
             str(ROOT / "ocr_glyph.c")],
            capture_output=True, text=True)
        if build.returncode != 0:
            fail(f"host build failed: {build.stderr.strip().splitlines()[:2]}")
            return 1
        run = subprocess.run([str(host), str(sub)], capture_output=True, text=True)
        if run.returncode != 0:
            fail(f"host run failed: {run.stderr.strip().splitlines()[:1]}")
            return 1
        cmp_ = subprocess.run([sys.executable, str(ROOT / "tools" / "compare.py"),
                               str(sub)], capture_output=True, text=True)
        tail = [l for l in cmp_.stdout.splitlines() if l.strip()]
        for line in tail[-8:]:
            print(f"    {line}")
        if cmp_.returncode != 0:
            fail(f"{len(keep)} cases at the sketch's parameters are NOT "
                 "bit-identical")
            return 1
        ok(f"{len(keep)}/{len(rows)} corpus cases use the sketch's parameters "
           "and are bit-identical")
        notes.append(f"{len(rows) - len(keep)} corpus cases use other "
                     "parameters (bright ink / STRETCH / pinned threshold) and "
                     "are out of scope for this replay")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
