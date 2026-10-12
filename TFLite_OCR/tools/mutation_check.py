#!/usr/bin/env python3
"""Does the parity corpus have teeth?

A corpus that passes a wrong implementation is worse than no corpus: it
certifies a bug.  This applies one deliberately-wrong edit at a time to
ocr_glyph.c — each edit is a plausible thing a re-implementation would do,
or the exact "equivalent-looking" spelling the Python docstrings warn about
— rebuilds, replays the corpus, and reports how many cases each mutation
breaks.

    python3 tools/mutation_check.py <corpus_dir> [--build DIR]

Every mutation MUST be caught.  A mutation reported CAUGHT BY 0 CASES is a
hole in the corpus, not a passing test.
"""
from __future__ import annotations

import argparse
import subprocess
import sys
import tempfile
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SRC = ROOT / "ocr_glyph.c"
HOST = ROOT / "tools" / "ocr_glyph_host.c"

# (label, exact old text, replacement) — exact match, so a refactor that
# moves this code fails loudly here instead of silently testing nothing.
MUTATIONS = [
    ("ink: dark '<=' -> '<'  (the stale-docstring spelling)",
     "ink[i] = (gray[i] <= t) ? 1u : 0u;",
     "ink[i] = (gray[i] < t) ? 1u : 0u;"),
    ("ink: bright '>' -> '>='",
     "ink[i] = (gray[i] > t) ? 1u : 0u;",
     "ink[i] = (gray[i] >= t) ? 1u : 0u;"),
    ("otsu: strict '>' tie-break -> '>='  (last maximiser wins)",
     "if (bn_cmp(&lhs, &rhs) > 0) {",
     "if (bn_cmp(&lhs, &rhs) >= 0) {"),
    ("otsu: empty classes scored instead of skipped",
     "if (d0 == 0) continue;",
     "if (0) continue;"),
    ("otsu: 64-bit score instead of the bignum (truncating product)",
     "otsu_score(&lhs, num, best_d);",
     "lhs.l[0] = (uint32_t)(num * num * best_d); memset(lhs.l + 1, 0, 16);"),
    ("resize: exact rational centre map -> (o*in)/out",
     "return (int)(((int64_t)(2 * o + 1) * (int64_t)in_len) /\n"
     "               (int64_t)(2 * out_len));",
     "return (int)(((int64_t)o * (int64_t)in_len) / (int64_t)out_len);"),
    ("canon: round-half-up new_len -> floor",
     "int nh = (2 * h * fit + long_side) / (2 * long_side);",
     "int nh = (2 * h * fit) / (2 * long_side);"),
    ("canon: paste offset (out-n)/2 -> ceil",
     "const int dst_y0 = (out_size - nh) / 2;",
     "const int dst_y0 = (out_size - nh + 1) / 2;"),
    ("canon: centre-of-mass rounding negative branch",
     "return (int)(-((-2 * num + den) / (2 * den)));",
     "return (int)((-2 * num + den) / (2 * den));"),
    ("seg: union keeps the FIRST run's top row",
     "if (y0 < acc.y1) acc.y1 = y0;",
     ""),
    ("seg: union keeps the FIRST run's bottom row",
     "if (y1 + 1 > acc.y2) acc.y2 = y1 + 1;",
     ""),
    ("seg: union keeps the FIRST run's right column",
     "acc.x2 = e;                    /* rightmost KEPT run wins */",
     ""),
    ("seg: min_run comparison off by one",
     "if (e - s < min_run) continue;",
     "if (e - s <= min_run) continue;"),
    ("luma: BT.601 weights perturbed by one",
     "uint32_t lum = (r * 30u + g * 59u + b * 11u + 50u) / 100u;",
     "uint32_t lum = (r * 29u + g * 59u + b * 12u + 50u) / 100u;"),
    ("luma: rounding term dropped",
     "uint32_t lum = (r * 30u + g * 59u + b * 11u + 50u) / 100u;",
     "uint32_t lum = (r * 30u + g * 59u + b * 11u) / 100u;"),
    ("int8: zero point -128 -> -127",
     "out[i] = (int8_t)((int)glyph[i] - 128);",
     "out[i] = (int8_t)((int)glyph[i] - 127);"),
]


def run_case(corpus: Path, build: Path, label: str, mutated_src: str):
    src = build / "ocr_glyph.c"
    src.write_text(mutated_src)
    exe = build / "host"
    cc = subprocess.run(
        ["cc", "-O2", "-std=c99", "-w", "-I", str(ROOT), "-o", str(exe),
         str(HOST), str(src)],
        capture_output=True, text=True)
    if cc.returncode != 0:
        return None, f"build failed: {cc.stderr.strip().splitlines()[:2]}"
    run = subprocess.run([str(exe), str(corpus)], capture_output=True, text=True)
    if run.returncode != 0:
        return -1, run.stderr.strip().splitlines()[:1]
    cmp_ = subprocess.run([sys.executable, str(ROOT / "tools" / "compare.py"),
                           str(corpus)],
                          capture_output=True, text=True)
    out = cmp_.stdout
    if "bit-identical" in out:
        for line in out.splitlines():
            if line.startswith("bit-identical"):
                n = int(line.split(":")[1])
                total = int([l for l in out.splitlines()
                             if l.startswith("cases")][0].split(":")[1])
                return total - n, None
    return None, out.strip().splitlines()[-1:]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus", type=Path)
    args = ap.parse_args()

    original = SRC.read_text()
    failures = []
    print(f"{'mutation':62s} result")
    print("-" * 88)
    with tempfile.TemporaryDirectory() as td:
        build = Path(td)
        for label, old, new in MUTATIONS:
            if old not in original:
                print(f"{label:62s} SKIPPED — anchor text not found!")
                failures.append(label)
                continue
            n, err = run_case(args.corpus, build, label, original.replace(old, new, 1))
            if n is None:
                print(f"{label:62s} ERROR {err}")
                failures.append(label)
            elif n <= 0:
                print(f"{label:62s} *** NOT CAUGHT ***")
                failures.append(label)
            else:
                print(f"{label:62s} caught by {n} cases")
    print()
    if failures:
        print(f"RESULT: {len(failures)} mutation(s) not caught — corpus has holes:")
        for f in failures:
            print(f"  - {f}")
        return 1
    print(f"RESULT: all {len(MUTATIONS)} mutations caught")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
