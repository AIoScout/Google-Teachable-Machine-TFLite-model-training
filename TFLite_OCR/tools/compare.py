#!/usr/bin/env python3
"""Compare the C port's outputs against the Python golden corpus.

    python3 tools/compare.py <corpus_dir>

Reports divergences per STAGE, not per file, because the stages fail very
differently: a threshold that differs by one count is a decision flip and is
reported as its own class, while a tensor that differs only in the border
rows means the resize drifted.  Exit status is 0 only when every case is
bit-identical in every stage.
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np


def parse_meta(text: str) -> dict:
    out = {}
    for tok in text.split():
        if "=" in tok:
            k, v = tok.split("=", 1)
            out[k] = v
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("corpus", type=Path)
    ap.add_argument("-v", "--verbose", type=int, default=8,
                    help="how many divergence examples to print per class")
    args = ap.parse_args()

    man = (args.corpus / "manifest.tsv").read_text().splitlines()[1:]
    n = ok = 0
    thresh_bad = box_bad = ok_bad = tensor_bad = 0
    shown = {"thresh": 0, "box": 0, "ok": 0, "tensor": 0, "missing": 0}
    pixel_deltas = []

    def show(kind, msg):
        if shown[kind] < args.verbose:
            print(f"    {kind}: {msg}")
            shown[kind] += 1

    for line in man:
        if not line.strip():
            continue
        name, kind, w, h, out_size = line.split("\t")[:5]
        out_size = int(out_size)
        n += 1
        cout = args.corpus / f"{name}.cout"
        cmeta = args.corpus / f"{name}.cmeta"
        if not cout.exists() or not cmeta.exists():
            shown["missing"] += 1
            print(f"  MISSING C OUTPUT for {name}")
            continue

        golden = np.frombuffer((args.corpus / f"{name}.golden").read_bytes(),
                               dtype=np.uint8)
        got = np.frombuffer(cout.read_bytes(), dtype=np.uint8)
        py = parse_meta((args.corpus / f"{name}.gmeta").read_text())
        cc = parse_meta(cmeta.read_text())

        this_ok = True
        if int(py["thresh"]) != int(cc["thresh"]):
            thresh_bad += 1
            this_ok = False
            show("thresh", f"{name} ({w}x{h}): python t={py['thresh']} "
                           f"C t={cc['thresh']}")
        if int(py["ok"]) != int(cc["ok"]):
            ok_bad += 1
            this_ok = False
            show("ok", f"{name}: python ok={py['ok']} C ok={cc['ok']}")
        if py["box"] != cc["box"]:
            box_bad += 1
            this_ok = False
            show("box", f"{name}: python {py['box']} C {cc['box']}")
        if int(py["boxes"]) != int(cc["boxes"]):
            box_bad += 1
            this_ok = False
            show("box", f"{name}: python boxes={py['boxes']} "
                        f"C boxes={cc['boxes']}")

        if golden.size != got.size:
            tensor_bad += 1
            this_ok = False
            show("tensor", f"{name}: size {golden.size} vs {got.size}")
        elif not np.array_equal(golden, got):
            tensor_bad += 1
            this_ok = False
            d = np.abs(golden.astype(np.int32) - got.astype(np.int32))
            pixel_deltas.append(int(d.max()))
            if out_size == 28 and shown["tensor"] < args.verbose:
                a = golden.reshape(out_size, out_size)
                b = got.reshape(out_size, out_size)
                diff_rows = np.flatnonzero((a != b).any(axis=1))
                show("tensor", f"{name}: {int((a != b).sum())}/{a.size} px "
                               f"differ (rows {diff_rows[:6].tolist()}), "
                               f"max|d|={int(d.max())}")

        # The last stage: what the interpreter is actually fed.
        qpath = args.corpus / f"{name}.cq8"
        if qpath.exists():
            got8 = np.frombuffer(qpath.read_bytes(), dtype=np.int8)
            want8 = (golden.astype(np.int16) - 128).astype(np.int8)
            if not np.array_equal(got8, want8):
                tensor_bad += 1
                this_ok = False
                show("tensor", f"{name}: int8 tensor differs "
                               f"({int((got8 != want8).sum())} px)")
        else:
            shown["missing"] += 1
            print(f"  MISSING int8 OUTPUT for {name}")

        if this_ok:
            ok += 1

    print()
    print(f"cases           : {n}")
    print(f"bit-identical   : {ok}")
    print(f"threshold flips : {thresh_bad}")
    print(f"ok-flag flips   : {ok_bad}")
    print(f"box divergences : {box_bad}")
    print(f"tensor divergences: {tensor_bad}")
    if pixel_deltas:
        print(f"max |delta| over divergent tensors: {max(pixel_deltas)}")
    bad = thresh_bad + ok_bad + box_bad + tensor_bad + shown["missing"]
    print("RESULT:", "ALL PASS" if bad == 0 else f"{bad} DIVERGENCES")
    return 0 if bad == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
