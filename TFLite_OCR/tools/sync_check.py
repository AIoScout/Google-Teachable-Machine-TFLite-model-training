#!/usr/bin/env python3
"""The sketches are separate folders. What has to stay in lockstep?

Splitting the pipeline into self-contained sketches means every shared file
lives in more than one place.  Not all of them *should* be identical — the
whole point is that each folder carries its own exported model — so this check
is a three-way classification, not a plain diff:

    SHARED      image_provider.{cpp,h}, frame_resize.h, main_functions.h,
                arduino_main.cpp — one source of truth, three identical copies.  A pipeline fix
                that lands in only one folder is the exact bug CLAUDE.md's
                "keep all three in lockstep" warns about, and it fails
                silently: every sketch still builds.

    PER-SKETCH  model_settings.{h,cpp}, model_resolver.h, tm_model_data.{h,cpp}
                Written by the AItraining export.  These are SUPPOSED to
                differ; what must not happen is the wrong export landing in a
                folder (caught at compile time by the TM_OCR_GLYPH_MODEL /
                TM_SIGN_OCR_MODEL guards in each .ino, and reported here as a
                hint).

    EXCLUSIVE   ocr_glyph.{c,h}  — TFLite_OCR/ only.
                sign_glyph.{c,h} — TFLite_Sign/ only.
                Present in the wrong folder means the split was undone.

The inventory check is the one that earns its keep: any file in a sketch
folder that is not in one of those lists is reported.  Adding `foo.h` to one
sketch and forgetting the others is invisible to a diff of the files you
remembered to list.

    python3 tools/sync_check.py                        # compare (exit 1 on drift)
    python3 tools/sync_check.py --write --from TFLite  # propagate shared files
"""
from __future__ import annotations

import argparse
import difflib
import hashlib
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]        # TFLite_OCR/
REPO = ROOT.parent
SKETCHES = {
    "TFLite": REPO / "TFLite",
    "TFLite_OCR": ROOT,
    "TFLite_Sign": REPO / "TFLite_Sign",
}

SHARED = ("image_provider.cpp", "image_provider.h", "frame_resize.h",
          "main_functions.h", "arduino_main.cpp")
PER_SKETCH = ("model_settings.h", "model_settings.cpp", "model_resolver.h",
              "tm_model_data.h", "tm_model_data.cpp")
EXCLUSIVE = {
    "TFLite": (),
    "TFLite_OCR": ("ocr_glyph.c", "ocr_glyph.h"),
    "TFLite_Sign": ("sign_glyph.c", "sign_glyph.h"),
}
# Neither sketch source nor shared: kept in the folder by Arduino convention or
# by hand, and not the business of this check.
IGNORED_SUFFIX = (".ino", ".md", ".docx", ".pdf", ".py", ".txt", ".json")

# Which export each folder is supposed to hold.  `required` lists the macro
# that must be present; every listed macro must equal its expected value when
# present.  A pre-macro legacy export (no flags at all) is legal only for
# TFLite/.
GUARDS = {
    "TFLite":      (("TM_OCR_GLYPH_MODEL", 0), ("TM_SIGN_OCR_MODEL", 0)),
    "TFLite_OCR":  (("TM_OCR_GLYPH_MODEL", 1), ("TM_SIGN_OCR_MODEL", 0)),
    "TFLite_Sign": (("TM_SIGN_OCR_MODEL", 1), ("TM_OCR_GLYPH_MODEL", 0)),
}
REQUIRED = {
    "TFLite": (),
    "TFLite_OCR": ("TM_OCR_GLYPH_MODEL",),
    "TFLite_Sign": ("TM_SIGN_OCR_MODEL",),
}


def digest(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def inventory(folder: Path) -> set[str]:
    return {p.name for p in folder.iterdir()
            if p.is_file() and not p.name.startswith(".")
            and p.suffix.lower() not in IGNORED_SUFFIX}


def macro_value(text: str, macro: str) -> str | None:
    """The value of `#define macro value`, or None when it is not defined."""
    prefix = f"#define {macro}"
    for line in text.splitlines():
        s = line.strip()
        if s.startswith(prefix):
            rest = s[len(prefix):].strip()
            return rest.split()[0] if rest else None
    return None


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--write", action="store_true",
                    help="copy the shared files from --from into the other sketches")
    ap.add_argument("--from", dest="src", choices=tuple(SKETCHES),
                    help="which folder is the source of truth for --write")
    args = ap.parse_args()
    if args.write and not args.src:
        sys.exit(f"--write needs --from {'|'.join(SKETCHES)} — the direction is "
                 "a decision, not a default")

    problems: list[str] = []
    labels = list(SKETCHES)

    # ── 1. shared files must be byte-identical across all sketches ──────
    print("── shared (must be identical) ──────────────────────────────")
    for name in SHARED:
        paths = {label: folder / name for label, folder in SKETCHES.items()}
        missing = [f"{label}/{name}" for label, p in paths.items()
                   if not p.exists()]
        if missing:
            print(f"  MISSING  {', '.join(missing)}")
            problems.append(f"{name} missing from {', '.join(m.split('/')[0] for m in missing)}")
            continue
        digests = {label: digest(p) for label, p in paths.items()}
        same = len(set(digests.values())) == 1
        print(f"  {name:20s} " +
              "  ".join(digests[label][:12] for label in labels) +
              f"  {'same' if same else 'DRIFT'}")
        if same:
            continue
        if args.write:
            for label, p in paths.items():
                if label != args.src:
                    shutil.copyfile(paths[args.src], p)
                    print(f"    copied {args.src}/{name} -> {label}/{name}")
            continue
        problems.append(f"{name} differs across the sketch folders")
        base_label = labels[0]
        base_lines = paths[base_label].read_text(errors="replace").splitlines()
        for label in labels[1:]:
            if digests[label] == digests[base_label]:
                continue
            other_lines = paths[label].read_text(errors="replace").splitlines()
            for line in list(difflib.unified_diff(
                    base_lines, other_lines, str(paths[base_label]),
                    str(paths[label]), lineterm="", n=1))[:80]:
                print(f"    {line}")

    # ── 2. per-sketch exports: allowed to differ, must not be swapped ────
    # "differ" is the expected verdict for every file here, but identity is not
    # equally alarming across them.  tm_model_data.h is a size-free pair of
    # `extern` declarations — the byte count lives in the .cpp — so every
    # export emits it identically and same-ness there says nothing.  For the
    # others, two sketches with byte-identical exports means one export was
    # copied into both folders (or the placeholder never got replaced).
    SAME_IS_FINE = {"tm_model_data.h"}
    print("\n── per-sketch export (may differ) ──────────────────────────")
    for name in PER_SKETCH:
        paths = {label: folder / name for label, folder in SKETCHES.items()}
        missing = [f"{label}/{name}" for label, p in paths.items()
                   if not p.exists()]
        if missing:
            print(f"  MISSING  {', '.join(missing)}")
            problems.append(f"{name} missing from {', '.join(m.split('/')[0] for m in missing)}")
            continue
        by_digest: dict[str, list[str]] = {}
        for label, p in paths.items():
            by_digest.setdefault(digest(p), []).append(label)
        dupes = [group for group in by_digest.values() if len(group) > 1]
        if not dupes:
            note = "differ (expected)"
        elif name in SAME_IS_FINE:
            note = (f"same in {'/'.join(dupes[0])} (expected — size-free extern "
                    "decl; the byte count is in the .cpp)")
        else:
            note = f"IDENTICAL in {'/'.join(dupes[0])} — did one export land in two folders?"
        print(f"  {name:20s} {note}")
        if dupes and name not in SAME_IS_FINE:
            problems.append(f"{name} is identical in {'/'.join(dupes[0])} — the "
                            "sketches' exports should not be the same file")

    # ── 3. the export guards must actually be armed ─────────────────────
    print("\n── export guards ───────────────────────────────────────────")
    for label, folder in SKETCHES.items():
        header = folder / "model_settings.h"
        text = header.read_text(errors="replace") if header.exists() else ""
        found = {macro: macro_value(text, macro) for macro, _ in GUARDS[label]}
        shown = "  ".join(
            f"{macro}={'<absent>' if value is None else value}"
            for macro, value in found.items())
        print(f"  {label}/model_settings.h: {shown}")
        for macro, want in GUARDS[label]:
            got = found[macro]
            if got is None:
                if macro in REQUIRED[label]:
                    print(f"    MISSING required guard: {macro} (want {want})")
                    problems.append(
                        f"{label}/model_settings.h does not define {macro}; "
                        "that sketch's .ino will refuse to build it")
                continue
            try:
                ok = int(got) == want
            except ValueError:
                ok = False
            if not ok:
                print(f"    WRONG EXPORT: {macro} = {got} (want {want})")
                problems.append(
                    f"{label}/ holds a {macro}={got} export; that sketch's "
                    ".ino will refuse to build it")

    # ── 4. nothing unseen in any folder, and no exclusive file misplaced ─
    print("\n── inventory ───────────────────────────────────────────────")
    known = set(SHARED) | set(PER_SKETCH)
    for label, folder in SKETCHES.items():
        allowed = known | set(EXCLUSIVE[label])
        extra = inventory(folder) - allowed
        if extra:
            print(f"  {label}/: unclassified {sorted(extra)}")
            problems.append(f"{label}/ has files this check does not know "
                            f"about ({sorted(extra)}) — if one is shared, add it "
                            "to SHARED; if it is per-sketch, to PER_SKETCH")
        else:
            print(f"  {label}/: every file classified")
    for owner, names in EXCLUSIVE.items():
        for label, folder in SKETCHES.items():
            if label == owner:
                continue
            for name in names:
                if (folder / name).exists():
                    print(f"  {label}/{name} exists — {name} belongs to "
                          f"{owner}/ only")
                    problems.append(f"{name} is in {label}/ — the split was undone")

    print()
    if problems:
        print(f"RESULT: {len(problems)} problem(s):")
        for p in problems:
            print(f"  - {p}")
        return 1
    print("RESULT: shared files identical, each sketch holds its own export, "
          "no unclassified files.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
