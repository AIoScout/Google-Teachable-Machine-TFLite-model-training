from __future__ import annotations

import argparse
import base64
import os
import sys
import time


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--port", required=True)
    ap.add_argument("--baud", type=int, default=921600)
    ap.add_argument("--out", default="")
    ap.add_argument("--out-dir", default="")
    ap.add_argument("--cmd", default="get_last")
    ap.add_argument("--timeout", type=float, default=10.0)
    args = ap.parse_args()

    try:
        import serial
    except Exception as e:
        sys.stderr.write(f"pyserial not available: {e}\n")
        return 2

    with serial.Serial(port=args.port, baudrate=args.baud, timeout=0.2) as ser:
        try:
            ser.reset_input_buffer()
        except Exception:
            pass
        ser.write((args.cmd.strip() + "\n").encode("utf-8"))
        ser.flush()

        begin_line = _read_line_with_prefixes(ser, ["BEGIN ", "BEGIN_BUNDLE "], timeout_s=args.timeout)
        if begin_line is None:
            sys.stderr.write("timeout waiting for BEGIN\n")
            return 1

        if begin_line.startswith("BEGIN_BUNDLE "):
            if not args.out_dir:
                sys.stderr.write("--out-dir is required for bundle\n")
                return 2
            os.makedirs(os.path.abspath(args.out_dir), exist_ok=True)
            return _recv_bundle(ser, begin_line, out_dir=args.out_dir, timeout_s=args.timeout)

        if not args.out:
            sys.stderr.write("--out is required for single file\n")
            return 2
        os.makedirs(os.path.dirname(os.path.abspath(args.out)) or ".", exist_ok=True)
        return _recv_single(ser, begin_line, out_path=args.out, timeout_s=args.timeout)


def _recv_single(ser, begin_line: str, out_path: str, timeout_s: float) -> int:
    parts = begin_line.strip().split(" ")
    if len(parts) < 3:
        sys.stderr.write(f"invalid BEGIN line: {begin_line}\n")
        return 1
    remote_path = parts[1]
    size = int(parts[2])

    b64_chunks: list[str] = []
    while True:
        line = _read_line(ser, timeout_s=timeout_s)
        if line is None:
            sys.stderr.write("timeout waiting for END\n")
            return 1
        s = line.strip()
        if s == "END":
            break
        if not s:
            continue
        b64_chunks.append(s)

    data = base64.b64decode("".join(b64_chunks), validate=False)
    if size >= 0 and len(data) != size:
        sys.stderr.write(f"size mismatch: expected={size} got={len(data)}\n")
    with open(out_path, "wb") as f:
        f.write(data)
    sys.stdout.write(f"saved {out_path} from {remote_path} ({len(data)} bytes)\n")
    return 0


def _recv_bundle(ser, begin_line: str, out_dir: str, timeout_s: float) -> int:
    parts = begin_line.strip().split(" ")
    if len(parts) < 3:
        sys.stderr.write(f"invalid BEGIN_BUNDLE line: {begin_line}\n")
        return 1
    root = parts[1]
    file_count = int(parts[2])
    saved = 0
    sys.stdout.write(f"bundle root={root} files={file_count}\n")

    while True:
        line = _read_line(ser, timeout_s=timeout_s)
        if line is None:
            sys.stderr.write("timeout waiting for bundle content\n")
            return 1
        s = line.strip()
        if not s:
            continue
        if s == "END_BUNDLE":
            break
        if not s.startswith("FILE "):
            continue
        f_parts = s.split(" ")
        if len(f_parts) < 3:
            sys.stderr.write(f"invalid FILE header: {s}\n")
            return 1
        rel_path = f_parts[1]
        size = int(f_parts[2])
        b64_chunks: list[str] = []
        while True:
            line2 = _read_line(ser, timeout_s=timeout_s)
            if line2 is None:
                sys.stderr.write("timeout waiting for ENDFILE\n")
                return 1
            t = line2.strip()
            if t == "ENDFILE":
                break
            if not t:
                continue
            b64_chunks.append(t)
        data = base64.b64decode("".join(b64_chunks), validate=False)
        if size >= 0 and len(data) != size:
            sys.stderr.write(f"size mismatch: {rel_path} expected={size} got={len(data)}\n")

        out_path = os.path.join(os.path.abspath(out_dir), rel_path.lstrip("/"))
        os.makedirs(os.path.dirname(out_path) or ".", exist_ok=True)
        with open(out_path, "wb") as f:
            f.write(data)
        saved += 1
        sys.stdout.write(f"saved {out_path} ({len(data)} bytes)\n")

    sys.stdout.write(f"bundle saved files={saved}\n")
    return 0


def _read_line_with_prefixes(ser, prefixes: list[str], timeout_s: float) -> str | None:
    start = time.time()
    while time.time() - start < timeout_s:
        line = _read_line(ser, timeout_s=max(0.2, timeout_s - (time.time() - start)))
        if line is None:
            continue
        for p in prefixes:
            if line.startswith(p):
                return line
    return None


def _read_line(ser, timeout_s: float) -> str | None:
    start = time.time()
    buf = bytearray()
    while time.time() - start < timeout_s:
        b = ser.read(1)
        if not b:
            continue
        if b == b"\n":
            try:
                return buf.decode("utf-8", errors="replace")
            finally:
                buf.clear()
        if b != b"\r":
            buf.extend(b)
    return None


if __name__ == "__main__":
    raise SystemExit(main())
