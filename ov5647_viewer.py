#!/usr/bin/env python3
"""
OV5647 Serial Frame Viewer / Recorder

Reads raw frames from the OV5647 camera over serial and converts them to images.

Protocol (from ESP32_P4_OV5647 library):
    Sync header:  0xAA 0x55 0xAA  (3 bytes)
    RGB payload:  IMG_SIZE × IMG_SIZE × 3 bytes   (default 96×96×3 = 27648)
    Gray payload: IMG_SIZE × IMG_SIZE bytes        (default 96×96   =  9216)

Usage:
    # Live view (press 'q' to quit, 's' to save current frame)
    python ov5647_viewer.py --port /dev/cu.usbmodem*

    # Record frames to directory
    python ov5647_viewer.py --port /dev/cu.usbmodem* --record ./frames/

    # Grayscale mode (when using send_gray_to_serial)
    python ov5647_viewer.py --port /dev/cu.usbmodem* --gray

    # Custom image size
    python ov5647_viewer.py --port /dev/cu.usbmodem* --size 160 --rgb

Dependencies:  pip install pyserial pillow numpy
"""

from __future__ import annotations

import argparse
import io
import os
import struct
import sys
import time
from pathlib import Path

import numpy as np
from PIL import Image

# ── Protocol constants ──────────────────────────────────────────────
HEADER = bytes([0xAA, 0x55, 0xAA])
HEADER_LEN = len(HEADER)  # 3


def _find_serial_port() -> str | None:
    """Try to auto-detect a serial port."""
    import glob

    patterns: list[str] = []
    if sys.platform == "darwin":
        patterns = ["/dev/cu.usbmodem*", "/dev/cu.usbserial*", "/dev/cu.*"]
    elif sys.platform.startswith("linux"):
        patterns = ["/dev/ttyUSB*", "/dev/ttyACM*", "/dev/ttyAMA*"]
    elif sys.platform == "win32":
        return None  # Windows needs explicit port

    for pat in patterns:
        matches = sorted(glob.glob(pat))
        if matches:
            return matches[0]
    return None


class OV5647FrameReader:
    """Reads OV5647 frames from a serial port."""

    def __init__(
        self,
        port: str,
        baud: int = 921600,
        img_size: int = 96,
        rgb: bool = True,
    ) -> None:
        self._port = port
        self._baud = baud
        self._img_size = img_size
        self._rgb = rgb
        self._frame_size = img_size * img_size * (3 if rgb else 1)
        self._ser = None
        self._buf = bytearray()

    @property
    def frame_size(self) -> int:
        return self._frame_size

    @property
    def img_size(self) -> int:
        return self._img_size

    @property
    def is_rgb(self) -> bool:
        return self._rgb

    def open(self) -> None:
        import serial

        self._ser = serial.Serial(port=self._port, baudrate=self._baud, timeout=0.1)
        self._buf = bytearray()
        try:
            self._ser.reset_input_buffer()
        except Exception:
            pass

    def close(self) -> None:
        if self._ser is not None:
            try:
                self._ser.close()
            finally:
                self._ser = None

    def read_frame(self, timeout_s: float = 5.0) -> bytes:
        """Read one complete frame, blocking until found or timeout."""
        if self._ser is None:
            raise RuntimeError("Serial not opened")

        start = time.time()
        fs = self._frame_size

        while time.time() - start < timeout_s:
            chunk = self._ser.read(4096)
            if chunk:
                self._buf.extend(chunk)
                if len(self._buf) > 262144:
                    self._buf = self._buf[-65536:]
            else:
                time.sleep(0.004)

            while True:
                idx = self._buf.find(HEADER)
                if idx < 0:
                    if len(self._buf) > HEADER_LEN:
                        keep = max(0, HEADER_LEN - 1)
                        self._buf = self._buf[-keep:] if keep else bytearray()
                    break

                after = idx + HEADER_LEN
                need = after + fs
                if len(self._buf) < need:
                    if idx > 0:
                        del self._buf[:idx]
                    break

                # Validate: next header should follow (if enough data)
                if len(self._buf) >= need + HEADER_LEN:
                    if self._buf[need : need + HEADER_LEN] != HEADER:
                        del self._buf[: idx + 1]
                        continue

                frame = bytes(self._buf[after:need])
                del self._buf[:need]
                return frame

        raise TimeoutError("Timeout waiting for frame header")


def frame_to_image(frame: bytes, img_size: int, rgb: bool = True) -> Image.Image:
    """Convert raw frame bytes to PIL Image."""
    if rgb:
        expected = img_size * img_size * 3
        if len(frame) == expected:
            arr = np.frombuffer(frame, dtype=np.uint8).reshape((img_size, img_size, 3))
            return Image.fromarray(arr, mode="RGB")
        else:
            raise ValueError(
                f"Expected {expected} bytes for RGB, got {len(frame)}. "
                f"Try --gray if the camera is sending grayscale."
            )
    else:
        expected = img_size * img_size
        if len(frame) == expected:
            arr = np.frombuffer(frame, dtype=np.uint8).reshape((img_size, img_size))
            return Image.fromarray(arr, mode="L")
        elif len(frame) == img_size * img_size * 3:
            # Auto-detect: received RGB-sized data in grayscale mode
            print(
                f"  [note] Received {len(frame)} bytes — looks like RGB, "
                f"treating as grayscale (use --rgb for colour)",
                file=sys.stderr,
            )
            arr = np.frombuffer(frame, dtype=np.uint8).reshape((img_size, img_size, 3))
            # Convert to grayscale: BT.601 luminance
            gray = (
                arr[:, :, 0].astype(np.float32) * 0.299
                + arr[:, :, 1].astype(np.float32) * 0.587
                + arr[:, :, 2].astype(np.float32) * 0.114
            ).astype(np.uint8)
            return Image.fromarray(gray, mode="L")
        else:
            raise ValueError(
                f"Expected {expected} bytes for grayscale, got {len(frame)}. "
                f"Try --rgb if the camera is sending RGB."
            )


def live_view(reader: OV5647FrameReader, record_dir: str | None = None) -> None:
    """Live preview window.  Press q=quit, s=save frame, Space=pause."""
    import cv2  # optional — falls back to PIL if missing

    print(f"OV5647 Viewer — {reader.img_size}×{reader.img_size} "
          f"{'RGB' if reader.is_rgb else 'grayscale'}")
    print(f"Port: {reader._port}  Baud: {reader._baud}")
    print("Keys: q=quit  s=save frame  Space=pause")
    print("Waiting for frames...")

    reader.open()
    paused = False
    frame_count = 0
    saved_count = 0

    if record_dir:
        os.makedirs(record_dir, exist_ok=True)

    cv2_window = "OV5647" if "cv2" in sys.modules else None
    try:
        import cv2

        cv2.namedWindow(cv2_window, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(cv2_window, 480, 480)
    except Exception:
        cv2_window = None

    try:
        while True:
            if not paused:
                try:
                    raw = reader.read_frame(timeout_s=1.0)
                except TimeoutError:
                    print("  (waiting for data...)", file=sys.stderr)
                    continue

                img = frame_to_image(raw, reader.img_size, reader.is_rgb)
                frame_count += 1

                if record_dir:
                    path = os.path.join(record_dir, f"frame_{saved_count:06d}.png")
                    img.save(path)
                    saved_count += 1
                    print(f"  saved {path}  ({frame_count} frames received)", file=sys.stderr)

                if cv2_window:
                    arr = np.array(img)
                    if not reader.is_rgb:
                        arr = cv2.cvtColor(arr, cv2.COLOR_GRAY2BGR)
                    # Upscale for easier viewing
                    display = cv2.resize(arr, (480, 480), interpolation=cv2.INTER_NEAREST)
                    cv2.putText(
                        display, f"Frame: {frame_count}", (5, 20),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1,
                    )
                    cv2.imshow(cv2_window, display)

            key = cv2.waitKey(1) & 0xFF if cv2_window else -1
            if key == ord("q"):
                break
            elif key == ord("s"):
                path = f"ov5647_frame_{saved_count:06d}.png"
                img.save(path)
                saved_count += 1
                print(f"  saved {path}")
            elif key == ord(" "):  # Space
                paused = not paused
                print(f"  {'paused' if paused else 'resumed'}")
    except KeyboardInterrupt:
        print("\nInterrupted.")
    finally:
        reader.close()
        if cv2_window:
            cv2.destroyAllWindows()

    print(f"Done. {frame_count} frames received, {saved_count} saved.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="OV5647 Serial Frame Viewer",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument(
        "--port", "-p",
        default=None,
        help="Serial port (auto-detected if omitted)",
    )
    parser.add_argument(
        "--baud", "-b",
        type=int,
        default=921600,
        help="Baud rate (default: 921600)",
    )
    parser.add_argument(
        "--size", "-s",
        type=int,
        default=96,
        help="Image width/height in pixels (default: 96)",
    )
    parser.add_argument(
        "--rgb", "-r",
        action="store_true",
        default=True,
        help="RGB mode — 3 bytes per pixel (default)",
    )
    parser.add_argument(
        "--gray", "-g",
        action="store_false",
        dest="rgb",
        help="Grayscale mode — 1 byte per pixel",
    )
    parser.add_argument(
        "--record",
        default=None,
        help="Directory to save every frame (e.g. ./frames/)",
    )
    parser.add_argument(
        "--list", "-l",
        action="store_true",
        help="List available serial ports and exit",
    )

    args = parser.parse_args()

    if args.list:
        try:
            from serial.tools import list_ports

            for p in list_ports.comports():
                print(f"  {p.device}  {p.description}")
        except Exception:
            print("pyserial not installed. Install with: pip install pyserial")
        return

    port = args.port or _find_serial_port()
    if not port:
        print(
            "No serial port found. Specify one with --port, or install pyserial:\n"
            "  pip install pyserial",
            file=sys.stderr,
        )
        sys.exit(1)

    print(f"Using port: {port}")
    reader = OV5647FrameReader(
        port=port,
        baud=args.baud,
        img_size=args.size,
        rgb=args.rgb,
    )
    print(f"Frame size: {reader.frame_size} bytes "
          f"({'RGB' if reader.is_rgb else 'grayscale'})")

    if args.record:
        live_view(reader, record_dir=args.record)
    else:
        live_view(reader)


if __name__ == "__main__":
    main()
