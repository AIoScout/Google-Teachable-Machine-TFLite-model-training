# S3_UART_Receiver

ESP32-S3 receives inference results from the ESP32-P4 and sends control messages back.

The P4-side TFLite firmware now performs 3-layer Out-of-Distribution (OOD) rejection
before queuing every packet. **Receivers MUST decode the `flags` field first** —
it is the only authoritative way to distinguish a real sign from a "No Sign" frame,
even if `label_id` / `confidence` look like a confident prediction (softmax
hallucinates a high-confidence class on any input, including empty desks).

## Wiring

- P4 TX(GPIO11) → S3 RX(GPIO44)
- P4 RX(GPIO10) ← S3 TX(GPIO43)
- P4 PWM(GPIO9) → S3 ADC(GPIO8)  (presence detection)
- GND → GND

## Protocols

### P4 → S3 (9 bytes)

```
 0    1    2    3    4    5        6           7      8
| AA | 55 | 01 | frame_id LE16 | label_id | confidence | flags | checksum(XOR 0-7) |
```

- `msg_type = 0x01` (inference)
- `checksum  = XOR of bytes 0..7`
- `label_id` = class index for the image/OCR sketches; for the multi-plate
  sign sketch it is the **matched plate index** on MATCH, `0xFE` on
  MISMATCH and `0xFF` on NO SIGN (see the contract below)

### S3 → P4 (5 bytes)

```
 0    1    2    3        4
| AA | 55 | 02 | command | checksum(XOR 0-3) |
```

- `0x01` ACK_STOP — sign confirmed, P4 stops TX (inference + SD write still run)
- `0x02` RESUME_JUNCTION — tasks done, P4 resumes TX

Both commands retry up to 3 times with RX-line idleness verification on the S3 side.

## Flags + label_id Contract (apply before using confidence)

`flags` byte layout (shared with the P4 TFLite firmware + SD cards `labels.csv`):

```
  7  6  5  4   3  2  1  0
 ├───verdict──┤ ├─sign_ready─┤
```

| Bit range  | Name         | Values                                            |
|---|---|---|
| bits 7–4   | verdict      | `0x0` → MATCH<br>`0xE` → MISMATCH<br>`0xF` → NO SIGN / OUT of distribution |
| bits 3–0   | sign_ready   | `0x2` → sign_ready (legacy value, preserved) |

**Common values (multi-plate sign firmware):**

| `flags` | Meaning | `label_id` (byte 5) |
|---|---|---|
| `0x02`  | **MATCH** — one allowed plate read exactly | matched **plate index** `0 … kTargetSignCount-1` |
| `0xE2`  | **MISMATCH** — a sign is present, no allowed plate matched | `0xFE` |
| `0xF2`  | **NO SIGN** — no ink / rejected | `0xFF` |

`confidence` (byte 6) is always the weakest slot's best raw score (0..255);
ignore it on MISMATCH / NO SIGN.

`label_id` is resolved through this folder's `model_settings.h`, which is a
copy of the sign export: `kTargetSignCount` and
`kTargetSignSequences[kTargetSignCount][4]` name the plate, e.g.
`label=1(B18)`.  Keep that copy in step with the export flashed to the P4.  A
legacy P4 build that still sends a class index in `label_id` is resolved
through `kCategoryLabels` as before.

**In code, always use these helpers (defined in `S3_UART_Receiver.ino`):**

```cpp
pkt_is_real_sign(p.flags)   // true  → MATCH,     OK to use label/confidence
pkt_is_mismatch(p.flags)    // true  → MISMATCH,  never confirms a sign
pkt_is_no_sign(p.flags)     // true  → NO SIGN,   ignore label/confidence
label_is_plate_index(p.label_id, pkt_is_real_sign(p.flags))
```

Other low-nibble values are reserved; treat them the same as `0x02` for future
compat.  Any high-nibble value other than `0x0` / `0xE` / `0xF` is reserved
and should currently be treated as "No Sign" to be safe.

## Sign Confirmation

Confidence + continuity (no junction/sign phase flags), with the additional
constraint that **MISMATCH and NO SIGN frames never count**, even if softmax
hallucinated a confident, stable label for 10 consecutive frames on an empty
scene.

Trigger: the same **matched plate index** for `kSignConfirmFrames`
consecutive frames where `pkt_is_real_sign(flags) == true` AND
`confidence >= kSignConfirmConfidence`.  Any MISMATCH/NO SIGN frame, any
low-confidence frame, or any plate change resets the counter.

| Constant | Default | Description |
|---|---|---|
| `kSignConfirmFrames`    | 5   | Consecutive real-sign frames required to confirm |
| `kSignConfirmConfidence`| 120 | Minimum uint8 confidence per frame (0-255). int8 quantized models typically output subdued scores; 120/255 ≈ 47%. Tune per model. |
| `kTaskDurationMs`       | 3000 | Duration of S3-side task execution after ACK_STOP and before RESUME_JUNCTION. Tune to your real workload (display, actuator, logging). |

### Example sequence

```
frame=41 label=0(A12) conf=221 flags=0x02 [REAL SIGN]  cnt=1
frame=42 label=0(A12) conf=218 flags=0x02 [REAL SIGN]  cnt=2
frame=43 label=0(A12) conf=230 flags=0x02 [REAL SIGN]  cnt=3
frame=44 label=NO_SIGN conf=214 flags=0xF2 [NO SIGN (OOD)]  cnt=0   ← hand moved, reset
frame=45 label=0(A12) conf=241 flags=0x02 [REAL SIGN]  cnt=1
…
frame=49..53: 5 in a row MATCH A12 ≥ 120 → ACK_STOP sent
```

A different allowed plate resets continuity too — the counter tracks the
**matched plate index**, so switching cars mid-stream cannot confirm the
wrong one.  MISMATCH frames print `label=MISMATCH(NO MATCH)` with
`flags=0xE2` and never advance the counter.

## Baud Rates

- P4↔S3 UART: 921600
- S3 debug (USB CDC): 921600 (configurable via `kDebugBaud`, defaulted to match P4 so one Serial monitor setting works for both MCUs)
