# S3_UART_Receiver

ESP32-S3 receives inference results from P4 and sends control messages back.

## Wiring

- P4 TX(GPIO11) → S3 RX(GPIO44)
- P4 RX(GPIO10) ← S3 TX(GPIO43)
- P4 PWM(GPIO9) → S3 ADC(GPIO8)  (presence detection)
- GND → GND

## Protocols

### P4 → S3 (9 bytes)
`0xAA 0x55 0x01` + frame_id(LE16) + label + confidence + flags + checksum(XOR 0-7)

### S3 → P4 (5 bytes)
`0xAA 0x55 0x02` + command + checksum(XOR 0-3)
- `0x01` ACK_STOP — sign confirmed, P4 stops TX
- `0x02` RESUME_JUNCTION — tasks done, P4 resumes TX

## Sign Confirmation

Confidence-only (no phase flags). Trigger: same label for `kSignConfirmFrames` consecutive frames with confidence ≥ `kSignConfirmConfidence`.

| Constant | Default | Description |
|---|---|---|
| `kSignConfirmFrames` | 5 | Frames to confirm |
| `kSignConfirmConfidence` | 120 | Confidence threshold (0-255) |
| `kTaskDurationMs` | 3000 | Task duration before RESUME |

Both ACK_STOP and RESUME_JUNCTION have 3x retry with RX-line verification.

## Baud Rates

- P4↔S3 UART: 921600
- S3 debug (USB CDC): 115200 (configurable via `kDebugBaud`)
