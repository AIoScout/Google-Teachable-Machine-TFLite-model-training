# S3\_UART\_Receiver (ESP32-S3)

This sketch receives 9-byte inference packets from ESP32-P4 over UART, prints the decoded result, **and sends control messages back to P4** to gate transmission and switch crop modes.

It also includes a simple **P4 presence check**:

- S3 reads an ADC pin (`analogRead`) until it detects a stable value indicating the P4 handshake signal is present
- After detection, S3 starts UART and decodes labels as usual

## Wiring

Connect **GND to GND** between the two boards.

Then connect the handshake (analog) line:

- P4 handshake PWM (GPIO9) -> S3 ADC input (GPIO8)

Then connect UART (bidirectional — P4 also has a UART RX task that reads S3 control packets):

- P4 TX (GPIO11) -> S3 RX (GPIO44)
- P4 RX (GPIO10) <- S3 TX (GPIO43)

## UART pins used in this project

The current default pins are defined in [S3\_UART\_Receiver.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/S3_UART_Receiver.ino):

- UART (to/from P4): `Serial0`, **RX=GPIO44**, **TX=GPIO43**, **921600 baud**
- Debug log: `Serial` (USB CDC), **115200 baud**

## Arduino IDE settings (ESP32-S3)

Use the Arduino IDE Tools menu settings shown in [setting\_s3.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/setting_s3.png).

![setting\_s3](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/setting_s3.png)

## P4 presence check (analog)

S3 waits for:

- `analogRead(GPIO8) >= 2000` (using an average of multiple samples)

After that, it starts UART and begins parsing label packets.

## P4 → S3 Packet format (9 bytes, inference results)

- Byte0: `0xAA`
- Byte1: `0x55`
- Byte2: `msg_type` (`0x01` = inference result)
- Byte3-4: `frame_id` (uint16, little-endian)
- Byte5: `label_id` (uint8)
- Byte6: `confidence` (uint8, 0-255)
- Byte7: `flags` (uint8, stage state)
  - `1`: junction stage active
  - `2`: sign stage active
- Byte8: `checksum` (uint8, XOR of Byte0..Byte7)

## S3 → P4 Packet format (5 bytes, control messages)

S3 sends control packets back to P4 to gate inference transmission and switch crop modes:

- Byte0: `0xAA`
- Byte1: `0x55`
- Byte2: `msg_type` (`0x02` = control)
- Byte3: `command`
  - `0x01` = `ACK_STOP` — sign confirmed, P4 should stop transmitting
  - `0x02` = `RESUME_JUNCTION` — tasks complete, P4 should resume + switch to junction mode
- Byte4: `checksum` (uint8, XOR of Byte0..Byte3)

## Sign Confirmation & Task Cycle

The S3 runs a sign confirmation state machine in `uart_task`:

1. **Detection**: When P4 is in sign_ready phase (`flags >= 2`) and confidence exceeds the threshold, S3 tracks consecutive frames of the same sign class.

2. **Confirmation**: After `kSignConfirmFrames` (default: 5) consecutive frames of the same sign class with confidence ≥ `kSignConfirmConfidence` (default: 180/255 ≈ 70%), the sign is confirmed.

3. **ACK_STOP**: S3 sends `ACK_STOP` to P4, which disables UART TX (inference continues, but packets are dropped).

4. **Tasks**: S3 performs its local tasks for `kTaskDurationMs` (default: 3000 ms). Replace this delay with real work (display update, logging, actuator control, etc.).

5. **RESUME_JUNCTION**: S3 sends `RESUME_JUNCTION` to P4, which re-enables UART TX and switches crop mode to `CROP_MODE_JUNCTION`.

6. **Reset**: S3 resets its confirmation state and waits for the next sign.

### Tunable Constants (in `S3_UART_Receiver.ino`)

| Constant | Default | Description |
|---|---|---|
| `kSignConfirmFrames` | 5 | Consecutive frames needed to confirm a sign |
| `kSignConfirmConfidence` | 180 | Minimum confidence (0-255, ~70%) |
| `kTaskDurationMs` | 3000 | Simulated task time before sending RESUME |

## Label names

This project includes `model_settings.cpp/.h`. `label_id` is mapped to a string by `kCategoryLabels[]`. Class types (`kClassTypes`) distinguish sign classes (0) from road classes (1).
