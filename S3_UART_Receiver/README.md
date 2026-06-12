# S3_UART_Receiver (ESP32-S3)

This sketch receives 8-byte inference packets from ESP32-P4 over UART and prints the decoded result.

It also includes a simple **P4 presence check**:

- S3 reads an ADC pin (`analogRead`) until it detects a stable value indicating the P4 handshake signal is present
- After detection, S3 starts UART and decodes labels as usual

## Wiring

Connect **GND to GND** between the two boards.

Then connect the handshake (analog) line:

- P4 handshake PWM (GPIO9) -> S3 ADC input (GPIO8)

Then connect UART:

- P4 TX -> S3 RX
- P4 RX <- S3 TX

## UART pins used in this project

The current default pins are defined in [S3_UART_Receiver.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/S3_UART_Receiver.ino):

- UART (from P4): `Serial1`, **RX=GPIO44**, **TX=GPIO43**, **921600 baud**
- Debug log: `Serial` (USB CDC), **115200 baud**

## Arduino IDE settings (ESP32-S3)

Use the Arduino IDE Tools menu settings shown in [setting_s3.png](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/setting_s3.png).

![setting_s3](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/setting_s3.png)

## P4 presence check (analog)

S3 waits for:

- `analogRead(GPIO8) >= 2000` (using an average of multiple samples)

After that, it starts UART and begins parsing label packets.

## Packet format (8 bytes)

- Byte0: `0xAA`
- Byte1: `0x55`
- Byte2: `msg_type` (default: `0x01`)
- Byte3-4: `frame_id` (uint16, little-endian)
- Byte5: `label_id` (uint8)
- Byte6: `confidence` (uint8)
- Byte7: `flags` (uint8, bit0 = stop)

## Label names

This project includes `model_settings.cpp/.h`. `label_id` is mapped to a string by `kCategoryLabels[]`.
