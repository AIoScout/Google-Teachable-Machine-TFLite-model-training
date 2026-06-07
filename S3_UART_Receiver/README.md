# S3_UART_Receiver (ESP32-S3)

This sketch receives 8-byte inference packets from ESP32-P4 over UART and prints the decoded result.

## Wiring

Connect **GND to GND** between the two boards.

Then connect UART:

- P4 TX -> S3 RX
- P4 RX <- S3 TX

## UART pins used in this project

The current default pins are defined in [S3_UART_Receiver.ino](file:///Users/koil/Google-Teachable-Machine-TFLite-model-training/S3_UART_Receiver/S3_UART_Receiver.ino):

- UART (from P4): `Serial1`, **RX=GPIO12**, **TX=GPIO13**, **921600 baud**
- Debug log: `Serial0`, **115200 baud**

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

