# Speed Data Communication Protocol Specification

This document defines the communication protocol between the Android Tag Application and the DWM1000 MCU (Receiver) via USB Serial.

## 1. Hardware Interface
- **Physical Link**: USB OTG (Android Host) to USB-Serial Bridge (MCU/DWM1000).
- **Communication Type**: UART over USB.

## 2. Serial Configuration
The receiver (MCU) must configure its UART interface with the following parameters:
- **Baud Rate**: 115200 bps
- **Data Bits**: 8 bits
- **Stop Bits**: 1 bit
- **Parity**: None
- **Flow Control**: None

## 3. Data Format
The Android application sends speed data as an **ASCII String** followed by a newline character.

- **Payload Structure**: `[Speed_Value]\n`
- **Units**: Kilometers per hour (km/h)
- **Precision**: 2 decimal places.
- **Delimiter**: `\n` (Line Feed, Hex: `0x0A`)

### Example Payloads:
- `12.34\n` (Represents 12.34 km/h)
- `0.00\n` (Represents 0.00 km/h)
- `105.80\n` (Represents 105.80 km/h)

## 4. Transmission Frequency
- **Interval**: Approximately **100ms** (10Hz).
- **Note**: The frequency depends on the Android device's GPS update capability. The MCU should be able to handle asynchronous bursts of data.

## 5. Receiver Implementation Guide (MCU side)

### Logic Flow:
1. Initialize UART at 115200 baud.
2. Buffer incoming bytes until a `\n` (0x0A) character is detected.
3. Convert the buffered ASCII string to a floating-point number.
4. Use the floating-point value for DWM1000 ranging or logic.

### Example C++ (Arduino-style) Snippet:
```cpp
void loop() {
  if (Serial.available() > 0) {
    String data = Serial.readStringUntil('\n');
    if (data.length() > 0) {
      float currentSpeed = data.toFloat();
      // Process the speed (e.g., adjust DWM1000 transmit power or frequency)
      Serial.print("Received Speed: ");
      Serial.println(currentSpeed);
    }
  }
}
```

### Example C (Bare Metal) Logic:
```c
char rx_buffer[16];
uint8_t rx_idx = 0;

void UART_IRQHandler() {
    char c = UART_ReceiveByte();
    if (c == '\n') {
        rx_buffer[rx_idx] = '\0';
        float speed = atof(rx_buffer);
        process_speed(speed);
        rx_idx = 0;
    } else if (rx_idx < 15) {
        rx_buffer[rx_idx++] = c;
    }
}
```

## 6. Error Handling
- **Invalid Data**: If the received string cannot be parsed as a float, the packet should be discarded.
- **Buffer Overflow**: Ensure the receive buffer is at least 16 bytes to accommodate long strings and delimiters.
