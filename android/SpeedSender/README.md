# SpeedSender

Android Java app for sending smartphone speed data to an ESP32 tag.

## Modes

- USB C-to-C: phone acts as USB host and writes serial data to the ESP32.
- Bluetooth: phone writes serial data to a paired Bluetooth SPP device.

## Payload

The app sends one line per second:

```text
TAG,<tagId>,SPD,<speed_mps>,TS,<timestamp_ms>
```

Example:

```text
TAG,2,SPD,1.42,TS,1710000000000
```

## ESP32 Serial Parsing Idea

```cpp
if (Serial.available()) {
  String line = Serial.readStringUntil('\n');
  if (line.startsWith("TAG,")) {
    // Parse TAG,<id>,SPD,<mps>,TS,<ms>
  }
}
```

For Bluetooth mode, the ESP32 side should expose Bluetooth Classic SPP, for example with `BluetoothSerial`.
For USB C-to-C mode, the ESP32 board must appear to Android as a USB serial device. Use a data-capable C-to-C cable, not a charge-only cable.

## Android Studio

Open `android/SpeedSender` in Android Studio, sync Gradle, then run on a phone.

For USB mode, use a data-capable C-to-C cable and a board that appears as a USB serial device.
