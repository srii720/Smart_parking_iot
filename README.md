# Smart Parking ESP32 - first sensor (P1)

This project is the first bench-tested slice of the smart parking system: one ESP32-WROOM-32 reads one HC-SR04 ultrasonic sensor, prints readings to the Serial Monitor, and serves a small live dashboard to a phone or laptop.

The current version is intentionally for one physical slot. Add the remaining sensors after P1 has been wired, measured, and calibrated.

## Hardware

- ESP32 DevKit / ESP32-WROOM-32 (38-pin `esp32dev` board)
- One HC-SR04 ultrasonic sensor
- Three 5.6 kΩ resistors for the Echo voltage divider (one upper resistor and two lower resistors in series)
- Breadboard and jumper wires
- USB cable for the ESP32

## Wiring for P1

| HC-SR04 | ESP32 / breadboard |
|---|---|
| VCC | ESP32 `5V` or `VIN` |
| GND | ESP32 `GND` |
| TRIG | GPIO 14 |
| ECHO | GPIO 34 through the divider below |
| LED anode (long leg) | GPIO 13 through a series resistor |
| LED cathode (short leg / flat side) | ESP32 GND |

GPIO 34 is input-only, which is appropriate for the sensor's Echo output. Do not connect a 5 V Echo signal directly to an ESP32 GPIO.

```text
HC-SR04 ECHO ──[5.6 kΩ]──●── ESP32 GPIO 34
                       |
                     [5.6 kΩ]
                       |
                     [5.6 kΩ]
                       |
HC-SR04 GND ───────────●── ESP32 GND
```

Put one 5.6 kΩ resistor between ECHO and the shared junction. Connect GPIO 34 and one end of a second 5.6 kΩ resistor to that same junction. Connect the second resistor end-to-end with the third 5.6 kΩ resistor, then connect the third resistor to GND. Connect sensor GND and ESP32 GND together. This 5.6 kΩ / 11.2 kΩ divider reduces a 5 V Echo signal to about 3.3 V. The fourth resistor is spare.

Power the sensor from the ESP32's 5 V/VIN pin while the ESP32 is powered by USB. Check the labels on your particular ESP32 board before connecting power.

## Set up and upload

Open this folder in VS Code with the PlatformIO extension, or use the PlatformIO command line from this folder:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

The firmware starts an access point by default, so no router credentials are needed for the first test:

- Wi-Fi name: `SmartParking-Setup`
- Password: `parking123`
- Dashboard: `http://192.168.4.1/`

If your phone warns that the Wi-Fi has no internet, stay connected and open the dashboard address above. This firmware uses the ESP32 access point directly; router credentials are not configured in this version.

### If firmware upload fails

Close the Serial Monitor before uploading; it holds the USB serial port open. The project uses a conservative `115200` upload speed to improve reliability. If the ESP32 does not enter its bootloader automatically, hold the board's `BOOT` button when PlatformIO prints `Connecting...` and release it once writing begins. Use a USB data cable and the port shown by `pio device list`.

An upload must finish with `SUCCESS` before the new firmware is running.

## First sensor test

1. Keep the ESP32 unplugged while making the connections.
2. Check each wire against the table and divider diagram, especially ECHO to GPIO 34.
3. Connect the ESP32 by USB and upload the firmware.
4. Open the Serial Monitor at `115200` baud. You should see P1 distance readings about ten times per second.
5. Hold a flat object 20-50 cm in front of the sensor, then move it away. The distance should change consistently.
6. Connect your phone to `SmartParking-Setup` and open `http://192.168.4.1/` to see the same reading in the dashboard.

For the eventual parking setup, mount the sensor above the bay facing downward. With no vehicle, measure the floor distance. Then measure the roof or bonnet distance with a vehicle or a suitable test object. Set the occupied and vacant thresholds in `src/main.cpp` so they fall between the measured empty and occupied ranges. The defaults are demonstration values, not calibrated values for your parking space.

The firmware takes a reading every 100 ms, uses the median of up to three valid readings to reduce spikes, and requires two consistent samples before changing the slot state. This makes occupancy updates quicker while retaining basic noise filtering. It uses separate occupied/vacant thresholds (20 cm and 25 cm by default). If no Echo pulse arrives, Serial Monitor prints `NO ECHO` with the GPIO 34 level and wiring checks. Valid readings clear the sensor error automatically.

## Dashboard and status endpoint

- `/` serves the phone-friendly P1 dashboard.
- `/api/status` returns the current sensor reading, occupancy state, and sensor-error state as JSON.

The dashboard is served locally by the ESP32. This version does not use a cloud service, database, MQTT gateway, mobile app installation, magnetic sensors, or gate servo.

## Configuration

In `src/main.cpp`:

- `TRIG_PIN` and `ECHO_PIN` select the sensor pins (`14` and `34` for the current wiring).
- `OCCUPIED_BELOW_CM` is the occupied threshold.
- `VACANT_ABOVE_CM` is the release threshold; keep it several centimeters above the occupied threshold.
- `SAMPLE_INTERVAL_MS`, `MEDIAN_SIZE`, `DEBOUNCE_SAMPLES`, and `ERROR_AFTER_BAD_READS` control sampling and filtering.
- The setup access point starts automatically: `SmartParking-Setup` / `parking123`.
- `LED_ENABLED` is `true`. The LED on GPIO 13 lights when P1 is marked occupied. Connect the LED anode (long leg) to GPIO 13 through a series resistor, and its cathode (short leg / flat side) to GND. A 5.6 kΩ resistor is safe but may make the LED dim; 330 Ω to 1 kΩ gives a brighter indicator.

GPIO 14 avoids the GPIO 5 boot-strapping pin. GPIO 34 is input-only, which suits the ECHO signal. Never connect ECHO directly to the ESP32; keep the divider in place.

## Extending to more spaces

After P1 works reliably, add one sensor at a time and keep each Echo line on its own divider and input-only ESP32 pin. The original four-slot pin plan was:

| Slot | TRIG | ECHO |
|---|---:|---:|
| P1 | GPIO 14 | GPIO 34 |
| P2 | GPIO 16 | GPIO 35 |
| P3 | GPIO 17 | GPIO 36 |
| P4 | GPIO 25 | GPIO 39 |

GPIO 34-39 are input-only on the ESP32-WROOM-32. Keep all sensor and ESP32 grounds connected together. Do not add all four sensors until P1's distance and thresholds are stable.
