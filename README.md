# Smart Car Parking — ESP32-WROOM-32

This PlatformIO project monitors four parking slots with HC-SR04 ultrasonic sensors and serves a live dashboard directly from the ESP32. The optional magnetic-sensor input path is present but disabled by default.

## Safe 38-pin pinout

| Function | Slot 1 | Slot 2 | Slot 3 | Slot 4 | Notes |
|---|---:|---:|---:|---:|---|
| HC-SR04 Trig | GPIO13 | GPIO14 | GPIO16 | GPIO17 | ESP32 output |
| HC-SR04 Echo | GPIO34 | GPIO35 | GPIO36 | GPIO39 | Input-only pins; use a divider |
| Magnetic input (optional) | GPIO32 | GPIO33 | GPIO25 | GPIO26 | Active-low with `INPUT_PULLUP` |

This mapping avoids flash-connected GPIO6–11 and boot-strapping GPIO0, 2, 5, 12 and 15. GPIO34–39 are used only as inputs. GPIO16/17 are suitable on the ESP32-WROOM-32 target used by `esp32dev`.

### HC-SR04 voltage warning

Power the HC-SR04 modules according to their module requirements, but do not connect a 5 V Echo output directly to an ESP32 GPIO. Use a resistor divider for every Echo line, for example 1 kΩ from Echo to the ESP32 pin and 2 kΩ from the ESP32 pin to GND. Connect all grounds together.

The firmware considers a slot occupied when the debounced distance is below `OCCUPIED_DISTANCE_CM` (20 cm). Adjust that constant in `src/main.cpp` for your physical mounting height.

## Build, flash and monitor on macOS

From this project directory:

```sh
pio run
pio run -t upload
pio device monitor -b 115200
```

If PlatformIO is not installed:

```sh
python3 -m pip install --user platformio
```

Set `WIFI_SSID` and `WIFI_PASSWORD` near the top of `src/main.cpp` before flashing. After boot, open the printed `http://<ESP32-IP>/` address on a phone or laptop connected to the same Wi-Fi. If station connection fails for 15 seconds, connect to Wi-Fi network `SmartParking-Setup` with password `parking123`, then open `http://192.168.4.1/`.

## Magnetic upgrade

`USE_MAGNETIC_SENSORS` is `false` by default, so the four ultrasonic sensors are the only occupancy source. Change it to `true` after wiring active-low magnetic sensors to the optional pins in the table. The magnetic path then updates debounced occupancy while ultrasonic readings remain visible for diagnostics.

The code is structured around `serviceUltrasonic()`, `serviceMagneticSensors()`, and the WebSocket transport, so a BLE service can later publish the same status model without changing the sensor state machine.
