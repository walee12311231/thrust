# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Build & Flash Commands

This is a PlatformIO project targeting the Raspberry Pi Pico (RP2040) with the Arduino framework (earlephilhower core).

```bash
# Build
pio run

# Build and upload to connected Pico
pio run --target upload

# Open serial monitor (115200 baud)
pio device monitor

# Clean build artifacts
pio run --target clean
```

PlatformIO uses the community `platform-raspberrypi` platform from `maxgerhardt/platform-raspberrypi` (not the official one) with `board_build.core = earlephilhower`.

**Git remote (SSH):** `git@github.com:walee12311231/thrust.git`

## Visualizer

`host/visualizer.py` — pygame 3D CubeSat orientation display. Reads the Pico's USB serial output and renders a rotating 6U model.

```bash
python3 host/visualizer.py              # auto-detects first /dev/cu.usbmodem* port
python3 host/visualizer.py /dev/cu.usbmodemXXX   # explicit port
```

Keybindings: `R` resets orientation, `ESC` quits. Uses a complementary filter (α = 0.97) — accelerometer corrects roll/pitch drift, yaw is gyro-only.

## Architecture

The project is a sensor data acquisition firmware intended to read IMU and thermocouple data from a Pico board, presumably for thrust measurement.

**`src/main.cpp`** — Entry point. Initializes I2C on pins SDA=4, SCL=5, brings up the MPU6050, and streams calibrated samples to USB serial at 50 Hz (every 20 ms). Serial format is bare CSV: `ax,ay,az,gx,gy,gz\n` (g and deg/s) so `visualizer.py` can parse it directly.

**`lib/Sensors/`** — Project-local sensor drivers, compiled as a static library by PlatformIO:

- **`MPU6050`** — I2C IMU driver (accelerometer + gyro + temperature). Key design points:
  - `autoInit()` is the one-call setup: configures I2C, verifies WHO_AM_I, sets ranges, and runs `calibrate()`.
  - `calibrate()` averages `N` samples at rest to zero out accel/gyro bias; accel Z offset is corrected for 1g gravity.
  - `streamSample()` prints a human-readable CSV-ish line to any `Stream` (Serial, etc.).
  - All I2C operations return `bool`; false means bus error.
  - Default I2C address: `0x68`. Alternate address (`0x69`) available via constructor.

- **`MAX31856`** — SPI thermocouple amplifier driver. `begin(type)` puts the chip in continuous-conversion mode and verifies the CR1 readback. `read(sample)` burst-reads the 19-bit linearised TC temp and 14-bit cold-junction temp. Supports types B/E/J/K/N/R/S/T via the `ThermocoupleType` enum.

## Conventions

- Sensor drivers take a reference to the hardware bus object (`TwoWire&`, and eventually `SPIClass&`) in their constructor — do not instantiate the bus inside the driver.
- All register constants are in an anonymous namespace in the `.cpp` file, not exposed in the header.
- Return `bool` from all hardware I/O functions; never throw or assert on bus errors.

