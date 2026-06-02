# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

Hardware-In-the-Loop (HIL) actuator test harness for the **Cursr-V Antenna Tracker**. The ESP32-S3 firmware drives a pan/tilt rig and exchanges newline-delimited JSON with the companion Web UI in `hil_panel/`. There is no flight-software entry point in this branch — `src/main.cpp` is the test harness only.

## Build / Flash / Monitor (PlatformIO)

```powershell
pio run                       # build firmware for env esp32s3usbotg
pio run -t upload             # flash over USB-CDC
pio device monitor            # serial monitor @ 115200
pio run -t clean              # clean build artifacts
```

No unit test framework is wired up — `test/` contains only PlatformIO's placeholder README.

## Web UI

`hil_panel/index.html` is a self-contained Web Serial client. Open it directly in Chrome or Edge (Web Serial API is not available in Firefox/Safari). It speaks the JSON protocol defined in `src/main.cpp`.

## Architecture

### Hardware configuration (`include/config.h`)

Fixed hardware stack — no compile-time variant selection remains:

- **Pan**: NEMA17 stepper via TB6600 (DRIVER mode), AS5048A encoder on SPI
- **Tilt**: DS51150 servo at 300 Hz via MCPWM, 2:1 belt reduction → 135° physical travel

All pin maps, mechanical constants, and PID gains live in `config.h`. No ifdefs gate them.

### Layered actuator stack

```
main.cpp  ──►  Actuators (lib/ActuatorControl)
                  ├── AccelStepper (pan)         → PID speed control, anti-windup
                  ├── AS5048A via SPI (encoder)  → with simulated-encoder fallback
                  └── ESP-IDF MCPWM (tilt servo) → legacy driver/mcpwm.h API
           ──►  GNSSManager (lib/GNSSManager)    → u-blox SAM-M10Q via I2C
           ──►  MagManager  (lib/MagManager)     → MMC5983MA via I2C (shared bus)
```

Pan control has two parallel update paths: `updatePan()` reads the real AS5048A; `updatePanWithPosition(deg)` accepts an externally supplied angle so the loop can run when the encoder isn't present. `main.cpp` picks between them each PID tick based on the runtime `hasEncoder` flag. The simulated path reads back the stepper's commanded position via `getStepperPositionDeg()`, which is exactly what HIL mode uses by default.

The tilt servo is open-loop. Telemetry reports `el_c == el_t` by design — there is no servo feedback path.

**MagManager** wraps the MMC5983MA in 10 Hz continuous mode. `update()` calls `readFieldsXYZ` (non-blocking register read) and computes True North heading via `atan2(X, -Y) + MAGNETIC_DECLINATION_DEG`. Only called when `magHardwareOk && hasMagnetometer`; result appears in telemetry as `mag_hdg`.

### Navigation pipeline (`lib/Navigation`)

Target ingestion via the `inject` command runs the full **WGS84 → ECEF → ENU → Az/El** transform. `computePointing` is the entry point; the rest (`geodeticToECEF`, `ecefToENU`, `primeVerticalRadius`) are building blocks. All public API takes degrees; math is in radians internally.

### Serial protocol (newline-delimited JSON, 115200 baud)

Inbound `cmd` values handled in `processCommand`:
- `direct` — `{az, el}` (either or both)
- `inject` — `{lat, lon, alt}` → routed through Navigation
- `set_base` — overrides base station for `inject`
- `set_pid` — `{kp, ki, kd}`
- `set_sensors` — `{enc, mag, gps}` toggles which sensors the firmware *pretends* to have
- `home`, `sweep_az`, `sweep_el`, `stop`

Outbound message types: `ready` (one-shot on boot), `tel` (10 Hz, `TELEMETRY_INTERVAL_MS`), `ack`, `err`. Telemetry numeric fields use `serialized(String(x, n))` to fix decimal precision — keep this pattern if you add new float fields.

Additional `tel` fields (conditional):
- `mag_hdg` — True North heading in degrees; present when `magHardwareOk && hasMagnetometer`
- `gnss_fix`, `gnss_sats`, `gnss_hacc` — present when `gnssHardwareOk`
- `tgt_lat`, `tgt_lon`, `tgt_alt` — last injected target; present after first `inject` command

`ready` message also carries `mag_hw` (bool) and `gps_hw` (bool) to tell the dashboard which hardware was found at boot.

### Loop timing

- PID tick: 200 Hz (`PID_INTERVAL_US = 5000`)
- Stepper pulses: every `loop()` iteration via `actuators.runPan()` — must remain non-blocking
- Telemetry: 10 Hz
- Serial parse: non-blocking, char-at-a-time accumulation in a 256-byte `String` buffer

### Unused-in-HIL components

`lib/TelemetryPacket` defines the 22-byte LoRa binary packet for the real receiver (UART2 from a Heltec gateway); it's not referenced by the HIL `main.cpp`. `lib/SignalFilter` (alpha-beta filter, tuned via `AB_ALPHA/BETA/DT` in config) is similarly unused here. Both are present for the production firmware that lives elsewhere — leave the includes available, but don't expect them on any code path the HIL harness exercises.

### Misc

- `lib copy/` is a stray duplicate containing only the placeholder README. Ignore it.
- `BOARD_HAS_PSRAM` and `ARDUINO_USB_CDC_ON_BOOT=1` are required build flags — the second is what makes `Serial` go to the USB-CDC port the Web UI connects to.
