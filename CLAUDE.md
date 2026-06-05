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

`hil_panel/` holds two self-contained Web Serial panels plus a launcher (`index.html`). Open any of them directly in Chrome or Edge (Web Serial API is not available in Firefox/Safari) — they have **no external dependencies** (no CDN scripts, no web fonts) and run fully offline from `file://`:

- **`test_bench.html`** (Version A) — per-peripheral checkout with raw + interpreted readouts and a one-click self-test that fires every command and verifies the replies (incl. timed sweeps and error branches).
- **`mission.html`** (Version B) — integrated Setup→Standby→Active mission console with a dependency-free Canvas-2D tracker sphere, trajectory simulator, and pipeline view. A single source-of-truth state keeps az / coordinate / sphere consistent.

The JSON protocol both panels speak is the single source of truth in **`hil_panel/PROTOCOL.md`**; keep it, the firmware, and the panels in sync.

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
```

Pan control has two parallel update paths: `updatePan()` reads the real AS5048A; `updatePanWithPosition(deg)` accepts an externally supplied angle so the loop can run when the encoder isn't present. `main.cpp` picks between them each PID tick based on the runtime `hasEncoder` flag. The simulated path reads back the stepper's commanded position via `getStepperPositionDeg()`, which is exactly what HIL mode uses by default.

The tilt servo is open-loop. Telemetry reports `el_c == el_t` by design — there is no servo feedback path.

> The MMC5983MA magnetometer — and its `MagManager` library, `MAGNETIC_DECLINATION_DEG`, and `Navigation::computeTrueHeading` — has been **removed from this project**. The live sensors are the AS5048A encoder and the u-blox GNSS only.

### Navigation pipeline (`lib/Navigation`)

Target ingestion via the `inject` command runs the full **WGS84 → ECEF → ENU → Az/El** transform. `computePointing` is the entry point; the rest (`geodeticToECEF`, `ecefToENU`, `primeVerticalRadius`) are building blocks. All public API takes degrees; math is in radians internally.

### Serial protocol (newline-delimited JSON, 115200 baud)

The full schema lives in **`hil_panel/PROTOCOL.md`** — that file is authoritative; this is a summary.

Inbound `cmd` values handled in `processCommand`:
- `direct` — `{az, el}` (either or both)
- `inject` — `{lat, lon, alt}` → routed through Navigation
- `set_base` — overrides base station for `inject`
- `set_pid` — `{kp, ki, kd}`
- `set_sensors` — `{enc, gps}` toggles which sensors the firmware uses as live sources
- `home`, `sweep_az`, `sweep_el`, `stop`

Outbound message types: `ready` (one-shot on boot), `tel` (10 Hz, `TELEMETRY_INTERVAL_MS`), `ack`, `err`. Telemetry numeric fields use `serialized(String(x, n))` to fix decimal precision — keep this pattern if you add new float fields.

Telemetry emits each sensor's **raw** wire value beside the actuator state so the panels can show raw + interpreted:
- pan stepper: `pid`, `sps`, `step` (raw count), `pos` (interpreted deg)
- encoder: `enc_raw` (14-bit counts), `enc_deg` (interpreted) — always present (a real SPI read each tick)
- tilt servo: `srv_us` (raw commanded pulse width)
- GNSS (when `gnssHardwareOk`): `gnss_fix`, `gnss_sats`, `gnss_lat_e7`, `gnss_lon_e7`, `gnss_alt_mm`, `gnss_hacc_mm` (all raw integers; panels interpret)
- `base_lat/lon/alt`, `base_src`; `tgt_lat/lon/alt` after the first `inject`

`ready` carries `gps_hw` (bool) to tell the dashboard whether the GNSS module answered at boot.

### Loop timing

- PID tick: 200 Hz (`PID_INTERVAL_US = 5000`)
- Stepper pulses: every `loop()` iteration via `actuators.runPan()` — must remain non-blocking
- Telemetry: 10 Hz
- Serial parse: non-blocking, char-at-a-time accumulation in a 256-byte `String` buffer

### Unused-in-HIL components

`lib/TelemetryPacket` defines the 22-byte LoRa binary packet for the real receiver (UART2 from a Heltec gateway); it's not referenced by the HIL `main.cpp`. It is present for the production firmware that lives elsewhere — leave the include available, but don't expect it on any HIL code path.

`lib/SignalFilter` (alpha-beta filter, tuned via `AB_ALPHA/BETA` in config) **is now exercised in HIL** on the `track` command path: `main.cpp` runs three instances (one per E/N/U axis, `dt = 1/TRACK_STREAM_HZ`) to smooth the streamed flight-replay position before the WGS84 reconstruction and pointing. See the flight-replay pipeline below.

### Flight replay (`track` command)

`hil_panel/mission.html` can stream a recorded rocket flight as if it were live telemetry. `tools/convert_flight.py` resamples `flight_data.csv` (local Cartesian X=East, Y=North, Z=Up MSL; launch pad 700 m N / 200 m E of the tracker, ground 892 m) to a fixed 15 Hz and emits the generated `hil_panel/flight_trajectory.js`. The panel's **Flight Replay** player streams `{cmd:"track", e, n, u}` (local ENU metres, optional synthetic GPS noise) at 15 Hz. The firmware pipeline: **alpha-beta filter (metres) → `enuToGeodetic` (flat-earth ENU→WGS84) → `computePointing` (WGS84→Az/El) → PID pan + servo tilt**, echoing raw vs filtered ENU in telemetry (`trk`, `trk_*_raw`, `trk_*`). Regenerate the trajectory with `python tools/convert_flight.py`; do not hand-edit `flight_trajectory.js`.

### Misc

- `lib copy/` is a stray duplicate containing only the placeholder README. Ignore it.
- `BOARD_HAS_PSRAM` and `ARDUINO_USB_CDC_ON_BOOT=1` are required build flags — the second is what makes `Serial` go to the USB-CDC port the Web UI connects to.
