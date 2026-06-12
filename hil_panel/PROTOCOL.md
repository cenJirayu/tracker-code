# HIL Serial Protocol

Newline-delimited JSON over USB-CDC at **115200 baud**. Every message is one
compact JSON object terminated by `\n`. This file is the single source of truth:
the firmware (`src/main.cpp`) and the web panels (`mission.html`,
`test_bench.html`, `component_check.html`) all implement exactly what is written
here.

The magnetometer has been removed from the project. Sensors are the **AS5048A
encoder** (pan feedback) and the **u-blox SAM-M10Q GNSS** (base station). The
actuators are the **NEMA17/TB6600 pan stepper** and the **DS51150 tilt servo**.

For every sensor the firmware emits both a **raw** value (as read off the wire)
and the panels show the **interpreted** value next to it, so the link can be
debugged end to end.

---

## Commands — host → firmware

| `cmd`         | Fields                          | Effect |
|---------------|---------------------------------|--------|
| `set_sensors` | `enc` bool, `gps` bool          | Toggle which sensors the firmware uses as live sources. |
| `direct`      | `az` deg, `el` deg (either/both)| Command pan/tilt angles directly; cancels any sweep. |
| `inject`      | `lat`, `lon` deg, `alt` m       | Target coordinate → Az/El via WGS84→ENU navigation. |
| `track`       | `e`, `n`, `u` metres            | Streamed local ENU position (rel. base, incl. launch offset) → alpha-beta filter → ENU→WGS84 → Az/El. Used by the flight-replay player. |
| `set_base`    | `lat`, `lon` deg, `alt` m       | Override the base-station origin used by `inject` / `track`. |
| `set_pid`     | `kp`, `ki`, `kd` (≥0)           | Retune the pan speed PID. |
| `home`        | —                               | Return to Az=0, El=0. |
| `sweep_az`    | —                               | Sweep azimuth 0→360 once, then stop. |
| `sweep_el`    | —                               | Sweep elevation 0→135→0 once, then stop. |
| `stop`        | —                               | Halt any active sweep. |

Malformed JSON or an unknown/missing `cmd` produces an `err` reply.

## Responses — firmware → host

Every response carries a `t` (type) discriminator.

### `ready` — emitted once at boot, after the self-test
```json
{ "t":"ready", "ver":"2.0.0", "board":"ESP32-S3",
  "motor":"NEMA17/TB6600", "servo":"DS51150/300Hz",
  "enc":false, "gps":false, "gps_hw":false }
```
`gps_hw` reports whether the GNSS module answered on I2C at boot (independent of
the runtime `gps` toggle).

### `ack` — command accepted; echoes the resolved values
```json
{ "t":"ack", "cmd":"direct", "az":45.0, "el":30.0 }
```

### `err` — command rejected
```json
{ "t":"err", "msg":"unknown command" }
```

### `tel` — telemetry, 10 Hz
```json
{ "t":"tel", "up":12345,
  "az_t":45.0, "az_c":44.6, "el_t":30.0, "el_c":30.0, "sweep":"none",
  "pid":12.50, "sps":320.0, "step":8123, "pos":44.6,
  "enc":false, "enc_raw":8190, "enc_deg":12.3,
  "srv_us":1444,
  "gps":false, "gnss_fix":3, "gnss_sats":9,
  "gnss_lat_e7":137563000, "gnss_lon_e7":1005018000,
  "gnss_alt_mm":10000, "gnss_hacc_mm":1500,
  "base_lat":13.756300, "base_lon":100.501800, "base_alt":10.0, "base_src":"manual",
  "tgt_lat":13.765300, "tgt_lon":100.501800, "tgt_alt":500.0 }
```

#### Field reference

**Pointing**
| Field   | Unit / range        | Meaning |
|---------|---------------------|---------|
| `up`    | ms                  | Uptime since boot. |
| `az_t`  | deg, 1 dp           | Azimuth target. |
| `az_c`  | deg, 1 dp           | Azimuth current (encoder if `enc`, else stepper position). Encoder value is referenced to the heading captured at boot (`0°` = power-on heading). |
| `el_t`  | deg, 1 dp           | Elevation target. |
| `el_c`  | deg, 1 dp           | Elevation current — equals `el_t` (servo is open-loop). |
| `sweep` | `none`\|`az`\|`el`  | Active sweep state. |

**Pan stepper** — raw control + interpreted position
| Field  | Kind        | Meaning |
|--------|-------------|---------|
| `pid`  | raw         | PID output (deg/s). |
| `sps`  | raw         | Commanded stepper speed (steps/s). |
| `step` | raw         | Absolute step count (`AccelStepper::currentPosition`). |
| `pos`  | interpreted | Stepper position in degrees `[0,360)`. |

**Encoder (AS5048A)** — always emitted (a real SPI read each tick)
| Field     | Kind        | Meaning |
|-----------|-------------|---------|
| `enc`     | flag        | Whether the encoder is the active pan feedback source. |
| `enc_raw` | raw         | Untouched 14-bit angle counts `[0,16383]` straight off the chip. |
| `enc_deg` | interpreted | Direction-corrected azimuth referenced to the heading captured at boot (`0°` = power-on heading). Equals `az_c` when `enc` is active. |

**Tilt servo (DS51150)** — open-loop
| Field    | Kind        | Meaning |
|----------|-------------|---------|
| `srv_us` | raw         | Commanded pulse width (µs, 500–2500). Interpreted angle = `el_c`. |

**GNSS (SAM-M10Q)** — present only when `gps_hw` was true at boot
| Field          | Kind        | Meaning |
|----------------|-------------|---------|
| `gps`          | flag        | Whether GNSS is the active base-station source. |
| `gnss_fix`     | raw         | Fix type code: 0 none, 2 2-D, 3 3-D, 4 GNSS+DR. |
| `gnss_sats`    | raw         | Satellites used. |
| `gnss_lat_e7`  | raw         | Latitude, 1e-7 deg (native wire integer). |
| `gnss_lon_e7`  | raw         | Longitude, 1e-7 deg. |
| `gnss_alt_mm`  | raw         | Altitude MSL, mm. |
| `gnss_hacc_mm` | raw         | Horizontal accuracy estimate, mm. |

Panels interpret these as: fix label, `lat = gnss_lat_e7 / 1e7`,
`hAcc = gnss_hacc_mm / 1000` m, etc.

**Base station / target**
| Field      | Kind        | Meaning |
|------------|-------------|---------|
| `base_lat` | interpreted | Base latitude (deg, 6 dp). |
| `base_lon` | interpreted | Base longitude (deg, 6 dp). |
| `base_alt` | interpreted | Base altitude (m). |
| `base_src` | flag        | `gnss` when a live GNSS fix drives the base, else `manual`. |
| `tgt_lat`/`tgt_lon`/`tgt_alt` | echo | Last `inject`/`track` target; present after the first such command. For `track` this is the WGS84 fix reconstructed from the filtered ENU. |

**Flight-replay track** — `trk` is always present; the rest only while a replay is active
| Field          | Kind        | Meaning |
|----------------|-------------|---------|
| `trk`          | flag        | A `track` replay is streaming. The fields below are present only when true. |
| `trk_e_raw`/`trk_n_raw`/`trk_u_raw` | raw | Last streamed local ENU position (m), **before** filtering. |
| `trk_e`/`trk_n`/`trk_u` | interpreted | Alpha-beta-**filtered** ENU position (m) actually fed to the WGS84→Az/El transform. |

---

# Mission mode (standalone firmware, `pio run -e mission`)

The mission firmware (`src/mission/mission_main.cpp`) runs a real launch with
no laptop. Rocket position arrives as binary **GroundLinkPacket** frames over
hardware UART (GPIO 44 RX, 115200 8N1) from the ground Heltec — see
`heltec_uplink/README.md` for the 28-byte wire layout. USB-CDC stays a pure
monitor port (`monitor.html` is the matching read-only viewer).

## State machine

`wait_link → pad_lock → armed → tracking ↔ signal_lost`

- **pad_lock** — incoming fixes are averaged into the pad position.
- **armed** — low-power standby: stepper de-energised, servo PWM stopped.
- **tracking** — entered automatically on launch detection (altitude above pad
  or sustained climb rate); same filter→navigation→PID pipeline as `track`.
- **signal_lost** — uplink stale > 2 s: pointing holds, resumes on next frame.

## Commands — restricted

Only `set_base` and `set_pid`, and only **before** tracking starts. Everything
else (and anything mid-flight) answers `err "mission mode: read-only"`.

## Extra `tel` fields

| Field | Meaning |
|-------|---------|
| `mode` | `"mission"` (absent/`"hil"` from the HIL harness). |
| `mstate` | State name as listed above. |
| `mot` | Motors energised (false in armed standby). |
| `rx_ok` / `rx_bad` | Valid / rejected uplink frames since boot. |
| `age_ms` | ms since the last valid frame (-1 before first). |
| `seq`, `rssi`, `snr` | From the last GroundLinkPacket (link diagnostics). |
| `rkt_lat/lon/alt`, `clb` | Last rocket fix and EMA climb rate (m/s). |
| `pad_lat/lon/alt`, `pad_n` | Locked pad position and packets averaged. |
| `base_src` | `gnss`, `manual` (set_base), or `default` (no fix yet). |

`ready` carries `mode:"mission"`; `trk`/`trk_*` fields behave as in HIL, with
the uplink stream as the source.
