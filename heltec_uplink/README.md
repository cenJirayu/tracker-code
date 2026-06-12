# TrackerUplink — Ground Heltec → Tracker UART module

Single-header transmitter module for the ground-station Heltec V4. Your
existing firmware keeps doing the LoRa work; this module only handles the
last hop: framing each decoded rocket fix and sending it over UART to the
Cursr-V tracker's mission firmware.

```
rocket ─(LoRa, your code)─► ground Heltec ─(this module, UART)─► tracker GPIO 44
```

## Wiring

| Heltec V4            | Tracker (ESP32-S3) |
|----------------------|--------------------|
| TX pin (your choice, e.g. GPIO 19) | GPIO 44 (`UART_RX_PIN`) |
| GND                  | GND                |

115200 baud, 8N1, one-way. Avoid Heltec pins 8–14 (LoRa SPI) and 17/18/21 (OLED).

## Usage

Copy `TrackerUplink.h` into your sketch/project:

```cpp
#include "TrackerUplink.h"

TrackerUplink uplink(Serial1);

void setup() {
    uplink.begin(/*txPin=*/19);          // 115200 8N1
}

// call from your LoRa receive handler:
void onLoRaPacket(double lat, double lon, float altM) {
    uplink.send(lat, lon, altM, radio.getRSSI(), radio.getSNR());
}
```

`rssi`/`snr` are optional (default 0) — they only feed the tracker's link-quality
display. Send at whatever rate fixes arrive; the tracker expects ~10 Hz
(`LINK_RATE_HZ` in the tracker's `include/config.h` — keep them matched).

## Wire format — GroundLinkPacket (28 bytes, little-endian)

| Offset | Type    | Field      | Notes                          |
|--------|---------|------------|--------------------------------|
| 0      | uint8   | startByte  | always `0xAA`                  |
| 1      | uint16  | seq        | wraps; tracker detects loss    |
| 3      | double  | lat        | WGS84 degrees                  |
| 11     | double  | lon        | WGS84 degrees                  |
| 19     | float   | alt        | metres MSL                     |
| 23     | int16   | rssi_dbm   | 0 if unknown                   |
| 25     | int8    | snr_db_x4  | SNR × 4 dB, 0 if unknown       |
| 26     | uint8   | flags      | reserved, 0                    |
| 27     | uint8   | checksum   | XOR of bytes 1..26             |

The authoritative tracker-side copy lives in
`lib/TelemetryPacket/TelemetryPacket.h` — **keep the two definitions in sync**.

## Testing without the rocket

`example_ground.ino` sends a fixed Bangkok coordinate at 10 Hz — flash it to
the Heltec, wire TX→44, and the tracker's `monitor.html` should show packets
arriving (`rx_ok` counting up) and the state advancing to `pad_lock`/`armed`.
Alternatively skip the Heltec entirely with `tools/uart_replay.py`, which
replays a recorded flight through a USB-UART adapter.
