// ============================================================================
// TrackerUplink.h — Ground Heltec → Cursr-V Tracker UART transmitter module
//
// Drop this single file into your ground-station Heltec firmware (Arduino or
// PlatformIO). It frames rocket position fixes as 28-byte GroundLinkPacket
// frames and sends them over a hardware UART to the tracker.
//
//   Wiring:  Heltec TX pin ──► tracker GPIO 44 (UART_RX_PIN),  GND ──► GND
//   Link:    115200 baud, 8N1, one-way (the tracker never talks back)
//
//   Usage:
//     #include "TrackerUplink.h"
//     TrackerUplink uplink(Serial1);
//     void setup() { uplink.begin(/*txPin=*/19); }
//     // from your LoRa receive handler:
//     uplink.send(lat, lon, altM, rssiDbm, snrDb);
//
// The packet layout is mirrored in lib/TelemetryPacket/TelemetryPacket.h of
// the tracker project — KEEP THE TWO IN SYNC.
// ============================================================================
#ifndef TRACKER_UPLINK_H
#define TRACKER_UPLINK_H

#include <Arduino.h>

// --------------------------------------------------------------------------
// GroundLinkPacket — 28 bytes on the wire (must match the tracker's copy)
// --------------------------------------------------------------------------
struct __attribute__((packed)) GroundLinkPacket {
    uint8_t  startByte;   //  1 byte  — sync marker, always 0xAA
    uint16_t seq;         //  2 bytes — sequence number (wraps; tracker detects loss)
    double   lat;         //  8 bytes — WGS84 latitude  (degrees)
    double   lon;         //  8 bytes — WGS84 longitude (degrees)
    float    alt;         //  4 bytes — altitude MSL    (metres)
    int16_t  rssi_dbm;    //  2 bytes — LoRa RSSI (dBm), 0 if unknown
    int8_t   snr_db_x4;   //  1 byte  — LoRa SNR × 4 (dB), 0 if unknown
    uint8_t  flags;       //  1 byte  — reserved, 0
    uint8_t  checksum;    //  1 byte  — XOR of bytes [1..26]
};

static_assert(sizeof(GroundLinkPacket) == 28, "GroundLinkPacket must be 28 bytes");

// --------------------------------------------------------------------------
// TrackerUplink
// --------------------------------------------------------------------------
class TrackerUplink {
public:
    explicit TrackerUplink(HardwareSerial& port) : _port(port) {}

    /// Open the UART. Only TX is used; pass the GPIO wired to the tracker's
    /// pin 44. rxPin = -1 leaves the receive side unassigned.
    void begin(int txPin, long baud = 115200, int rxPin = -1) {
        _port.begin(baud, SERIAL_8N1, rxPin, txPin);
    }

    /// Frame and send one position fix. Call this from wherever your LoRa
    /// receive handler delivers a decoded rocket position. RSSI/SNR are
    /// optional link diagnostics shown on the tracker's monitor.
    void send(double lat, double lon, float altM,
              int16_t rssiDbm = 0, float snrDb = 0.0f) {
        GroundLinkPacket pkt;
        pkt.startByte = 0xAA;
        pkt.seq       = _seq++;
        pkt.lat       = lat;
        pkt.lon       = lon;
        pkt.alt       = altM;
        pkt.rssi_dbm  = rssiDbm;
        pkt.snr_db_x4 = (int8_t)constrain((int)lroundf(snrDb * 4.0f), -128, 127);
        pkt.flags     = 0;
        pkt.checksum  = _xor(pkt);
        _port.write(reinterpret_cast<const uint8_t*>(&pkt), sizeof(pkt));
    }

    /// Packets sent since boot.
    uint32_t sentCount() const { return _seq; }

private:
    static uint8_t _xor(const GroundLinkPacket& pkt) {
        const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
        uint8_t cs = 0;
        for (size_t i = 1; i < sizeof(GroundLinkPacket) - 1; ++i) cs ^= raw[i];
        return cs;
    }

    HardwareSerial& _port;
    uint16_t _seq = 0;
};

#endif // TRACKER_UPLINK_H
