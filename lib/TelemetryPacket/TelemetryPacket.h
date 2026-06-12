// ============================================================================
// TelemetryPacket.h — Binary Telemetry Protocol Definition
// Cursr-V Antenna Tracker
//
// Two wire formats share the 0xAA-sync + XOR-checksum convention:
//   - TelemetryPacket (22 B)  — flight computer → rocket-side radio (legacy,
//     layout frozen; do not change).
//   - GroundLinkPacket (28 B) — ground Heltec → tracker over hardware UART.
//     Adds a sequence number and LoRa link quality so the tracker can detect
//     packet loss and surface RSSI/SNR in its monitoring telemetry.
//     The transmitter-side copy lives in heltec_uplink/TrackerUplink.h —
//     keep the two layouts in sync.
//
// PacketStreamParser<P> turns a byte stream into validated packets of either
// type (sync hunt → fixed-length accumulate → checksum, resync on failure).
// ============================================================================
#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <cstdint>
#include <cstddef>
#include <cstring>

// --------------------------------------------------------------------------
// Packet Constants
// --------------------------------------------------------------------------
static constexpr uint8_t  SYNC_BYTE    = 0xAA;
static constexpr size_t   PACKET_SIZE  = 22;   // Total bytes on the wire

// --------------------------------------------------------------------------
// Packed Telemetry Struct — must match the Heltec transmitter layout exactly
// --------------------------------------------------------------------------
struct __attribute__((packed)) TelemetryPacket {
    uint8_t  startByte;   //  1 byte  — sync marker (must be 0xAA)
    double   lat;         //  8 bytes — WGS84 latitude  (degrees)
    double   lon;         //  8 bytes — WGS84 longitude (degrees)
    float    alt;         //  4 bytes — altitude MSL     (metres)
    uint8_t  checksum;    //  1 byte  — XOR of bytes [1..20]
};

static_assert(sizeof(TelemetryPacket) == PACKET_SIZE,
              "TelemetryPacket size mismatch — check struct packing");

// --------------------------------------------------------------------------
// Compute XOR checksum over all payload bytes (excludes startByte itself
// and the trailing checksum byte).
// --------------------------------------------------------------------------
inline uint8_t computeChecksum(const TelemetryPacket& pkt) {
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t cs = 0;
    // XOR bytes 1 through 20 (skip byte 0 = startByte, skip byte 21 = checksum)
    for (size_t i = 1; i < PACKET_SIZE - 1; ++i) {
        cs ^= raw[i];
    }
    return cs;
}

// --------------------------------------------------------------------------
// Validate a received packet: sync byte + checksum match.
// --------------------------------------------------------------------------
inline bool validatePacket(const TelemetryPacket& pkt) {
    if (pkt.startByte != SYNC_BYTE) return false;
    return (computeChecksum(pkt) == pkt.checksum);
}

// ============================================================================
// GroundLinkPacket — ground Heltec → tracker UART frame (28 bytes)
// ============================================================================
static constexpr size_t GROUND_LINK_PACKET_SIZE = 28;

struct __attribute__((packed)) GroundLinkPacket {
    uint8_t  startByte;   //  1 byte  — sync marker (must be 0xAA)
    uint16_t seq;         //  2 bytes — transmitter sequence number (wraps)
    double   lat;         //  8 bytes — WGS84 latitude  (degrees)
    double   lon;         //  8 bytes — WGS84 longitude (degrees)
    float    alt;         //  4 bytes — altitude MSL    (metres)
    int16_t  rssi_dbm;    //  2 bytes — LoRa RSSI (dBm), 0 if unknown
    int8_t   snr_db_x4;   //  1 byte  — LoRa SNR × 4 (dB), 0 if unknown
    uint8_t  flags;       //  1 byte  — reserved, send 0
    uint8_t  checksum;    //  1 byte  — XOR of bytes [1 .. size-2]
};

static_assert(sizeof(GroundLinkPacket) == GROUND_LINK_PACKET_SIZE,
              "GroundLinkPacket size mismatch — check struct packing");

// XOR over all payload bytes (skips startByte and the checksum byte itself).
// Works for any packed packet whose first byte is the sync and last byte is
// the checksum.
template <typename P>
inline uint8_t computeXorChecksum(const P& pkt) {
    const uint8_t* raw = reinterpret_cast<const uint8_t*>(&pkt);
    uint8_t cs = 0;
    for (size_t i = 1; i < sizeof(P) - 1; ++i) {
        cs ^= raw[i];
    }
    return cs;
}

template <typename P>
inline bool validateXorPacket(const P& pkt) {
    if (pkt.startByte != SYNC_BYTE) return false;
    return (computeXorChecksum(pkt) == pkt.checksum);
}

// ============================================================================
// PacketStreamParser — incremental framer for a fixed-size sync+checksum
// packet arriving as a byte stream. Feed one byte at a time; when a full
// valid frame has accumulated, it is copied to `out` and feed() returns true.
// Garbage and corrupt frames are skipped by re-scanning the buffer for the
// next sync byte, so the parser self-recovers from any misalignment.
// ============================================================================
template <typename P>
class PacketStreamParser {
public:
    bool feed(uint8_t b, P& out) {
        if (_len == 0 && b != SYNC_BYTE) return false;  // hunt for sync
        _buf[_len++] = b;
        if (_len < sizeof(P)) return false;

        P pkt;
        memcpy(&pkt, _buf, sizeof(P));
        if (validateXorPacket(pkt)) {
            _len = 0;
            out = pkt;
            ++okCount;
            return true;
        }
        ++badCount;
        resync();
        return false;
    }

    uint32_t okCount  = 0;   // frames accepted
    uint32_t badCount = 0;   // checksum/sync failures

private:
    // Drop the leading byte(s) of a bad frame and keep any bytes from the
    // next sync onward, so an interleaved valid frame is not lost.
    void resync() {
        size_t next = 1;
        while (next < _len && _buf[next] != SYNC_BYTE) ++next;
        _len -= next;
        memmove(_buf, _buf + next, _len);
    }

    uint8_t _buf[sizeof(P)];
    size_t  _len = 0;
};

#endif // TELEMETRY_PACKET_H
