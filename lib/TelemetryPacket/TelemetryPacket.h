// ============================================================================
// TelemetryPacket.h — Binary Telemetry Protocol Definition
// Cursr-V Antenna Tracker
//
// Defines the 22-byte packed struct received from the Heltec LoRa gateway
// over hardware UART. Includes XOR checksum for data integrity validation.
// ============================================================================
#ifndef TELEMETRY_PACKET_H
#define TELEMETRY_PACKET_H

#include <cstdint>
#include <cstddef>

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

#endif // TELEMETRY_PACKET_H
