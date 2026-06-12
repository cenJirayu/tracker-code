// ============================================================================
// example_ground.ino — minimal TrackerUplink integration example
//
// Shows where to call uplink.send() in a typical ground-station sketch.
// Replace the FAKE_FIX block with your real LoRa receive handler — wherever
// your existing code decodes a rocket position, call uplink.send() there.
//
// Wiring: UPLINK_TX_PIN ──► tracker GPIO 44, GND ──► GND, 115200 baud.
// ============================================================================
#include "TrackerUplink.h"

// Pick any free GPIO on your Heltec V4 (avoid the LoRa SPI pins 8-14 and the
// OLED pins 17/18/21).
static const int UPLINK_TX_PIN = 19;

TrackerUplink uplink(Serial1);

void setup() {
    Serial.begin(115200);
    uplink.begin(UPLINK_TX_PIN);
}

void loop() {
    // ---- FAKE_FIX: stand-in for your LoRa receive handler ------------------
    // In your real firmware, delete this block and instead call uplink.send()
    // from the spot where a LoRa packet has just been decoded, e.g.:
    //
    //   void onLoRaPacket(double lat, double lon, float alt) {
    //       uplink.send(lat, lon, alt, radio.getRSSI(), radio.getSNR());
    //   }
    //
    static unsigned long lastMs = 0;
    if (millis() - lastMs >= 100) {            // 10 Hz, matches LINK_RATE_HZ
        lastMs = millis();
        uplink.send(13.7563, 100.5018, 892.0f, /*rssi*/ -72, /*snr*/ 9.5f);
    }
    // ------------------------------------------------------------------------
}
