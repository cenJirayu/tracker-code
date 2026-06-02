// ============================================================================
// MagManager.cpp
// ============================================================================
#include "MagManager.h"
#include "config.h"
#include <cmath>

// 18-bit midpoint (2^17)
static constexpr float MAG_MID = 131072.0f;

MagManager::MagManager() : _headingDeg(0.0f), _ready(false) {}

bool MagManager::begin(TwoWire& wire) {
    wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);  // idempotent on ESP32

    if (!_mag.begin(wire)) return false;

    _mag.softReset();
    delay(10);
    _mag.enableAutomaticSetReset();
    _mag.setFilterBandwidth(100);       // 100 Hz BW for continuous mode
    _mag.setContinuousModeFrequency(10); // 10 Hz autonomous updates
    _mag.enableContinuousMode();

    _ready = true;
    return true;
}

bool MagManager::update() {
    if (!_ready) return false;

    uint32_t rawX, rawY, rawZ;
    if (!_mag.readFieldsXYZ(&rawX, &rawY, &rawZ)) return false;

    // Centre on zero. Divide by mid so values are in [-1, +1] (units cancel).
    float cx = ((float)rawX - MAG_MID) / MAG_MID;
    float cy = ((float)rawY - MAG_MID) / MAG_MID;

    // Heading: magnetic north aligned with sensor +Y axis.
    // atan2(X, -Y) → clockwise angle from North in [-180, 180].
    float hdg = atan2f(cx, -cy) * (180.0f / (float)M_PI);

    // Apply local magnetic declination and normalise to [0, 360).
    hdg += (float)MAGNETIC_DECLINATION_DEG;
    if (hdg < 0.0f)    hdg += 360.0f;
    if (hdg >= 360.0f) hdg -= 360.0f;

    _headingDeg = hdg;
    return true;
}
