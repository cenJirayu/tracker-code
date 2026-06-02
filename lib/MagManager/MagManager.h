// ============================================================================
// MagManager.h — MMC5983MA Magnetometer Abstraction
// Cursr-V Antenna Tracker
//
// Wraps the SparkFun MMC5983MA library. Call begin() once, then update()
// at the desired rate (telemetry rate is fine). Continuous mode is used so
// update() is a fast non-blocking register read.
//
// Output: True North heading in degrees [0, 360) via getHeadingDeg().
// ============================================================================
#ifndef MAG_MANAGER_H
#define MAG_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_MMC5983MA_Arduino_Library.h>

class MagManager {
public:
    MagManager();

    /// Initialise I2C and the MMC5983MA in 10 Hz continuous mode.
    /// @return true on successful communication with the module
    bool begin(TwoWire& wire = Wire);

    /// Read the latest measurement and recompute heading.
    /// Non-blocking in continuous mode.
    /// @return true when a fresh reading was obtained
    bool update();

    bool  isReady()      const { return _ready; }

    /// True North heading in degrees [0, 360), applying MAGNETIC_DECLINATION_DEG.
    float getHeadingDeg() const { return _headingDeg; }

private:
    SFE_MMC5983MA _mag;
    float         _headingDeg;
    bool          _ready;
};

#endif // MAG_MANAGER_H
