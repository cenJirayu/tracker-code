// ============================================================================
// GNSSManager.h — u-blox SAM-M10Q GNSS Abstraction
// Cursr-V Antenna Tracker
//
// Wraps the SparkFun u-blox GNSS v3 library.  Call begin() once, then
// update() every loop() iteration.  All position getters are valid only
// after isValid() returns true.
//
// Position I/O: latitude / longitude in decimal degrees (double),
//               altitude in metres MSL (float).
// ============================================================================
#ifndef GNSS_MANAGER_H
#define GNSS_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_u-blox_GNSS_v3.h>

// --------------------------------------------------------------------------
//  GNSS fix snapshot — populated on every successful PVT poll
// --------------------------------------------------------------------------
struct GNSSFix {
    double   lat;           // decimal degrees, north-positive
    double   lon;           // decimal degrees, east-positive
    float    altMSL;        // metres above mean sea level
    float    horizAccM;     // 1-σ horizontal accuracy estimate (metres)
    uint8_t  fixType;       // 0=none,2=2D,3=3D,4=GNSS+DR
    uint8_t  satsUsed;
    bool     valid;         // fixType ≥ 2 AND gnssFixOK bit set
};

// --------------------------------------------------------------------------
//  GNSSManager
// --------------------------------------------------------------------------
class GNSSManager {
public:
    GNSSManager();

    /// Initialise I2C and the SAM-M10Q.
    /// @param wire   TwoWire bus (default Wire; SDA=I2C_SDA_PIN, SCL=I2C_SCL_PIN)
    /// @param rateHz Navigation update rate 1–10 Hz (default 1)
    /// @return true on successful communication with the module
    bool begin(TwoWire& wire = Wire, uint8_t rateHz = 1);

    /// Non-blocking poll — call every loop() iteration.
    /// @return true when a fresh PVT message has been parsed
    bool update();

    // ---- Accessors (snapshot from last successful update) ----
    bool     isValid()        const { return _fix.valid; }
    uint8_t  getFixType()     const { return _fix.fixType; }
    uint8_t  getSatsUsed()    const { return _fix.satsUsed; }
    double   getLat()         const { return _fix.lat; }
    double   getLon()         const { return _fix.lon; }
    float    getAlt()         const { return _fix.altMSL; }
    float    getHorizAccM()   const { return _fix.horizAccM; }

    /// Fill lat/lon/alt in one call; returns isValid().
    bool getLocation(double& lat, double& lon, float& alt) const;

    /// Milliseconds since the last valid fix was received (UINT32_MAX if never).
    uint32_t msSinceLastFix() const;

    /// Direct access to the underlying SparkFun driver for advanced use.
    SFE_UBLOX_GNSS& driver() { return _gnss; }

private:
    SFE_UBLOX_GNSS _gnss;
    GNSSFix        _fix;
    uint32_t       _lastFixMs;   // millis() at last valid fix
    bool           _everHadFix;
};

#endif // GNSS_MANAGER_H
