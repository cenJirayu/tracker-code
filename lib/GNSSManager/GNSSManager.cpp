// ============================================================================
// GNSSManager.cpp
// ============================================================================
#include "GNSSManager.h"
#include "config.h"

static constexpr uint8_t UBLOX_I2C_ADDR = 0x42;

// Minimum fix type accepted as "valid" (2 = 2-D, 3 = 3-D, 4 = GNSS+DR).
static constexpr uint8_t MIN_FIX_TYPE = 2;

GNSSManager::GNSSManager()
    : _fix{}, _lastFixMs(0), _everHadFix(false)
{
    _fix.valid = false;
}

bool GNSSManager::begin(TwoWire& wire, uint8_t rateHz)
{
    wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

    if (!_gnss.begin(wire, UBLOX_I2C_ADDR)) {
        return false;
    }

    // Suppress all NMEA output on I2C; we use UBX-only polling.
    _gnss.setI2COutput(COM_TYPE_UBX);

    // Disable periodic auto-messages we don't need — keeps I2C buffer lean.
    _gnss.setAutoPVT(false);

    // Set navigation update rate.
    uint16_t periodMs = (rateHz > 0 && rateHz <= 10) ? (1000u / rateHz) : 1000u;
    _gnss.setMeasurementRate(periodMs);
    _gnss.setNavigationRate(1);  // one solution per measurement

    // SAM-M10Q default dynamic model is portable (0); leave it unless the
    // caller overrides via driver().setDynamicModel() after begin().

    _gnss.saveConfigSelective(VAL_CFG_SUBSEC_IOPORT);

    return true;
}

bool GNSSManager::update()
{
    // checkUblox() drains the I2C FIFO; getPVT() returns true only when a
    // fresh UBX-NAV-PVT message has been fully parsed.
    _gnss.checkUblox();

    if (!_gnss.getPVT()) {
        return false;
    }

    _fix.fixType   = _gnss.getFixType();
    _fix.satsUsed  = _gnss.getSIV();
    _fix.valid     = (_fix.fixType >= MIN_FIX_TYPE) && _gnss.getGnssFixOk();

    // Always update position fields so the caller can inspect raw data even
    // during a degraded fix; isValid() guards production use.
    _fix.lat       = _gnss.getLatitude()  * 1e-7;   // degrees
    _fix.lon       = _gnss.getLongitude() * 1e-7;   // degrees
    _fix.altMSL    = _gnss.getAltitudeMSL() * 1e-3f; // mm → m
    _fix.horizAccM = _gnss.getHorizontalAccEst() * 1e-3f; // mm → m

    if (_fix.valid) {
        _lastFixMs   = millis();
        _everHadFix  = true;
    }

    return true;
}

bool GNSSManager::getLocation(double& lat, double& lon, float& alt) const
{
    lat = _fix.lat;
    lon = _fix.lon;
    alt = _fix.altMSL;
    return _fix.valid;
}

uint32_t GNSSManager::msSinceLastFix() const
{
    if (!_everHadFix) {
        return UINT32_MAX;
    }
    return millis() - _lastFixMs;
}
