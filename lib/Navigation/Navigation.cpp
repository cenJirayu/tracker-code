// ============================================================================
// Navigation.cpp — Geographic Coordinate Transformations
// Cursr-V Antenna Tracker
// ============================================================================
#include "Navigation.h"
#include <cmath>

// --------------------------------------------------------------------------
// WGS84 Ellipsoid Constants
// --------------------------------------------------------------------------
static constexpr double WGS84_A  = 6378137.0;                  // semi-major axis (m)
static constexpr double WGS84_F  = 1.0 / 298.257223563;       // flattening
static constexpr double WGS84_B  = WGS84_A * (1.0 - WGS84_F); // semi-minor axis
static constexpr double WGS84_E2 = 1.0 - (WGS84_B * WGS84_B) / (WGS84_A * WGS84_A);  // first eccentricity squared

static constexpr double DEG2RAD = M_PI / 180.0;
static constexpr double RAD2DEG = 180.0 / M_PI;

// --------------------------------------------------------------------------
// Prime vertical radius of curvature
// --------------------------------------------------------------------------
static double primeVerticalRadius(double sinLat) {
    return WGS84_A / sqrt(1.0 - WGS84_E2 * sinLat * sinLat);
}

// --------------------------------------------------------------------------
// WGS84 Geodetic → ECEF
// --------------------------------------------------------------------------
Vec3d geodeticToECEF(double latDeg, double lonDeg, double altM) {
    double lat = latDeg * DEG2RAD;
    double lon = lonDeg * DEG2RAD;

    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double sinLon = sin(lon);
    double cosLon = cos(lon);

    double N = primeVerticalRadius(sinLat);

    Vec3d ecef;
    ecef.x = (N + altM) * cosLat * cosLon;
    ecef.y = (N + altM) * cosLat * sinLon;
    ecef.z = (N * (1.0 - WGS84_E2) + altM) * sinLat;
    return ecef;
}

// --------------------------------------------------------------------------
// ECEF difference vector → local ENU
//
// Rotation matrix from ECEF to ENU at reference point (φ, λ):
//   | -sinλ         cosλ          0     |
//   | -sinφ·cosλ   -sinφ·sinλ    cosφ   |
//   |  cosφ·cosλ    cosφ·sinλ    sinφ   |
// --------------------------------------------------------------------------
Vec3d ecefToENU(const Vec3d& dECEF, double refLatDeg, double refLonDeg) {
    double lat = refLatDeg * DEG2RAD;
    double lon = refLonDeg * DEG2RAD;

    double sinLat = sin(lat);
    double cosLat = cos(lat);
    double sinLon = sin(lon);
    double cosLon = cos(lon);

    Vec3d enu;
    enu.x = -sinLon * dECEF.x + cosLon * dECEF.y;                                    // East
    enu.y = -sinLat * cosLon * dECEF.x - sinLat * sinLon * dECEF.y + cosLat * dECEF.z; // North
    enu.z =  cosLat * cosLon * dECEF.x + cosLat * sinLon * dECEF.y + sinLat * dECEF.z; // Up
    return enu;
}

// --------------------------------------------------------------------------
// Compute pointing angles from base to target
// --------------------------------------------------------------------------
PointingAngles computePointing(double baseLat, double baseLon, double baseAlt,
                                double tgtLat,  double tgtLon,  double tgtAlt) {
    Vec3d baseECEF = geodeticToECEF(baseLat, baseLon, baseAlt);
    Vec3d tgtECEF  = geodeticToECEF(tgtLat,  tgtLon,  tgtAlt);

    Vec3d dECEF;
    dECEF.x = tgtECEF.x - baseECEF.x;
    dECEF.y = tgtECEF.y - baseECEF.y;
    dECEF.z = tgtECEF.z - baseECEF.z;

    Vec3d enu = ecefToENU(dECEF, baseLat, baseLon);

    PointingAngles pa;

    // Azimuth: measured clockwise from North (atan2 of East/North)
    pa.azimuth = atan2(enu.x, enu.y) * RAD2DEG;
    if (pa.azimuth < 0.0) pa.azimuth += 360.0;

    // Elevation: angle above the horizon
    double horizDist = sqrt(enu.x * enu.x + enu.y * enu.y);
    pa.elevation = atan2(enu.z, horizDist) * RAD2DEG;

    return pa;
}

// --------------------------------------------------------------------------
// Local ENU offset (metres) → WGS84 geodetic (flat-earth approximation)
//
// Inverse-ish of the East/North columns of ecefToENU, accurate to <0.1 % over
// a few km. Vec3d is reused as a geodetic carrier: x=lat°, y=lon°, z=alt m.
// --------------------------------------------------------------------------
Vec3d enuToGeodetic(double e, double n, double u,
                    double baseLat, double baseLon, double baseAlt) {
    static constexpr double M_PER_DEG = 111320.0;  // metres per degree of latitude

    Vec3d geo;
    geo.x = baseLat + n / M_PER_DEG;                              // latitude  (deg)
    geo.y = baseLon + e / (M_PER_DEG * cos(baseLat * DEG2RAD));   // longitude (deg)
    geo.z = baseAlt + u;                                          // altitude  (m)
    return geo;
}
