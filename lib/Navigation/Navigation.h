// ============================================================================
// Navigation.h — Geographic Coordinate Transformations & Pointing Math
// Cursr-V Antenna Tracker
//
// Implements the full WGS84 → ECEF → ENU pipeline and computes
// Azimuth / Elevation pointing angles from base station to target.
// All angular I/O is in DEGREES; internal math uses radians.
// ============================================================================
#ifndef NAVIGATION_H
#define NAVIGATION_H

// --------------------------------------------------------------------------
// 3-D vector (double precision)
// --------------------------------------------------------------------------
struct Vec3d {
    double x;
    double y;
    double z;
};

// --------------------------------------------------------------------------
// Pointing result returned to the caller
// --------------------------------------------------------------------------
struct PointingAngles {
    double azimuth;    // degrees, 0 = North, clockwise
    double elevation;  // degrees, 0 = horizon, 90 = zenith
};

// --------------------------------------------------------------------------
// Public API
// --------------------------------------------------------------------------

/// Convert WGS84 geodetic coordinates to ECEF Cartesian.
/// @param latDeg   Latitude  (degrees, north positive)
/// @param lonDeg   Longitude (degrees, east  positive)
/// @param altM     Altitude above WGS84 ellipsoid (metres)
Vec3d geodeticToECEF(double latDeg, double lonDeg, double altM);

/// Transform an ECEF difference vector into local ENU coordinates
/// centred at the given reference (base station) geodetic position.
/// @param dECEF     Target ECEF minus base ECEF
/// @param refLatDeg Reference latitude  (degrees)
/// @param refLonDeg Reference longitude (degrees)
Vec3d ecefToENU(const Vec3d& dECEF, double refLatDeg, double refLonDeg);

/// Compute azimuth and elevation from base station to target.
/// @param baseLat, baseLon, baseAlt  Base station WGS84 position
/// @param tgtLat,  tgtLon,  tgtAlt   Target WGS84 position
PointingAngles computePointing(double baseLat, double baseLon, double baseAlt,
                                double tgtLat,  double tgtLon,  double tgtAlt);

/// Convert a local ENU offset (metres, relative to the base) back to WGS84
/// geodetic coordinates using the flat-earth approximation (the same
/// 111320 / 111320·cosφ convention the web panels use; <0.1 % error over a few
/// km). Lets a streamed local-position target be reconstructed as a real WGS84
/// fix and fed through computePointing().
/// @return Vec3d reused as geodetic: { x = latitude°, y = longitude°, z = altitude m }.
Vec3d enuToGeodetic(double e, double n, double u,
                    double baseLat, double baseLon, double baseAlt);

#endif // NAVIGATION_H
