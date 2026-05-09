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

/// Compute magnetic heading from raw magnetometer X/Y readings,
/// applying magnetic declination to return True North heading.
/// @param magX         Calibrated magnetometer X axis (µT or Gauss — units cancel)
/// @param magY         Calibrated magnetometer Y axis
/// @param declinationDeg  Local magnetic declination (east-positive, degrees)
/// @return Heading in degrees [0, 360) from True North, clockwise
double computeTrueHeading(double magX, double magY, double declinationDeg);

#endif // NAVIGATION_H
