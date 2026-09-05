// Latitude/longitude to UTM.
//
// Equations from USGS Bulletin 1532, after Chuck Gantz's public implementation
// as carried by ros gps_common. Modified BSD.
#pragma once

#include <cmath>
#include <cstdio>

namespace gnocchi::utm
{

constexpr double kA = 6378137.0;            // WGS84 major axis
constexpr double kE2 = 0.00669437999014;    // first eccentricity squared
constexpr double kK0 = 0.9996;              // UTM scale factor

inline char letterDesignator(double lat)
{
    static const char* bands = "CDEFGHJKLMNPQRSTUVWX";
    if (lat < -80.0 || lat > 84.0) return 'Z';
    int index = static_cast<int>((lat + 80.0) / 8.0);
    if (index > 19) index = 19;
    return bands[index];
}

inline void toUtm(double lat, double lon, double& northing, double& easting, char* zone)
{
    constexpr double kDeg = M_PI / 180.0;
    const double lon_wrapped = lon - std::floor((lon + 180.0) / 360.0) * 360.0;
    const double lat_rad = lat * kDeg;
    const double lon_rad = lon_wrapped * kDeg;

    int zone_number = static_cast<int>((lon_wrapped + 180.0) / 6.0) + 1;
    // Norway and Svalbard widen their zones.
    if (lat >= 56.0 && lat < 64.0 && lon_wrapped >= 3.0 && lon_wrapped < 12.0) zone_number = 32;
    if (lat >= 72.0 && lat < 84.0)
    {
        if (lon_wrapped >= 0.0 && lon_wrapped < 9.0) zone_number = 31;
        else if (lon_wrapped < 21.0) zone_number = 33;
        else if (lon_wrapped < 33.0) zone_number = 35;
        else if (lon_wrapped < 42.0) zone_number = 37;
    }
    const double origin = ((zone_number - 1) * 6 - 180 + 3) * kDeg;
    if (zone) std::snprintf(zone, 8, "%d%c", zone_number, letterDesignator(lat));

    const double ep2 = kE2 / (1.0 - kE2);
    const double N = kA / std::sqrt(1.0 - kE2 * std::sin(lat_rad) * std::sin(lat_rad));
    const double T = std::tan(lat_rad) * std::tan(lat_rad);
    const double C = ep2 * std::cos(lat_rad) * std::cos(lat_rad);
    const double A = std::cos(lat_rad) * (lon_rad - origin);
    const double e4 = kE2 * kE2;
    const double e6 = e4 * kE2;
    const double M =
        kA * ((1.0 - kE2 / 4.0 - 3.0 * e4 / 64.0 - 5.0 * e6 / 256.0) * lat_rad -
              (3.0 * kE2 / 8.0 + 3.0 * e4 / 32.0 + 45.0 * e6 / 1024.0) * std::sin(2.0 * lat_rad) +
              (15.0 * e4 / 256.0 + 45.0 * e6 / 1024.0) * std::sin(4.0 * lat_rad) -
              (35.0 * e6 / 3072.0) * std::sin(6.0 * lat_rad));

    easting = kK0 * N *
                  (A + (1.0 - T + C) * A * A * A / 6.0 +
                   (5.0 - 18.0 * T + T * T + 72.0 * C - 58.0 * ep2) * std::pow(A, 5) / 120.0) +
              500000.0;
    northing = kK0 * (M + N * std::tan(lat_rad) *
                              (A * A / 2.0 + (5.0 - T + 9.0 * C + 4.0 * C * C) * std::pow(A, 4) / 24.0 +
                               (61.0 - 58.0 * T + T * T + 600.0 * C - 330.0 * ep2) * std::pow(A, 6) / 720.0));
    if (lat < 0.0) northing += 10000000.0;
}

}  // namespace gnocchi::utm
