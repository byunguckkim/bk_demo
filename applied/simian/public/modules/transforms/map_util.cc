#include "applied/simian/public/modules/transforms/map_util.h"

#include <iostream>

#include "GeographicLib/Geocentric.hpp"
#include "GeographicLib/LocalCartesian.hpp"
#include "GeographicLib/UTMUPS.hpp"

namespace applied {
namespace {
constexpr bool kUseMgrsLimits = false;
}  // namespace

void ConvertWGS84LatLngToUTM(double lat, double lng, int* zone, bool* north, double* x, double* y,
                             double* meridian_convergence) {
  double unused_scale;
  GeographicLib::UTMUPS::Forward(lat, lng, *zone, *north, *x, *y, *meridian_convergence,
                                 unused_scale);
}

void ConvertUTMToWGS84LatLng(int zone, bool north, double x, double y, double* lat, double* lng,
                             double* meridian_convergence) {
  double unused_scale;
  GeographicLib::UTMUPS::Reverse(zone, north, x, y, *lat, *lng, *meridian_convergence, unused_scale,
                                 kUseMgrsLimits);
}

void ConvertWGS84LatLngToECEF(double lat, double lng, double h, double* x, double* y, double* z) {
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
  earth.Forward(lat, lng, h, *x, *y, *z);
}

void ConvertECEFToWGS84LatLng(double x, double y, double z, double* lat, double* lng, double* h) {
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
  earth.Reverse(x, y, z, *lat, *lng, *h);
}

void ConvertWGS84LatLngToLocalCartesian(double lat0, double lng0, double lat, double lng, double h,
                                        double* x, double* y, double* z) {
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
  GeographicLib::LocalCartesian proj(lat0, lng0, 0, earth);
  proj.Forward(lat, lng, h, *x, *y, *z);
}

void ConvertLocalCartesianToWGS84LatLng(double lat0, double lng0, double x, double y, double z,
                                        double* lat, double* lng, double* h) {
  GeographicLib::Geocentric earth(GeographicLib::Constants::WGS84_a(),
                                  GeographicLib::Constants::WGS84_f());
  GeographicLib::LocalCartesian proj(lat0, lng0, 0, earth);
  proj.Reverse(x, y, z, *lat, *lng, *h);
}

}  // namespace applied
