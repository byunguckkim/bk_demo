#pragma once
/**
 * @brief Map Coordinate Conversion Utilities
 *
 */
namespace applied {

// Convert WGS84 Lat Long to UTM
// Inputs:
//   lat: Latitude of the converted point.
//   lng: Longitude of the converted point.
// Returns:
//   Nothing. Results are stored in the zone, north, x, y, and meridian_convergence variables.
// Outputs:
//   int zone: Resulting UTM zone of the converted point.
//   bool north: Resulting UTM north bool of the converted point.
//   double x: Resulting x coordinate of the converted point to convert to lat/long.
//   double y: Resulting y coordinate of the converted point to convert to lat/long.
//   meridian_convergence: Double to be added to the heading / direction of the point.
void ConvertWGS84LatLngToUTM(double lat, double lng, int* zone, bool* north, double* x, double* y,
                             double* meridian_convergence);

// Convert UTM to WGS84 Lat Long
// Inputs:
//   int zone: UTM zone that the scenario was run in. You can extract the UTM zone from the
//     scenario by querying the startup_options_ field like given in the below example:
//     startup_options_.map_config().transforms().utm().zone()
//   bool north: You can extract the UTM north bool from the scenario by querying the
//     startup_options_ field like given in the below example:
//     startup_options_.map_config().transforms().utm().north()
//   double x: The x coordinate of the point to convert to lat/long.
//   double y: The y coordinate of the point to convert to lat/long.
// Returns:
//   Nothing. Results are stored in the lat, lng, and meridian_convergence variables.
// Outputs:
//   lat: Resulting latitude of the converted point.
//   lng: Resulting longitude of the converted point.
//   meridian_convergence: Double to be added to the heading / direction of the point.
void ConvertUTMToWGS84LatLng(int zone, bool north, double x, double y, double* lat, double* lng,
                             double* meridian_convergence);

// Convert WGS84 Lat Long to ECEF
// Inputs:
//   lat: Latitude of the point to convert.
//   lng: Longitude of the point to convert.
//   h: Altitude of the point to convert.
// Returns:
//   Nothing. Results are stored in the x, y, and z variables.
// Outputs:
//   double x: Resulting x coordinate of the converted point in ECEF frame.
//   double y: Resulting y coordinate of the converted point in ECEF frame.
//   double z: Resulting z coordinate of the converted point in ECEF frame.
void ConvertWGS84LatLngToECEF(double lat, double lng, double h, double* x, double* y, double* z);

// Convert ECEF to WGS84 Lat Long
// Inputs:
//   double x: The x coordinate of the point to convert in ECEF frame.
//   double y: The y coordinate of the point to convert in ECEF frame.
//   double z: The z coordinate of the point to convert in ECEF frame.
// Returns:
//   Nothing. Results are stored in the lat, lng, and h variables.
// Outputs:
//   lat: Resulting latitude of the converted point.
//   lng: Resulting longitude of the converted point.
//   h: Resulting altitude of the converted point.
void ConvertECEFToWGS84LatLng(double x, double y, double z, double* lat, double* lng, double* h);

// Convert WGS84 Lat Long to Local Cartesian
// Inputs:
//   lat0: Latitude of the origin point of the local cartesian frame.
//   lng0: Longitude of the origin point of the local cartesian frame.
//   lat: Latitude of the point to convert.
//   lng: Longitude of the point to convert.
//   h: Altitude of the point to convert.
// Returns:
//   Nothing. Results are stored in the x, y, and z variables.
// Outputs:
//   double x: Resulting x coordinate of the point in the local cartesian frame.
//   double y: Resulting y coordinate of the point in the local cartesian frame.
//   double z: Resulting z coordinate of the point in the local cartesian frame.
void ConvertWGS84LatLngToLocalCartesian(double lat0, double lng0, double lat, double lng, double h,
                                        double* x, double* y, double* z);

// Convert Local Cartesian to WGS84 Lat Long
// Inputs:
//   lat0: Latitude of the origin point of the local cartesian frame.
//   lng0: Longitude of the origin point of the local cartesian frame.
//   double x: The x coordinate of the point to convert in the local cartesian frame.
//   double y: The y coordinate of the point to convert in the local cartesian frame.
//   double z: The z coordinate of the point to convert in the local cartesian frame.
// Returns:
//   Nothing. Results are stored in the lat, lng, and h variables.
// Outputs:
//   lat: Resulting latitude of the converted point.
//   lng: Resulting longitude of the converted point.
//   h: Resulting altitude of the converted point.
void ConvertLocalCartesianToWGS84LatLng(double lat0, double lng0, double x, double y, double z,
                                        double* lat, double* lng, double* h);

/*
 * DEPRECATED
 * The following functions are all deprecated and support for them will be removed in a future
 * release. We recommend updating your interface code to use the newly named functions instead.
 */

// Renamed to ConvertWGS84LatLngToUTM.
inline void convert_latlng_to_utm(double lat, double lng, int* zone, bool* north, double* x,
                                  double* y, double* meridian_convergence) {
  ConvertWGS84LatLngToUTM(lat, lng, zone, north, x, y, meridian_convergence);
}

// Renamed to ConvertUTMToWGS84LatLng.
inline void convert_utm_to_latlng(int zone, bool north, double x, double y, double* lat,
                                  double* lng, double* meridian_convergence) {
  ConvertUTMToWGS84LatLng(zone, north, x, y, lat, lng, meridian_convergence);
}

// Renamed to ConvertWGS84LatLngToECEF.
inline void convert_latlng_to_ecef(double lat, double lng, double h, double* x, double* y,
                                   double* z) {
  ConvertWGS84LatLngToECEF(lat, lng, h, x, y, z);
}

// Renamed to ConvertWGS84LatLngToECEF.
inline void convert_ecef_to_latlng(double x, double y, double z, double* lat, double* lng,
                                   double* h) {
  ConvertECEFToWGS84LatLng(x, y, z, lat, lng, h);
}

// Renamed to ConvertWGS84LatLngToLocalCartesian.
inline void convert_latlng_to_localcartesian(double lat0, double lng0, double lat, double lng,
                                             double h, double* x, double* y, double* z) {
  ConvertWGS84LatLngToLocalCartesian(lat0, lng0, lat, lng, h, x, y, z);
}

// Renamed to ConvertLocalCartesianToWGS84LatLng.
inline void convert_localcartesian_to_latlng(double lat0, double lng0, double x, double y, double z,
                                             double* lat, double* lng, double* h) {
  ConvertLocalCartesianToWGS84LatLng(lat0, lng0, x, y, z, lat, lng, h);
}

}  // namespace applied
