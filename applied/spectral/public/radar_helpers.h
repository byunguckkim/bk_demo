// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <functional>
#include <string>

#include <nlohmann/json_fwd.hpp>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/spectral/public/radar_point.h"

namespace applied {

/**
 * @brief Calculates the number of RadarPoint instances inside a given RadarCloud.
 *
 * @param radar_track The RadarTrack to count points of
 *
 * @return An std::pair where .first is the total point count and .second is a status
 * string (empty on success, non-empty on failure).
 */
std::pair<int, std::string> GetNumRadarPoints(
    const simian_public::sensor_model::SensorOutput::RadarTrack& radar_track);

/**
 * @brief Invokes a given function on each RadarPoint in the given RadarTrack.
 *
 * @param radar_track The RadarTrack to iterate through
 * @param func A function to invoke on a single RadarPoint instance. Returns std::string.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string ForEachRadarPoint(
    const ::simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    std::function<std::string(const RadarPoint&)> func);

/**
 * @brief Generates an unrolled-JSON object from a RadarTack.
 * @details Example object format:
 * {
 *   "sim_time": {
 *     "seconds": 0,
 *     "nanoseconds": 100000000,
 *   },
 *   "x": [
 *     10
 *     11.1,
 *   ],
 *   "y": [
 *     4.31
 *     2,
 *   ],
 *   ...
 * }
 *
 * @param radar_track
 * @param requested_fields A bitmask of RadarPoint fields to include in the JSON. When
 * requested_fields != ALL only the requested fields will be populated.
 * @param out_json JSON object to write output to.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string GetRadarPointsAsUnrolledJson(
    const ::simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    const RadarPoint::JSONFields requested_fields, nlohmann::json* out_json);

/**
 * @brief Saves a JSON object in the format described by GetRadarPointsAsUnrolledJson to a plaintext
 * or gzip'd file. On error, a non-empty string is returned.
 *
 * @param radar_track The RadarTrack data to use,
 * @param requested_fields A bitmask of RadarPoint fields to include in the JSON. When
 *                         requested_fields != ALL only the requested fields will be populated.
 * @param output_path Full-path output destination, including the filename ending with ".json".
 *                    If ending with ".gz", the save file will be gzip compressed.
 *                    E.g. "/dir/filename.json.gz".
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SaveRadarOutputAsUnrolledJson(
    const simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    const RadarPoint::JSONFields requested_fields, const std::string& output_path);

}  // namespace applied
