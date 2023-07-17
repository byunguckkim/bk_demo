// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/radar_helpers.h"

#include <cstring>
#include <functional>
#include <string>

#include <cmath>
#include <nlohmann/json.hpp>

#include "applied/spectral/public/spectral_shared_memory_helper.h"
#include "applied/spectral/public/utils.h"

namespace applied {

namespace {

void prepare_raw_radar_point(RadarPoint& p) {
  constexpr float kDegreeToRadian = M_PI / 180.0;
  // Spectral actually publishes these points in spherical coordinates
  const float range = p.x;
  const float azimuth = p.y;
  const float elevation = p.z;
  double theta = elevation * kDegreeToRadian;
  double rcostheta = range * cos(theta);
  p.x = rcostheta * cos(kDegreeToRadian * azimuth);
  p.y = rcostheta * sin(kDegreeToRadian * azimuth);
  p.z = range * sin(theta);
  // Left hand unreal coordinates to Simian right hand.
  p.y = -p.y;
  // These actually come in as raw linear values
  p.snr_dB = 10.0 * log10(p.snr_dB);
  p.rcs_dBsm = 10.0 * log10(p.rcs_dBsm);
}

}  // namespace

std::pair<int, std::string> GetNumRadarPoints(
    const simian_public::sensor_model::SensorOutput::RadarTrack& radar_track) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(radar_track);
  const char* bin_data = data_pointer.memory_address;
  const size_t bin_size = data_pointer.size;

  RadarPointsHeader header;
  if (bin_size < sizeof(header)) {
    return {-1, "radar data was too small to contain header"};
  }
  memcpy(&header, bin_data, sizeof(RadarPointsHeader));

  return {header.num_points, ""};
}

std::string ForEachRadarPoint(
    const ::simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    std::function<std::string(const RadarPoint&)> func) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(radar_track);
  const char* bin_data = data_pointer.memory_address;
  const size_t bin_size = data_pointer.size;

  RadarPointsHeader header;
  if (bin_size < sizeof(RadarPointsHeader)) {
    return "radar data was too small to contain header";
  }
  memcpy(&header, bin_data, sizeof(header));
  size_t offset = sizeof(header);

  const size_t point_stride = header.radar_point_size_words * sizeof(float);
  if ((bin_size - offset) % point_stride != 0) {
    return "radar points data was not a valid size";
  }
  if (point_stride < sizeof(RadarPoint)) {
    return "radar points data was not a valid size";
  }

  RadarPoint p;
  memset(&p, 0, sizeof(p));
  const size_t copied_size = std::min<size_t>(sizeof(p), point_stride);
  for (; offset < bin_size; offset += point_stride) {
    memcpy(&p, bin_data + offset, copied_size);
    prepare_raw_radar_point(p);
    std::string err_msg = func(p);
    if (!err_msg.empty()) {
      return err_msg;
    }
  }
  return "";
}

std::string GetRadarPointsAsUnrolledJson(
    const ::simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    const RadarPoint::JSONFields requested_fields, nlohmann::json* out_json) {
  out_json->clear();
  (*out_json)["sim_time"] = {{"seconds", radar_track.metadata().sensor_timestamp().seconds()},
                             {"nanoseconds", radar_track.metadata().sensor_timestamp().nanos()}};
  const bool want_all = requested_fields == RadarPoint::ALL;
  auto point_handler = [requested_fields, want_all, out_json](const RadarPoint& point) {
    if (want_all || (requested_fields & RadarPoint::X)) {
      (*out_json)["x"].push_back(point.x);
    }
    if (want_all || (requested_fields & RadarPoint::Y)) {
      (*out_json)["y"].push_back(point.y);
    }
    if (want_all || (requested_fields & RadarPoint::Z)) {
      (*out_json)["z"].push_back(point.z);
    }
    if (want_all || (requested_fields & RadarPoint::EXT_RANGE)) {
      (*out_json)["ext_range"].push_back(point.ext_range);
    }
    if (want_all || (requested_fields & RadarPoint::EXT_WIDTH)) {
      (*out_json)["ext_width"].push_back(point.ext_width);
    }
    if (want_all || (requested_fields & RadarPoint::EXT_HEIGHT)) {
      (*out_json)["ext_height"].push_back(point.ext_height);
    }
    if (want_all || (requested_fields & RadarPoint::VELOCITY)) {
      (*out_json)["velocity"].push_back(point.velocity);
    }
    if (want_all || (requested_fields & RadarPoint::SNR_DB)) {
      (*out_json)["snr_dB"].push_back(point.snr_dB);
    }
    if (want_all || (requested_fields & RadarPoint::RCS_DBSM)) {
      (*out_json)["rcs_dBsm"].push_back(point.rcs_dBsm);
    }
    if (want_all || (requested_fields & RadarPoint::ACTOR_ID)) {
      (*out_json)["actor_id"].push_back(point.actor_id);
    }
    if (want_all || (requested_fields & RadarPoint::SEMANTIC_CLASS)) {
      (*out_json)["semantic_class"].push_back(point.semantic_class);
    }
    return std::string("");
  };
  std::string err_msg = ForEachRadarPoint(radar_track, point_handler);
  if (!err_msg.empty()) {
    return "error serializing track to unrolled JSON: " + err_msg;
  }
  return "";
}

std::string SaveRadarOutputAsUnrolledJson(
    const simian_public::sensor_model::SensorOutput_RadarTrack& radar_track,
    const RadarPoint::JSONFields requested_fields, const std::string& output_path) {
  // Generate JSON
  nlohmann::json radar_json;
  std::string err_msg =
      ::applied::GetRadarPointsAsUnrolledJson(radar_track, ::applied::RadarPoint::ALL, &radar_json);
  if (!err_msg.empty()) {
    return "error getting json for radar track: " + err_msg;
  }

  // Write JSON to disk
  err_msg = ::applied::utils::WriteToFile(output_path, radar_json);
  if (!err_msg.empty()) {
    return "error wrting json for radar track: " + err_msg;
  }

  return "";
}

}  // namespace applied
