// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/lidar_helpers.h"

#include <functional>
#include <memory>
#include <string>
#include <vector>

#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "applied/spectral/public/spectral_shared_memory_helper.h"
#include "applied/spectral/public/utils.h"

namespace applied {
namespace {

// Toggles between left/right handed coordinates systems.
void convert_handedness(LidarPoint& p) {
  p.x = -p.x;
  p.y = -p.y;
  std::swap(p.x, p.y);
}

/**
 * @brief Calculates the number of bytes in a LidarPoint which can differs
 * based on if include_groundtruth flag is toggled on or off.
 *
 * @param lidar_cloud The LidarCloud holding the points.
 *
 * @return An integer describing the number of bytes in a lidar point.
 */
size_t GetLidarPointSize(const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(lidar_cloud);
  const char* bin_data = data_pointer.memory_address;
  LidarPointsHeader header;
  memcpy(&header, bin_data, sizeof(LidarPointsHeader));
  const size_t lidar_point_bytes = header.lidar_point_size_words * sizeof(uint32_t);
  return lidar_point_bytes;
}

/**
 * @brief Describes the start position of the Lidar points field by
 * calculating the offset in bytes of the LidarHeader + LidarChannels.
 *
 * @param lidar_cloud The LidarCloud holding the LidarHeader and LidarChannels.
 *
 * @return An integer describing the LidarHeader + LidarChannels offset in bytes.
 */
size_t GetLidarPointsArrayStartOffset(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(lidar_cloud);
  const char* bin_data = data_pointer.memory_address;
  LidarPointsHeader header;
  memcpy(&header, bin_data, sizeof(LidarPointsHeader));
  size_t offset = sizeof(LidarPointsHeader);
  // Move past individual channel point counts
  offset += header.channel_count * sizeof(uint32_t);
  return offset;
}

}  // namespace

std::pair<int, std::string> GetNumLidarPoints(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(lidar_cloud);
  const size_t bin_size = data_pointer.size;
  if (bin_size < sizeof(LidarPointsHeader)) {
    return {-1, "lidar data was too small to contain header"};
  }
  size_t lidar_point_bytes = GetLidarPointSize(lidar_cloud);
  size_t offset = GetLidarPointsArrayStartOffset(lidar_cloud);
  if ((bin_size - offset) % lidar_point_bytes != 0) {
    return {-1, "lidar points data was not a valid size"};
  }
  return {(bin_size - offset) / lidar_point_bytes, ""};
}

std::string GetLidarPointFromLidarCloud(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    size_t lidar_point_index, LidarPoint& point) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(lidar_cloud);
  const char* bin_data = data_pointer.memory_address;
  const size_t bin_size = data_pointer.size;
  if (bin_size < sizeof(LidarPointsHeader)) {
    return "lidar data was too small to contain header";
  }
  size_t lidar_point_bytes = GetLidarPointSize(lidar_cloud);
  size_t offset = GetLidarPointsArrayStartOffset(lidar_cloud);
  if ((bin_size - offset) % lidar_point_bytes != 0) {
    return "lidar points data was not a valid size";
  }
  size_t num_lidar_points = (bin_size - offset) / lidar_point_bytes;
  if (lidar_point_index >= num_lidar_points) {
    return "lidar point index was out of lidar cloud range";
  }

  memset(&point, 0, sizeof(point));
  offset += (lidar_point_bytes * lidar_point_index);
  memcpy(&point, bin_data + offset, lidar_point_bytes);
  convert_handedness(point);
  return "";
}

std::string ForEachLidarPoint(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    std::function<std::string(const LidarPoint&)> func) {
  const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(lidar_cloud);
  const char* bin_data = data_pointer.memory_address;
  const size_t bin_size = data_pointer.size;
  if (bin_size < sizeof(LidarPointsHeader)) {
    return "lidar data was too small to contain header";
  }
  size_t lidar_point_bytes = GetLidarPointSize(lidar_cloud);
  size_t offset = GetLidarPointsArrayStartOffset(lidar_cloud);
  if ((bin_size - offset) % lidar_point_bytes != 0) {
    return "lidar points data was not a valid size";
  }

  LidarPoint p;
  memset(&p, 0, sizeof(p));
  for (; offset < bin_size; offset += lidar_point_bytes) {
    memcpy(&p, bin_data + offset, lidar_point_bytes);

    // Simian use right-handed coordinate system, convert to left-handed convention
    // to align with current LidarCloud convention.
    convert_handedness(p);

    const std::string err_msg = func(p);
    if (!err_msg.empty()) {
      return err_msg;
    }
  }
  return "";
}

std::string ModifyEachLidarPoint(
    ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    std::function<std::string(LidarPoint&)> func) {
  if (lidar_cloud.has_shared_memory_pointer()) {
    return "cannot modify lidar cloud with data in shared memory";
  }
  const size_t bin_size = lidar_cloud.points().size();
  std::vector<char> points_array(lidar_cloud.points().begin(), lidar_cloud.points().end());
  char* bin_data = points_array.data();
  if (bin_size < sizeof(LidarPointsHeader)) {
    return "lidar data was too small to contain header";
  }
  size_t offset = GetLidarPointsArrayStartOffset(lidar_cloud);
  const size_t lidar_point_bytes = GetLidarPointSize(lidar_cloud);
  if ((bin_size - offset) % lidar_point_bytes != 0) {
    return "lidar points data was not a valid size";
  }

  LidarPoint p;
  memset(&p, 0, sizeof(p));
  if (lidar_point_bytes > sizeof(p)) {
    return "too many bytes per lidar point to decode";
  }
  for (; offset < bin_size; offset += lidar_point_bytes) {
    // In case GT was not enabled, so we only want copy up a certain number of bytes
    // instead of reinterpret casting directly in the memory buffer.
    memcpy(&p, bin_data + offset, lidar_point_bytes);

    // Simian use right-handed coordinate system, convert to left-handed convention
    // to align with current LidarCloud convention.
    convert_handedness(p);

    const std::string err_msg = func(p);
    if (!err_msg.empty()) {
      return err_msg;
    }

    // Convert back to left-handed convention the LidarCloud originally stores the
    // point in
    convert_handedness(p);

    // Write updated point back to bin_data
    memcpy(bin_data + offset, &p, lidar_point_bytes);
  }
  // Copy bin_data back to lidarCloud at the very end which makes this function atomic.
  lidar_cloud.mutable_points()->assign(points_array.begin(), points_array.end());
  return "";
}

std::string FillLidarCloud(
    const uint32_t frame_number, const float horizontal_angle_deg, bool ground_truth_enabled,
    const std::string& sensor_name, const ::simian_public::spatial::Pose& sensor_pose,
    const std::vector<std::vector<LidarPoint>>& lidar_points,
    ::simian_public::sensor_model::SensorOutput_LidarCloud& dest_lidar_cloud) {
  // Set sensor_name
  dest_lidar_cloud.mutable_metadata()->set_sensor_name(sensor_name);

  // Set pose
  *(dest_lidar_cloud.mutable_metadata()->mutable_sensor_pose()) = sensor_pose;

  // Generate header
  LidarPointsHeader header;
  header.frame_number = frame_number;
  header.horizontal_angle_deg = horizontal_angle_deg;
  header.channel_count = lidar_points.size();
  const size_t lidar_point_size = (ground_truth_enabled)
                                      ? sizeof(LidarPoint)
                                      : sizeof(LidarPoint) - LidarPoint::kGroundTruthBytesSize;
  header.lidar_point_size_words = static_cast<uint32_t>(lidar_point_size / sizeof(uint32_t));

  // Calculate total byte size of LidarCloud.points
  //
  // There are 3 sections (see LidarCloudPointsFormat):
  //  - LidarPointsHeader
  //  - uint32[NumChannels]     // Num LidarPoints per channel
  //  - LidarPoint[]            // LidarPoints for all channels
  constexpr size_t kHeaderSize = sizeof(LidarPointsHeader);
  size_t total_size = kHeaderSize + header.channel_count * sizeof(uint32_t);
  for (const auto& points_single_channel : lidar_points) {
    total_size += points_single_channel.size() * lidar_point_size;
  }

  // Write directly to points() field in destination LidarCloud.
  auto dest_str = dest_lidar_cloud.mutable_points();
  dest_str->clear();
  dest_str->reserve(total_size);

  // Copy header
  dest_str->append(reinterpret_cast<char*>(&header), sizeof(header));

  // Copy channel point counts
  for (const auto& points_single_channel : lidar_points) {
    auto channel_point_count = static_cast<uint32_t>(points_single_channel.size());
    dest_str->append(reinterpret_cast<char*>(&channel_point_count), sizeof(channel_point_count));
  }

  // Copy points
  //
  // Ideally we could just memcpy the remaining bytes, but LidarPoint can be two
  // different sizes (if Ground-Truth was enabled/disabled), and we need to update
  // the coordinate system convention for each point. Thus we have to copy each point
  // individually.
  for (const auto& points_single_channel : lidar_points) {
    for (auto lidar_point : points_single_channel) {
      // Simian use right-handed coordinate system, convert to left-handed convention
      // to align with current LidarCloud convention.
      convert_handedness(lidar_point);
      dest_str->append(reinterpret_cast<char*>(&lidar_point), lidar_point_size);
    }
  }

  return "";
}

std::string GetLidarPointsAsUnrolledJson(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    const LidarPoint::JSONFields requested_fields, nlohmann::json* out_json) {
  out_json->clear();
  (*out_json)["sim_time"] = {{"seconds", lidar_cloud.metadata().sensor_timestamp().seconds()},
                             {"nanoseconds", lidar_cloud.metadata().sensor_timestamp().nanos()}};
  auto point_handler = [requested_fields, out_json](const LidarPoint& point) {
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::X)) {
      (*out_json)["x"].push_back(point.x);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::Y)) {
      (*out_json)["y"].push_back(point.y);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::Z)) {
      (*out_json)["z"].push_back(point.z);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::INTENSITY)) {
      (*out_json)["intensity"].push_back(point.intensity);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::CHANNEL)) {
      (*out_json)["channel"].push_back(point.channel);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::INSTANCE_ID)) {
      (*out_json)["instance_id"].push_back(point.instance_id);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::SEMANTIC_CLASS)) {
      (*out_json)["semantic_class"].push_back(point.semantic_class);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::AMBIENT)) {
      (*out_json)["ambient"].push_back(point.ambient);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::RANGE_GT)) {
      (*out_json)["range_gt"].push_back(point.range_gt);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::AZIMUTH_GT)) {
      (*out_json)["azimuth_gt"].push_back(point.azimuth_gt);
    }
    if (requested_fields == LidarPoint::ALL || (requested_fields & LidarPoint::ELEVATION_GT)) {
      (*out_json)["elevation_gt"].push_back(point.elevation_gt);
    }
    return std::string("");
  };
  const std::string err_msg = ForEachLidarPoint(lidar_cloud, point_handler);
  if (!err_msg.empty()) {
    return "error serializing point cloud to unrolled JSON: " + err_msg;
  }
  return "";
}

std::string SaveLidarOutputAsUnrolledJson(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud,
    const LidarPoint::JSONFields requested_fields, const std::string& output_path) {
  // Generate JSON from lidar cloud
  nlohmann::json lidar_json;
  std::string err_msg =
      ::applied::GetLidarPointsAsUnrolledJson(lidar_cloud, ::applied::LidarPoint::ALL, &lidar_json);
  if (!err_msg.empty()) {
    return "error getting json for lidar cloud: " + err_msg;
  }

  // Write JSON to disk
  err_msg = ::applied::utils::WriteToFile(output_path, lidar_json);
  if (!err_msg.empty()) {
    return "error wrting json for lidar cloud: " + err_msg;
  }

  return "";
}

std::pair<cv::Mat, std::string> GetLidarPointsAsImage(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    const LidarImageParams& params) {
  std::vector<float> x;
  std::vector<float> y;
  std::vector<cv::Scalar> colors;
  auto point_handler = [&x, &y, &colors, &params](const LidarPoint& point) {
    x.push_back(point.x);
    y.push_back(point.y);
    colors.push_back(params.get_color_for_point(point));
    return std::string("");
  };
  const std::string err_msg = ForEachLidarPoint(lidar_cloud, point_handler);
  if (!err_msg.empty()) {
    return {{}, err_msg};
  }
  cv::Mat result(params.size_y(), params.size_x(), CV_8UC3, cv::Scalar(0, 0, 0));
  if (x.size() == 0) {
    return {result, ""};
  }
  // It might be useful to extend this with a mode to keep a constant scale based on the lidar's
  // max range rather than zooming in on the points in this way based on per frame min/max.
  const float x_min = *std::min_element(x.begin(), x.end());
  const float x_max = *std::max_element(x.begin(), x.end());
  const float y_min = *std::min_element(y.begin(), y.end());
  const float y_max = *std::max_element(y.begin(), y.end());

  for (size_t i = 0; i < x.size(); ++i) {
    const float x_scaled = (x[i] - x_min) / (x_max - x_min);
    const float y_scaled = (y[i] - y_min) / (y_max - y_min);
    const int row = static_cast<int>(floor((1.0 - y_scaled) * params.size_y()));
    const int col = static_cast<int>(floor(x_scaled * params.size_x()));
    cv::circle(result, {col, row}, 1, colors[i]);
  }
  return {result, ""};
}

std::string SaveLidarOutputAsImage(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud,
    const LidarImageParams& params, const std::string& output_path) {
  if (!::applied::utils::EndsWith(output_path, ".png", true)) {
    return "expected output_path to end with .png: " + output_path;
  }
  const auto lidar_points_result = GetLidarPointsAsImage(lidar_cloud, params);
  const cv::Mat& image_data = lidar_points_result.first;
  const std::string& err_msg = lidar_points_result.second;
  if (!err_msg.empty()) {
    return "error serializing lidar data to image: " + err_msg;
  }
  static const std::vector<int> use_default_png_params;
  const bool success = cv::imwrite(output_path, image_data, use_default_png_params);
  if (!success) {
    // Typically this is because the parent directory hasn't been created yet.
    return "error writing png to: " + output_path;
  }
  return "";
}

}  // namespace applied
