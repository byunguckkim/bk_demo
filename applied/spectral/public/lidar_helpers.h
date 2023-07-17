// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <functional>
#include <string>

#include <nlohmann/json_fwd.hpp>
#include <opencv2/core/mat.hpp>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"
#include "applied/spectral/public/lidar_point.h"

namespace applied {

/**
 * @brief Calculates the number of LidarPoint instances inside a given LidarCloud.
 *
 * @param lidar_cloud The LidarCloud to count points of
 *
 * @return An std::pair where .first is the total point count and .second is a status
 * string (empty on success, non-empty on failure).
 */
std::pair<int, std::string> GetNumLidarPoints(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud);

/**
 * @brief Invokes a given function on each LidarPoint in the given LidarCloud.
 *
 * @param lidar_cloud The LidarCloud to iterate through
 * @param func A function to invoke on a single LidarPoint instance. Returns std::string.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string ForEachLidarPoint(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    std::function<std::string(const LidarPoint&)> func);

/**
 * @brief Invokes a given function on each LidarPoint with write-access to the lidar
 * point in memory in the given LidarCloud. If the LidarCloud points are accessed
 * through the shared memory pointer, it will be written back through the bytes field.
 *
 * @param lidar_cloud The LidarCloud to iterate through
 * @param func A function to invoke on a single LidarPoint instance. Returns std::string.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string ModifyEachLidarPoint(
    ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    std::function<std::string(LidarPoint&)> func);

/**
 * @brief Gets the LidarPoint at position `lidar_point_index` in the lidar_cloud.
 *  Returns the point in RHS (simian) coordinate system.
 *
 * @param lidar_cloud The LidarCloud.
 * @param lidar_point_index The position of the LidarPoint to retrieve.
 * @param lidar_point The LidarPoint to be copied over.
 *
 * @returns Empty string on success, nonempty string on failure.
 */
std::string GetLidarPointFromLidarCloud(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    size_t lidar_point_index, LidarPoint& point);

/**
 * @brief Given a vector of LidarPoints and required properties, fills an existing LidarCloud
 * proto structure that can support visualization in Applied products like Strada and Logstream.
 * Stores the points in LHS (spectral) coordinate system.
 *
 * @param frame_number The Spectral frame number for the LidarCloud.
 * @param horizontal_angle_deg The horizontal azimuth angle for the LidarCloud (in degrees).
 * @param ground_truth_enabled Set true if the LidarPoints were simulated with ground-truth enabled.
 * @param sensor_name Name of lidar sensor (should match name of LIDAR channel specified in the
 * `v2_api_basic` section of the scenario).
 * @param sensor_pose Vehicle-to-sensor pose of the lidar sensor when the data was recorded.
 * @param lidar_points A 2-D vector of LidarPoints for all channels of the lidar, where
 * lidar_points[i][j] = jth point of ith lidar channel.
 * @param dest_lidar_cloud The LidarCloud proto message who's 'points' field to write to.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string FillLidarCloud(
    const uint32_t frame_number, const float horizontal_angle_deg, bool ground_truth_enabled,
    const std::string& sensor_name, const ::simian_public::spatial::Pose& sensor_pose,
    const std::vector<std::vector<LidarPoint>>& lidar_points,
    ::simian_public::sensor_model::SensorOutput_LidarCloud& dest_lidar_cloud);

/**
 * @brief Generates an unrolled-JSON object from a LidarCloud.
 * @details Example object format:
 *   {
 *     "sim_time": {
 *       "seconds": 0,
 *       "nanoseconds": 100000000,
 *     },
 *     "x": [
 *       10
 *       11.1,
 *     ],
 *     "y": [
 *       4.31
 *       2,
 *     ],
 *     ...
 *   }
 *
 * @param lidar_cloud
 * @param requested_fields A bitmask of LidarPoint fields to include in the JSON. When
 * requested_fields != ALL only the requested fields will be populated.
 * @param out_json JSON object to write output to.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string GetLidarPointsAsUnrolledJson(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    const LidarPoint::JSONFields requested_fields, nlohmann::json* out_json);

/**
 * @brief Saves a JSON object in the format described by GetLidarPointsAsUnrolledJson to a plaintext
 * or gzip'd file. On error, a non-empty string is returned.
 *
 * @param lidar_cloud The LidarCloud data to use,
 * @param requested_fields A bitmask of LidarPoint fields to include in the JSON. When
 *                         requested_fields != ALL only the requested fields will be populated.
 * @param output_path Full-path output destination, including the filename ending with ".json".
 *                    If ending with ".gz", the save file will be gzip compressed.
 *                    E.g. "/dir/filename.json.gz".
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SaveLidarOutputAsUnrolledJson(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud,
    const LidarPoint::JSONFields requested_fields, const std::string& output_path);

/**
 * @brief Customizes images created with GetLidarPointsAsImage().
 */
class LidarImageParams final {
 public:
  LidarImageParams() = default;
  ~LidarImageParams() = default;
  size_t size_x() const { return size_x_; }
  size_t size_y() const { return size_y_; }
  cv::Scalar get_color_for_point(const LidarPoint& p) const {
    (void)p;

    // It would also be natural to adopt intensity or semantic label based
    // coloring schemes.
    return constant_point_color_;
  }

  void set_size_x(size_t size_x) {
    if (size_x > 0) {
      size_x_ = size_x;
    }
  }
  void set_size_y(size_t size_y) {
    if (size_y > 0) {
      size_y_ = size_y;
    }
  }
  // Note that this color is in BGR per OpenCV's convention.
  void set_constant_color(const cv::Scalar& color) { constant_point_color_ = color; }

 private:
  size_t size_x_{1000u};
  size_t size_y_{1000u};
  cv::Scalar constant_point_color_{200, 250, 100};
};

/**
 * @brief Renders the given LidarCloud as an image. Useful for lidar visualization without a 3D
 * renderer.
 *
 * @param lidar_cloud The LidarCloud to render.
 * @param params THe LidarImageParams instance to describe the desired output image.
 *
 * @returns An std::pair of the image as a cv::Mat, and std::string which will be empty on success,
 *          and non-empty on failure.
 */
std::pair<cv::Mat, std::string> GetLidarPointsAsImage(
    const ::simian_public::sensor_model::SensorOutput_LidarCloud& lidar_cloud,
    const LidarImageParams& params);

/**
 * @brief Saves a PNG representing the given LidarCLoud to the given output path.  On error, a
 * non-empty string is returned.
 *
 * @param lidar_cloud The LidarCloud data to use.
 * @param params THe LidarImageParams instance to describe the desired output image.
 * @param output_path Full-path output destination, include the filename.
 *                    Must end with ".png".
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SaveLidarOutputAsImage(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud,
    const LidarImageParams& params, const std::string& output_path);

}  // namespace applied
