// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cassert>
#include <cstdint>

namespace applied {

/**
 * @brief Header of LidarCloud.points byte array
 *
 * Note: sizeof(LidarPoint) will change if ground-truth is enabled in scenario config.
 * See LidarPointsHeader.lidar_point_size_words.
 */
#pragma pack(push, 1)
struct LidarPointsHeader final {
  /** @brief Frame number assigned by Spectral */
  uint64_t frame_number;

  /** @brief Horizontal azimuth angle from Spectral (in degrees) */
  float horizontal_angle_deg;

  /** @brief Number of channels (or lasers) of the lidar. */
  uint32_t channel_count;

  /**
   * @brief Size of LidarPoint struct in words (32-bits/4 bytes).
   *
   * @details sizeof(LidarPoint) changes by LidarPoint::kGroundTruthBytesSize based on if
   * ground-truth is enabled in the lidar model scenario config. Use this field to determine the
   * size of LidarPoint when unpacking LidarCloud.points.
   */
  uint32_t lidar_point_size_words;
};
static_assert(sizeof(LidarPointsHeader) == 20, "Unexpected size of LidarPointsHeader");
#pragma pack(pop)

/**
 * @brief Describes the format of a single lidar point as adapted by lidar_helpers.h.
 */
#pragma pack(push, 1)
struct LidarPoint final {
  /**
   * @brief Positional data.
   *
   * @details Reported in meters from the sensors origin, where Simian's coordinate system
   * uses x as forward, y as left, and z as up.
   */
  float x;
  float y;
  float z;

  /**
   * @brief Intensity of the point
   *
   * @details See sensor paramaters for configuration options:
   * https://home.applied.co/manual/spectral/latest/#/sensor_types/lidar/parameters
   */
  float intensity;

  /**
   * @brief Channel index value
   */
  float channel;

  /**
   * @brief Semantic information of the Simian actor this point intersected with.
   *        See Spectral manual for more details.
   *
   * @details
   * https://home.applied.co/manual/spectral/latest/#/sensors_overview/semantic_segmentation/semantic_segmentation?id=semantic-segmentation
   */
  uint16_t instance_id;
  uint8_t reserved;
  uint8_t semantic_class;

  /**
   * @brief Background power in dBW for a given beam. Note: this field is disabled by default.
   */
  float ambient;

  /**
   * @brief [OPTIONAL] Ground-truth positional data
   *
   * @details These fields are only present when ground-truth is enabled in the scenario config.
   * They indicate the ground truth range (in meters), azimuth and elevation (in radians) before
   * noise models are applied. If not ground-truth is not enabled, then these fields are not
   * transmitted (i.e. sizeof(LidarPoint) will reduce by kGroundTruthBytesSize).
   */
  float range_gt;
  float azimuth_gt;
  float elevation_gt;

  /** @brief Class constant representing how many bytes are represented by ground-truth fields */
  static constexpr size_t kGroundTruthBytesSize = 3 * sizeof(float);
  static_assert(kGroundTruthBytesSize % 4 == 0,
                "kGroundTruthBytesSize not word-aligned (4-byte aligned)");

  /**
   * @brief Bitmask mapping of all fields in LidarPoint which are transcribable
   * to JSON via GetLidarPointsAsUnrolledJson and SaveLidarOutputAsUnrolledJson.
   */
  enum JSONFields : uint64_t {
    ALL = 0,
    X = 1 << 0,
    Y = 1 << 1,
    Z = 1 << 2,
    INTENSITY = 1 << 3,
    CHANNEL = 1 << 4,
    INSTANCE_ID = 1 << 5,
    SEMANTIC_CLASS = 1 << 6,
    AMBIENT = 1 << 7,
    RANGE_GT = 1 << 8,
    AZIMUTH_GT = 1 << 9,
    ELEVATION_GT = 1 << 10,
  };
};
// Ground-Truth Disabled
static_assert((sizeof(LidarPoint) - LidarPoint::kGroundTruthBytesSize) == 28,
              "Unexpected size of LidarPoint - kGroundTruthBytesSize");
// Ground-Truth Enabled
static_assert(sizeof(LidarPoint) == 40, "Unexpected size of LidarPoint");
#pragma pack(pop)

/**
 * @brief (Reference) Wire format of LidarCloud.points bytes array
 */
struct LidarCloudPointsFormat {
  /**
   * @brief Describes the lidar channel count, firing angle, and LidarPoint size
   */
  LidarPointsHeader header;

  /**
   * @brief For each channel, the number of points in following "points" array for that specific
   *        channel.
   * @details E.g. if channel_point_count[0] is 10, then the first 10 points in points array below
   *          are for the first channel
   */
  uint32_t* channel_point_count;

  /**
   * @brief Array of LidarPoint instances organized by channel.
   * @details E.g. if channel_point_cound = [1, 2, 3], points[0] is from first channel,
   *          points[1-2] are from second channel, and points[3-5] are from the third channel.
   */
  LidarPoint* points;
};

}  // namespace applied
