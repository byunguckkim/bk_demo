// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cstdint>

namespace applied {
/**
 * @brief Header of RadarTrack.tracks byte array
 */
#pragma pack(push, 1)
struct RadarPointsHeader final {
  uint32_t reserved0;
  uint32_t reserved1;

  /**
   * @brief Size of RadarPoint struct in words (32-bits/4 bytes).
   */
  uint32_t radar_point_size_words;

  /**
   * @brief Number of RadarPoints present in this RadarTrack.
   */
  uint32_t num_points;
};
#pragma pack(pop)

/**
 * @brief Describes the format of a single radar point.
 */
#pragma pack(push, 1)
struct RadarPoint final {
  // Cartesian positon of the point relative to the sensor origin
  // Simian's coordinate system (meters, x forward, y left, z up).
  float x;
  float y;
  float z;
  // The depth resolution of this scan beam.
  float ext_range;
  // The width and height, in degrees, of this scan beam.
  float ext_width;
  float ext_height;
  float velocity;
  float snr_dB;
  float rcs_dBsm;

  uint32_t reserved_0;

  /**
   * @brief Semantic information of the Simian actor this point intersected with.
   *        See Spectral manual for more details.
   */
  // Points on actors will be tagged with an instance ID that matches
  // their Simian actor ID.
  uint16_t actor_id;
  uint8_t reserved_1;
  // See manual for list of semantic classes.
  uint8_t semantic_class;

  enum JSONFields : uint64_t {
    ALL = 0,
    X = 1 << 0,
    Y = 1 << 1,
    Z = 1 << 2,
    EXT_RANGE = 1 << 3,
    EXT_WIDTH = 1 << 4,
    EXT_HEIGHT = 1 << 5,
    VELOCITY = 1 << 6,
    SNR_DB = 1 << 7,
    RCS_DBSM = 1 << 8,
    ID = 1 << 9,
    ACTOR_ID = 1 << 10,
    SEMANTIC_CLASS = 1 << 11,
  };
};
#pragma pack(pop)
static_assert(sizeof(RadarPoint) == 44, "Unexpected RadarPoint size");

/**
 * @brief (Reference) Wire format of RadarTrack.points bytes array
 */
struct RadarTrackPointsFormat {
  /**
   * @brief Describes the number of points/samples per point.
   */
  RadarPointsHeader header;

  /**
   * @brief Array of RadarPoint points.
   */
  RadarPoint* points;
};

}  // namespace applied
