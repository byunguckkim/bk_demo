// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "applied/simian/public/proto/v2/io.pb.h"

/**
 * @brief Flag name and default boolean-value.
 */
using ExtraDataFlag = std::pair<std::string, bool>;

/**
 * @brief Helper class to manage flags specified via "extra_data" fields in YAML.
 *
 * @details Example extra data flag in YAML:
 *
 *   extra_data:
 *     count_lidar_points: true
 *     write_camera_images: true
 */
class ExtraDataFlags {
 public:
  /**
   * @brief Constructor that accepts a list of fields to parse.
   *
   * @param flags A vector of ExtraDataFlag to initialize against.
   */
  ExtraDataFlags(const std::vector<ExtraDataFlag>& default_flags = {}) {
    for (const auto& [flag_key, default_value] : default_flags) {
      flags_map_[flag_key] = FlagState{default_value, default_value};
    }
  }

  /**
   * @brief Updates flag states based on interface startup-options specified in YAML.
   *
   * @param startup_options The V2 InterfaceStartupOptions to load values from.
   * @returns Empty string on success, non-empty string on failure.
   */
  std::string parse_startup_options(
      const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) {
    if (!startup_options.has_scenario_extra_data() ||
        !startup_options.scenario_extra_data().has_struct_value()) {
      return "";
    }

    for (const auto& [flag_key, proto_value] :
         startup_options.scenario_extra_data().struct_value().fields()) {
      // Check flag is supported
      auto it = flags_map_.find(flag_key);
      if (it == flags_map_.end()) {
        std::cout << "[Warning] Ignoring unrecognized extra_data flag: " << flag_key << std::endl;
        continue;
      }

      // Update the flag state
      it->second.current_state = proto_value.bool_value();
    }

    return "";
  }

  /**
   * @brief Use ["flag_key"] to read a flag's state
   */
  bool operator[](const std::string& flag_key) const {
    return flags_map_.at(flag_key).current_state;
  }

  /**
   * @brief Use ["flag_key"] = <bool> to update a flag's state
   */
  bool& operator[](const std::string& flag_key) { return flags_map_.at(flag_key).current_state; }

  /**
   * @brief Prints the current state of each flag to an ostream
   */
  friend std::ostream& operator<<(std::ostream& os, const ExtraDataFlags& ed_flags) {
    // Print the map entries in sorted order. To do so, first sort the keys
    std::vector<std::string> flag_keys;
    for (const auto& [flag_key, flag_state] : ed_flags.flags_map_) {
      flag_keys.emplace_back(flag_key);
    }
    std::sort(flag_keys.begin(), flag_keys.end());

    // Print the keys using the now-sorted vector
    for (const auto& flag_key : flag_keys) {
      const auto& flag_state = ed_flags.flags_map_.at(flag_key);
      os << " " << std::left << std::setw(40) << flag_key + ": "
         << ((flag_state.current_state) ? "true " : "false");
      const bool is_default = (flag_state.current_state == flag_state.initial_state);
      (is_default) ? os << " (default)" << std::endl : os << std::endl;
    }

    return os;
  }

 private:
  /**
   * @brief Describes latest state and internal metadata of a flag
   */
  struct FlagState {
    bool initial_state;
    bool current_state;
  };

  /**
   * @brief Mapping betwen extra_data flag strings and their FlagState data
   */
  std::unordered_map<std::string, FlagState> flags_map_;
};
