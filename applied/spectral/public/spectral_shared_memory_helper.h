// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cstdint>
#include <map>
#include <mutex>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/sensor_sim/basic_types.pb.h"
#include "applied/simian/public/proto/sim_data.pb.h"
#include "applied/spectral/public/mmap_region.h"

namespace applied {
namespace spectral {

/**
 * @brief Pointer to a shared-memory location mapped by Spectral.
 */
struct DataPointer {
  DataPointer() = default;
  DataPointer(const char* memory_addr, size_t sz) : memory_address(memory_addr), size(sz) {}

  /**
   * @brief Resolved memory address
   */
  const char* memory_address = nullptr;

  /**
   * @brief Size in bytes of data at memory_address
   */
  size_t size = 0u;
};

/**
 * @brief Resolves a simian_public::sensor_sim::basic_types::SharedMemoryPointer proto message
 *        generated from Spectral into a DataPointer which can be used to access shared-memory
 *        location and its contents.
 */
class SMHelper {
 public:
  SMHelper() = default;
  ~SMHelper() = default;

  /**
   * @brief Creates a DataPointer that can access a shared-memory contents a SharedMemoryPointer
   *        proto field.
   *
   * @param data_pointer The SharedMemoryPointer to resolve.
   * @return On success, a valid DataPointer. On failure, a DataPointer with nullptr for
   *         memory_address.
   */
  DataPointer ReadDataPointer(
      const simian_public::sensor_sim::basic_types::SharedMemoryPointer& shared_memory_pointer);

  // DEPRECATED - Please refer to simian_public::sensor_sim::basic_types::SharedMemoryPointer as
  // simian_public::sensor_model::SharedMemoryPointer is deprecated and all references to this type
  // will be removed in release 1.29.
  //
  // Returns a DataPointer from a given SharedMemoryPointer.
  // If data_pointer is somehow invalid, the memory_address will be a nullptr
  DataPointer ReadDataPointer(
      const simian_public::sensor_model::SharedMemoryPointer& deprecated_shared_memory_pointer);

  /**
   * @brief Creates a DataPointer that can access a shared-memory contents a given CameraImage.
   *        Note: if shared-memory was not enabled, DataPointer will point to the original protobuf
   *        content instead of a memory-mapped address (for CameraImage,
   *        CameraImage.image().image_bytes()).
   *
   * @param camera_output The CameraImage to resolve.
   * @return On success, a valid DataPointer. On failure, a DataPointer with nullptr for
   *         memory_address.
   */
  DataPointer ReadDataPointer(
      const simian_public::sensor_model::SensorOutput_CameraImage& camera_output);

  /**
   * @brief Creates a DataPointer that can access a shared-memory contents a given LidarCloud.
   *        Note: if shared-memory was not enabled, DataPointer will point to the original protobuf
   *        content instead of a memory-mapped address (for LidarCloud, LidarCloud.points())
   *
   * @param lidar_output The LidarCloud to resolve.
   * @return On success, a valid DataPointer. On failure, a DataPointer with nullptr for
   *         memory_address.
   */
  DataPointer ReadDataPointer(
      const simian_public::sensor_model::SensorOutput_LidarCloud& lidar_output);

  /**
   * @brief Creates a DataPointer that can access a shared-memory contents a given RadarTrack.
   *        Note: if shared-memory was not enabled, DataPointer will point to the original protobuf
   *        content instead of a memory-mapped address (for RadarTrack, RadarTrack.points())
   *
   * @param radar_output The RadarTrack to resolve.
   * @return On success, a valid DataPointer. On failure, a DataPointer with nullptr for
   *         memory_address.
   */
  DataPointer ReadDataPointer(
      const simian_public::sensor_model::SensorOutput_RadarTrack& radar_output);

  /**
   * @brief Creates a DataPointer that can access a shared-memory contents a given SensorDataOutput.
   *
   * @param output The SensorDataOutput to resolve.
   * @return On success, a valid DataPointer. On failure, a DataPointer with nullptr for
   *         memory_address.
   */
  DataPointer ReadDataPointer(
      const ::simian_public::sensor_sim::basic_types::SensorDataOutput& output);

 private:
  /**
   * @brief Memory maps the filename (if not done so already)
   *
   * @param filename The filename of the mmap'd region.
   * @param offset Byte offset within region.
   * @return Pointer to the memory-mapped region + offset.
   */
  char* const GetMappedDataPointer(const std::string& filename, size_t offset);

  /**
   * @brief Mutex for synchronizing read/writes to mmap_regions_
   */
  std::mutex mmap_regions_mutex_;

  /**
   * @brief Mapping of filenames and MMap regions. Sensors tend to come from the same files, so we
   *        keep the memory mapping open for the duration of the simulation.
   */
  std::map<std::string, MmapRegion> mmap_regions_;
};

/**
 * @brief Singleton wrapper of SMHelper which can be used in place of managing an instance of
 *        SMHelper.
 *
 * @details Example usage:
 *             auto data_pointer = SMHelperSingleton::GetInstance().ReadDataPointer(...);
 */
class SMHelperSingleton final {
 public:
  static SMHelper& GetInstance() {
    static SMHelper instance{};
    return instance;
  }

  explicit SMHelperSingleton() = delete;
};

}  // namespace spectral
}  // namespace applied
