// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "spectral_shared_memory_helper.h"

#include <stdio.h>
#include <unistd.h>

#include <mutex>
#include <string>

namespace applied {
namespace spectral {

DataPointer SMHelper::ReadDataPointer(
    const simian_public::sensor_sim::basic_types::SharedMemoryPointer& shared_memory_pointer) {
  const char* map_addr =
      GetMappedDataPointer(shared_memory_pointer.filename(), shared_memory_pointer.offset());
  if (map_addr == nullptr) {
    printf("WARNING: could not resolve pointer to shared memory in %s\n",
           shared_memory_pointer.filename().c_str());
    fflush(stdout);
    return DataPointer{nullptr, 0};
  }
  return DataPointer{map_addr, shared_memory_pointer.size()};
}

// DEPRECATED - Please refer to simian_public::sensor_sim::basic_types::SharedMemoryPointer as
// simian_public::sensor_model::SharedMemoryPointer is deprecated and all references to this type
// will be removed in release 1.29.
DataPointer SMHelper::ReadDataPointer(
    const simian_public::sensor_model::SharedMemoryPointer& deprecated_shared_memory_pointer) {
  printf(
      "WARNING: simian_public::sensor_model::SharedMemoryPointer is deprecated and will be removed "
      "in release 1.29 (refer to simian_public::sensor_sim::basic_types::SharedMemoryPointer)");
  simian_public::sensor_sim::basic_types::SharedMemoryPointer shared_memory_pointer;
  shared_memory_pointer.set_filename(deprecated_shared_memory_pointer.filename());
  shared_memory_pointer.set_offset(deprecated_shared_memory_pointer.offset());
  shared_memory_pointer.set_size(deprecated_shared_memory_pointer.size());
  return ReadDataPointer(shared_memory_pointer);
}

DataPointer SMHelper::ReadDataPointer(
    const simian_public::sensor_model::SensorOutput_CameraImage& camera_output) {
  // Check if camera has no image_bytes filled already, and has non-empty
  // filename specified by the data_pointer.
  if (camera_output.image().image_bytes().size() == 0 &&
      camera_output.image().shared_memory_pointer().filename().length() > 0) {
    return ReadDataPointer(camera_output.image().shared_memory_pointer());
  }
  // Return the raw image bytes otherwise
  return DataPointer{camera_output.image().image_bytes().data(),
                     camera_output.image().image_bytes().size()};
}

DataPointer SMHelper::ReadDataPointer(
    const simian_public::sensor_model::SensorOutput_LidarCloud& lidar_output) {
  // Check if lidar has no points filled already and has non-empty filename
  // specified by the shared_memory_pointer.
  if (lidar_output.points().size() == 0 &&
      lidar_output.shared_memory_pointer().filename().length() > 0) {
    return ReadDataPointer(lidar_output.shared_memory_pointer());
  }
  // Return the raw points bytes otherwise
  return DataPointer{lidar_output.points().data(), lidar_output.points().size()};
}

DataPointer SMHelper::ReadDataPointer(
    const simian_public::sensor_model::SensorOutput_RadarTrack& radar_output) {
  // Check if radar has no tracks filled already and has non-empty filename
  // specified by the shared_memory_pointer.
  if (radar_output.tracks().size() == 0 &&
      radar_output.shared_memory_pointer().filename().length() > 0) {
    return ReadDataPointer(radar_output.shared_memory_pointer());
  }
  // Return the raw track bytes otherwise
  return DataPointer(radar_output.tracks().data(), radar_output.tracks().size());
}

DataPointer SMHelper::ReadDataPointer(
    const ::simian_public::sensor_sim::basic_types::SensorDataOutput& output) {
  if (output.has_shmem_pointer()) {
    return ReadDataPointer(output.shmem_pointer());
  }
  return DataPointer(output.data().data(), output.data().size());
}

char* const SMHelper::GetMappedDataPointer(const std::string& filename, size_t offset) {
  std::lock_guard<std::mutex> guard(mmap_regions_mutex_);
  auto it = mmap_regions_.find(filename);
  if (it == mmap_regions_.end()) {
    std::unique_ptr<MmapRegion> region = MmapRegion::MapFileToMemory(filename);
    mmap_regions_.emplace(filename, std::move(*region));
    it = mmap_regions_.find(filename);
  }
  return it->second.GetPointer<char>(offset);
}

}  // namespace spectral
}  // namespace applied
