// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cstddef>
#include <memory>
#include <string>

#include "applied/spectral/public/scoped_descriptor.h"

namespace applied {

/**
 * @brief MmapRegion represents a memory mapped file's in-process footprint.
 */
class MmapRegion final {
 public:
  /**
   * Create an MmapRegion.
   *
   * @param path path to file to mmap() into this process
   * @returns a pointer to a valid MmapRegion instance, or nullptr on error
   */
  static std::unique_ptr<MmapRegion> MapFileToMemory(const std::string& path);
  MmapRegion(MmapRegion&& rhs) noexcept;
  MmapRegion(const MmapRegion& rhs) = delete;
  MmapRegion& operator=(const MmapRegion& rhs) = delete;
  ~MmapRegion();

  /**
   * @brief Remove this memory mapping from our process's address space.
   *
   * @return true if the file is no longer mapped
   */
  bool Reset();
  /**
   * @return true if a file is currently mapping into memory by this MmapRegion.
   */
  bool IsValid() const { return region_ != nullptr; }
  /**
   * @brief Get a pointer into the region at a given offset.
   *
   * @param offset desired number of bytes from the begining of the file
   * @return nullptr if no such byte is mapped, or a pointer to the requested byte
   */
  void* GetVoidPointer(size_t offset) const;

  /**
   * @brief Helper function to get a pointer to a field with a given type within a region.
   *
   * @param offset desired number of bytes from the begining of the file
   * @return nullptr if no such byte is mapped, or a pointer to the requested byte
   */
  template <typename T>
  T* GetPointer(size_t offset) const {
    return (T*)GetVoidPointer(offset);
  }

 private:
  explicit MmapRegion(void* region, size_t region_size);

  void* region_ = nullptr;
  size_t region_size_ = 0;
};

}  // namespace applied
