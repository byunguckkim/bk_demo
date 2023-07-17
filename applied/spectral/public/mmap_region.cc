// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt
#include "applied/spectral/public/mmap_region.h"

#include <iostream>
#include <limits>

#if defined(__unix__)
#include <sys/mman.h>
#include <sys/stat.h>
#elif defined(_WIN32)

#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif  // WIN32_LEAN_AND_MEAN
#include <windows.h>

#endif

#if !defined(DEBUG_MMAP_REGION)
// Allow people to turn on the debugging messages, but leave it on by default;
// the failure modes are pretty arcane.
#define DEBUG_MMAP_REGION 1
#endif

namespace applied {

MmapRegion::MmapRegion(void* region, size_t region_size)
    : region_(region), region_size_(region_size) {}
MmapRegion::MmapRegion(MmapRegion&& rhs) noexcept
    : region_(rhs.region_), region_size_(rhs.region_size_) {
  rhs.region_ = nullptr;
  rhs.region_size_ = 0;
}
MmapRegion::~MmapRegion() { (void)Reset(); }

#if defined(__unix__)
std::unique_ptr<MmapRegion> MmapRegion::MapFileToMemory(const std::string& path) {
  ScopedDescriptor fd(path.c_str(), O_RDONLY);
  if (!fd.IsValid()) {
    return nullptr;
  }

  struct stat64 stat_buf;
  if (fstat64(fd.GetFd(), &stat_buf) == -1) {
    return nullptr;
  }
  void* ret = mmap64(nullptr, stat_buf.st_size, PROT_READ, MAP_SHARED, fd.GetFd(), 0);
  if (ret == MAP_FAILED || ret == nullptr) {
    return nullptr;
  }
  return std::unique_ptr<MmapRegion>(new MmapRegion(ret, stat_buf.st_size));
}
#elif defined(_WIN32)

// https://docs.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-createfilemappinga
// https://docs.microsoft.com/en-us/windows/win32/api/winbase/nf-winbase-openfilemappinga
// https://docs.microsoft.com/en-us/windows/win32/debug/system-error-codes--0-499-
std::unique_ptr<MmapRegion> MmapRegion::MapFileToMemory(const std::string& path) {
  HANDLE opened_file =
      CreateFile(path.c_str(),
                 GENERIC_READ,  // open for reading
                 // Have to use the least restriction sharing settings or it is an error.
                 FILE_SHARE_WRITE | FILE_SHARE_READ,
                 NULL,                   // default security
                 OPEN_EXISTING,          // existing file only
                 FILE_ATTRIBUTE_NORMAL,  // normal file
                 NULL);                  // no attr. template
  if (opened_file == INVALID_HANDLE_VALUE) {
#if DEBUG_MMAP_REGION
    std::cerr << "MmapRegion::MapFileToMemory() failed: CreateFile() failed: " << GetLastError()
              << std::endl;
#endif
    return nullptr;
  }
  LARGE_INTEGER windows_legacy_size{};
  if (!GetFileSizeEx(opened_file, &windows_legacy_size)) {
#if DEBUG_MMAP_REGION
    std::cerr << "MmapRegion::MapFileToMemory() failed: GetFileSizeEx() failed: " << GetLastError()
              << std::endl;
#endif
    CloseHandle(opened_file);
    return nullptr;
  }
  const size_t file_size = windows_legacy_size.QuadPart;

  HANDLE file_mapping = CreateFileMappingA(opened_file,
                                           nullptr,  // default security
                                           PAGE_READONLY, 0, 0, nullptr);
  const int32_t create_file_mapping_err = GetLastError();
  CloseHandle(opened_file);
  if (file_mapping == nullptr) {
#if DEBUG_MMAP_REGION
    std::cerr << "MmapRegion::MapFileToMemory() failed: GetFileMapping() failed: "
              << create_file_mapping_err << std::endl;
#endif
    return nullptr;
  }
  // https://docs.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-mapviewoffile
  void* region = MapViewOfFile(file_mapping,   // handle to map object
                               FILE_MAP_READ,  // read only permission
                               0,              // file offset high dword
                               0,              // file offset low dword
                               0);             // mapped buffer len, 0 -> map to end of the file
  const int32_t map_view_of_file_err = GetLastError();
  // From https://docs.microsoft.com/en-us/windows/win32/api/memoryapi/nf-memoryapi-unmapviewoffile
  //   Although an application may close the file handle used to create a file
  //   mapping object, the system holds the corresponding file open until the
  //   last view of the file is unmapped.
  CloseHandle(file_mapping);
  if (region == nullptr) {
#if DEBUG_MMAP_REGION
    std::cerr << "MmapRegion::MapFileToMemory() failed: MapViewOfFile() failed: "
              << map_view_of_file_err << std::endl;
#endif
    return nullptr;
  }
  return std::unique_ptr<MmapRegion>(new MmapRegion(region, file_size));
}
#endif

bool MmapRegion::Reset() {
  if (region_ == nullptr) {
    return true;
  }
#if defined(__unix__)
  const bool success = munmap(region_, region_size_) == 0;
#elif defined(_WIN32)
  UnmapViewOfFile(region_);
  const bool success = true;
#else
#error unsupported OS
#endif
  region_ = nullptr;
  region_size_ = 0;
  return success;
}

void* MmapRegion::GetVoidPointer(size_t offset) const {
  if (region_ == nullptr || offset >= region_size_) {
    return nullptr;
  }
  return (void*)(uint64_t(region_) + uint64_t(offset));
}

}  // namespace applied
