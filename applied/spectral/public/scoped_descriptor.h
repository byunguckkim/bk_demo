// Copyright (C) 2020 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <fcntl.h>
#include <unistd.h>

namespace applied {

/**
 * ScopedDescriptor closes a file descriptor when the ScopedDescriptor goes out of scope.
 */
class ScopedDescriptor final {
 public:
  ScopedDescriptor() = default;
  /**
   * @brief Constructor that accepts an existing, open file descriptor.
   *
   * @param existing_file_descriptor a file desciptor that can be closed with `close()`.
   */
  explicit ScopedDescriptor(int existing_file_descriptor) : fd_(existing_file_descriptor) {}
  /**
   * @brief Constructor that accepts a path to open.
   *
   * @param path null terminated C string, for example "/tmp/foo/bar.txt"
   * @param flags see documentation for the open() system call (i.e. `man open`).
   */
  explicit ScopedDescriptor(const char* path, int flags) { fd_ = open(path, flags); }
  ScopedDescriptor(ScopedDescriptor&& rhs) noexcept : fd_(rhs.fd_) { rhs.fd_ = -1; }
  ScopedDescriptor& operator=(ScopedDescriptor&& rhs) {
    (void)Reset();
    fd_ = rhs.fd_;
    rhs.fd_ = -1;
    return *this;
  }
  ScopedDescriptor(const ScopedDescriptor& rhs) = delete;
  ScopedDescriptor& operator=(const ScopedDescriptor& rhs) = delete;
  ~ScopedDescriptor() { Reset(); }

  /**
   * @brief Close any currently held file descriptor and take ownership of the argument.
   *
   * @param fd a file descriptor that can be closed with `close()`.
   */
  void TakeOwnershipOf(int fd) {
    (void)Reset();
    fd_ = fd;
  }
  /**
   * @return the integer file descriptor that this ScopedDescriptor owns or -1.
   */
  int GetFd() const { return fd_; }
  /**
   * @return whether this ScopedDescriptor owns an open file descriptor.
   */
  bool IsValid() const { return fd_ >= 0; }
  /**
   * @brief `close()` any currently owned file descriptor.
   *
   * @return true if this ScopedDescriptor does not own a file descriptor after this call.
   */
  bool Reset() {
    if (!IsValid()) {
      return true;
    }
    const bool success = close(fd_) == 0;
    fd_ = -1;
    return success;
  }

 private:
  int fd_ = -1;
};

}  // namespace applied
