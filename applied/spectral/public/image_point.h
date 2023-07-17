// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cmath>

namespace applied {

struct ImagePoint {
  double x{};
  double y{};

  ImagePoint operator+(const ImagePoint& other) const { return {x + other.x, y + other.y}; }
  ImagePoint operator-(const ImagePoint& other) const { return {x - other.x, y - other.y}; }
  ImagePoint operator*(double scale) const { return {scale * x, scale * y}; }

  double norm() const { return std::sqrt(norm2()); }
  double norm2() const { return x * x + y * y; }

  ImagePoint normalized() const {
    const double r = norm();
    return {x / r, y / r};
  }

  ImagePoint& operator+=(const ImagePoint& other) {
    x += other.x;
    y += other.y;
    return *this;
  }
};  // struct ImagePoint

std::ostream& operator<<(std::ostream& os, const ImagePoint& m);

}  // namespace applied
