// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include <iostream>

#include "applied/spectral/public/image_point.h"

namespace applied {

std::ostream& operator<<(std::ostream& os, const ImagePoint& m) {
  return os << "ImagePoint(" << m.x << "," << m.y << ")";
}

}  // namespace applied
