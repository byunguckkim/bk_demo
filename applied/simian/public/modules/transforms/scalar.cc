// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/simian/public/modules/transforms/scalar.h"

#include <algorithm>
#include <limits>
#include <ostream>
#include <utility>

namespace applied {

double Mod2pi(double angle) {
  angle = std::fmod(angle, 2.0 * M_PI);
  if (angle > M_PI) {
    return angle - 2.0 * M_PI;
  }
  if (angle <= -M_PI) {
    return angle + 2.0 * M_PI;
  }
  return angle;
}

double Mod2piPositive(double angle) {
  if (angle < 0.0) {
    return 2.0 * M_PI + std::fmod(angle, 2.0 * M_PI);
  }
  return std::fmod(angle, 2.0 * M_PI);
}

AngleInterval::AngleInterval(double start, double length) {
  if (length < 0.0) {
    unrolled_ = Interval::Empty();
  } else {
    start = Mod2piPositive(start);
    unrolled_ = Interval(start, start + length);
  }
}

bool AngleInterval::Contains(double angle) const {
  angle = Mod2piPositive(angle);
  return unrolled_.Contains(angle) || unrolled_.Contains(angle + 2.0 * M_PI);
}

bool AngleInterval::Contains(const AngleInterval& other) const {
  if (unrolled_.Contains(other.unrolled_)) {
    return true;
  }
  const Interval shifted(Interval::Shift(other.unrolled_, 2.0 * M_PI));
  return unrolled_.Contains(shifted);
}

bool AngleInterval::ContainsIncludingTolerance(double angle, double tolerance) const {
  angle = Mod2piPositive(angle);
  return unrolled_.ContainsIncludingTolerance(angle, tolerance) ||
         unrolled_.ContainsIncludingTolerance(angle + 2.0 * M_PI, tolerance);
}

bool AngleInterval::Overlaps(const AngleInterval& other) const {
  if (unrolled_.Overlaps(other.unrolled_)) {
    return true;
  }
  for (int ii = -2; ii <= 2; ii += 4) {
    if (unrolled_.Overlaps(Interval::Shift(other.unrolled_, ii * M_PI))) {
      return true;
    }
  }
  return false;
}

void AngleInterval::GrowToInclude(double angle) {
  angle = Mod2piPositive(angle);
  if (unrolled_.IsEmpty()) {
    unrolled_.GrowToInclude(angle);
    return;
  }

  const Interval candidates[3] = {
      Interval::SpanningUnion(unrolled_, angle),
      Interval::SpanningUnion(unrolled_, angle + 2.0 * M_PI),
      Interval::SpanningUnion(Interval::Shift(unrolled_, 2.0 * M_PI), angle)};
  const Interval* best_candidate = &candidates[0];
  double best_length = best_candidate->Length();
  for (int ii = 1; ii < 3; ++ii) {
    const Interval& candidate = candidates[ii];
    const double length = candidate.Length();
    if (length < best_length) {
      best_length = length;
      best_candidate = &candidate;
    }
  }
  unrolled_ = *best_candidate;
}

void AngleInterval::GrowToInclude(const AngleInterval& other) {
  if (unrolled_.IsEmpty()) {
    unrolled_ = other.unrolled_;
    return;
  }

  const Interval candidates[3] = {
      Interval::SpanningUnion(unrolled_, other.unrolled_),
      Interval::SpanningUnion(unrolled_, Interval::Shift(other.unrolled_, 2.0 * M_PI)),
      Interval::SpanningUnion(Interval::Shift(unrolled_, 2.0 * M_PI), other.unrolled_)};
  const Interval* best_candidate = &candidates[0];
  double best_length = best_candidate->Length();
  for (int ii = 1; ii < 3; ++ii) {
    const Interval& candidate = candidates[ii];
    const double length = candidate.Length();
    if (length < best_length) {
      best_length = length;
      best_candidate = &candidate;
    }
  }
  unrolled_ = *best_candidate;
}

AngleInterval AngleInterval::FromTo(double from, double to) {
  if (to < from) {
    std::swap(from, to);
  }
  from = Mod2piPositive(from);
  to = Mod2piPositive(to);
  double length = to - from;
  if (length < 0.0) {
    length += 2.0 * M_PI;
  }
  return AngleInterval(from, length);
}

}  // namespace applied

namespace std {

ostream& operator<<(ostream& os, const applied::Interval& interval) {
  if (interval.IsEmpty()) {
    os << "empty";
  } else {
    if (interval.lower == numeric_limits<double>::lowest()) {
      os << "-inf";
    } else {
      os << interval.lower;
    }
    if (interval.upper == numeric_limits<double>::max()) {
      os << " +inf";
    } else {
      os << " " << interval.upper;
    }
  }
  return os;
}

ostream& operator<<(ostream& os, const applied::AngleInterval& angle_interval) {
  if (angle_interval.IsUnlimited()) {
    os << "unlimited";
  } else {
    os << angle_interval.Unrolled();
  }
  return os;
}

}  // namespace std
