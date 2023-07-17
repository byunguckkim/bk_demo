// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cmath>

#include <algorithm>
#include <iosfwd>
#include <limits>

#include "applied/simian/public/utils/optional.h"

namespace applied {

// Convert an angle (in radians) to the equivalent angle within the interval (-pi, pi].
double Mod2pi(double angle);

// Convert an angle (in radians) to the equivalent angle within the interval [0, 2*pi).
double Mod2piPositive(double angle);

// This interval is made for non-periodic domains. For angles, use
// AngleInterval instead.
struct Interval {
  double lower, upper;

  // Specifying _lower > _upper means the interval is treated as empty.
  Interval() : lower(1.0), upper(-1.0) {}
  Interval(double _lower, double _upper) : lower(_lower), upper(_upper) {}

  // Given two potentially out of order values, create an Interval where lower takes the value of
  // the lower-magnitude input.
  static Interval MakeNonEmptyInterval(double bound_a, double bound_b) {
    if (bound_a <= bound_b) {
      return Interval(bound_a, bound_b);
    }
    return Interval(bound_b, bound_a);
  }

  bool IsEmpty() const { return lower > upper; }

  double Length() const { return upper - lower; }

  bool Contains(double value) const {
    // No need to check IsEmpty().
    return lower <= value && upper >= value;
  }

  bool Contains(const Interval& other) const {
    // No need to check IsEmpty(), but we could trip over
    // other.IsEmpty() if our bounds happen to be big enough to
    // contain both sentinal values.
    return !other.IsEmpty() && Contains(other.lower) && Contains(other.upper);
  }

  bool ContainsIncludingTolerance(double value, double epsilon) const {
    // Expands the interval by a small amount on both sides to account for floating point
    // imprecision.
    return !IsEmpty() && lower - epsilon <= value && upper + epsilon >= value;
  }

  double Clamp(double value) const {
    if (IsEmpty()) {
      return value;
    }
    return std::max(lower, std::min(upper, value));
  }

  bool Overlaps(const Interval& other) const {
    return !IsEmpty() && !other.IsEmpty() &&
           (Contains(other.lower) || Contains(other.upper) || other.Contains(lower) ||
            other.Contains(upper));
  }

  void GrowToInclude(double value) { *this = SpanningUnion(*this, value); }
  void GrowToInclude(const Interval& other) { *this = SpanningUnion(*this, other); }

  static Interval Empty() { return Interval(); }

  static Interval Unlimited() {
    return Interval((std::numeric_limits<double>::lowest)(), (std::numeric_limits<double>::max)());
  }

  static Interval Shift(const Interval& orig, double shift) {
    return Interval(orig.lower + shift, orig.upper + shift);
  }

  static Interval SpanningUnion(const Interval& orig, double value) {
    if (orig.IsEmpty()) {
      return Interval(value, value);
    }
    return Interval(std::min(orig.lower, value), std::max(orig.upper, value));
  }

  static Interval SpanningUnion(const Interval& aa, const Interval& bb) {
    if (aa.IsEmpty()) {
      return bb;
    }
    if (bb.IsEmpty()) {
      return aa;
    }
    return Interval(std::min(aa.lower, bb.lower), std::max(aa.upper, bb.upper));
  }

  // Given two intervals, determine the signed clearance; overlapping intervals return 0.
  //
  // @param interval_1: An Interval that defines positive and negative such that if interval_2 is
  // greater than interval 1, then the result is positive. If interval 2 is less than interval 1,
  // then the result is negative.
  // @param interval_2: An Interval.
  //
  // @return The signed clearance if both inputs are valid (non-empty) intervals, otherwise returns
  // an empty std::optional.
  static nonstd::optional<double> SignedClearance(const Interval& interval_1,
                                                  const Interval& interval_2) {
    // Equation from stack overflow:
    // https://stackoverflow.com/questions/4449285/efficient-algorithm-for-shortest-distance-between-two-line-segments-in-1d
    if (interval_1.IsEmpty() || interval_2.IsEmpty()) return nonstd::nullopt;

    // Assume interval_1 is "first" in magnitude, meaning interval_1 is either less than interval_2
    // (positive result expected) or they overlap (zero result expected).
    Interval first = interval_1;
    Interval second = interval_2;
    int negative_modifier = 1;
    if (interval_1.lower > interval_2.lower) {
      // interval_1 is either greater than interval_2 (negative result expected) or they overlap
      // (zero result expected).
      first = interval_2;
      second = interval_1;
      negative_modifier = -1;
    }
    const double signed_clearance = negative_modifier * std::max(0.0, second.lower - first.upper);
    return signed_clearance;
  }

  // If both intervals are empty, then they are equal.
  // If only one of the two intervals is empty, then they are not equal.
  // If neither is empty, then they are equal only if their lower and upper fields are both equal.
  bool operator==(const Interval& other_interval) {
    if (IsEmpty() && other_interval.IsEmpty()) return true;
    if (IsEmpty() || other_interval.IsEmpty()) return false;
    return lower == other_interval.lower && upper == other_interval.upper;
  }
};

// Angles are interesting, because they wrap around. And we need
// numerical fudging due to the modulo operations. And those can both
// happen at once.
class AngleInterval {
 public:
  AngleInterval() : unrolled_(Interval::Empty()) {}
  AngleInterval(double start, double length);

  bool operator==(const AngleInterval& rhs) const {
    return (IsEmpty() && rhs.IsEmpty()) ||
           (unrolled_.lower == rhs.unrolled_.lower && unrolled_.upper == rhs.unrolled_.upper);
  }

  double Start() const { return unrolled_.lower; }
  double Length() const { return unrolled_.Length(); }

  bool IsEmpty() const { return unrolled_.IsEmpty(); }
  bool IsUnlimited() const { return unrolled_.Length() >= 2.0 * M_PI; }
  bool Contains(double angle) const;
  bool Contains(const AngleInterval& other) const;
  bool ContainsIncludingTolerance(double angle, double tolerance) const;
  bool Overlaps(const AngleInterval& other) const;

  // Grows whichever side requires a smaller change of length.
  void GrowToInclude(double angle);
  void GrowToInclude(const AngleInterval& other);
  static AngleInterval GrowToInclude(const AngleInterval& orig, const AngleInterval& other) {
    AngleInterval grown = orig;
    grown.GrowToInclude(other);
    return grown;
  }

  static AngleInterval Empty() { return AngleInterval(); }
  static AngleInterval Unlimited() { return AngleInterval(0.0, 2.1 * M_PI); }
  static AngleInterval FromTo(double from, double to);

  const Interval& Unrolled() const { return unrolled_; }

 private:
  Interval unrolled_;
};

}  // namespace applied

namespace std {

ostream& operator<<(ostream& os, const applied::Interval& interval);

ostream& operator<<(ostream& os, const applied::AngleInterval& angle_interval);

}  // namespace std
