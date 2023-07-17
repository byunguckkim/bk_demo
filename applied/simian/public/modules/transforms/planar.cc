// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/simian/public/modules/transforms/planar.h"

#include <algorithm>
#include <ostream>
#include <vector>

namespace applied {

double IntersectPositiveXAxisWithLineSegment(const Vector2d& point_a, const Vector2d& point_b) {
  if (point_a.y() * point_b.y() > 0.0) {
    // Endpoints lie on same side of ray => no intersection.
    return -1;
  }
  if (std::abs(point_b.x() - point_a.x()) < 1e-6) {
    // Segment crosses ray at (nearly) right angles. Distance is same
    // as either segment points' x coordinate. May or may not
    // intersect (x may be negative).
    return point_b.x();
  }
  const double dy = point_b.y() - point_a.y();
  if (std::abs(dy) < 1e-6) {
    // Segment lies (nearly) on ray. Distance is the min of the
    // segment points' x coordinate. If that min is negative but one
    // of the endpoints' x is positive, return 0 instead.
    const double dist = std::min(point_b.x(), point_a.x());
    if (dist >= 0.0) {
      return dist;
    }
    if (std::max(point_b.x(), point_a.x()) >= 0.0) {
      return 0.0;
    }
    return -1.0;
  }

  // With all those pesky special cases out of the way...
  return (point_a.x() * point_b.y() - point_a.y() * point_b.x()) / dy;
}

double IntersectRayWithLineSegments(const Vector2d& ray_origin, double ray_angle,
                                    const std::vector<LineSegment2d>& segments) {
  const Pose2d ray_t_ref = MakePose2d(ray_origin.x(), ray_origin.y(), ray_angle).inverse();
  double mindist = -1.0;
  for (const auto& segment : segments) {
    const double dist = IntersectPositiveXAxisWithLineSegment(ray_t_ref * segment.point_a,
                                                              ray_t_ref * segment.point_b);
    if (dist < 0.0) {
      continue;
    }
    if (mindist < 0.0) {
      mindist = dist;
    } else {
      mindist = std::min(mindist, dist);
    }
  }
  return mindist;
}

const State2d ReexpressPose(const Pose2d& pose, const State2d& state2d) {
  // Note: no translational offset is applied when re-expressing velocity and acceleration.
  return {state2d.Acceleration(), state2d.Velocity(), state2d.Yawrate(), pose * state2d.Pose()};
}

double PoseAngleDiff(const Pose2d& world_T_ref, const Pose2d& world_T_b) {
  const Pose2d ref_T_b = world_T_ref.inverse() * world_T_b;
  return ComputePose2dAngle(ref_T_b);
}

double AbsPoseAngleDiff(const Pose2d& world_T_a, const Pose2d& world_T_b) {
  return std::abs(PoseAngleDiff(world_T_a, world_T_b));
}

}  // namespace applied

namespace std {

ostream& operator<<(std::ostream& os, const applied::Pose2d& pose) {
  os << pose.translation().x() << " " << pose.translation().y() << " "
     << applied::ComputePose2dAngle(pose);
  return os;
}

ostream& operator<<(ostream& os, const applied::Vector2d& point) {
  os << point.x() << " " << point.y();
  return os;
}

ostream& operator<<(ostream& os, const applied::Screw2d& screw) {
  os << std::get<0>(screw).x() << " " << std::get<0>(screw).y() << " " << std::get<1>(screw);
  return os;
}

ostream& operator<<(ostream& os, const applied::State2d& state) {
  os << "State2d:\n";
  os << "  acceleration: " << state.AccelerationScrew() << "\n";
  os << "  velocity: " << state.VelocityScrew() << "\n";
  os << "  pose: " << state.Pose() << "\n";
  return os;
}

}  // namespace std
