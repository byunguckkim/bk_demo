// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <algorithm>
#include <iosfwd>
#include <vector>

#include "applied/simian/public/modules/transforms/eigen.h"

namespace applied {

using Vector2d = Eigen::Matrix<double, 2, 1, Eigen::DontAlign>;

// TODO(rolo) Consider building our own Pose2d class from Eigen::Rotation2D and
// a Vector2d. Just like Pose3d does in spatial. OTOH this appears to be good
// enough for now.
using Pose2d = Eigen::Transform<double, 2, Eigen::Affine, Eigen::DontAlign>;
// Screw2d is [(tx, ty), rz] i.e. [(linear X component, linear Y component), angular Z component]
using Screw2d = std::tuple<Vector2d, double>;

inline const Pose2d MakePose2d(double x, double y, double angle) {
  return Eigen::Translation2d(Vector2d(x, y)) * Eigen::Rotation2D<double>(angle);
}

inline const Screw2d MakeScrew2d(double tx, double ty, double rz) {
  return std::make_tuple(Vector2d(tx, ty), rz);
}

inline const Screw2d MakeScrew2d(Vector2d translation, double rotation) {
  return std::make_tuple(translation, rotation);
}

// Class representation of state, which is passed down via pybind into python
// and can be converted to a State2d proto message.
class State2d {
 public:
  State2d()
      : pose_(MakePose2d(0, 0, 0)),
        velocity_(MakeScrew2d(0, 0, 0)),
        acceleration_(MakeScrew2d(0, 0, 0)) {}
  State2d(const double acceleration, const double velocity, const double yawrate, const double x,
          const double y, const double heading)
      : pose_(MakePose2d(x, y, heading)),
        velocity_(MakeScrew2d(velocity, 0, yawrate)),
        acceleration_(MakeScrew2d(acceleration, 0, 0)) {}
  State2d(const double acceleration, const double velocity, const double yawrate,
          const Pose2d& pose)
      : pose_(pose),
        velocity_(MakeScrew2d(velocity, 0, yawrate)),
        acceleration_(MakeScrew2d(acceleration, 0, 0)) {}
  State2d(const Pose2d& pose, const Screw2d& velocity, const Screw2d& acceleration)
      : pose_(pose), velocity_(velocity), acceleration_(acceleration) {}
  double Acceleration() const { return std::get<0>(acceleration_).x(); }
  Screw2d AccelerationScrew() const { return acceleration_; }
  double Velocity() const { return std::get<0>(velocity_).x(); }
  Screw2d VelocityScrew() const { return velocity_; }
  double Yawrate() const { return std::get<1>(velocity_); }
  Pose2d Pose() const { return pose_; }
  void SetAcceleration(const double acceleration) {
    acceleration_ =
        MakeScrew2d(acceleration, std::get<0>(acceleration_).y(), std::get<1>(acceleration_));
  }
  void SetAccelerationScrew(const Screw2d& acceleration) { acceleration_ = acceleration; }
  void SetVelocity(const double velocity) {
    velocity_ = MakeScrew2d(velocity, std::get<0>(velocity_).y(), std::get<1>(velocity_));
  }
  void SetVelocityScrew(const Screw2d& velocity) { velocity_ = velocity; }
  void SetYawrate(const double yawrate) {
    velocity_ = MakeScrew2d(std::get<0>(velocity_), yawrate);
  }
  void SetPose(const Pose2d& pose) { pose_ = pose; }

 private:
  Pose2d pose_;
  Screw2d velocity_;
  Screw2d acceleration_;
};

// TODO(rolo) Implement a proper Pose2d class and cache its angle so
// we don't need to recompute this all the time. See planar_py.cc
// (move that approach into the core c++ impl).
inline double ComputePose2dAngle(const Pose2d& tt) {
  return std::atan2(tt.linear()(1, 0), tt.linear()(0, 0));
}

inline bool Pose2dNear(const Pose2d& lhs, const Pose2d& rhs, double epsilon) {
  const Pose2d delta = lhs.inverse() * rhs;
  return delta.translation().norm() <= epsilon && std::abs(ComputePose2dAngle(delta)) <= epsilon;
}

// Technically an approximate comparison because involves a hardcoded choice of epsilon.
inline bool Pose2dEq(const Pose2d& lhs, const Pose2d& rhs) { return Pose2dNear(lhs, rhs, 1e-12); }

// Screw2d is an alias for std::tuple<Vector2d, double>
// Checking near equality by checking near equality of the vectors (via the norm) and of each
// double.
inline bool Screw2dNear(const Screw2d& lhs, const Screw2d& rhs, double epsilon) {
  return (std::get<0>(lhs) - std::get<0>(rhs)).norm() <= epsilon &&
         std::abs(std::get<1>(lhs) - std::get<1>(rhs)) <= epsilon;
}

inline bool operator==(const State2d& lhs, const State2d& rhs) {
  constexpr double kEpsilon = 1e-12;
  return std::fabs(lhs.Acceleration() - rhs.Acceleration()) < kEpsilon &&
         std::fabs(lhs.Velocity() - rhs.Velocity()) < kEpsilon &&
         std::fabs(lhs.Yawrate() - rhs.Yawrate()) < kEpsilon && Pose2dEq(lhs.Pose(), rhs.Pose());
}

inline bool operator!=(const State2d& lhs, const State2d& rhs) { return !(lhs == rhs); }

// Intersects the X-axis positive half-line with the given line
// segment. Returns the distance from the origin to the intersection
// point, or a negative number if there is no intersection.
double IntersectPositiveXAxisWithLineSegment(const Vector2d& point_a, const Vector2d& point_b);

struct LineSegment2d {
  LineSegment2d(const Vector2d& aa, const Vector2d& bb) : point_a(aa), point_b(bb) {}
  LineSegment2d(double px, double py, double qx, double qy) : point_a(px, py), point_b(qx, qy) {}
  const Vector2d point_a;
  const Vector2d point_b;
};

// Intersects a ray (half-line) with a collection of line
// segments. Returns the distance from the ray_origin to the nearest
// intersection point. If the ray intersects none of the segments, a
// negative number is returned.
double IntersectRayWithLineSegments(const Vector2d& ray_origin, double ray_angle,
                                    const std::vector<LineSegment2d>& segments);

// Re-express a State2d from the frame described by the provided pose to the parent frame
// of the pose.
//
// Let a_T_b be a Pose2d defining the transform from frame A to frame B.
// Let b_S_c be a State2d providing the state of frame/body C expressed in the frame B.
// Let a_S_c be a State2d providing the state of frame/body C expressed in the ego frame.
// This function is used as follows:
//     State2d a_S_c = Reexpress(a_T_b, b_S_c);
//
// The translational offset between frames is included when re-expressing the MotionState's pose,
// but not when re-expressing the State2d's velocity or acceleration.
//
// Note: This function does not affect the frame in which velocities and accelerations are observed.
//
// Returns a_S_c
const State2d ReexpressPose(const Pose2d& a_T_b, const State2d& b_S_c);

// Calculates the angle difference between the poses, using the first pose as the reference.
double PoseAngleDiff(const Pose2d& world_T_ref, const Pose2d& world_T_b);

// Calculates the absolute value of the angle difference between the poses.
double AbsPoseAngleDiff(const Pose2d& world_T_a, const Pose2d& world_T_b);

}  // namespace applied

namespace std {

ostream& operator<<(ostream& os, const applied::Pose2d& pose);

ostream& operator<<(ostream& os, const applied::Vector2d& point);

ostream& operator<<(ostream& os, const applied::Screw2d& screw);

ostream& operator<<(ostream& os, const applied::State2d& state);
}  // namespace std
