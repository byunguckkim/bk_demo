// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/simian/public/modules/transforms/spatial.h"

#include <ostream>
#include <tuple>

namespace applied {

namespace {
constexpr double kNearZero = 1e-12;
}  // namespace

bool QuaternionIsNormalized(const Quaternion& qq, double tolerance) {
  return std::abs(qq.norm() - 1.0) < tolerance;
}

bool SameOrientation(const Quaternion& lhs, const Quaternion& rhs) {
  // To understand why (w, x, y, z) and (-w, -x, -y, -z) are equivalent,
  // consider the rotation matrix expressed using a quaternion:
  // https://en.wikipedia.org/wiki/Rotation_matrix
  // clang-format off
  // R = [xx - yy - zz + ww, 2xy - 2zw,         2xz + 2yw,
  //      2xy + 2zw,        -xx + yy - zz + ww, 2yz - 2xw,
  //      2xz - 2yw,         2yz + 2xw,        -xx - yy + zz + ww];
  // clang-format on
  // Each term is a multiplication between two elements in the quaternion.
  if (lhs.w() == rhs.w() && lhs.x() == rhs.x() && lhs.y() == rhs.y() && lhs.z() == rhs.z()) {
    return true;
  }
  return lhs.w() == -rhs.w() && lhs.x() == -rhs.x() && lhs.y() == -rhs.y() && lhs.z() == -rhs.z();
}

bool AlmostEqual(const Quaternion& lhs, const Quaternion& rhs, double angle_tol) {
  return lhs.angularDistance(rhs) <= angle_tol;
}

const Quaternion AxisAngleToQuaternion(const Vector3d& axis, double angle) {
  const double sqn = axis.squaredNorm();
  if (sqn < kNearZero) {
    return IdentityQuaternion();
  }
  if (std::abs(sqn - 1.0) < kNearZero) {
    return Quaternion(Eigen::AngleAxis<double>(angle, axis));
  }
  return Quaternion(Eigen::AngleAxis<double>(angle, axis / std::sqrt(sqn)));
}

const Quaternion RollPitchYawToQuaternion(double roll, double pitch, double yaw) {
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  yaw *= 0.5;
  pitch *= 0.5;
  roll *= 0.5;
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  return Quaternion(cy * cr * cp + sy * sr * sp, cy * sr * cp - sy * cr * sp,
                    cy * cr * sp + sy * sr * cp, sy * cr * cp - cy * sr * sp);
}

const Quaternion PitchRollYawToQuaternion(double pitch, double roll, double yaw) {
  // https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
  // Using 3-1-2 quaternion multiplication order instead for Yaw-Roll-Pitch.
  yaw *= 0.5;
  pitch *= 0.5;
  roll *= 0.5;
  const double cy = std::cos(yaw);
  const double sy = std::sin(yaw);
  const double cr = std::cos(roll);
  const double sr = std::sin(roll);
  const double cp = std::cos(pitch);
  const double sp = std::sin(pitch);
  Quaternion qyaw = Quaternion(cy, 0, 0, sy);
  Quaternion qpitch = Quaternion(cp, 0, sp, 0);
  Quaternion qroll = Quaternion(cr, sr, 0, 0);
  return qyaw * qroll * qpitch;
}

const Vector3d UnitQuaternionToRollPitchYaw(const Quaternion& quaternion) {
  const double qw = quaternion.w();
  const double qx = quaternion.x();
  const double qy = quaternion.y();
  const double qz = quaternion.z();

  // To avoid NaN when RPY is near gymbal lock, i.e. abs(pitch)=pi/2,
  // replace it with special case expressions. Set roll=0 in those, to
  // resolve the ambiguity of roll and yaw acting along the same axis
  // when gymbal locked.
  const double sin_pitch = 2.0 * (qw * qy - qz * qx);
  if (std::abs(sin_pitch) > 1.0 - 1e-6) {
    // Summary of the approach: find the x and y components of the
    // fully rotated Z unit vector. That'll basically represent the
    // yaw if we assume roll=0. To find those components, use e.g. the
    // expressions for going from quaternions to rotation matrices, or
    // work through the Hamiltonian
    // product. E.g.
    // https://euclideanspace.com/maths/geometry/rotations/conversions/quaternionToMatrix/index.htm
    //
    // Note that both zx and zy below should be twice as big really,
    // but we end up just plugging them into atan2() so magnitude does
    // not really matter and we can skip that operation.
    const double zx = qw * qy + qx * qz;
    const double zy = qy * qz - qw * qx;
    if (sin_pitch > 0.0) {
      return Vector3d(-std::atan2(zy, zx), +M_PI / 2.0, 0.0);
    }
    return Vector3d(std::atan2(-zy, -zx), -M_PI / 2.0, 0.0);
  }

  const double aa = std::atan2(2.0 * (qw * qx + qy * qz), 1.0 - 2.0 * (qx * qx + qy * qy));
  const double bb = std::asin(sin_pitch);
  const double cc = std::atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  return Vector3d(aa, bb, cc);
}

// Inspired by
// https://www.euclideanspace.com/maths/geometry/rotations/conversions/quaternionToAngle/index.htm
void UnitQuaternionToAxisAngle(const Quaternion& quaternion, Vector3d* axis, double* angle) {
  if (quaternion.w() > 1.0) {
    // Assume this is numerical noise, treat like w==1 (i.e. no
    // rotation).
    *angle = 0.0;
    *axis << 1.0, 0.0, 0.0;
    return;
  }

  *angle = 2.0 * std::acos(quaternion.w());
  const double scale = std::sqrt(1.0 - quaternion.w() * quaternion.w());
  if (scale < kNearZero) {
    *angle = 0.0;
    *axis << 1.0, 0.0, 0.0;
    return;
  }
  *axis = quaternion.vec();
  *axis /= scale;
}

double UnitQuaternionToHeading(const Quaternion& unit_quaternion) {
  const auto ux = unit_quaternion._transformVector(Vector3d::UnitX());
  return std::atan2(ux.y(), ux.x());
}

const Vector3d UnitQuaternionToRotationVector(const Quaternion& src, const Quaternion& dst) {
  assert(std::abs(src.norm() - 1.0) < 1e-10);
  assert(std::abs(dst.norm() - 1.0) < 1e-10);
  const Quaternion delta = dst * src.inverse();
  assert(std::abs(delta.norm() - 1.0) < 1e-10);
  const double theta = 2.0 * std::acos(delta.w());
  if (std::abs(theta) < 1e-12 || std::isnan(theta)) {
    // No rotation, avoid NaN.
    return Vector3d(0.0, 0.0, 0.0);
  }
  return (theta / sin(theta / 2)) * Vector3d(delta.x(), delta.y(), delta.z());
}

// TODO(rolo,mike) Implement this more cleanly when it gets used in
// the context of screw motions (i.e. combined translations and
// rotations). Using the axis-angle as intermediare works fine for
// current use cases.
const Quaternion IntegrateRotationalVelocity(const Quaternion& src, const Vector3d& omega,
                                             double duration) {
  // TODO(dembia): To support any 3D rotation, we should use:
  ///  quaternion += duration * quaterniondot
  // where quaterniondot is 0.5 * [-qx, qy, qz; qw, qz, -qy; qw, qx, -qz; qw, qy, -qx] * omega
  const double angle = duration * omega.norm();
  if (angle < 1e-12) {
    return src;
  }

  // Note hat the order here is surprising. But verified, see comments
  // in PR #18734.
  return AxisAngleToQuaternion(omega, angle) * src;
}

Pose3d::Pose3d() : translation_(Vector3d::Zero()), unit_quaternion_(IdentityQuaternion()) {}

Pose3d::Pose3d(const Vector3d& translation, const Quaternion& quaternion)
    : translation_(translation), unit_quaternion_(quaternion.normalized()) {
  // TODO(dembia): I think the original intent here was that zero quaternions become identity,
  // but the actual behavior is that zero quaternions become NaN (because of the normalization
  // above), for which the comparison below is false.
  // TODO(#29976) Test this out and also do the right thing on NaN
  // components.
  if (std::abs(unit_quaternion_.squaredNorm() - 1.0) > 1e-8) {
    // Eigen leaves the coefficients unchanged in case it cannot
    // normalize. So replace our unit quaternion in case something looks
    // fishy, when someone passed a zero quaternion.
    //
    // Because unit_quaternion_ is normalized in the initializer, this branch should not happen
    // except when the passed quaternion is not set or otherwise unnormalizable, which is what makes
    // this scary-looking substitution not really scary.  But we don't want to do it in any ordinary
    // situation, which is the reason for the relatively high tolerance of 1e-8.
    unit_quaternion_ = IdentityQuaternion();
  }
}

const Pose3d Pose3d::Inverse() const {
  const Quaternion invq = unit_quaternion_.conjugate();
  return Pose3d(invq._transformVector(-translation_), invq);
}

const Vector3d Pose3d::operator*(const Vector3d& rhs_vector) const {
  return unit_quaternion_._transformVector(rhs_vector) + translation_;
}

const Screw Pose3d::operator*(const Screw& rhs_screw) const { return unit_quaternion_ * rhs_screw; }

const Pose3d Pose3d::operator*(const Pose3d& rhs_pose) const {
  return Pose3d(unit_quaternion_._transformVector(rhs_pose.translation_) + translation_,
                unit_quaternion_ * rhs_pose.unit_quaternion_);
}

const Pose3d MakePose3dPlanarXY(const Pose2d& other) {
  const double yaw = ComputePose2dAngle(other);
  return MakePose3dRollPitchYaw(other.translation().x(), other.translation().y(), 0.0, 0.0, 0.0,
                                yaw);
}

const Pose3d MakePose3dLeftUp(double tx, double ty, double tz, double lx, double ly, double lz,
                              double ux, double uy, double uz) {
  Vector3d unit_y(lx, ly, lz);
  unit_y.normalize();
  Vector3d unit_z(ux, uy, uz);
  unit_z.normalize();
  Eigen::Matrix3d rr;
  rr.block<3, 1>(0, 0) = unit_y.cross(unit_z);
  rr.block<3, 1>(0, 1) = unit_y;
  rr.block<3, 1>(0, 2) = unit_z;
  return Pose3d(Vector3d(tx, ty, tz), Quaternion(rr));
}

const Pose3d MakePose3dForwardUp(const Vector3d& translation, const Vector3d& unit_x,
                                 const Vector3d& unit_z) {
  Eigen::Matrix3d rr;
  rr.block<3, 1>(0, 0) = unit_x;
  rr.block<3, 1>(0, 1) = unit_z.cross(unit_x);
  rr.block<3, 1>(0, 2) = unit_z;
  return Pose3d(translation, Quaternion(rr));
}

const Pose3d MakePose3dForwardUp(double tx, double ty, double tz, double fx, double fy, double fz,
                                 double ux, double uy, double uz) {
  Vector3d unit_x(fx, fy, fz);
  unit_x.normalize();
  Vector3d unit_z(ux, uy, uz);
  unit_z.normalize();
  return MakePose3dForwardUp(Vector3d(tx, ty, tz), unit_x, unit_z);
}

const SphericalCoordinates CartesianToSpherical(const Vector3d& point) {
  SphericalCoordinates result;
  result.distance = point.norm();
  if (result.distance < kNearZero) {
    return result;
  }
  const Vector3d unit = point / result.distance;
  result.yaw = std::atan2(unit.y(), unit.x());
  result.pitch = std::atan2(unit.z(), std::sqrt(unit.x() * unit.x() + unit.y() * unit.y()));
  return result;
}

const Vector3d SphericalToCartesian(const SphericalCoordinates& point) {
  const double cospitch = std::cos(point.pitch);
  return point.distance * Vector3d(std::cos(point.yaw) * cospitch, std::sin(point.yaw) * cospitch,
                                   std::sin(point.pitch));
}

const Pose3d MakePose3dSquashXY(const Pose3d& other) {
  const double heading = UnitQuaternionToHeading(other.UnitQuaternion());
  return MakePose3dRollPitchYaw(other.Translation().x(), other.Translation().y(), 0.0, 0.0, 0.0,
                                heading);
}

const Pose2d MakePose2dSquashXY(const Pose3d& other) {
  const double heading = UnitQuaternionToHeading(other.UnitQuaternion());
  return MakePose2d(other.Translation().x(), other.Translation().y(), heading);
}

bool operator==(const MotionState& lhs, const MotionState& rhs) {
  return lhs.pose == rhs.pose && lhs.velocity == rhs.velocity &&
         lhs.acceleration == rhs.acceleration;
}

const MotionState Reexpress(const Pose3d& pose, const MotionState& motion_state) {
  // Note: no translational offset is applied when re-expressing velocity and acceleration.
  return {pose * motion_state.pose, pose * motion_state.velocity, pose * motion_state.acceleration};
}

const Screw ReexpressScrew(const Pose3d& pose, const Screw& screw) { return pose * screw; }

const MotionState MakeMotionStateFromState2d(const State2d& state) {
  Pose3d pose3d(Vector3d(state.Pose().translation().x(), state.Pose().translation().y(), 0),
                RollPitchYawToQuaternion(0, 0, ComputePose2dAngle(state.Pose())));

  Screw velocity(
      Vector3d(std::get<0>(state.VelocityScrew()).x(), std::get<0>(state.VelocityScrew()).y(), 0),
      Vector3d(0, 0, std::get<1>(state.VelocityScrew())));
  Screw acceleration(Vector3d(std::get<0>(state.AccelerationScrew()).x(),
                              std::get<0>(state.AccelerationScrew()).y(), 0),
                     Vector3d(0, 0, std::get<1>(state.AccelerationScrew())));
  return MotionState(pose3d, velocity, acceleration);
}

const State2d MakeState2dFromMotionState(const MotionState& motion_state) {
  const double heading = UnitQuaternionToHeading(motion_state.pose.UnitQuaternion());
  const Pose2d pose =
      MakePose2d(motion_state.pose.Translation().x(), motion_state.pose.Translation().y(), heading);
  const Screw2d velocity =
      MakeScrew2d(motion_state.velocity.translation.x(), motion_state.velocity.translation.y(),
                  motion_state.velocity.rotation.z());
  const Screw2d acceleration = MakeScrew2d(motion_state.acceleration.translation.x(),
                                           motion_state.acceleration.translation.y(),
                                           motion_state.acceleration.rotation.z());
  return State2d(pose, velocity, acceleration);
}

}  // namespace applied

namespace std {

ostream& operator<<(ostream& os, const applied::SphericalCoordinates& spherical) {
  os << spherical.distance << " " << spherical.yaw << " " << spherical.pitch;
  return os;
}

}  // namespace std
