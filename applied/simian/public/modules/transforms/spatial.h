// Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <iosfwd>

#include "applied/simian/public/modules/transforms/eigen.h"
#include "applied/simian/public/modules/transforms/planar.h"

namespace applied {

using Vector3d = Eigen::Matrix<double, 3, 1, Eigen::DontAlign>;

inline const Vector3d MakeVector3dNormalized(const Vector3d& v) {
  const double sqn = v.squaredNorm();
  if (sqn < 1e-12) {
    return Vector3d(0.0, 0.0, 0.0);
  }
  return v / std::sqrt(sqn);
}

// Note that the Eigen Quaternion ctor takes (w, x, y, z) as
// arguments, in that order. And their data storage implementation
// uses [x, y, z, w] but I don't expect us to be exposed to that
// difference, as long as we don't use the raw data ctor.
using Quaternion = Eigen::Quaternion<double, Eigen::DontAlign>;

inline Quaternion operator+(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(q1.w() + q2.w(), q1.x() + q2.x(), q1.y() + q2.y(), q1.z() + q2.z());
}

inline Quaternion operator-(const Quaternion& q1, const Quaternion& q2) {
  return Quaternion(q1.w() - q2.w(), q1.x() - q2.x(), q1.y() - q2.y(), q1.z() - q2.z());
}

inline Quaternion operator*(double coeff, const Quaternion& qq) {
  return Quaternion(coeff * qq.w(), coeff * qq.x(), coeff * qq.y(), coeff * qq.z());
}

inline const Quaternion IdentityQuaternion() { return Quaternion(1.0, 0.0, 0.0, 0.0); }

// Returns true if the magnitude of the quaternion is within tolerance of 1.0.
// tolerance should be in [0, 1), but this is not checked.
bool QuaternionIsNormalized(const Quaternion& qq, double tolerance);

// This function returns true if the two quaternions specify exactly
// the same orientation/rotation.
// Precondition: both quaternions are normalized.
// The function accounts for the fact
// that there are two ways to represent a single rotation using
// quaternions. Comparing (w, x, y, z) and (-w, -x, -y, -z) returns true.
bool SameOrientation(const Quaternion& lhs, const Quaternion& rhs);

// This function returns true if the two quaternions are within an epsilon angle distance of each
// other.
// Precondition: both quaternions are normalized.
// angle_tol: Tolerance for the angle between lhs and rhs quaternions.
bool AlmostEqual(const Quaternion& lhs, const Quaternion& rhs, double angle_tol);

// The axis does NOT need to be normalized. If the axis is
// practically zero, identity is returned.
const Quaternion AxisAngleToQuaternion(const Vector3d& axis, double angle);

inline const Quaternion AxisAngleToQuaternion(double ax, double ay, double az, double angle) {
  return AxisAngleToQuaternion(Vector3d(ax, ay, az), angle);
}

// Create a quaternion from roll, pitch, and yaw angles,
// applied in a Z-Y'-X'' (yaw-pitch-roll) rotation sequence.
// See the description of UnitQuaternionToRollPitchYaw() for a description of this convention.
// Typically this is the commonly used euler angle -> quaternion transformation.
const Quaternion RollPitchYawToQuaternion(double roll, double pitch, double yaw);

// Create a quaternion from pitch, roll, and yaw angles,
// applied in a Z-X'-Y'' (yaw-roll-pitch) rotation sequence.
const Quaternion PitchRollYawToQuaternion(double pitch, double roll, double yaw);

// Compute the roll angle (corresponds to X'' in the Z-Y'-X'' rotation
// sequence). If you need all 3 RPY angles, or if you want more robust
// handling of gymbal lock, use UnitQuaternionToRollPitchYaw()
// instead.
//
// Precondition: qq is a unit quaternion (this is not checked).
inline double UnitQuaternionToRoll(const Quaternion& qq) {
  return std::atan2(2.0 * (qq.w() * qq.x() + qq.y() * qq.z()),
                    1.0 - 2.0 * (qq.x() * qq.x() + qq.y() * qq.y()));
}

// Compute the pitch angle (corresponds to Y' in the Z-Y'-X'' rotation
// sequence). If you need all 3 RPY angles, or if you want more robust
// handling of gymbal lock, use UnitQuaternionToRollPitchYaw()
// instead. This function here has a simpler heuristic to avoid NaN.
//
// Because the order of rotations is yaw, pitch, roll, this pitch
// is also the angle from the XY plane of frame A to the x axis of frame B.
//
// Precondition: a_q_b is a unit quaternion (this is not checked).
inline double UnitQuaternionToPitch(const Quaternion& a_q_b) {
  const double sin_pitch = 2.0 * (a_q_b.w() * a_q_b.y() - a_q_b.z() * a_q_b.x());
  if (sin_pitch > 1.0) {
    return M_PI / 2.0;
  }
  if (sin_pitch < -1.0) {
    return -M_PI / 2.0;
  }
  return std::asin(sin_pitch);
}

// Compute the yaw angle (corresponds to Z in the Z-Y'-X'' rotation
// sequence). If you need all 3 RPY angles, or if you want more robust
// handling of gymbal lock, use UnitQuaternionToRollPitchYaw()
// instead.
//
// Note that "yaw" is exactly the same as "heading" (see
// UnitQuaternionToHeading()).
//
// Precondition: qq is a unit quaternion (this is not checked).
inline double UnitQuaternionToYaw(const Quaternion& qq) {
  return std::atan2(2.0 * (qq.w() * qq.z() + qq.x() * qq.y()),
                    1.0 - 2.0 * (qq.y() * qq.y() + qq.z() * qq.z()));
}

// Compute the angles required to obtain the given rotation, if we
// were to rotate first by yaw around Z, then by pitch around the
// rotated Y, and finally by roll around the rotated X. This is a
// Z-Y'-X'' convention (if you're familiar with Tait-Bryan that may
// help, just keep in mind in Simian the global Z is pointing up).
//
// You should generally avoid using RPY because of its problems with
// gymbal lock (pitch angles close to +pi/2 or -pi/2). Use quaternions
// instead. Note that it is OK to use RPY for /specifying/
// orientations. But if you go from quaternion to RPY, getting near to
// this singularity can be a real problem: as you get closer to the
// singularity, tiny changes in input can lead to large changes in
// output, so floating point roundoff gets amplified. In practice,
// this is especially bad if you then tweak the RPY a bit and convert
// back to quaternions: you may well end up in a spot that is farther
// removed from the starting point than intended.
//
// To alleviate the numerical issues around gymbal lock, this function
// handles abs(pitch) close to pi/2 by setting roll=0 and snapping
// pitch to +pi/2 or -pi/2. I.e. the computed yaw then represents the
// "interesting" bit of the rotation. Note that the unit tests for
// this function check that this approximation is within one nanometer
// if applied to a lever arm of one meter.
//
// If you need only individual angles, AND you do not need to worry
// about gymbal lock, then UnitQuaternionToRoll(),
// UnitQuaternionToPitch(), or UnitQuaternionToYaw() can be more
// convenient.
//
// Precondition: quaternion is a unit quaternion (this is not checked).
const Vector3d UnitQuaternionToRollPitchYaw(const Quaternion& quaternion);

// Note that the returned axis is always normalized (and there is a
// test for that).
//
// Precondition: quaternion is a unit quaternion (this is not checked).
void UnitQuaternionToAxisAngle(const Quaternion& quaternion, Vector3d* axis, double* angle);

// Convenience method for conversions from 3D to 2D. Based on
// projecting the unit_x vector onto the XY plane, so it is pretty
// robust even at significant pitch and roll. Assumes the quaternion
// is normalized.
//
// This is equivalent to UnitQuaternionToYaw() and maintained for historical reasons.
//
// Precondition: unit_quaternion is a unit quaternion (this is not checked).
double UnitQuaternionToHeading(const Quaternion& unit_quaternion);

// Computes a 3D vector that is a rotational velocity (omega) in the
// following sense: applying that omega for a duration of 1s will
// rotate a body such that its orientation ends up at `dst`,
// assuming it was `src` at the beginning.
//
// The returned omega is expressed in implicit outer frame (e.g. "map"
// in typical use cases). That is the frame in which both `src` and
// `dst` orientations are expressed.
//
// Precondition: src and dst are both unit quaternions (this is not checked).
const Vector3d UnitQuaternionToRotationVector(const Quaternion& src, const Quaternion& dst);

// Compute the orientation that results from applying the given a
// rotational velocity (usually termed "omega") over the given
// duration. Basically the inverse of
// UnitQuaternionToRotationVector(), i.e. it returns the "dst" of
// that.
//
// Precondition: src is a unit quaternion (this is not checked).
const Quaternion IntegrateRotationalVelocity(const Quaternion& src, const Vector3d& omega,
                                             double duration);

// Screws can represent spatial (linear and angular) velocities,
// accelerations, and forces.
struct Screw {
  Screw() : translation(0.0, 0.0, 0.0), rotation(0.0, 0.0, 0.0) {}
  Screw(const Vector3d& tra, const Vector3d& rot) : translation(tra), rotation(rot) {}

  // This vector holds a translational or linear quantity.
  Vector3d translation;
  // This vector holds a rotational or angular quantity.
  Vector3d rotation;
};

inline Screw operator*(const Quaternion& qq, const Screw& ss) {
  return Screw(qq * ss.translation, qq * ss.rotation);
}

inline bool operator==(const Screw& lhs, const Screw& rhs) {
  return lhs.translation == rhs.translation && lhs.rotation == rhs.rotation;
}

inline bool operator!=(const Screw& lhs, const Screw& rhs) { return !(lhs == rhs); }

// This function returns true if the two screws are almost equal by checking if
// the norms of their translations and rotations are within epsilon of each other
inline bool AlmostEqual(const Screw& lhs, const Screw& rhs, double epsilon) {
  return (lhs.translation - rhs.translation).norm() <= epsilon &&
         (lhs.rotation - rhs.rotation).norm() <= epsilon;
}

class Pose3d {
 public:
  // Default ctor initializes to identity.
  Pose3d();

  // The quaternion gets normalized for you. If you pass something
  // that cannot be normalized, the identity quaternion is used
  // instead.
  Pose3d(const Vector3d& translation, const Quaternion& quaternion);

  const Pose3d Inverse() const;
  // Re-express a position vector from the frame described by this
  // pose to the parent frame of this pose.
  //
  // Let world_T_ego be a Pose3d defining the transform from the world
  // frame to the ego frame.
  // Let sensor_pos_in_ego be a Vector3d providing the sensor's mounting
  // position expressed in the ego frame.
  // Let sensor_pos_in_world be a Vector3d providing the sensor's mounting
  // position expressed in the world frame.
  // This operator is used as follows:
  //     Vector3d sensor_pos_in_world = world_T_ego * sensor_pos_in_ego;
  //
  // Do not use this operator for velocities, accelerations, etc.
  const Vector3d operator*(const Vector3d& rhs_vector) const;
  // Re-express a screw from the frame described by this pose
  // to the parent frame of this pose.
  //
  // Let world_T_ego be a Pose3d defining the transform from the world
  // frame to the ego frame.
  // Let sensor_vel_in_ego be a Screw providing the linear velocity of the
  // sensor mounting point and the angular velocity of the ego, expressed
  // in the ego frame.
  // Let sensor_vel_in_world be a Screw providing the linear velocity of the
  // sensor mounting point and the angular velocity of the ego, expressed
  // in the world frame.
  // This operator is used as follows:
  //     Screw sensor_vel_in_world = world_T_ego * sensor_vel_in_ego;
  //
  // This operator does *not* add in the pose's translation.
  const Screw operator*(const Screw& rhs_screw) const;

  // If this Pose3d object represents the transform a_T_b and rhs_pose represents the transform
  // b_T_c, this operator returns a_T_c = a_T_b * b_T_c.
  const Pose3d operator*(const Pose3d& rhs_pose) const;

  const Vector3d& Translation() const { return translation_; }
  const Quaternion& UnitQuaternion() const { return unit_quaternion_; }

  // This does not change the rotation encoded by the unit quaternion.
  void NegateQuaternionInplace() { unit_quaternion_ = -1.0 * unit_quaternion_; }

  // Non-const access only for translation. Because they can be
  // trivially changed. The same is not true for rotations. If needed,
  // provide a way to overwrite the quaternion wholesale, and in that
  // method enforce that it is a unit quaternion.
  Vector3d& Translation() { return translation_; }

  // If this pose represents the transform a_T_b, this function returns the x unit vector of frame b
  // expressed in frame a.
  // For example, if a_T_b = world_T_ego, this function returns a unit vector that coincides with
  // the (+x) longitudinal axis of the ego, expressed in the world frame.
  Vector3d UnitX() const { return unit_quaternion_._transformVector(Vector3d::UnitX()); }
  // If this pose represents the transform a_T_b, this function returns the y unit vector of frame b
  // expressed in frame a.
  // For example, if a_T_b = world_T_ego, this function returns a unit vector that coincides with
  // the (+y) lateral axis of the ego, expressed in the world frame.
  Vector3d UnitY() const { return unit_quaternion_._transformVector(Vector3d::UnitY()); }
  // If this pose represents the transform a_T_b, this function returns the z unit vector of frame b
  // expressed in frame a.
  // For example, if a_T_b = world_T_ego, this function returns a unit vector that coincides with
  // the (+z) vertical axis of the ego, expressed in the world frame.
  Vector3d UnitZ() const { return unit_quaternion_._transformVector(Vector3d::UnitZ()); }

 private:
  Vector3d translation_;
  Quaternion unit_quaternion_;
};

inline bool operator==(const Pose3d& lhs, const Pose3d& rhs) {
  return lhs.Translation() == rhs.Translation() &&
         SameOrientation(lhs.UnitQuaternion(), rhs.UnitQuaternion());
}

inline bool operator!=(const Pose3d& lhs, const Pose3d& rhs) { return !(lhs == rhs); }

inline bool AlmostEqual(const Pose3d& lhs, const Pose3d& rhs, double epsilon) {
  return (lhs.Translation() - rhs.Translation()).norm() <= epsilon &&
         AlmostEqual(lhs.UnitQuaternion(), rhs.UnitQuaternion(), epsilon);
}

// Thin wrapper around Pose3d ctor. Returns a 3D transform based on a
// (tx,ty,tz) translation and a quaternion specified as (qw,qx,qy,qz).
inline const Pose3d MakePose3dQuaternion(double tx, double ty, double tz, double qw, double qx,
                                         double qy, double qz) {
  return Pose3d(Vector3d(tx, ty, tz), Quaternion(qw, qx, qy, qz));
}

// Returns a 3D transform based on a translation and a rotation around
// an axis by the given angle. The axis gets normalized for you, and
// if its norm is too small it is ignored and a pure translation
// returned instead.
inline const Pose3d MakePose3dAxisAngle(const Vector3d& translation, const Vector3d& axis,
                                        double angle) {
  return Pose3d(translation, AxisAngleToQuaternion(axis, angle));
}

inline const Pose3d MakePose3dAxisAngle(double tx, double ty, double tz, double axis_x,
                                        double axis_y, double axis_z, double angle) {
  return Pose3d(Vector3d(tx, ty, tz), AxisAngleToQuaternion(axis_x, axis_y, axis_z, angle));
}

// The orientation is determined by applying yaw around Z, then
// pitch around the rotated Y, then roll around the rotated X.
inline const Pose3d MakePose3dRollPitchYaw(double tx, double ty, double tz, double roll,
                                           double pitch, double yaw) {
  return Pose3d(Vector3d(tx, ty, tz), RollPitchYawToQuaternion(roll, pitch, yaw));
}

// The orientation is specified by the left-pointing vector (lx,ly,lz)
// and the up-pointing vector (ux,uy,uz). Both those vectors get
// normalized to yield unit Y and Z, which then defines unit X.
//
// Precondition: (lx,ly,lz) and (ux,uy,uz) must be non-zero and
// perpendicular to each other. This is NOT checked by the
// implementation.
const Pose3d MakePose3dLeftUp(double tx, double ty, double tz, double lx, double ly, double lz,
                              double ux, double uy, double uz);

// The orientation is specified by the unit vectors of the rotated X and Z axes.
//
// Precondition: unit_x and unit_z must be perpendicular to each other
// and each of length one. This is NOT checked by the implementation.
const Pose3d MakePose3dForwardUp(const Vector3d& translation, const Vector3d& unit_x,
                                 const Vector3d& unit_z);

// The orientation is specified by the forward-pointing vector
// (fx,fy,fz) and the up-pointing vector (ux,uy,uz). Both those
// vectors get normalized to yield unit X and Z, and the call gets
// forwarded to the vector-based MakePose3dForwardUp() above.
//
// Precondition: (fx,fy,fz) and (ux,uy,uz) must be non-zero and
// perpendicular to each other. This is NOT checked by the
// implementation.
const Pose3d MakePose3dForwardUp(double tx, double ty, double tz, double fx, double fy, double fz,
                                 double ux, double uy, double uz);

// Convert a Pose2d into a Pose3d.
const Pose3d MakePose3dPlanarXY(const Pose2d& pose2d);

inline const Pose3d MakePose3dTranslation(const Vector3d& tt) {
  return Pose3d(tt, IdentityQuaternion());
}

inline const Pose3d MakePose3dTranslation(double x, double y, double z) {
  return Pose3d(Vector3d(x, y, z), IdentityQuaternion());
}

// Discard non-planar information from a Pose3d. The approximation is
// acceptable when pitch and roll are both very small.
const Pose3d MakePose3dSquashXY(const Pose3d& other);

// Create a Pose2d by discarding non-planar information from a
// Pose3d. The approximation is acceptable when pitch and roll are
// both very small.
const Pose2d MakePose2dSquashXY(const Pose3d& other);

struct SphericalCoordinates {
  SphericalCoordinates() : distance(0.0), yaw(0.0), pitch(0.0) {}
  SphericalCoordinates(double distance_, double yaw_, double pitch_)
      : distance(distance_), yaw(yaw_), pitch(pitch_) {}

  double distance;
  double yaw;    // rotation around Z
  double pitch;  // elevation above XY plane
};

// Transforms the given Cartesian point to spherical coordinates,
// with distance>=0, -pi<=yaw<=pi, and -pi/2<=pitch<=pi/2.
const SphericalCoordinates CartesianToSpherical(const Vector3d& point);

// Transform the given spherical coordinates to a Cartesian
// point. Happily accepts spherical coordinate ranges outside of the
// ones returned by CartesianToSpherical() -- it is up to you to make
// sure your code is not surprised by that.
const Vector3d SphericalToCartesian(const SphericalCoordinates& point);

struct MotionState {
  MotionState() : pose(), velocity(), acceleration() {}
  MotionState(const Pose3d& pp, const Screw& vv, const Screw& aa)
      : pose(pp), velocity(vv), acceleration(aa) {}

  Pose3d pose;
  Screw velocity;
  Screw acceleration;
};

// Creates a 3D MotionState from a State2d with zero elevation, roll, and pitch.
const MotionState MakeMotionStateFromState2d(const State2d& state2d);
// Creates a State2d from a MotionState by the z component.
const State2d MakeState2dFromMotionState(const MotionState& motion_state);

bool operator==(const MotionState& lhs, const MotionState& rhs);

inline bool operator!=(const MotionState& lhs, const MotionState& rhs) { return !(lhs == rhs); }

// Re-express a MotionState from the frame described by the provided pose to the parent frame
// of the pose.
//
// Let a_T_b be a Pose3d defining the transform from frame A to frame B.
// Let b_S_c be a MotionState providing the state of frame/body C expressed in the frame B.
// Let a_S_c be a MotionState providing the state of frame/body C expressed in the frame A.
// This function is used as follows:
//     Screw a_S_c = Reexpress(a_T_b, b_S_c);
//
// The translational offset between frames is included when re-expressing the MotionState's pose,
// but not when re-expressing the MotionState's velocity or acceleration.
//
// Note: This function does not affect the frame in which velocities and accelerations are observed.
//
// Returns a_S_c
const MotionState Reexpress(const Pose3d& a_T_b, const MotionState& b_S_c);

// Re-express a Screw from the frame described by the provided pose to the parent frame
// of the pose.
//
// Let a_T_b be a Pose3d defining the transform from frame A to frame B.
// Let b_S_c be a Screw providing the state of frame/body C expressed in the frame B.
// Let a_S_c be a Screw providing the state of frame/body C expressed in the frame A.
// This function is used as follows:
//     Screw a_S_c = ReexpressScrew(a_T_b, b_S_c);
//
// Returns a_S_c
const Screw ReexpressScrew(const Pose3d& a_T_b, const Screw& b_S_c);

// This function relates the linear velocity of two points fixed to a frame.
// Given the velocity observed in frame N of point B0 fixed in frame B (c_basis_n_v_b0),
// obtain the velocity observed in frame N of point B1 also fixed to frame B (c_basis_n_v_b1).
// This calculation requires the angular velocity of frame B in frame N (c_basis_n_w_b),
// and the position of B1 from B0 (c_basis_b0_p_b1).
// All quantities must be expressed in the same frame (named C in the parameters,
// which could be N, B, or any other frame);
// the returned velocity is expressed in the same frame as the arguments.
//
// We use the notation <ExpressedIn>_basis_<ObservedIn>_<QuantityType>_<Target>.
inline const Vector3d LinearVelocityTwoPointsFixed(const Vector3d& c_basis_n_v_b0,
                                                   const Vector3d& c_basis_b0_p_b1,
                                                   const Vector3d& c_basis_n_w_b) {
  return c_basis_n_v_b0 + c_basis_n_w_b.cross(c_basis_b0_p_b1);
}

// This function relates the linear acceleration of two points fixed to a frame.
// Given the acceleration observed in frame N of point B0 fixed in frame B (c_basis_n_a_b0),
// obtain the acceleration observed in frame N of point B1 also fixed to frame B (c_basis_n_a_b1).
// This calculation requires the angular velocity (c_basis_n_w_b) and
// angular acceleration (c_basis_n_alpha_b) of frame B in frame N,
// and the position of B1 from BO (c_basis_b0_p_b1).
// All quantities must be expressed in the same frame (named C in the parameters,
// which could be N, B, or any other frame);
// the returned acceleration is expressed in the same frame as the arguments.
//
// We use the notation <ExpressedIn>_basis_<ObservedIn>_<QuantityType>_<Target>.
inline const Vector3d LinearAccelerationTwoPointsFixed(const Vector3d& c_basis_n_a_b0,
                                                       const Vector3d& c_basis_b0_p_b1,
                                                       const Vector3d& c_basis_n_w_b,
                                                       const Vector3d& c_basis_n_alpha_b) {
  return c_basis_n_a_b0 + c_basis_n_alpha_b.cross(c_basis_b0_p_b1) +
         c_basis_n_w_b.cross(c_basis_n_w_b.cross(c_basis_b0_p_b1));
}

}  // namespace applied

namespace std {

ostream& operator<<(ostream& os, const applied::SphericalCoordinates& spherical);

}  // namespace std
