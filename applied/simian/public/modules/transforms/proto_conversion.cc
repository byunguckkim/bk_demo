// Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/simian/public/modules/transforms/proto_conversion.h"

#include <cmath>
#include <iostream>

namespace applied {

Vector3d FromProto(const simian_public::spatial::Point& point_proto) {
  return {point_proto.x(), point_proto.y(), point_proto.z()};
}

void ToProto(const Vector3d& vec, simian_public::spatial::Point* point_proto) {
  point_proto->set_x(vec.x());
  point_proto->set_y(vec.y());
  point_proto->set_z(vec.z());
}

bool FromProto(const simian_public::spatial::PoseSpec& in, Pose3d* out) {
  const Vector3d tt(in.px(), in.py(), in.pz());
  Quaternion qq;
  const auto orientation_case = in.orientation_case();
  switch (orientation_case) {
    case simian_public::spatial::PoseSpec::ORIENTATION_NOT_SET:
      // Historical note:  This branch used to throw an error, but there was code in the scenario
      // compiler that worked around the error by setting the field to `no_rotation` if it wasn't
      // set.  For testing convenience, let's just do what we would wind up doing in the actual
      // program anyway.
      qq = IdentityQuaternion();
      break;
    case simian_public::spatial::PoseSpec::kRpy: {
      const auto& rpy = in.rpy();
      qq = RollPitchYawToQuaternion(rpy.roll(), rpy.pitch(), rpy.yaw());
      break;
    }
    case simian_public::spatial::PoseSpec::kQuaternion: {
      const auto& quaternion = in.quaternion();
      if (std::isnan(quaternion.qw()) || std::isnan(quaternion.qx()) ||
          std::isnan(quaternion.qy()) || std::isnan(quaternion.qz())) {
        std::cerr << "ERROR:  quaternion with nan component specified (probably omitted).  All "
                     "quaternion components must be set."
                  << std::endl;
        return false;
      }
      qq = Quaternion(quaternion.qw(), quaternion.qx(), quaternion.qy(), quaternion.qz());
      // The tolerance of 1.0e-2 here is meant to cover you if you specify two significant figures
      // of each quaternion component; in that case we'll normalize it for you.  If you're less
      // accurate than that, you probably want to know.
      if (!QuaternionIsNormalized(qq, 1.0e-2)) {
        std::cerr << "ERROR: Unnormalized quaternion specified.  qw=" << qq.w() << " qx=" << qq.x()
                  << " qy=" << qq.y() << " qz=" << qq.z() << std::endl;
        return false;
      }
      break;
    }
    case simian_public::spatial::PoseSpec::kNoRotation:
      qq = IdentityQuaternion();
      break;
    default:
      std::cerr << "ERROR: unhandled "
                   "simian_public::spatial::PoseSpec::OrientationCase "
                << orientation_case << "\n";
      return false;
  }
  *out = Pose3d(tt, qq);
  return true;
}

void ToProto(const Pose3d& in, simian_public::spatial::PoseSpec* out) {
  const Vector3d& tt = in.Translation();
  out->set_px(tt.x());
  out->set_py(tt.y());
  out->set_pz(tt.z());

  const Quaternion& qq = in.UnitQuaternion();
  out->mutable_quaternion()->set_qw(qq.w());
  out->mutable_quaternion()->set_qx(qq.x());
  out->mutable_quaternion()->set_qy(qq.y());
  out->mutable_quaternion()->set_qz(qq.z());
}

void FromProtoNoCheck(const simian_public::spatial::Pose& in, Pose3d* out) {
  const Quaternion qq(in.qw(), in.qx(), in.qy(), in.qz());
  *out = Pose3d(Vector3d(in.px(), in.py(), in.pz()), qq);
}

bool FromProto(const simian_public::spatial::Pose& in, Pose3d* out) {
  const Quaternion qq(in.qw(), in.qx(), in.qy(), in.qz());
  if (qq.squaredNorm() < 1e-6) {
    // Too close to zero, this quaternion comes from a bad source.
    return false;
  }
  *out = Pose3d(Vector3d(in.px(), in.py(), in.pz()), qq);
  return true;
}

void ToProto(const Pose3d& in, simian_public::spatial::Pose* out) {
  const Vector3d& tt = in.Translation();
  const Quaternion& qq = in.UnitQuaternion();
  out->set_px(tt.x());
  out->set_py(tt.y());
  out->set_pz(tt.z());
  out->set_qw(qq.w());
  out->set_qx(qq.x());
  out->set_qy(qq.y());
  out->set_qz(qq.z());
}

bool FromProto(const simian_public::spatial::Screw& in, Screw* out) {
  out->translation << in.tx(), in.ty(), in.tz();
  out->rotation << in.rx(), in.ry(), in.rz();
  return true;
}

void ToProto(const Screw& in, simian_public::spatial::Screw* out) {
  out->set_tx(in.translation.x());
  out->set_ty(in.translation.y());
  out->set_tz(in.translation.z());
  out->set_rx(in.rotation.x());
  out->set_ry(in.rotation.y());
  out->set_rz(in.rotation.z());
}

bool FromProto(const simian_public::spatial::State& in, MotionState* out) {
  if (!FromProto(in.pose(), &out->pose)) {
    return false;
  }
  if (!FromProto(in.velocity(), &out->velocity)) {
    return false;
  }
  if (!FromProto(in.acceleration(), &out->acceleration)) {
    return false;
  }
  return true;
}

void ToProto(const MotionState& in, simian_public::spatial::State* out) {
  ToProto(in.pose, out->mutable_pose());
  ToProto(in.velocity, out->mutable_velocity());
  ToProto(in.acceleration, out->mutable_acceleration());
}

bool FromProto(const simian_public::spatial::TransformForest& in, TransformForest* out) {
  for (const auto& entry : in.entries()) {
    Pose3d src_T_dst;
    if (!FromProto(entry.src_t_dst(), &src_T_dst)) {
      return false;
    }
    if (!out->SetOrAdd(entry.src(), entry.dst(), src_T_dst)) {
      return false;
    }
  }
  for (const auto& spec : in.specs()) {
    Pose3d src_T_dst;
    if (!FromProto(spec.src_t_dst(), &src_T_dst)) {
      return false;
    }
    if (!out->SetOrAdd(spec.src(), spec.dst(), src_T_dst)) {
      return false;
    }
  }
  return true;
}

void ToProto(const TransformForest& in, simian_public::spatial::TransformForest* out) {
  TransformForest::VisitCallback cb = [out](const std::string& src, const std::string& dst,
                                            const Pose3d& src_T_dst) {
    auto* entry = out->add_entries();
    entry->set_src(src);
    entry->set_dst(dst);
    ToProto(src_T_dst, entry->mutable_src_t_dst());
  };
  in.Visit(cb);
}

void ToProtoDiff(const TransformForest& in, simian_public::spatial::TransformForest* out) {
  TransformForest::VisitCallback cb = [out](const std::string& src, const std::string& dst,
                                            const Pose3d& src_T_dst) {
    auto* entry = out->add_entries();
    entry->set_src(src);
    entry->set_dst(dst);
    ToProto(src_T_dst, entry->mutable_src_t_dst());
  };
  in.VisitDiff(cb);
}

bool FromProto(const simian_public::planar::State2d& in, State2d* out) {
  Screw2d velocity;
  if (in.has_velocity_screw()) {
    if (!FromProto(in.velocity_screw(), &velocity)) {
      return false;
    }
  } else {
    velocity = MakeScrew2d(in.velocity(), 0, in.yawrate());
  }

  Screw2d acceleration;
  if (in.has_acceleration_screw()) {
    if (!FromProto(in.acceleration_screw(), &acceleration)) {
      return false;
    }
  } else {
    acceleration = MakeScrew2d(in.acceleration(), 0, 0.0);
  }
  Pose2d pose;
  if (!FromProto(in.pose(), &pose)) {
    return false;
  }
  *out = State2d(pose, velocity, acceleration);
  return true;
}
void ToProto(const State2d& in, simian_public::planar::State2d* out) {
  out->set_acceleration(in.Acceleration());
  out->set_velocity(in.Velocity());
  out->set_yawrate(in.Yawrate());
  ToProto(in.Pose(), out->mutable_pose());
  ToProto(in.VelocityScrew(), out->mutable_velocity_screw());
  ToProto(in.AccelerationScrew(), out->mutable_acceleration_screw());
}

bool FromProto(const simian_public::planar::Screw2d& in, Screw2d* out) {
  std::get<0>(*out) << in.tx(), in.ty();
  std::get<1>(*out) = in.rz();
  return true;
}
void ToProto(const Screw2d& in, simian_public::planar::Screw2d* out) {
  out->set_tx(std::get<0>(in).x());
  out->set_ty(std::get<0>(in).y());
  out->set_rz(std::get<1>(in));
}

bool FromProto(const simian_public::planar::Pose2d& in, Pose2d* out) {
  *out = MakePose2d(in.x(), in.y(), in.heading());
  return true;
}
void ToProto(const Pose2d& in, simian_public::planar::Pose2d* out) {
  out->set_x(in.translation().x());
  out->set_y(in.translation().y());
  out->set_heading(ComputePose2dAngle(in));
}

Vector3d GetScaleFromScaledPoseSpec(
    const simian_public::spatial::ScaledPoseSpec& scaled_pose_spec) {
  using ScaleTypeCase = simian_public::spatial::ScaledPoseSpec::ScaleTypeCase;
  switch (scaled_pose_spec.scale_type_case()) {
    case ScaleTypeCase::SCALE_TYPE_NOT_SET:
      return Vector3d{1.0, 1.0, 1.0};
    case ScaleTypeCase::kScale:
      return Vector3d{scaled_pose_spec.scale(), scaled_pose_spec.scale(), scaled_pose_spec.scale()};
    case ScaleTypeCase::kScaleVector:
      return Vector3d{scaled_pose_spec.scale_vector().x(), scaled_pose_spec.scale_vector().y(),
                      scaled_pose_spec.scale_vector().z()};
  }
  // Should be unreachable
  throw std::runtime_error("Internal error:  unrecognized scale type");
  return Vector3d{0.0, 0.0, 0.0};
}

}  // namespace applied
