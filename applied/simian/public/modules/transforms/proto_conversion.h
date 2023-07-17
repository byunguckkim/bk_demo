// Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include "applied/simian/public/proto/planar.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"
#include "applied/simian/public/modules/transforms/planar.h"
#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/simian/public/modules/transforms/transform_forest.h"

namespace applied {

// Convert a point proto message to an applied::Vector3d.
Vector3d FromProto(const simian_public::spatial::Point& point_proto);
void ToProto(const Vector3d& vec, simian_public::spatial::Point* point_proto);

// TODO(fahhem,rolo-applied) Either add an appropriate form of
// [[nodiscard]] that is compatible with older compilers, or switch
// to exceptions for signalling conversion errors.
bool FromProto(const simian_public::spatial::PoseSpec& in, Pose3d* out);
// Populates the quaternion oneof in the output PoseSpec.
void ToProto(const Pose3d& in, simian_public::spatial::PoseSpec* out);

// Does not check the validity of the quaternion.
void FromProtoNoCheck(const simian_public::spatial::Pose& in, Pose3d* out);
// If the squared magnitude of the quaternion is less than 10^(-6), returns false,
// and `out` is not modified.
// Returns true otherwise.
// Note: this function does not ensure that the magnitude of the quaternion is close to 1.
bool FromProto(const simian_public::spatial::Pose& in, Pose3d* out);
void ToProto(const Pose3d& in, simian_public::spatial::Pose* out);

bool FromProto(const simian_public::spatial::Screw& in, Screw* out);
void ToProto(const Screw& in, simian_public::spatial::Screw* out);

bool FromProto(const simian_public::spatial::State& in, MotionState* out);
void ToProto(const MotionState& in, simian_public::spatial::State* out);

bool FromProto(const simian_public::spatial::TransformForest& in, TransformForest* out);
void ToProto(const TransformForest& in, simian_public::spatial::TransformForest* out);

bool FromProto(const simian_public::planar::State2d& in, State2d* out);
void ToProto(const State2d& in, simian_public::planar::State2d* out);

bool FromProto(const simian_public::planar::Screw2d& in, Screw2d* out);
void ToProto(const Screw2d& in, simian_public::planar::Screw2d* out);

bool FromProto(const simian_public::planar::Pose2d& in, Pose2d* out);
void ToProto(const Pose2d& in, simian_public::planar::Pose2d* out);

// Populate the proto with entries for only the links in the transform forest that have
// be updated since the most recent call to IncrementVersion() on the transform forest.
void ToProtoDiff(const TransformForest& in, simian_public::spatial::TransformForest* out);

// Recover the scales for the x, y, and z coordinates from a `ScaledPoseSpec`.
// Defaults to a scale of 1.0 for each coordinate if scale is not present.
Vector3d GetScaleFromScaledPoseSpec(const simian_public::spatial::ScaledPoseSpec& scaled_pose_spec);

}  // namespace applied
