// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/image_projector.h"

namespace applied {
using namespace ::applied::implementation;

ImageProjector::ImageProjector(const Pose3d& camera_T_frame,
                               const RectilinearParams& rectilinear_params,
                               const DistortionParams& distortion_params)
    : camera_T_frame_(camera_T_frame),
      frame_T_camera_(camera_T_frame_.Inverse()),
      pin_hole_projector_(std::make_unique<PinHoleProjector>(rectilinear_params)),
      distorter_(std::make_unique<Distorter>(rectilinear_params, distortion_params)),
      cube_mapper_(std::make_unique<CubeMapper>(rectilinear_params, distortion_params)),
      use_opencv_(distortion_params.use_opencv) {}

ImageProjector& ImageProjector::operator=(const ImageProjector& rhs) {
  camera_T_frame_ = rhs.camera_T_frame_;
  frame_T_camera_ = rhs.frame_T_camera_;
  use_opencv_ = rhs.use_opencv_;
  pin_hole_projector_.reset();
  if (rhs.pin_hole_projector_) {
    pin_hole_projector_ = std::make_unique<PinHoleProjector>(*rhs.pin_hole_projector_);
  }
  distorter_.reset();
  if (rhs.distorter_) {
    distorter_ = std::make_unique<Distorter>(*rhs.distorter_);
  }
  cube_mapper_.reset();
  if (rhs.cube_mapper_) {
    cube_mapper_ = std::make_unique<CubeMapper>(*rhs.cube_mapper_);
  }
  return *this;
}

ImageProjector::ImageProjector(const ImageProjector& rhs)
    : camera_T_frame_(rhs.camera_T_frame_),
      frame_T_camera_(rhs.frame_T_camera_),
      use_opencv_(rhs.use_opencv_) {
  if (rhs.pin_hole_projector_) {
    pin_hole_projector_ = std::make_unique<PinHoleProjector>(*rhs.pin_hole_projector_);
  }
  if (rhs.distorter_) {
    distorter_ = std::make_unique<Distorter>(*rhs.distorter_);
  }
  if (rhs.cube_mapper_) {
    cube_mapper_ = std::make_unique<CubeMapper>(*rhs.cube_mapper_);
  }
}

std::pair<ImageProjector, std::string> ImageProjector::Make(
    const Pose3d& camera_T_frame,
    const simian_public::sensor_model::Description::CameraModel& camera_model) {
  RectilinearParams rectilinear_params(camera_model);
  const auto parse_result = DistortionParams::Parse(camera_model, rectilinear_params);
  const DistortionParams& distortion_params = parse_result.first;
  const std::string& err_msg = parse_result.second;
  if (!err_msg.empty()) {
    return {{}, err_msg};
  }
  return {{camera_T_frame, rectilinear_params, distortion_params}, ""};
}

std::pair<ImageProjector, std::string> ImageProjector::Make(
    const Pose3d& camera_T_frame,
    const simian_public::sensor_model::SensorOutput::CameraImage& image) {
  std::string validate_size_msg = ValidateSizeConsistency(image);
  if (!validate_size_msg.empty()) return {{}, validate_size_msg};
  return Make(camera_T_frame, image.metadata().sensor_description().camera_model());
}

bool ImageProjector::Project(const ::applied::Vector3d& point_in_frame,
                             ::applied::ImagePoint& out) const {
  if (!pin_hole_projector_ || !distorter_ || !cube_mapper_) {
    // User has failed to check an error returned from Make() above.
    return false;
  }
  const Vector3d point_in_camera = camera_T_frame_ * point_in_frame;
  if (cube_mapper_->UseCubeMap()) {
    return cube_mapper_->Project(point_in_camera, out);
  }

  return pin_hole_projector_->Project(point_in_camera, out) && distorter_->ApplyDistortions(out);
}

bool ImageProjector::Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                               ::applied::Vector3d& point_in_frame) const {
  if (!pin_hole_projector_ || !distorter_ || !cube_mapper_) {
    // User has failed to check an error returned from Make() above.
    return false;
  }
  Vector3d point_in_camera;
  if (cube_mapper_->UseCubeMap()) {
    if (!cube_mapper_->Unproject(image_point, radial_depth, point_in_camera)) {
      return false;
    }
  } else {
    ::applied::ImagePoint image_point_undistorted = image_point;
    if (!distorter_->RemoveDistortions(image_point_undistorted)) {
      return false;
    }
    if (!pin_hole_projector_->Unproject(image_point_undistorted, radial_depth, point_in_camera)) {
      return false;
    }
  }
  point_in_frame = frame_T_camera_ * point_in_camera;
  return true;
}

bool ImageProjector::UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                     ::applied::Vector3d& point_in_frame) const {
  if (!pin_hole_projector_ || !distorter_ || !cube_mapper_) {
    // User has failed to check an error returned from Make() above.
    return false;
  }
  Vector3d point_in_camera;
  if (cube_mapper_->UseCubeMap()) {
    if (!cube_mapper_->UnprojectZDepth(image_point, z_depth, point_in_camera)) {
      return false;
    }
  } else {
    ::applied::ImagePoint image_point_undistorted = image_point;
    if (!distorter_->RemoveDistortions(image_point_undistorted)) {
      return false;
    }
    if (!pin_hole_projector_->UnprojectZDepth(image_point_undistorted, z_depth, point_in_camera)) {
      return false;
    }
  }
  point_in_frame = frame_T_camera_ * point_in_camera;
  return true;
}

double ImageProjector::ZDepthOfPointInFrame(const ::applied::Vector3d& point_in_frame) const {
  return (camera_T_frame_ * point_in_frame).x();
}

double ImageProjector::RadialDepthOfPointInFrame(const ::applied::Vector3d& point_in_frame) const {
  return (camera_T_frame_ * point_in_frame).norm();
}

}  // namespace applied
