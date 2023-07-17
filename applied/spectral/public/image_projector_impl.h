// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <string>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include "Eigen/Core"
#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/spectral/public/image_point.h"
// These will change without advance warning across releases. Your code will
// break if you make it depend on these.
namespace applied {
namespace implementation {
std::string ValidateSizeConsistency(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image);
class RectilinearParams {
 public:
  RectilinearParams(const ::simian_public::sensor_model::Description::CameraModel& camera_model);
  RectilinearParams(const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image);
  uint32_t image_width;
  uint32_t image_height;
  double fov_x;
  double fov_y;
  double fx;
  double fy;
  double cx;
  double cy;
  double crop_factor;
  Eigen::Matrix3d GetIntrinsicsMatrix() const;
  ImagePoint GetImageCenter() const;
};

class DistortionParams {
 public:
  static std::pair<DistortionParams, std::string> Parse(
      const simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
      const RectilinearParams& rectilinear_params);

  static std::pair<DistortionParams, std::string> Parse(
      const simian_public::sensor_model::Description::CameraModel& camera_model,
      const RectilinearParams& rectilinear_params);

  // These are taken from new-style radial_distortion_params.
  // NOTE: we keep only four of these values, whereas the YAML supports nine.
  // There's a reason for that.  Essentially a_0 is redundant with focal length or FOV, and the
  // a_{2n+1} values don't make physical sense.  For example, if you have a nonzero a_1,
  // then the mapping from 3D points to pixel points is not continuously differentiable at
  // the center of the image, which is the exact point where we would expect the lens to
  // be best behaved.  This isn't hard to change at some point in the future if we really
  // need to.
  const float a_2;
  const float a_4;
  const float a_6;
  const float a_8;
  const simian_public::sensor_model::CameraLensParams::RadialDistortion::RadialUnits radial_units;
  const bool has_radial_distortion;

  const std::vector<float> tangential_distortion;
  const std::vector<float> affine_scaling;
  const bool is_equidistant;
  const bool use_cubemap;
  const bool use_opencv;
  ::simian_public::camera_model::OpenCVDistortionParams opencv_distortion_params;

 private:
  DistortionParams(const simian_public::sensor_model::Description::CameraModel& camera_model,
                   const simian_public::sensor_model::CameraLensParams& lens_params);
};

class PinHoleProjector final {
 public:
  explicit PinHoleProjector(const RectilinearParams& params);

  // Returns false if the point cannot be projected into an image
  // (e.g. points behind a pinhole camera have no valid mapping).
  [[nodiscard]] bool Project(const ::applied::Vector3d& in_camera_frame,
                             ::applied::ImagePoint& out) const;
  [[nodiscard]] bool Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                               ::applied::Vector3d& in_camera_frame);

  [[nodiscard]] bool UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                     ::applied::Vector3d& in_camera_frame);

 private:
  Eigen::Matrix3d intrinsics_;
  Eigen::Matrix3d intrinsics_inverse_;
};

class RadialDistorter {
 public:
  RadialDistorter(float a_2, float a_4, float a_6, float a_8);
  bool has_radial_distortion() const { return has_radial_distortion_; }
  float max_r() const { return max_r_; }
  float scale_factor(float r) const;
  float distorted_r(float r) const;
  float inverse_distorted_r(float distorted_r) const;

 private:
  const float a_2_;
  const float a_4_;
  const float a_6_;
  const float a_8_;
  const bool has_radial_distortion_;
  float max_r_;

  // delta_r_ is the step size of r in distorted_r_graph_.
  const float delta_r_;
  std::vector<std::pair<float, float>> distorted_r_graph_;
  float distorted_r_prime(float r) const;
  float distorted_r_doubleprime(float r) const;
};

class Distorter {
 public:
  Distorter(const RectilinearParams& rectilinear_params, const DistortionParams& distortion_params);
  [[nodiscard]] bool ApplyDistortions(ImagePoint& image_point) const;
  [[nodiscard]] bool RemoveDistortions(ImagePoint& image_point) const;

 private:
  const RadialDistorter radial_distorter_;
  const DistortionParams distortion_params_;
  const RectilinearParams rectilinear_params_;
  const float crop_factor_;
  // TODO(mike):  Allow separate fov_x_ and fov_y_?
  const double fov_x_;
  const double width_;
  const double maxdim_;
  const double focal_length_;
  const bool use_opencv;
  const ImagePoint image_center_;
  [[nodiscard]] bool ApplyOpenCVDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool RemoveOpenCVDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeApplyTangentialDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeRemoveTangentialDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeApplyRadialDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeRemoveRadialDistortion(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeApplyAffineScaling(ImagePoint& diff) const;
  [[nodiscard]] bool MaybeRemoveAffineScaling(ImagePoint& diff) const;
  void MaybeConvertToEquidistant(ImagePoint& diff) const;
  void MaybeUnconvertToEquidistant(ImagePoint& diff) const;
  void MaybeCorrectForCrop(ImagePoint& diff) const;
  void MaybeUncorrectForCrop(ImagePoint& diff) const;
  bool MakeCameraMatrix(cv::Mat& camera_matrix) const;
  bool MakeDistortionMatrix(cv::Mat& distortion_params) const;
};

class CubeMapper {
 public:
  CubeMapper(const RectilinearParams& rectilinear_params,
             const DistortionParams& distortion_params);
  [[nodiscard]] bool Project(const Vector3d& point_in_camera_frame, ImagePoint& out) const;
  [[nodiscard]] bool Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                               ::applied::Vector3d& in_camera_frame);

  [[nodiscard]] bool UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                     ::applied::Vector3d& in_camera_frame);

  bool UseCubeMap() const { return use_cubemap_; }

 private:
  const bool use_cubemap_;
  const RectilinearParams rectilinear_params_;
  const ImagePoint image_center_;
  const RadialDistorter radial_distorter_;
  // Focal length in terms of x and y pixel dimension respectively.  Note this is different from the
  // corresponding values in rectilinear_params_;
  const double fx_;
  const double fy_;
};

}  // namespace implementation
}  // namespace applied
