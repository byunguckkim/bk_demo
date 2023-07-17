// Copyright (C) 2022 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <memory>
#include <string>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/spectral/public/image_point.h"
#include "applied/spectral/public/image_projector.h"

namespace applied {

/** @brief Object to convert between 3D points, and image points with pixel values encoding depth.
 */
class DepthImageProjector {
 public:
  /**
   * @brief Create a `DepthImageProjector` object.
   * @param camera_T_frame Transform from a given frame to the camera's frame.
   * @param camera_model Proto encoding information about the abstract camera.
   * @return Pair of a `DepthImageProjector` and an error message.  The error message is empty on
   * success.
   */
  static std::pair<DepthImageProjector, std::string> Make(
      const Pose3d& camera_T_frame,
      const simian_public::sensor_model::Description::CameraModel& camera_model);

  DepthImageProjector& operator=(const DepthImageProjector& rhs) = default;
  DepthImageProjector(DepthImageProjector&& rhs) = default;
  DepthImageProjector(const DepthImageProjector& rhs) = default;

  /**
   * @brief Project a 3D point to image space, and compute pixel value representing depth.
   *
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `DepthImageProjector` object was created.
   * @param out Image point in pixel coordinates.
   * @param pixel_value Pixel value encoding depth.
   * @return `true` on success, `false` on error.
   * @details Returns `false` if 3D point cannot be projected using the camera model (for example,
   * if using a pinhole camera when the point is behind the image plane, or if the point is so far
   * off-center that the radial distortion model breaks down).  Does not return `false` simply
   * because the point in image space is outside the bounds of the image.
   *
   * The coding of depth as a pixel value depends on the `DepthParams` in the `camera_model` used to
   * create the `DepthImageProjector`. */
  [[nodiscard]] bool Project(const ::applied::Vector3d& point_in_frame, ::applied::ImagePoint& out,
                             double& pixel_value) const;

  /**
   * @brief Inverse-project a point in image space to 3D space, using depth encoded by a pixel
   * value.
   * @param image_point Image point in pixel coordinates.
   * @param pixel_value Pixel value encoding depth.
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `DepthImageProjector` object was created.
   * @return `true` on success, `false` on error.
   */
  [[nodiscard]] bool Unproject(const ::applied::ImagePoint& image_point, double pixel_value,
                               ::applied::Vector3d& point_in_frame) const;

  // public for testing
  double PixelValueToDepth(double pixel_value) const;
  // public for testing
  double DepthToPixelValue(double depth) const;

 private:
  // Bogus default constructor for error cases.
  DepthImageProjector()
      : bit_depth_(0),
        max_pixel_value_(0.0),
        granularity_(0.0),
        log_of_base_(0.0),
        unit_is_uint_(false),
        use_radial_depth_(false) {}
  DepthImageProjector(
      ImageProjector&& image_projector,
      const simian_public::sensor_model::CameraSystemParams::DepthParams& depth_params,
      int original_sensor_color_depth, bool unit_is_uint, bool use_radial_depth)
      : image_projector_(new ImageProjector(image_projector)),
        depth_params_(depth_params),
        bit_depth_((depth_params_.bit_depth() > 0) ? depth_params_.bit_depth()
                                                   : original_sensor_color_depth),
        max_pixel_value_(pow(2.0, bit_depth_) - 1),
        granularity_(1.0 / max_pixel_value_),
        log_of_base_(std::log(depth_params_.log_base())),
        unit_is_uint_(unit_is_uint),
        use_radial_depth_(use_radial_depth) {}

  std::unique_ptr<ImageProjector> image_projector_;
  const simian_public::sensor_model::CameraSystemParams::DepthParams depth_params_;
  const int bit_depth_;
  const double max_pixel_value_;
  const double granularity_;
  const double log_of_base_;
  const bool unit_is_uint_;
  const bool use_radial_depth_;
};  // class DepthImageProjector

}  // namespace applied
