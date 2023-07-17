// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <memory>
#include <string>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/spectral/public/image_point.h"
#include "applied/spectral/public/image_projector_impl.h"

namespace applied {
/**
 * @brief Class provided for transforming points between image space and 3D camera space.
 */
class ImageProjector final {
 public:
  /**
   * @brief Create an `ImageProjector` object.
   * @param camera_T_frame Transform from a given frame to the camera's frame.
   * @param camera_image Proto encoding an image and information about the parent abstract camera.
   * @return Pair of an `ImageProjector` and an error message.  The error message is empty on
   * success.
   */
  static std::pair<ImageProjector, std::string> Make(
      const Pose3d& camera_T_frame,
      const simian_public::sensor_model::SensorOutput_CameraImage& image);

  /**
   * @brief Create an `ImageProjector` object.
   * @param camera_T_frame Transform from a given frame to the camera's frame.
   * @param camera_model Proto encoding information about the abstract camera.
   * @return Pair of an `ImageProjector` and an error message.  The error message is empty on
   * success.
   */
  static std::pair<ImageProjector, std::string> Make(
      const Pose3d& camera_T_frame,
      const simian_public::sensor_model::Description::CameraModel& camera_model);

  ImageProjector& operator=(const ImageProjector& rhs);
  ImageProjector(ImageProjector&& rhs) = default;
  ImageProjector(const ImageProjector& rhs);

  /**
   * @brief Project a 3D point to image space.
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `ImageProjector` object was created.
   * @param out Image point in pixel coordinates.
   * @return `true` on success, `false` on error.
   * @details Returns `false` if 3D point cannot be projected using the camera model (for example,
   * if using a pinhole camera when the point is behind the image plane, or if the point is so far
   * off-center that the radial distortion model breaks down).  Does not return `false` simply
   * because the point in image space is outside the bounds of the image.
   */
  [[nodiscard]] bool Project(const ::applied::Vector3d& point_in_frame,
                             ::applied::ImagePoint& out) const;

  /**
   * @brief Inverse-project a point in image space to 3D space, at a depth measured by
   *        the radial distance from the origin of the camera frame.
   * @param image_point Image point in pixel coordinates.
   * @param radial_depth Distance from origin of camera frame.
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `ImageProjector` object was created.
   * @return `true` on success, `false` on error.
   */
  [[nodiscard]] bool Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                               ::applied::Vector3d& point_in_frame) const;

  /**
   * @brief Inverse-project a point in image space to 3D space, at a depth measured by
   *        the cartesian distance on the z axis from the origin of the camera frame.
   * @param image_point Image point in pixel coordinates.
   * @param z_depth Cartesian distance in the z axis from the camera frame.
   * @param point_in_frame Point in 3D coordinates in the given camera frame provided when the
   * `ImageProjector` object was created.
   * @return `true` on success, `false` on error.
   */
  [[nodiscard]] bool UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                     ::applied::Vector3d& point_in_frame) const;

  /**
   * @brief Calculates cartesian distance on the z axis from a 3D point to the camera frame origin.
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `ImageProjector` object was created.
   * @return double Distance from the point_in_frame to the camera on the z axis.
   */
  double ZDepthOfPointInFrame(const ::applied::Vector3d& point_in_frame) const;

  /**
   * @brief Calculates radial distance from a 3D point to the camera frame origin.
   * @param point_in_frame Point in 3D coordinates in the given frame provided when the
   * `ImageProjector` object was created.
   * @return double Distance from the point_in_frame to the camera on the z axis.
   */
  double RadialDepthOfPointInFrame(const ::applied::Vector3d& point_in_frame) const;

 private:
  ImageProjector() = default;
  ImageProjector(const Pose3d& camera_T_frame,
                 const ::applied::implementation::RectilinearParams& rectilinear_params,
                 const ::applied::implementation::DistortionParams& distortion_params);

  Pose3d camera_T_frame_;
  Pose3d frame_T_camera_;
  std::unique_ptr<::applied::implementation::PinHoleProjector> pin_hole_projector_;
  std::unique_ptr<::applied::implementation::Distorter> distorter_;
  std::unique_ptr<::applied::implementation::CubeMapper> cube_mapper_;
  bool use_opencv_;
};  // class ImageProjector

}  // namespace applied
