// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/image_projector_impl.h"

namespace applied {
namespace implementation {

RectilinearParams::RectilinearParams(
    const ::simian_public::sensor_model::Description::CameraModel& camera_model) {
  const simian_public::sensor_model::StandardCameraParams& standard_params =
      camera_model.standard_params();
  const simian_public::spatial::AZEL& fov_params = standard_params.field_of_view();
  const auto& resolution = camera_model.standard_params().sensor_params().resolution();
  image_height = static_cast<uint32_t>(std::floor(resolution.y()));
  image_width = static_cast<uint32_t>(std::floor(resolution.x()));
  fov_x = fov_params.az();
  fov_y = fov_params.el();
  const simian_public::sensor_model::CameraLensParams& lens_params =
      camera_model.standard_params().lens_params();
  auto center_position = lens_params.center_position();
  crop_factor = lens_params.cropping();
  if (!standard_params.lens_params().has_camera_intrinsic_params()) {
    // TODO(#90173) fix computations when az and el not both specified.
    if (fov_x > 0.0 && fov_y > 0.0) {
      fx = (image_width / 2) / tan(fov_x / 2.0);
      fy = (image_height / 2) / tan(fov_y / 2.0);
    } else if (fov_x > 0.0) {
      // Assume a square pixel, defined by its horizontal FoV.
      fx = fy = (image_width / 2) / tan(fov_x / 2.0);
      fov_y = fov_x;
    } else if (fov_y > 0.0) {
      // Assume a square pixel, defined by its vertical FoV.
      fx = fy = (image_height / 2) / tan(fov_y / 2.0);
      fov_x = fov_y;
    } else {
      const simian_public::sensor_model::CameraSensorParams& sensor_params =
          standard_params.sensor_params();
      const simian_public::common::Vector2& sensor_size = sensor_params.size();
      const double focal_length = lens_params.focal_length();
      fx = image_width * focal_length / sensor_size.x();
      fov_x = 2 * atan(image_width / (2 * fx));
      fy = image_height * focal_length / sensor_size.y();
      fov_y = 2 * atan(image_height / (2 * fy));
    }
    cx = (0.5 - center_position.x() / crop_factor) * image_width;
    cy = (0.5 - center_position.y() / crop_factor) * image_height;
  } else {
    ::simian_public::camera_model::CameraIntrinsicParams camera_intrinsic_params =
        standard_params.lens_params().camera_intrinsic_params();
    fx = camera_intrinsic_params.fx();
    fy = camera_intrinsic_params.fy();
    cx = (camera_intrinsic_params.cx() - (image_width) / 2) / crop_factor + (image_width) / 2;
    cy = (camera_intrinsic_params.cy() - (image_height) / 2) / crop_factor + (image_height) / 2;
    fov_x = fov_params.az();
    fov_y = fov_params.el();
  }
}

RectilinearParams::RectilinearParams(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image)
    : RectilinearParams(camera_image.metadata().sensor_description().camera_model()) {}

Eigen::Matrix3d RectilinearParams::GetIntrinsicsMatrix() const {
  Eigen::Matrix3d result;
  // clang-format off
  result << fx , 0.0 , cx,
            0.0 , fy , cy,
            0.0 , 0.0 , 1.0;
  // clang-format on
  return result;
}

ImagePoint RectilinearParams::GetImageCenter() const {
  ImagePoint img_pt;
  img_pt.x = cx;
  img_pt.y = cy;
  return img_pt;
}

DistortionParams::DistortionParams(
    const simian_public::sensor_model::Description::CameraModel& camera_model,
    const simian_public::sensor_model::CameraLensParams& lens_params)
    : a_2(lens_params.radial_distortion_params().coefficients().a_2()),
      a_4(lens_params.radial_distortion_params().coefficients().a_4()),
      a_6(lens_params.radial_distortion_params().coefficients().a_6()),
      a_8(lens_params.radial_distortion_params().coefficients().a_8()),
      radial_units(lens_params.radial_distortion_params().units()),
      has_radial_distortion(a_2 != 0.0 || a_4 != 0.0 || a_6 != 0.0 || a_8 != 0.0),
      tangential_distortion(lens_params.tangential_distortion().begin(),
                            lens_params.tangential_distortion().end()),
      affine_scaling(lens_params.affine_scaling().begin(), lens_params.affine_scaling().end()),
      is_equidistant(lens_params.projection() ==
                     simian_public::sensor_model::CameraLensParams::EQUIDISTANT),
      use_cubemap(camera_model.fidelity().enable_cubemap()),
      use_opencv(lens_params.has_opencv_distortion_params()),
      opencv_distortion_params(lens_params.opencv_distortion_params()) {}

std::pair<DistortionParams, std::string> DistortionParams::Parse(
    const simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    const RectilinearParams& rectilinear_params) {
  std::string validate_size_msg = ValidateSizeConsistency(camera_image);
  if (!validate_size_msg.empty()) {
    return {DistortionParams(simian_public::sensor_model::Description::CameraModel(),
                             simian_public::sensor_model::CameraLensParams()),
            validate_size_msg};
  }
  const simian_public::sensor_model::Description::CameraModel& camera_model =
      camera_image.metadata().sensor_description().camera_model();
  return DistortionParams::Parse(camera_model, rectilinear_params);
}

std::pair<DistortionParams, std::string> DistortionParams::Parse(
    const simian_public::sensor_model::Description::CameraModel& camera_model,
    const RectilinearParams& rectilinear_params) {
  DistortionParams result(camera_model, camera_model.standard_params().lens_params());
  if (result.use_cubemap && !result.is_equidistant) {
    return {result, "cubemap is supported only for EQUIDISTANT cameras"};
  }
  const simian_public::spatial::AZEL& fov_params = camera_model.standard_params().field_of_view();
  // TODO(#90173) fix the logic when only one of these is specified.
  if (result.use_cubemap &&
      (std::abs(fov_params.az()) < 1.0e-3 || std::abs(fov_params.el()) < 1.0e-3)) {
    return {result, "when using cubemap, both fov_az and fov_el must be specified"};
  }

  const simian_public::sensor_model::CameraLensParams& lens_params =
      camera_model.standard_params().lens_params();
  if (lens_params.radial_distortion_size() > 0) {
    return {result,
            "the deprecated field radial_distortion is not supported in ImageProjector classes.  "
            "Use radial_distortion_params instead."};
  }
  const simian_public::sensor_model::CameraLensParams::RadialDistortion& radial_distortion_params =
      lens_params.radial_distortion_params();
  const simian_public::sensor_model::CameraLensParams::RadialDistortion::RadialCoefficients&
      coefficients = radial_distortion_params.coefficients();
  // Note that a_0 has a default value of 1, so we'll consider either 0 or 1 to mean it's not
  // specified.
  if ((coefficients.a_0() != 0.0 && std::abs(coefficients.a_0() - 1.0) > 1e-7) ||
      coefficients.a_1() != 0.0 || coefficients.a_3() != 0.0 || coefficients.a_5() != 0.0 ||
      coefficients.a_7() != 0.0) {
    // NOTE:  See note in image_projector_impl.h as to why we don't support these for the moment.
    return {result,
            "the only radial_distortion_params coefficients currently supported "
            "are a_2, a_4, a_6, and a_8"};
  }

  if (result.use_cubemap && result.has_radial_distortion &&
      result.radial_units !=
          simian_public::sensor_model::CameraLensParams_RadialDistortion_RadialUnits_PIXELS &&
      result.radial_units !=
          simian_public::sensor_model::CameraLensParams_RadialDistortion_RadialUnits_RADIANS) {
    return {result,
            "when using \"enable_cubemap: true\", the only radial_distortion_params units "
            "currently supported are RADIANS (with the deprecated synonym PIXELS)"};
  }
  if (!result.use_cubemap && result.has_radial_distortion &&
      result.radial_units !=
          simian_public::sensor_model::CameraLensParams_RadialDistortion_RadialUnits_NORMALIZED) {
    return {result,
            "when not using \"enable_cubemap: true\", the only radial_distortion_params units "
            "currently supported are NORMALIZED"};
  }
  if (result.use_cubemap && !lens_params.radial_distortion().empty()) {
    return {result, "deprecated \"radial_distortion\" field not supported for cubemap"};
  }
  if (result.has_radial_distortion && !result.use_cubemap &&
      result.radial_units !=
          simian_public::sensor_model::CameraLensParams_RadialDistortion_RadialUnits_NORMALIZED) {
    return {result, "unsupported radial units"};
  }
  if (result.affine_scaling.size() != 0 && result.affine_scaling.size() != 4) {
    return {result, "expected affine scaling to have 0 or 4 entries"};
  }
  return {result, ""};
}

PinHoleProjector::PinHoleProjector(const RectilinearParams& params)
    : intrinsics_(params.GetIntrinsicsMatrix()), intrinsics_inverse_(intrinsics_.inverse()) {}

bool PinHoleProjector::Project(const ::applied::Vector3d& in_camera_frame,
                               ::applied::ImagePoint& out) const {
  // Convert from Simian's axis conventions to conventional graphics conventions:
  // x forward, y left, z up to x right, y down, z into the screen.
  const ::applied::Vector3d in_graphics_frame(-in_camera_frame.y(), -in_camera_frame.z(),
                                              in_camera_frame.x());
  ::applied::Vector3d image_pointz = intrinsics_ * in_graphics_frame;
  if (image_pointz.z() <= 0) {
    return false;
  }
  out = {image_pointz.x() / image_pointz.z(), image_pointz.y() / image_pointz.z()};
  return true;
}

bool PinHoleProjector::Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                                 ::applied::Vector3d& in_camera_frame) {
  if (radial_depth <= 0.0) {
    return false;
  }
  // Homogeneous coordinates.
  ::applied::Vector3d image_pointz = {image_point.x, image_point.y, 1};
  const applied::Vector3d in_graphics_frame = intrinsics_inverse_ * image_pointz;
  in_camera_frame = {in_graphics_frame.z(), -in_graphics_frame.x(), -in_graphics_frame.y()};
  // Normalize to given radial_depth.
  in_camera_frame *= radial_depth / in_graphics_frame.norm();
  return true;
}

bool PinHoleProjector::UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                       ::applied::Vector3d& in_camera_frame) {
  if (z_depth <= 0.0) {
    return false;
  }
  // Homogeneous coordinates.
  ::applied::Vector3d image_pointz = {image_point.x, image_point.y, 1};
  const applied::Vector3d in_graphics_frame = intrinsics_inverse_ * image_pointz;
  in_camera_frame = {in_graphics_frame.z(), -in_graphics_frame.x(), -in_graphics_frame.y()};
  // Normalize to given radial_depth.
  in_camera_frame *= z_depth / in_graphics_frame.z();
  return true;
}

Distorter::Distorter(const RectilinearParams& rectilinear_params,
                     const DistortionParams& distortion_params)
    : radial_distorter_(distortion_params.a_2, distortion_params.a_4, distortion_params.a_6,
                        distortion_params.a_8),
      distortion_params_(distortion_params),
      rectilinear_params_(rectilinear_params),
      crop_factor_(rectilinear_params.crop_factor),
      fov_x_(rectilinear_params.fov_x),
      width_(rectilinear_params.image_width),
      maxdim_(std::max(rectilinear_params.image_height, rectilinear_params.image_width)),
      focal_length_(width_ / (2 * tan(fov_x_ / 2.))),
      use_opencv(distortion_params.use_opencv),
      image_center_(rectilinear_params.GetImageCenter()) {}

/**
 * @brief Build an openCV Matrix from rectilinear params
 *
 * @param camera_matrix
 * @return If the camera matrix can be made from the parameters
 */
bool Distorter::MakeCameraMatrix(cv::Mat& camera_matrix) const {
  camera_matrix.at<float>(0, 0) = rectilinear_params_.fx;
  camera_matrix.at<float>(1, 0) = 0;
  camera_matrix.at<float>(2, 0) = 0;

  camera_matrix.at<float>(0, 1) = 0;
  camera_matrix.at<float>(1, 1) = rectilinear_params_.fy;
  camera_matrix.at<float>(2, 1) = 0;

  camera_matrix.at<float>(0, 2) = rectilinear_params_.cx;
  camera_matrix.at<float>(1, 2) = rectilinear_params_.cy;
  camera_matrix.at<float>(2, 2) = 1;
  return true;
}

/**
 * @brief Build a distortion Matrix from distortion_params
 *
 * @param distortion_params
 * @return If the distortion params can be made from the parameters.
 */
bool Distorter::MakeDistortionMatrix(cv::Mat& distortion_params) const {
  distortion_params.at<double>(0) = distortion_params_.opencv_distortion_params.k1();
  distortion_params.at<double>(1) = distortion_params_.opencv_distortion_params.k2();
  distortion_params.at<double>(2) = distortion_params_.opencv_distortion_params.p1();
  distortion_params.at<double>(3) = distortion_params_.opencv_distortion_params.p2();
  distortion_params.at<double>(4) = distortion_params_.opencv_distortion_params.k3();
  distortion_params.at<double>(5) = distortion_params_.opencv_distortion_params.k4();
  distortion_params.at<double>(6) = distortion_params_.opencv_distortion_params.k5();
  distortion_params.at<double>(7) = distortion_params_.opencv_distortion_params.k6();
  return true;
}

/**
 * @brief Applies opencv distortion based on distortion coefficients by projecting
 * a mock 3d point onto the 2d image plane.
 *
 * @param out Image point to mutate by applying distortion. The original point has the origin at the
 * image center. The output point has the origin at (0,0)
 * @return
 */
bool Distorter::ApplyOpenCVDistortion(::applied::ImagePoint& out) const {
  // Need to multiply by an aspect ratio to compensate for non square pixels in the real world
  double aspect_ratio = rectilinear_params_.fx / rectilinear_params_.fy;
  std::vector<cv::Point3d> points = {{out.x, out.y * aspect_ratio, rectilinear_params_.fx}};
  std::vector<cv::Point2d> distorted_points;
  cv::Mat rvec = cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
  cv::Mat camera_matrix(3, 3, cv::DataType<float>::type);
  cv::Mat distortion_matrix(8, 1, cv::DataType<double>::type);
  MakeCameraMatrix(camera_matrix);
  MakeDistortionMatrix(distortion_matrix);
  cv::projectPoints(points, rvec, tvec, camera_matrix, distortion_matrix, distorted_points);
  out.x = distorted_points[0].x;
  out.y = distorted_points[0].y;
  return true;
}

/**
 * @brief Reverses distortion based on distortion coefficients.
 *
 * @param out Image point to mutate by removing distortion. Input point has the origin at (0,0).
 * Output is in pixel units with origin at the image center.
 * @return
 */
bool Distorter::RemoveOpenCVDistortion(::applied::ImagePoint& out) const {
  std::vector<cv::Point2d> points = {{out.x, out.y}};
  std::vector<cv::Point2d> undistorted_points;
  cv::Mat rvec = cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
  cv::Mat tvec = cv::Mat::zeros(cv::Size(3, 1), CV_64FC1);
  cv::Mat camera_matrix(3, 3, cv::DataType<float>::type);
  cv::Mat distortion_matrix(8, 1, cv::DataType<double>::type);
  MakeCameraMatrix(camera_matrix);
  MakeDistortionMatrix(distortion_matrix);
  cv::undistortPoints(points, undistorted_points, camera_matrix, distortion_matrix);
  out.x = undistorted_points[0].x * rectilinear_params_.fx;
  out.y = undistorted_points[0].y * rectilinear_params_.fy;
  return true;
}

bool Distorter::ApplyDistortions(ImagePoint& out) const {
  if (use_opencv) {
    // OpenCV expects points origin at the image center
    ImagePoint diff = out - image_center_;
    // Output with origin at (0,0)
    if (!ApplyOpenCVDistortion(diff)) return false;
    // Correcting for crop has to happen with origin at image center
    diff = diff - image_center_;
    MaybeCorrectForCrop(diff);
    out = diff + image_center_;
  } else {
    ImagePoint diff = out - image_center_;
    diff.y *= -1;
    if (!MaybeApplyTangentialDistortion(diff)) return false;
    if (!MaybeApplyRadialDistortion(diff)) return false;
    if (!MaybeApplyAffineScaling(diff)) return false;
    MaybeConvertToEquidistant(diff);
    MaybeCorrectForCrop(diff);
    diff.y *= -1;
    out = image_center_ + diff;
  }
  return true;
}

bool Distorter::MaybeApplyTangentialDistortion(ImagePoint& diff) const {
  if (distortion_params_.tangential_distortion.size() == 0 ||
      (distortion_params_.tangential_distortion.size() == 2 &&
       std::abs(distortion_params_.tangential_distortion[0]) < 1.0e-7 &&
       std::abs(distortion_params_.tangential_distortion[1]) < 1.0e-7)) {
    return true;
  }
  // TODO: Implement.
  return false;
}

bool Distorter::MaybeRemoveTangentialDistortion(ImagePoint& diff) const {
  if (distortion_params_.tangential_distortion.size() == 0 ||
      (distortion_params_.tangential_distortion.size() == 2 &&
       std::abs(distortion_params_.tangential_distortion[0]) < 1.0e-7 &&
       std::abs(distortion_params_.tangential_distortion[1]) < 1.0e-7)) {
    return true;
  }
  // TODO: Implement.
  return false;
}

bool Distorter::MaybeApplyAffineScaling(ImagePoint& diff) const {
  if (distortion_params_.affine_scaling.size() == 0 ||
      (distortion_params_.affine_scaling.size() == 4 &&
       std::abs(distortion_params_.affine_scaling[0] - 1) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[1]) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[2]) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[3] - 1) < 1.0e-7)) {
    return true;
  }
  // TODO:  Implement.
  return false;
}

bool Distorter::MaybeRemoveAffineScaling(ImagePoint& diff) const {
  if (distortion_params_.affine_scaling.size() == 0 ||
      (distortion_params_.affine_scaling.size() == 4 &&
       std::abs(distortion_params_.affine_scaling[0] - 1) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[1]) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[2]) < 1.0e-7 &&
       std::abs(distortion_params_.affine_scaling[3] - 1) < 1.0e-7)) {
    return true;
  }
  // TODO:  Implement.
  return false;
}

bool Distorter::MaybeApplyRadialDistortion(ImagePoint& diff) const {
  if (!radial_distorter_.has_radial_distortion()) {
    return true;
  }
  // NOTE:  This works only for NORMALZED units (and even there, need to figure out
  // what's going on with FCameraModel::CalcRadialImageScaleFactor() ).
  const float r = 2.0 * sqrt(diff.x * diff.x + diff.y * diff.y) / maxdim_;
  if (r > radial_distorter_.max_r()) {
    return false;
  }
  diff = diff * radial_distorter_.scale_factor(r);
  return true;
}

bool Distorter::MaybeRemoveRadialDistortion(ImagePoint& diff) const {
  if (!radial_distorter_.has_radial_distortion()) {
    return true;
  }

  // NOTE:  This works only for NORMALZED units (and even there, need to figure out
  // what's going on with FCameraModel::CalcRadialImageScaleFactor() ).
  const float r = 2.0 * sqrt(diff.x * diff.x + diff.y * diff.y) / maxdim_;
  if (std::abs(r) < 1.0e-10) return true;  // Avoid dividing 0 by 0.
  const float r_undistorted = radial_distorter_.inverse_distorted_r(r);
  if (r_undistorted < 0) return false;  // outside valid range.
  diff = diff * (r_undistorted / r);
  return true;
}

void Distorter::MaybeConvertToEquidistant(ImagePoint& diff) const {
  if (!distortion_params_.is_equidistant) {
    return;
  }
  const double r = diff.norm();
  // Avoid dividing 0 by 0.  Note that in theory we would want to multiply diff by
  // maxdim_ / (focal_length * fov_) or something like that.  But that's always going
  // to be a change of much less than a pixel, so just return without changing diff.
  if (std::abs(r) < 1.0e-10) return;
  const double theta = atan(r / focal_length_);
  const double new_r = (theta / fov_x_) * width_;
  diff = diff * (new_r / r);
}

void Distorter::MaybeUnconvertToEquidistant(ImagePoint& diff) const {
  if (!distortion_params_.is_equidistant) {
    return;
  }
  const double r = diff.norm();
  // Avoid dividing 0 by 0.  Note that in theory we would want to multiply diff by
  // (focal_length * fov_) / maxdim_ or something like that.  But that's always going
  // to be a change of much less than a pixel, so just return without changing diff.
  if (std::abs(r) < 1.0e-10) return;

  const double r_old = focal_length_ * tan(r * fov_x_ / width_);
  diff = diff * (r_old / r);
}

void Distorter::MaybeCorrectForCrop(ImagePoint& diff) const { diff = diff * (1.0 / crop_factor_); }
void Distorter::MaybeUncorrectForCrop(ImagePoint& diff) const { diff = diff * crop_factor_; }

bool Distorter::RemoveDistortions(ImagePoint& out) const {
  if (use_opencv) {
    // Correcting for cropping has to happen with origin around image center
    ImagePoint diff = out - image_center_;
    MaybeUncorrectForCrop(diff);
    // Undistorting expects origin to be around (0,0)
    diff = diff + image_center_;
    // Undistorting outputs origin around image center
    if (!RemoveOpenCVDistortion(diff)) return false;
    // Moving back to origin at (0,0)
    out = diff + image_center_;
  } else {
    ImagePoint diff = out - image_center_;
    diff.y *= -1;
    MaybeUncorrectForCrop(diff);
    MaybeUnconvertToEquidistant(diff);
    if (!MaybeRemoveAffineScaling(diff)) return false;
    if (!MaybeRemoveRadialDistortion(diff)) return false;
    if (!MaybeRemoveTangentialDistortion(diff)) return false;
    diff.y *= -1;
    out = image_center_ + diff;
  }
  return true;
}

CubeMapper::CubeMapper(const RectilinearParams& rectilinear_params,
                       const DistortionParams& distortion_params)
    : use_cubemap_(distortion_params.use_cubemap),
      rectilinear_params_(rectilinear_params),
      image_center_(rectilinear_params_.GetImageCenter()),
      radial_distorter_(distortion_params.a_2, distortion_params.a_4, distortion_params.a_6,
                        distortion_params.a_8),
      fx_(rectilinear_params_.image_width / rectilinear_params_.fov_x),
      fy_(rectilinear_params_.image_height / rectilinear_params_.fov_y) {}

bool CubeMapper::Project(const Vector3d& in_camera_frame, ImagePoint& out) const {
  if (!use_cubemap_) {
    return false;
  }
  // Convert from Simian's axis conventions to conventional graphics conventions:
  // x forward, y left, z up to x right, y down, z into the screen.
  const ::applied::Vector3d in_graphics_frame(-in_camera_frame.y(), -in_camera_frame.z(),
                                              in_camera_frame.x());
  const double rho = in_graphics_frame.norm();
  double theta = std::acos(in_graphics_frame.z() / rho);
  if (theta > radial_distorter_.max_r()) {
    return false;
  }
  if (radial_distorter_.has_radial_distortion()) {
    theta = theta * radial_distorter_.scale_factor(theta);
  }

  const double theta_cropped = theta / rectilinear_params_.crop_factor;
  const double r_proj = std::sqrt(in_graphics_frame.x() * in_graphics_frame.x() +
                                  in_graphics_frame.y() * in_graphics_frame.y());
  out.x = theta_cropped * fx_ * in_graphics_frame.x() / r_proj + image_center_.x;
  out.y = theta_cropped * fy_ * in_graphics_frame.y() / r_proj + image_center_.y;
  return true;
}

bool CubeMapper::Unproject(const ::applied::ImagePoint& image_point, double radial_depth,
                           ::applied::Vector3d& in_camera_frame) {
  if (!use_cubemap_) {
    return false;
  }
  const double theta_x = (image_point.x - image_center_.x) / fx_;
  const double theta_y = (image_point.y - image_center_.y) / fy_;
  const double theta_cropped = std::sqrt(theta_x * theta_x + theta_y * theta_y);
  const double theta_distorted = theta_cropped * rectilinear_params_.crop_factor;

  const double theta = (radial_distorter_.has_radial_distortion())
                           ? radial_distorter_.inverse_distorted_r(theta_distorted)
                           : theta_distorted;
  if (theta < 0.0) {
    return false;
  }
  const double in_graphics_frame_z = radial_depth * std::cos(theta);
  const double in_graphics_frame_r = radial_depth * std::sin(theta);
  const double phi = std::atan2(theta_y, theta_x);
  const double in_graphics_frame_x = in_graphics_frame_r * std::cos(phi);
  const double in_graphics_frame_y = in_graphics_frame_r * std::sin(phi);
  in_camera_frame = {in_graphics_frame_z, -in_graphics_frame_x, -in_graphics_frame_y};
  return true;
}

bool CubeMapper::UnprojectZDepth(const ::applied::ImagePoint& image_point, double z_depth,
                                 ::applied::Vector3d& in_camera_frame) {
  if (!use_cubemap_) {
    return false;
  }
  const double theta_x = (image_point.x - image_center_.x) / fx_;
  const double theta_y = (image_point.y - image_center_.y) / fy_;
  const double theta_cropped = std::sqrt(theta_x * theta_x + theta_y * theta_y);
  const double theta_distorted = theta_cropped * rectilinear_params_.crop_factor;
  const double theta = (radial_distorter_.has_radial_distortion())
                           ? radial_distorter_.inverse_distorted_r(theta_distorted)
                           : theta_distorted;
  if (theta < 0.0) {
    return false;
  }
  const double in_graphics_frame_z = z_depth;
  const double in_graphics_frame_r = z_depth * std::tan(theta);
  const double phi = std::atan2(theta_y, theta_x);
  const double in_graphics_frame_x = in_graphics_frame_r * std::cos(phi);
  const double in_graphics_frame_y = in_graphics_frame_r * std::sin(phi);
  in_camera_frame = {in_graphics_frame_z, -in_graphics_frame_x, -in_graphics_frame_y};
  return true;
}

// TODO(#220) add unit test for censoring in non-monotonic region.
RadialDistorter::RadialDistorter(float a_2, float a_4, float a_6, float a_8)
    : a_2_(a_2),
      a_4_(a_4),
      a_6_(a_6),
      a_8_(a_8),
      has_radial_distortion_(a_2_ != 0.0 || a_4_ != 0.0 || a_6_ != 0.0 || a_8_ != 0.0),
      delta_r_(0.01) {
  // We want the distorted_r function to be monotonic over the entire domain.  If distorted_r
  // ever turns back on itself -- that is, if distorted_r_prime goes negative -- we should
  // censor any points with an r past that point.
  //
  // In *current* usage, we're either using non-cubemap with NORMALIZED units, or else
  // cubemap with PIXELS units (which really means RADIANS).  In either case, it's hard to
  // see how an r greater than pi would ever make sense.   So we'll first scan for any negative
  // values of distorted_r_prime less than pi.  Increments of 0.01 should be fine.
  max_r_ = M_PI;
  distorted_r_graph_.reserve(static_cast<int>(std::ceil(M_PI / delta_r_)));
  for (float r = 0; r < M_PI; r += delta_r_) {
    distorted_r_graph_.emplace_back(std::pair<float, float>{distorted_r(r), r});
    if (distorted_r_prime(r) < 0.0) {
      max_r_ = r;
      break;
    }
  }
  // Now at this point we can do a finer search if we want, with any of a number of rootfinding
  // techniques.  That's why I implemented distorted_r_doubleprime in case we want to do
  // Newton's method or something.  With such a high-degree polynomial it's tricky to find
  // rigorous roots, and probably not necessary.  For now I'm not going to bother; we'll see
  // if we need it.
}

float RadialDistorter::scale_factor(float r) const {
  float retval = 0.0f;
  const float r2 = r * r;
  retval += a_8_;
  retval *= r2;
  retval += a_6_;
  retval *= r2;
  retval += a_4_;
  retval *= r2;
  retval += a_2_;
  retval *= r2;
  retval += 1.0;
  return retval;
}

float RadialDistorter::distorted_r(float r) const { return r * scale_factor(r); }

// The first derivative.
float RadialDistorter::distorted_r_prime(float r) const {
  float retval = 0.0f;
  const float r2 = r * r;
  retval += 9 * a_8_;
  retval *= r2;
  retval += 7 * a_6_;
  retval *= r2;
  retval += 5 * a_4_;
  retval *= r2;
  retval += 3 * a_2_;
  retval *= r2;
  retval += 1.0;
  return retval;
}

// The second derivative.
float RadialDistorter::distorted_r_doubleprime(float r) const {
  float retval = 0.0f;
  const float r2 = r * r;
  retval += 72 * a_8_;
  retval *= r2;
  retval += 42 * a_6_;
  retval *= r2;
  retval += 20 * a_4_;
  retval *= r2;
  retval *= 6 * a_2_;
  retval *= r;
  return retval;
}

float RadialDistorter::inverse_distorted_r(float distorted_r) const {
  const auto iR =
      std::upper_bound(distorted_r_graph_.cbegin(), distorted_r_graph_.cend(), distorted_r,
                       [](float r_dummy, const std::pair<float, float>& entry) -> bool {
                         return r_dummy < entry.first;
                       });
  if (iR == distorted_r_graph_.cbegin()) {
    // Shouldn't actually happen, but if it does, there's only one reasonable value.
    return 0.0f;
  } else if (iR == distorted_r_graph_.cend()) {
    // Can't find the root.
    return -1.0f;
  } else {
    const auto iRprev = iR - 1;
    const float r_0 = iRprev->second;
    const float distorted_r_0 = iRprev->first;
    const float r_1 = iR->second;
    const float distorted_r_1 = iR->first;
    return r_0 + (r_1 - r_0) / (distorted_r_1 - distorted_r_0) * (distorted_r - distorted_r_0);
  }
}

std::string ValidateSizeConsistency(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image) {
  const int width = camera_image.image_shape().width();
  const int height = camera_image.image_shape().height();
  const auto& resolution = camera_image.metadata()
                               .sensor_description()
                               .camera_model()
                               .standard_params()
                               .sensor_params()
                               .resolution();
  if (width != static_cast<int>(std::floor(resolution.x())))
    return "Inconsistent width for camera image";
  if (height != static_cast<int>(std::floor(resolution.y())))
    return "Inconsistent height for camera image";
  return "";
}
}  // namespace implementation
}  // namespace applied
