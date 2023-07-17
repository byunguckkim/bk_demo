#include "applied/spectral/public/depth_image_projector.h"

namespace applied {

namespace {
// https://en.cppreference.com/w/cpp/algorithm/clamp
// We require compatibility with C++14, so write our own implementation here.
template <class T>
constexpr const T& Clamp(const T& v, const T& lo, const T& hi) {
  return v < lo ? lo : hi < v ? hi : v;
}
}  // namespace
std::pair<DepthImageProjector, std::string> DepthImageProjector::Make(
    const Pose3d& camera_T_frame,
    const simian_public::sensor_model::Description::CameraModel& camera_model) {
  auto make_projector_res = ImageProjector::Make(camera_T_frame, camera_model);
  ImageProjector& projector = make_projector_res.first;
  const std::string& errmsg = make_projector_res.second;
  if (!errmsg.empty()) {
    return {DepthImageProjector(), errmsg};
  }
  if (!camera_model.standard_params().system_params().has_depth_params()) {
    return {DepthImageProjector(), "No depth params specified for camera"};
  }
  return {DepthImageProjector(
              std::move(projector), camera_model.standard_params().system_params().depth_params(),
              camera_model.standard_params().sensor_params().color_depth(),
              // Initialization of these boolean params needs to be kept in synch with how they are
              // treated in Spectral.
              /*unit_is_uint=*/camera_model.standard_params().system_params().data_type() ==
                  simian_public::sensor_model::CameraSystemParams_DataType_UINT,
              /*use_radial_depth=*/camera_model.fidelity().enable_cubemap()),
          ""};
}

double DepthImageProjector::PixelValueToDepth(double pixel_value) const {
  if (depth_params_.type() ==
      simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_RAW) {
    return pixel_value;
  }
  double r = pixel_value * granularity_;
  if (depth_params_.type() ==
      simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_HYP_SPLINE) {
    r = r * depth_params_.hyp_offset() / (1 - r);
  } else {
    if (depth_params_.type() ==
        simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_LOG) {
      r = std::exp(log_of_base_ * (r - 1));
    }
    if (unit_is_uint_) {
      r = r * depth_params_.max() + depth_params_.min();
    }
  }
  return r;
}

double DepthImageProjector::DepthToPixelValue(double depth) const {
  double r = depth;
  r = Clamp<double>(r, depth_params_.min(), depth_params_.max());
  if (depth_params_.type() ==
      simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_RAW) {
    return r;
  } else if (depth_params_.type() ==
             simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_HYP_SPLINE) {
    const double hyp_offset = depth_params_.hyp_offset();
    r = 1 - hyp_offset / (r + hyp_offset);
  } else {
    if (unit_is_uint_) {
      r = (r - depth_params_.min()) / depth_params_.max();
    }

    if (depth_params_.type() ==
        simian_public::sensor_model::CameraSystemParams_DepthParams_Mode_LOG) {
      r = 1 + (std::log(r) / log_of_base_);
    }
  }

  return max_pixel_value_ * r;
}

bool DepthImageProjector::Project(const ::applied::Vector3d& point_in_frame,
                                  ::applied::ImagePoint& out, double& pixel_value) const {
  bool ok_projected = image_projector_->Project(point_in_frame, out);
  if (!ok_projected) {
    return false;
  }
  double z_or_radial_depth = (use_radial_depth_)
                                 ? image_projector_->RadialDepthOfPointInFrame(point_in_frame)
                                 : image_projector_->ZDepthOfPointInFrame(point_in_frame);
  pixel_value = DepthToPixelValue(z_or_radial_depth);
  return true;
}

bool DepthImageProjector::Unproject(const ::applied::ImagePoint& image_point, double pixel_value,
                                    ::applied::Vector3d& point_in_frame) const {
  double z_or_radial_depth = PixelValueToDepth(pixel_value);
  bool success = false;
  if (use_radial_depth_) {
    success = image_projector_->Unproject(image_point, z_or_radial_depth, point_in_frame);
  } else {
    success = image_projector_->UnprojectZDepth(image_point, z_or_radial_depth, point_in_frame);
  }
  return success;
}
}  // namespace applied
