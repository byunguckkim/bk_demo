// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.

#include "applied/spectral/public/ros_helpers.h"

#if __has_include(<rclcpp/time.hpp>)
#include <rclcpp/time.hpp>
#include <sensor_msgs/image_encodings.hpp>

namespace applied {
namespace ros_time_ns = ::rclcpp;
}
#elif __has_include(<ros/time.h>)
#include <ros/time.h>
#include <sensor_msgs/image_encodings.h>

namespace applied {
namespace ros_time_ns = ::ros;
}
#endif  // __has_include(...)

#include "applied/spectral/public/image_helpers.h"
#include "applied/spectral/public/lidar_helpers.h"
#include "applied/spectral/public/radar_helpers.h"
#include "applied/spectral/public/spectral_shared_memory_helper.h"

#if defined(APPLIED_HAVE_ROS)

namespace applied {
namespace {

struct RosField {
  std::string name;
  uint8_t ros_type;
  size_t size_in_bytes;
};

using RosFieldList = std::vector<RosField>;

static const RosFieldList kROSLidarFields = {
    {"x", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"y", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"z", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"intensity", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"channel", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"instance_id", sensor_msgs_ns::PointField::UINT16, 2u},
    {"reserved", sensor_msgs_ns::PointField::UINT8, 1u},
    {"semantic_class", sensor_msgs_ns::PointField::UINT8, 1u},
    {"ambient", sensor_msgs_ns::PointField::FLOAT32, 4u},
};

static const RosFieldList kROSLidarFieldsIncludingGT = {
    {"x", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"y", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"z", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"intensity", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"channel", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"instance_id", sensor_msgs_ns::PointField::UINT16, 2u},
    {"reserved", sensor_msgs_ns::PointField::UINT8, 1u},
    {"semantic_class", sensor_msgs_ns::PointField::UINT8, 1u},
    {"ambient", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"range_gt", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"azimuth_gt", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"elevation_gt", sensor_msgs_ns::PointField::FLOAT32, 4u},
};

#pragma pack(push, 1)
struct ROSLidarPoint {
  float x;
  float y;
  float z;
  float intensity;
  float channel;
  // Points on actors will be tagged with an instance ID that matches
  // their Simian actor ID.
  uint16_t instance_id;
  uint8_t reserved;
  // See manual for list of semantic classes.
  uint8_t semantic_class;
  float ambient;
};
#pragma pack(pop)
static_assert(sizeof(ROSLidarPoint) == sizeof(float) * 7, "ROSLidarPoint should be tightly packed");

#pragma pack(push, 1)
struct ROSLidarPointIncludingGT : public ROSLidarPoint {
  float range_gt;
  float azimuth_gt;
  float elevation_gt;
};
#pragma pack(pop)
static_assert(sizeof(ROSLidarPointIncludingGT) == sizeof(ROSLidarPoint) + sizeof(float) * 3,
              "ROSLidarPointIncludingGT should be tightly packed");

static const RosFieldList kROSRadarFieldNames = {
    {"x", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"y", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"z", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"ext_range", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"ext_width", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"ext_height", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"velocity", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"snr", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"rcs", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"reserved_0", sensor_msgs_ns::PointField::FLOAT32, 4u},
    {"actor_id", sensor_msgs_ns::PointField::FLOAT32, 2u},
    {"reserved_1", sensor_msgs_ns::PointField::FLOAT32, 1u},
    {"semantic_class", sensor_msgs_ns::PointField::FLOAT32, 1u}};

double CalcFocalLengthRectilinear(double fov, double image_length) {
  return (image_length / 2.0) / (tan(fov / 2.0));
}
double CalcFocalLengthEquidistant(double fov, double image_length) {
  return (image_length / 2.0) / ((fov / 2.0));
}

void SetPointCloudFields(const RosFieldList& fields, sensor_msgs_ns::PointCloud2& out) {
  size_t offset = 0;
  out.fields.clear();
  out.fields.reserve(fields.size());
  for (const auto& field : fields) {
    out.fields.emplace_back();
    sensor_msgs_ns::PointField& ros_field = out.fields.back();
    ros_field.name = field.name;
    ros_field.datatype = field.ros_type;
    ros_field.count = 1;
    ros_field.offset = offset;
    offset += field.size_in_bytes;
  }
}

bool HasGroundTruthPointData(
    const simian_public::sensor_model::SensorOutput_LidarCloud& lidar_output) {
  return lidar_output.metadata()
      .sensor_description()
      .lidar_model()
      .format()
      .include_groundtruth_points();
}

}  // namespace

void CalculateFocalLengths(
    const simian_public::sensor_model::SensorOutput::CameraImage& camera_image, double& out_fx,
    double& out_fy, double& out_fov_x, double& out_fov_y) {
  const simian_public::sensor_model::Description::CameraModel& camera_model =
      camera_image.metadata().sensor_description().camera_model();
  const simian_public::sensor_model::ImageShape& image_shape = camera_image.image_shape();
  const simian_public::sensor_model::StandardCameraParams& standard_params =
      camera_model.standard_params();
  const simian_public::spatial::AZEL& fov_params = standard_params.field_of_view();
  const bool is_fisheye = standard_params.lens_params().projection() ==
                          ::simian_public::sensor_model::CameraLensParams::EQUIDISTANT;

  // Get a pointer to the correct focal length function for the specified projection.
  auto calculate_focal_length =
      (is_fisheye) ? CalcFocalLengthEquidistant : CalcFocalLengthRectilinear;
  out_fov_x = fov_params.az();
  out_fov_y = fov_params.el();
  if (out_fov_x > 0.0 && out_fov_y > 0.0) {
    out_fx = calculate_focal_length(out_fov_x, image_shape.width());
    out_fy = calculate_focal_length(out_fov_y, image_shape.height());
  } else if (out_fov_x > 0.0) {
    // Assume a square pixel, defined by its horizontal FoV.
    out_fx = out_fy = calculate_focal_length(out_fov_x, image_shape.width());
    out_fov_y = out_fov_x;
  } else if (out_fov_y > 0.0) {
    // Assume a square pixel, defined by its vertical FoV.
    out_fx = out_fy = calculate_focal_length(out_fov_y, image_shape.height());
    out_fov_x = out_fov_y;
  } else {
    const simian_public::sensor_model::CameraLensParams& lens_params =
        standard_params.lens_params();
    const simian_public::sensor_model::CameraSensorParams& sensor_params =
        standard_params.sensor_params();
    const simian_public::common::Vector2& sensor_size = sensor_params.size();
    const double focal_length = lens_params.focal_length();
    out_fx = image_shape.width() * focal_length / sensor_size.x();
    out_fy = image_shape.height() * focal_length / sensor_size.y();
  }
}

std::string BuildRosCameraInfo(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::CameraInfo& out) {
  out.header.frame_id = frame_id;
  const google::protobuf::Timestamp& sim_time = camera_image.metadata().sensor_timestamp();
  out.header.stamp = ros_time_ns::Time(sim_time.seconds(), sim_time.nanos());
  const simian_public::sensor_model::ImageShape& shape = camera_image.image_shape();
  out.width = shape.width();
  out.height = shape.height();
  const double cx = (double)(shape.width() / 2);
  const double cy = (double)(shape.height() / 2);
  double fx, fy, fov_x, fov_y;
  CalculateFocalLengths(camera_image, fx, fy, fov_x, fov_y);
#if __has_include(<rclcpp/time.hpp>)
  out.k = std::array<double, 9>
#elif 1  // __has_include(<ros/time.h>)
  out.K = boost::array<double, 9>
#endif   // __has_include(...)
      {
          fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0,
      };

  return "";
}

std::string BuildRosCompressedImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::CompressedImage& out) {
  const ::applied::spectral::DataPointer data_pointer =
      ::applied::spectral::SMHelperSingleton::GetInstance().ReadDataPointer(camera_image);
  const std::string err_msg = ::applied::SavePngToBuffer(data_pointer, camera_image, out.data);
  if (!err_msg.empty()) {
    return "error converting to png format: " + err_msg;
  }
  out.format = "png";
  out.header.frame_id = frame_id;
  const google::protobuf::Timestamp& sim_time = camera_image.metadata().sensor_timestamp();
  out.header.stamp = ros_time_ns::Time(sim_time.seconds(), sim_time.nanos());
  return "";
}

std::string BuildRosImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::Image& out) {
  const simian_public::sensor_model::ImageShape& shape = camera_image.image_shape();
  const ::applied::spectral::DataPointer data_pointer =
      ::applied::spectral::SMHelperSingleton::GetInstance().ReadDataPointer(camera_image);
  const size_t bytes_per_channel = static_cast<size_t>(camera_image.image().color_depth()) / 8u;
  const bool is_uint = camera_image.image().data_type() ==
                       simian_public::sensor_model::CameraSystemParams_DataType_UINT;

  std::string encoding;
  if (is_uint && bytes_per_channel == 1) {
    out.encoding = sensor_msgs::image_encodings::BGRA8;
  } else if (is_uint && bytes_per_channel == 2) {
    out.encoding = sensor_msgs::image_encodings::BGRA16;
  } else if (!is_uint && bytes_per_channel == 4) {
    out.encoding = sensor_msgs::image_encodings::TYPE_32FC4;
  } else {
    return "unsupported channel format: " + std::to_string(camera_image.image().color_depth()) +
           " color depth " + (is_uint ? "(uint)" : "(float)");
  }
  const size_t expected_size =
      shape.height() * shape.width() * shape.channels() * bytes_per_channel;
  if (expected_size != data_pointer.size) {
    return "expected " + std::to_string(expected_size) + " bytes of image data but got " +
           std::to_string(data_pointer.size);
  }
  out.header.frame_id = frame_id;
  const google::protobuf::Timestamp& sim_time = camera_image.metadata().sensor_timestamp();
  out.header.stamp = ros_time_ns::Time(sim_time.seconds(), sim_time.nanos());
  out.height = shape.height();
  out.width = shape.width();
  out.data.resize(data_pointer.size);
  out.data.assign(data_pointer.memory_address, data_pointer.memory_address + data_pointer.size);
  // This is the row length in bytes.
  out.step = shape.width() * bytes_per_channel * shape.channels();
  out.is_bigendian = false;
  return "";
}

std::string BuildRosLidarCloud(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_output,
    std::string_view frame_id, sensor_msgs_ns::PointCloud2& out) {
  size_t points_written = 0;
  const bool has_gt_data = HasGroundTruthPointData(lidar_output);
  const auto [num_points, err_msg] = applied::GetNumLidarPoints(lidar_output);
  if (!err_msg.empty()) {
    return err_msg;
  }
  if (has_gt_data) {
    SetPointCloudFields(kROSLidarFieldsIncludingGT, out);
    ROSLidarPointIncludingGT ros_point;
    out.data.resize(num_points * sizeof(ros_point));
    applied::ForEachLidarPoint(lidar_output, [&](const ::applied::LidarPoint& lidar_point) {
      ros_point.x = lidar_point.x;
      ros_point.y = lidar_point.y;
      ros_point.z = lidar_point.z;
      ros_point.intensity = lidar_point.intensity;
      ros_point.channel = lidar_point.channel;
      ros_point.instance_id = lidar_point.instance_id;
      ros_point.semantic_class = lidar_point.semantic_class;
      ros_point.ambient = lidar_point.ambient;
      ros_point.azimuth_gt = lidar_point.azimuth_gt;
      ros_point.elevation_gt = lidar_point.elevation_gt;
      ros_point.range_gt = lidar_point.range_gt;
      points_written++;
      memcpy(out.data.data() + (points_written - 1) * sizeof(ros_point), &ros_point,
             sizeof(ros_point));
      return "";
    });
    out.width = points_written;
    out.point_step = sizeof(ros_point);
    out.row_step = out.point_step * out.width;
  } else {
    SetPointCloudFields(kROSLidarFields, out);
    ROSLidarPoint ros_point;
    out.data.resize(num_points * sizeof(ros_point));
    applied::ForEachLidarPoint(lidar_output, [&](const ::applied::LidarPoint& lidar_point) {
      ros_point.x = lidar_point.x;
      ros_point.y = lidar_point.y;
      ros_point.z = lidar_point.z;
      ros_point.intensity = lidar_point.intensity;
      ros_point.channel = lidar_point.channel;
      ros_point.instance_id = lidar_point.instance_id;
      ros_point.semantic_class = lidar_point.semantic_class;
      ros_point.ambient = lidar_point.ambient;
      points_written++;
      memcpy(out.data.data() + (points_written - 1) * sizeof(ros_point), &ros_point,
             sizeof(ros_point));
      return "";
    });
    out.width = points_written;
    out.point_step = sizeof(ros_point);
    out.row_step = out.point_step * out.width;
  }
  out.header.frame_id = frame_id;
  const google::protobuf::Timestamp& sim_time = lidar_output.metadata().sensor_timestamp();
  out.header.stamp = ros_time_ns::Time(sim_time.seconds(), sim_time.nanos());
  out.is_dense = true;
  out.height = 1;
  out.is_bigendian = false;
  return "";
}

std::string BuildRosRadarCloud(
    const simian_public::sensor_model::SensorOutput::RadarTrack& radar_output,
    const std::string& frame_id, sensor_msgs_ns::PointCloud2& out) {
  const auto [num_points, err_msg] = applied::GetNumRadarPoints(radar_output);
  if (!err_msg.empty()) {
    return err_msg;
  }

  SetPointCloudFields(kROSRadarFieldNames, out);

  size_t points_written = 0;
  out.data.resize(sizeof(applied::RadarPoint) * num_points);
  applied::ForEachRadarPoint(radar_output, [&](const ::applied::RadarPoint& radar_point) {
    points_written++;
    memcpy(out.data.data() + (points_written - 1) * sizeof(radar_point), &radar_point,
           sizeof(radar_point));
    return "";
  });

  out.header.frame_id = frame_id;
  const google::protobuf::Timestamp& sim_time = radar_output.metadata().sensor_timestamp();
  out.header.stamp = ros_time_ns::Time(sim_time.seconds(), sim_time.nanos());
  out.is_dense = true;
  out.height = 1;
  out.is_bigendian = false;
  out.width = points_written;
  out.point_step = sizeof(::applied::RadarPoint);
  out.row_step = out.point_step * out.width;

  return "";
}

}  // namespace applied

#endif  // if defined(APPLIED_HAVE_ROS)
