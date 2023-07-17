// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.

#pragma once

#include <string>
#include <string_view>

#include <google/protobuf/struct.pb.h>

#if __has_include(<sensor_msgs/msg/image.hpp>)
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#define APPLIED_HAVE_ROS 1
namespace applied {
namespace sensor_msgs_ns = ::sensor_msgs::msg;
}

#elif __has_include(<sensor_msgs/Image.h>)
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>

#define APPLIED_HAVE_ROS 1
namespace applied {
namespace sensor_msgs_ns = ::sensor_msgs;
}
#endif  // __has_include(...)

#include "applied/simian/public/proto/sensor_model.pb.h"

#if defined(APPLIED_HAVE_ROS)

namespace applied {

/**
 * @brief Create a ROS CameraInfo object that corresponds to the given Spectral CameraImage.
 *
 * @param camera_image protocol buffer representing a camera output
 * @param frame_id name of the camera's tf2 reference frame
 * @param out object to mutate with data from the given CameraImage.
 * @returns an empty string on success or a non-empty error message.
 */
std::string BuildRosCameraInfo(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::CameraInfo& out);

/**
 * @brief Create a ROS CompressedImage from the given Spectral CameraImage
 *
 * Note that the CompressedImage will contain the given camera data as a PNG.
 *
 * @param camera_image protocol buffer representing a camera output
 * @param frame_id name of the camera's tf2 reference frame
 * @param out object to mutate with data from the given CameraImage.
 * @returns an empty string on success or a non-empty error message.
 */
std::string BuildRosCompressedImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::CompressedImage& out);

/**
 * @brief Create a ROS Image from the given Spectral CameraImage
 *
 * @param camera_image protocol buffer representing a camera output
 * @param frame_id name of the camera's tf2 reference frame
 * @param out object to mutate with data from the given CameraImage.
 * @returns an empty string on success or a non-empty error message.
 */
std::string BuildRosImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::string_view frame_id, sensor_msgs_ns::Image& out);

/**
 * @brief Create a ROS PointCloud2 from the given Spectral LidarCloud
 *
 * @param lidar_output protocol buffer representing a lidar output from Spectral
 * @param frame_id name of the lidar's tf2 reference frame
 * @param out object to mutate with data from the given LidarCloud
 * @returns an empty string on success or a non-empty error message.
 */
std::string BuildRosLidarCloud(
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_output,
    std::string_view frame_id, sensor_msgs_ns::PointCloud2& out);

/**
 * @brief Create a ROS PointCloud2 from the given Spectral RadarTrack
 *
 * @param radar_output protocol buffer representing a radar output from Spectral
 * @param frame_id name of the radar's tf2 reference frame
 * @param out object to mutate with data from the given RadarTrack
 * @returns an empty string on success or a non-empty error message.
 */
std::string BuildRosRadarCloud(
    const simian_public::sensor_model::SensorOutput::RadarTrack& radar_output,
    const std::string& frame_id, sensor_msgs_ns::PointCloud2& out);

/**
 * @brief Calculate focal length from camera image field of view and resolution
 *
 * @param camera_image Protobuf message containing an image and information about the camera
 * @param out_fx double to mutate with the camera's focal length in units of pixel width
 * @param out_fy double to mutate with the camera's focal length in units of pixel height
 * @param out_fov_x double to mutate with the horizontal field of view
 * @param out_fov_y double to mutate with the vertical field of view
 */
void CalculateFocalLengths(
    const simian_public::sensor_model::SensorOutput::CameraImage& camera_image, double& out_fx,
    double& out_fy, double& out_fov_x, double& out_fov_y);
}  // namespace applied

#endif  // if defined(APPLIED_HAVE_ROS)
