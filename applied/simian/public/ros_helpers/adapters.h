#pragma once

#include <string_view>

#include <google/protobuf/timestamp.pb.h>

#if __has_include(<geometry_msgs/msg/transform_stamped.hpp>)
#define APPLIED_HAVE_ROS2 1

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <rclcpp/time.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace applied::ros::adapters {
using ros_seconds_t = int32_t;
namespace geometry_msgs_ns = ::geometry_msgs::msg;
namespace sensor_msgs_ns = ::sensor_msgs::msg;
namespace std_msgs_ns = ::std_msgs::msg;
namespace time_ns = ::rclcpp;
namespace visualization_msgs_ns = ::visualization_msgs::msg;
}  // namespace applied::ros::adapters

#elif __has_include(<geometry_msgs/TransformStamped.h>)
#define APPLIED_HAVE_ROS1 1

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

namespace applied::ros::adapters {
using ros_seconds_t = uint32_t;
namespace geometry_msgs_ns = ::geometry_msgs;
namespace sensor_msgs_ns = ::sensor_msgs;
namespace std_msgs_ns = ::std_msgs;
namespace time_ns = ::ros;
namespace visualization_msgs_ns = ::visualization_msgs;
}  // namespace applied::ros::adapters

#endif  // __has_include(...)

#include "applied/simian/public/proto/actor.pb.h"
#include "applied/simian/public/proto/ego.pb.h"
#include "applied/simian/public/proto/perception.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"

#if defined(APPLIED_HAVE_ROS1) || defined(APPLIED_HAVE_ROS2)

namespace applied::ros::adapters {

/**
 * @brief Create a ROS Vector3 object from a Simian pose.
 *
 * @param pose object containing a position.
 * @returns Vector3 object
 */
geometry_msgs_ns::Vector3 AdaptVector3(const simian_public::spatial::Pose& pose);

/**
 * @brief Create a ROS Point object from a Simian pose.
 *
 * @param pose object containing a position.
 * @returns Point object
 */
geometry_msgs_ns::Point AdaptPoint(const simian_public::spatial::Pose& pose);

/**
 * @brief Create a ROS Quaternion object from a Simian orientation.
 *
 * @param pose object containing an orientation.
 * @returns Quaternion object
 */
geometry_msgs_ns::Quaternion AdaptQuaternion(const simian_public::spatial::Pose& pose);

/**
 * @brief Create a ROS Time object from a Simian timestamp.
 *
 * @param sim_time timestamp from Simian.
 * @returns Time object
 */
time_ns::Time AdaptRosTime(const ::google::protobuf::Timestamp& sim_time);

/**
 * @brief Create a ROS TransformStamped object from a Simian pose.
 *
 * @param pose parent_frame_id name of the reference frame.
 * @param pose child_frame_id name of the transform frame.
 * @param sim_time timestamp from Simian.
 * @param pose object from Simian.
 * @returns TransformStamped object.
 */
geometry_msgs_ns::TransformStamped AdaptStampedTransform(
    std::string_view parent_frame_id, std::string_view child_frame_id,
    const google::protobuf::Timestamp& sim_time, const simian_public::spatial::Pose& pose);

/**
 * @brief Create a ROS TransformStamped object from the ego's pose.
 *
 * @param sim_time timestamp from Simian.
 * @param pose object from Simian.
 * @returns TransformStamped object.
 */
geometry_msgs_ns::TransformStamped AdaptEgoTransform(const google::protobuf::Timestamp& sim_time,
                                                     const simian_public::spatial::Pose& ego_pose);

/**
 * @brief Create a ROS PoseStamped object from a Simian pose.
 *
 * This may be more useful than TransformStamped because RViz has a default plugin
 * for visualizing PoseStamped messages. However, note that PoseStamped has no
 * "child_frame_id", so it is more implicit.
 *
 * @param pose parent_frame_id name of the reference frame.
 * @param sim_time timestamp from Simian.
 * @param pose object from Simian.
 * @returns PoseStamped object.
 */
geometry_msgs_ns::PoseStamped AdaptStampedPose(std::string_view parent_frame_id,
                                               const google::protobuf::Timestamp& sim_time,
                                               const simian_public::spatial::Pose& pose);

/**
 * @brief Create a ROS Imu object to match the ego's imu.
 *
 * @param sim_time timestamp from Simian.
 * @param ego_section object from Simian representing the ego body.
 * @returns Imu object.
 */
sensor_msgs_ns::Imu AdaptEgoImu(const google::protobuf::Timestamp& sim_time,
                                const simian_public::ego::EgoOutput::Section& ego_section);

/**
 * @brief Create a green RGB color object.
 *
 * @returns ColorRGBA object.
 */
std_msgs_ns::ColorRGBA DefaultMarkerColor();

/**
 * @brief Create a ROS Marker object to match a Simian actor.
 *
 * @param sim_time timestamp from Simian.
 * @param actor object from Simian.
 * @param marker_color color to set the marker.
 * @returns Marker object
 */
visualization_msgs_ns::Marker AdaptActorMarker(const google::protobuf::Timestamp& sim_time,
                                               const simian_public::actor::Actor& actor,
                                               const std_msgs_ns::ColorRGBA& marker_color);

/**
 * @brief Create a ROS MarkerArray object to match a group of Simian actors.
 *
 * @param sim_time timestamp from Simian.
 * @param actor_sensor object from Simian containing a list of actors.
 * @param marker_color color to set the marker.
 * @returns MarkerArray object.
 */
visualization_msgs_ns::MarkerArray AdaptActorMarkers(
    const google::protobuf::Timestamp& sim_time,
    const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor,
    const std_msgs_ns::ColorRGBA& marker_color = DefaultMarkerColor());

}  // namespace applied::ros::adapters

#endif  // APPLIED_HAVE_ROS1 || APPLIED_HAVE_ROS2

// Alias applied::ros1::adapters as applied::ros::adapters for legacy support
#ifdef APPLIED_HAVE_ROS1
namespace applied {
namespace ros1 {
namespace adapters = ::applied::ros::adapters;
}  // namespace ros1
}  // namespace applied
#endif  // APPLIED_HAVE_ROS1
