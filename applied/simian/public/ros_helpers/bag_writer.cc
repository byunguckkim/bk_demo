#include "applied/simian/public/ros_helpers/bag_writer.h"

#if __has_include(<geometry_msgs/TransformStamped.h>)
// This is a system with ROS1 installed

#include "rosbag/stream.h"

namespace {

std::string GetTfTopicName(bool latched) {
  if (latched) {
    return "/tf_static";
  } else {
    return "/tf";
  }
}

}  // namespace

namespace applied::ros1 {

BagWriter::BagWriter(const std::string& path) : bag_path_(path) {}

bool BagWriter::Open() {
  bag_.open(bag_path_, rosbag::bagmode::Write);
  // If your version of rosbag supports it, you can reduce your bag size
  // considerably by enabling some compression.
  // bag_.setCompression(::rosbag::compression::LZ4);
  return IsOpen();
}

void BagWriter::WriteTransform(const geometry_msgs::TransformStamped& transform, bool latched,
                               const ros::Time& time) {
  // TransformBroadcaster wraps transforms into a TFMessage before publishing. We must do the same
  // when saving to a bag otherwise RVIZ will complain the messages are not the expected type.
  tf2_msgs::TFMessage tf2_msg;
  tf2_msg.transforms.push_back(transform);

  Write(GetTfTopicName(latched), tf2_msg, latched, time);
}

void BagWriter::WriteTransforms(const std::vector<geometry_msgs::TransformStamped>& transforms,
                                bool latched, const ros::Time& time) {
  // TransformBroadcaster wraps transforms into a TFMessage before publishing. We must do the same
  // when saving to a bag otherwise RVIZ will complain the messages are not the expected type.
  tf2_msgs::TFMessage tf2_msg;
  for (const geometry_msgs::TransformStamped& transform : transforms) {
    tf2_msg.transforms.push_back(transform);
  }
  Write(GetTfTopicName(latched), tf2_msg, latched, time);
}

}  // namespace applied::ros1

#endif  // if __has_include(<geometry_msgs/TransformStamped.h>)
