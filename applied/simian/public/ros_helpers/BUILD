package(default_visibility = ["//visibility:public"])

cc_library(
  name = "ros_helpers",
  deps = ["//simian/public/proto:actor_cc", "//simian/public/proto:ego_cc", "//simian/public/proto:perception_cc", "//simian/public/proto:spatial_cc", "@ros_common_msgs//:geometry_msgs_cc", "@ros_common_msgs//:sensor_msgs_cc", "@ros_common_msgs//:visualization_msgs_cc", "@ros_geometry2//:tf2_msgs_cc", "@roscomm//:rosbag", "@roscpp_core//:roscore"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/ros_helpers:adapters.cc", "//simian/public/ros_helpers:bag_writer.cc"],
  hdrs = ["//simian/public/ros_helpers:adapters.h", "//simian/public/ros_helpers:bag_writer.h"],
  include_prefix="applied/simian/public/ros_helpers",
)

