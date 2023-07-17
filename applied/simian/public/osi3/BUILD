package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "osi_common",
  deps = [],
  srcs = ["//simian/public/osi3:osi_common.proto"],
)

applied_proto_library(
  name = "osi_environment",
  deps = [],
  srcs = ["//simian/public/osi3:osi_environment.proto"],
)

applied_proto_library(
  name = "osi_groundtruth",
  deps = ["//simian/public/osi3:osi_common", "//simian/public/osi3:osi_environment", "//simian/public/osi3:osi_lane", "//simian/public/osi3:osi_object", "//simian/public/osi3:osi_occupant", "//simian/public/osi3:osi_roadmarking", "//simian/public/osi3:osi_trafficlight", "//simian/public/osi3:osi_trafficsign", "//simian/public/osi3:osi_version"],
  srcs = ["//simian/public/osi3:osi_groundtruth.proto"],
)

applied_proto_library(
  name = "osi_lane",
  deps = ["//simian/public/osi3:osi_common"],
  srcs = ["//simian/public/osi3:osi_lane.proto"],
)

applied_proto_library(
  name = "osi_object",
  deps = ["//simian/public/osi3:osi_common"],
  srcs = ["//simian/public/osi3:osi_object.proto"],
)

applied_proto_library(
  name = "osi_occupant",
  deps = ["//simian/public/osi3:osi_common"],
  srcs = ["//simian/public/osi3:osi_occupant.proto"],
)

applied_proto_library(
  name = "osi_roadmarking",
  deps = ["//simian/public/osi3:osi_common", "//simian/public/osi3:osi_trafficsign"],
  srcs = ["//simian/public/osi3:osi_roadmarking.proto"],
)

applied_proto_library(
  name = "osi3",
  deps = ["//simian/public/osi3:osi_common", "//simian/public/osi3:osi_environment", "//simian/public/osi3:osi_groundtruth", "//simian/public/osi3:osi_lane", "//simian/public/osi3:osi_object", "//simian/public/osi3:osi_occupant", "//simian/public/osi3:osi_roadmarking", "//simian/public/osi3:osi_trafficlight", "//simian/public/osi3:osi_trafficsign", "//simian/public/osi3:osi_version"],
  srcs = ["//simian/public/osi3:osi_sentinel.proto"],
)

applied_proto_library(
  name = "osi_trafficlight",
  deps = ["//simian/public/osi3:osi_common"],
  srcs = ["//simian/public/osi3:osi_trafficlight.proto"],
)

applied_proto_library(
  name = "osi_trafficsign",
  deps = ["//simian/public/osi3:osi_common"],
  srcs = ["//simian/public/osi3:osi_trafficsign.proto"],
)

applied_proto_library(
  name = "osi_version",
  deps = ["@com_google_protobuf//:descriptor_proto"],
  srcs = ["//simian/public/osi3:osi_version.proto"],
)

