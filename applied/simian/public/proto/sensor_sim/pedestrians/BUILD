package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "animations",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim/pedestrians:animations.proto"],
)

applied_proto_library(
  name = "body",
  deps = ["//simian/public/proto/sensor_sim/pedestrians:clothing", "//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim/pedestrians:body.proto"],
)

applied_proto_library(
  name = "clothing",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim/pedestrians:clothing.proto"],
)

applied_proto_library(
  name = "spectral_pedestrian",
  deps = ["//simian/public/proto/sensor_sim/pedestrians:animations", "//simian/public/proto/sensor_sim/pedestrians:body"],
  srcs = ["//simian/public/proto/sensor_sim/pedestrians:spectral_pedestrian.proto"],
)

