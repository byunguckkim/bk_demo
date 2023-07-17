package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "common_material_properties",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim/materials:common_material_properties.proto"],
)

applied_proto_library(
  name = "lane_line_material",
  deps = ["//simian/public/proto/sensor_sim/materials:common_material_properties"],
  srcs = ["//simian/public/proto/sensor_sim/materials:lane_line_material.proto"],
)

applied_proto_library(
  name = "material_description",
  deps = ["//simian/public/proto/sensor_sim/materials:lane_line_material", "//simian/public/proto/sensor_sim/materials:object_material", "//simian/public/proto/sensor_sim/materials:surface_material"],
  srcs = ["//simian/public/proto/sensor_sim/materials:material_description.proto"],
)

applied_proto_library(
  name = "object_material",
  deps = ["//simian/public/proto/sensor_sim/materials:common_material_properties", "//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim/materials:object_material.proto"],
)

applied_proto_library(
  name = "surface_material",
  deps = ["//simian/public/proto/sensor_sim/materials:common_material_properties", "//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim/materials:surface_material.proto"],
)

