package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "asset_tags",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:asset_tags.proto"],
)

applied_proto_library(
  name = "basic_types",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:basic_types.proto"],
)

applied_proto_library(
  name = "camera_model",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim:camera_model.proto"],
)

applied_proto_library(
  name = "fragment_description",
  deps = ["//simian/public/proto:planar"],
  srcs = ["//simian/public/proto/sensor_sim:fragment_description.proto"],
)

applied_proto_library(
  name = "image_mapping",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim:image_mapping.proto"],
)

applied_proto_library(
  name = "imported_vehicle_description",
  deps = ["//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:mesh_description"],
  srcs = ["//simian/public/proto/sensor_sim:imported_vehicle_description.proto"],
)

applied_proto_library(
  name = "keypoints",
  deps = ["//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto/sensor_sim:keypoints.proto"],
)

applied_proto_library(
  name = "labels",
  deps = ["//simian/public/proto/sensor_sim:basic_types", "//simian/public/proto/sensor_sim:models", "//simian/public/proto:common", "//simian/public/proto:spatial", "//simian/public/proto:spectral_assets", "//simian/public/proto/map:map_enums", "//simian/public/proto/map:map_lane", "//simian/public/proto/map:map_region", "//simian/public/proto/sensor_sim:keypoints", "//simian/public/proto/sensor_sim:model_output"],
  srcs = ["//simian/public/proto/sensor_sim:labels.proto"],
)

applied_proto_library(
  name = "mesh_description",
  deps = ["//simian/public/proto:geometry"],
  srcs = ["//simian/public/proto/sensor_sim:mesh_description.proto"],
)

applied_proto_library(
  name = "model_output",
  deps = ["//simian/public/proto/sensor_sim:model_spec", "//simian/public/proto:spectral_assets"],
  srcs = ["//simian/public/proto/sensor_sim:model_output.proto"],
)

applied_proto_library(
  name = "model_spec",
  deps = ["//simian/public/proto/sensor_sim:models", "//simian/public/proto/sensor_sim:pedestrian_props", "//simian/public/proto/sensor_sim:spectral_materials", "//simian/public/proto:common", "//simian/public/proto:geometry", "//simian/public/proto:spatial", "//simian/public/proto:spectral_assets", "//simian/public/proto/sensor_sim:fragment_description", "//simian/public/proto/sensor_sim:imported_vehicle_description", "//simian/public/proto/sensor_sim/materials:material_description", "//simian/public/proto/sensor_sim/pedestrians:spectral_pedestrian"],
  srcs = ["//simian/public/proto/sensor_sim:model_spec.proto"],
)

applied_proto_library(
  name = "models",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:models.proto"],
)

applied_proto_library(
  name = "pedestrian_props",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:pedestrian_props.proto"],
)

applied_proto_library(
  name = "sensor_behavior",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:sensor_behavior.proto"],
)

applied_proto_library(
  name = "spectral_map_config",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim:spectral_map_config.proto"],
)

applied_proto_library(
  name = "spectral_materials",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto/sensor_sim:spectral_materials.proto"],
)

applied_proto_library(
  name = "spectral_rendering",
  deps = [],
  srcs = ["//simian/public/proto/sensor_sim:spectral_rendering.proto"],
)

