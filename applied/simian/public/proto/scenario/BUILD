package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "common",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto/scenario:common.proto"],
)

applied_proto_library(
  name = "scenario_digest",
  deps = ["//simian/public/proto/scenario:common", "//simian/public/proto/scenario:sensor_config", "//simian/public/proto/scenario:simulink_config", "//simian/public/proto/scenario:vehicle_config", "//simian/public/proto/scenario:vehiclesim_config", "//simian/public/proto:environment", "//simian/public/proto:helix_common", "//simian/public/proto:motion_model", "//simian/public/proto:shape", "//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:image_mapping", "//simian/public/proto/sensor_sim:labels"],
  srcs = ["//simian/public/proto/scenario:scenario_digest.proto"],
)

applied_proto_library(
  name = "sensor_config",
  deps = ["//simian/public/proto:common", "//simian/public/proto:field_options_proto", "//simian/public/proto:planar", "//simian/public/proto:sensor_model", "//simian/public/proto:spatial", "//simian/public/proto/map:map_enums", "//simian/public/proto/scenario:common", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto/scenario:sensor_config.proto"],
)

applied_proto_library(
  name = "simulink_config",
  deps = ["@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto/scenario:simulink_config.proto"],
)

applied_proto_library(
  name = "vehicle_config",
  deps = ["//simian/public/proto/scenario:common", "//simian/public/proto/scenario:sensor_config", "//simian/public/proto/scenario:vehiclesim_config", "//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:composite_vehicle", "//simian/public/proto:field_options_proto", "//simian/public/proto:geometry", "//simian/public/proto:interactive_field", "//simian/public/proto:motion_model", "//simian/public/proto:motion_state_modifier", "//simian/public/proto:physics_engine", "//simian/public/proto:planar", "//simian/public/proto:shape", "//simian/public/proto:spatial", "//simian/public/proto:spectral_assets", "//simian/public/proto/sensor_sim:model_spec", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:empty_proto"],
  srcs = ["//simian/public/proto/scenario:vehicle_config.proto"],
)

applied_proto_library(
  name = "vehiclesim_config",
  deps = ["//simian/public/proto:common", "//simian/public/proto:field_options_proto"],
  srcs = ["//simian/public/proto/scenario:vehiclesim_config.proto"],
)

