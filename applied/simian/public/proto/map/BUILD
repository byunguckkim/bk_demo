package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "map",
  deps = ["//simian/public/proto/map:map_clear_area", "//simian/public/proto/map:map_crosswalk", "//simian/public/proto/map:map_junction", "//simian/public/proto/map:map_lane", "//simian/public/proto/map:map_line", "//simian/public/proto/map:map_parking_space", "//simian/public/proto/map:map_region", "//simian/public/proto/map:map_road", "//simian/public/proto/map:map_road_marking", "//simian/public/proto/map:map_sidewalk", "//simian/public/proto/map:map_sign", "//simian/public/proto/map:map_signal", "//simian/public/proto/map:map_signal_control_states", "//simian/public/proto/map:map_speed_bump", "//simian/public/proto/map:map_traversable_surface", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto/map:map.proto"],
)

applied_proto_library(
  name = "map_clear_area",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_clear_area.proto"],
)

applied_proto_library(
  name = "map_common",
  deps = [],
  srcs = ["//simian/public/proto/map:map_common.proto"],
)

applied_proto_library(
  name = "map_crosswalk",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_crosswalk.proto"],
)

applied_proto_library(
  name = "map_enums",
  deps = [],
  srcs = ["//simian/public/proto/map:map_enums.proto"],
)

applied_proto_library(
  name = "map_geometry",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto:geometry"],
  srcs = ["//simian/public/proto/map:map_geometry.proto"],
)

applied_proto_library(
  name = "map_id",
  deps = [],
  srcs = ["//simian/public/proto/map:map_id.proto"],
)

applied_proto_library(
  name = "map_junction",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_junction.proto"],
)

applied_proto_library(
  name = "map_lane",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_validation"],
  srcs = ["//simian/public/proto/map:map_lane.proto"],
)

applied_proto_library(
  name = "map_lane_markings",
  deps = ["//simian/public/proto/map:map_enums", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_lane"],
  srcs = ["//simian/public/proto/map:map_lane_markings.proto"],
)

applied_proto_library(
  name = "map_line",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_validation", "//simian/public/proto:common"],
  srcs = ["//simian/public/proto/map:map_line.proto"],
)

applied_proto_library(
  name = "map_parking_space",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_parking_space.proto"],
)

applied_proto_library(
  name = "map_region",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_validation", "//simian/public/proto:common"],
  srcs = ["//simian/public/proto/map:map_region.proto"],
)

applied_proto_library(
  name = "map_road",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_road.proto"],
)

applied_proto_library(
  name = "map_road_marking",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto:common", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto/map:map_road_marking.proto"],
)

applied_proto_library(
  name = "map_semantic",
  deps = ["//simian/public/proto/map:map_enums", "//simian/public/proto/map:map_validation", "//simian/public/proto:geometry", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto/map:map_semantic.proto"],
)

applied_proto_library(
  name = "map_sidewalk",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_sidewalk.proto"],
)

applied_proto_library(
  name = "map_sign",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_validation", "//simian/public/proto:spatial", "//simian/public/proto:spectral_assets"],
  srcs = ["//simian/public/proto/map:map_sign.proto"],
)

applied_proto_library(
  name = "map_signal",
  deps = ["//simian/public/proto/map:map_common", "//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id", "//simian/public/proto/map:map_validation", "//simian/public/osi3:osi_trafficlight", "//simian/public/proto:geometry", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto/map:map_signal.proto"],
)

applied_proto_library(
  name = "map_signal_control_states",
  deps = ["//simian/public/proto:traffic_light"],
  srcs = ["//simian/public/proto/map:map_signal_control_states.proto"],
)

applied_proto_library(
  name = "map_speed_bump",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_speed_bump.proto"],
)

applied_proto_library(
  name = "map_traversable_surface",
  deps = ["//simian/public/proto/map:map_geometry", "//simian/public/proto/map:map_id"],
  srcs = ["//simian/public/proto/map:map_traversable_surface.proto"],
)

applied_proto_library(
  name = "map_validation",
  deps = ["//simian/public/proto/map:map_enums", "//simian/public/proto/map:map_id", "//simian/public/proto:common", "//simian/public/proto:geometry"],
  srcs = ["//simian/public/proto/map:map_validation.proto"],
)

applied_proto_library(
  name = "procedural_map_proto",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:drawing", "//simian/public/proto:geometry", "//simian/public/proto:spatial", "//simian/public/proto/scenario:common", "//simian/public/proto/sensor_sim:asset_tags", "//simian/public/proto/sensor_sim:models"],
  srcs = ["//simian/public/proto/map:procedural_map.proto"],
)

