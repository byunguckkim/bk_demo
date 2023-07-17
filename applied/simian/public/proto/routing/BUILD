package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "error_code_proto",
  deps = [],
  srcs = ["//simian/public/proto/routing:error_code.proto"],
)

applied_proto_library(
  name = "header_proto",
  deps = ["//simian/public/proto/routing:error_code_proto"],
  srcs = ["//simian/public/proto/routing:header.proto"],
)

applied_proto_library(
  name = "routing_proto",
  deps = ["//simian/public/proto/routing:error_code_proto", "//simian/public/proto/routing:header_proto", "//simian/public/proto:geometry", "//simian/public/proto/map:map_parking_space"],
  srcs = ["//simian/public/proto/routing:routing.proto"],
)

