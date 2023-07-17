package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "customer_service_v2",
  deps = ["//simian/public/proto/v2:io", "//simian/public/proto:common", "//simian/public/proto:drawing", "//simian/public/proto:sim_command", "//simian/public/proto:stack_logs", "//simian/public/proto:triage_proto"],
  srcs = ["//simian/public/proto/v2:customer_service_v2.proto"],
)

applied_proto_library(
  name = "io",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:ego", "//simian/public/proto:integration_definition_compiled", "//simian/public/proto:map_config", "//simian/public/proto:sim_data", "//simian/public/proto:spatial", "//simian/public/proto:stack_state", "//simian/public/proto/scenario:scenario_digest", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:empty_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto/v2:io.proto"],
)

