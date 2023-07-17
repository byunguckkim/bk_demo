package(default_visibility = ["//visibility:public"])

cc_library(
  name = "simian_map_client_cc",
  deps = ["@com_github_grpc_grpc//:grpc++", "//simian/public/proto:public_map_service_proto_cc", "//simian/public/proto:public_map_service_proto_cc_grpc", "//simian/public/proto:terrain_config_cc", "@com_github_grpc_grpc//:grpc++_reflection"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/map:simian_map_client.cc"],
  hdrs = ["//simian/public/map:simian_map_client.h"],
  include_prefix="applied/simian/public/map",
)

