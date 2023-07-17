package(default_visibility = ["//visibility:public"])

cc_library(
  name = "api_def",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public:api_def.h"],
  include_prefix="applied/simian/public",
)

cc_library(
  name = "proto_def",
  deps = ["@com_google_protobuf//:protobuf"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public:proto_def.cc"],
  hdrs = ["//simian/public:proto_def.h"],
  include_prefix="applied/simian/public",
)

