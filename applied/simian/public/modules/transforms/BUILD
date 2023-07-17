package(default_visibility = ["//visibility:public"])

cc_library(
  name = "eigen_headers",
  deps = ["@eigen//:eigen"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public/modules/transforms:eigen.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "map_util",
  deps = ["@geographiclib//:geographiclib"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/modules/transforms:map_util.cc"],
  hdrs = ["//simian/public/modules/transforms:map_util.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "planar",
  deps = ["//simian/public/modules/transforms:eigen_headers"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/modules/transforms:planar.cc"],
  linkstatic = True,
  hdrs = ["//simian/public/modules/transforms:planar.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "proto_conversion",
  deps = ["//simian/public/modules/transforms:planar", "//simian/public/modules/transforms:spatial", "//simian/public/modules/transforms:transform_forest", "//simian/public/proto:planar_cc", "//simian/public/proto:spatial_cc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror=switch"],
  srcs = ["//simian/public/modules/transforms:proto_conversion.cc"],
  hdrs = ["//simian/public/modules/transforms:proto_conversion.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "scalar",
  deps = ["//simian/public/utils:optional"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/modules/transforms:scalar.cc"],
  linkstatic = True,
  hdrs = ["//simian/public/modules/transforms:scalar.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "spatial",
  deps = ["//simian/public/modules/transforms:eigen_headers", "//simian/public/modules/transforms:planar"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/modules/transforms:spatial.cc"],
  linkstatic = True,
  hdrs = ["//simian/public/modules/transforms:spatial.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

cc_library(
  name = "transform_forest",
  deps = ["//simian/public/modules/transforms:spatial", "//simian/public/utils:optional"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/modules/transforms:transform_forest.cc"],
  hdrs = ["//simian/public/modules/transforms:transform_forest.h"],
  include_prefix="applied/simian/public/modules/transforms",
)

