package(default_visibility = ["//visibility:public"])

cc_library(
  name = "optional",
  tags = ["third_party"],
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public/utils:optional.h"],
  include_prefix="applied/simian/public/utils",
)

cc_library(
  name = "string_view",
  tags = ["third_party"],
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public/utils:string_view.h"],
  include_prefix="applied/simian/public/utils",
)

