package(default_visibility = ["//visibility:public"])

cc_library(
  name = "simulink_automator_core",
  deps = ["@com_github_grpc_grpc//:grpc++", "//simian/public/proto:common_cc", "//simian/public/proto:simulink_automator_service_cc", "//simian/public/proto:simulink_automator_service_cc_grpc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/simulink/simulink_automator:simulink_automator_core.cc"],
  hdrs = ["//simian/public/simulink/simulink_automator:simulink_automator_core.h"],
  include_prefix="applied/simian/public/simulink/simulink_automator",
)

