package(default_visibility = ["//visibility:public"])

cc_library(
  name = "compatibility",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public/simulink:compatibility.h"],
  include_prefix="applied/simian/public/simulink",
)

cc_library(
  name = "ip_address_validator",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/simulink:ip_addr_validator.cc"],
  hdrs = ["//simian/public/simulink:ip_addr_validator.h"],
  include_prefix="applied/simian/public/simulink",
)

cc_library(
  name = "simulink_bridge_manager",
  deps = ["//simian/public/simulink:simulink_version", "//simian/public/simulink:zmq_services", "//simian/public/proto:common_cc", "//simian/public/proto:simulink_cc", "//simian/public/proto/scenario:simulink_config_cc", "//simian/public/simulink/simulink_automator:simulink_automator_core"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/simulink:simulink_bridge_manager.cc"],
  hdrs = ["//simian/public/simulink:simulink_bridge_manager.h"],
  include_prefix="applied/simian/public/simulink",
)

cc_library(
  name = "simulink_interface_helper",
  tags = ["nowindows"],
  deps = ["//simian/public/simulink:ip_address_validator", "//simian/public/simulink:simulink_bridge_manager", "//simian/public/utils:string_view"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/simulink:simulink_interface_helper.cc"],
  hdrs = ["//simian/public/simulink:simulink_interface_helper.h"],
  include_prefix="applied/simian/public/simulink",
)

cc_library(
  name = "simulink_version",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  hdrs = ["//simian/public/simulink:simulink_version.h"],
  include_prefix="applied/simian/public/simulink",
)

cc_library(
  name = "zmq_services",
  deps = ["//simian/public/simulink:compatibility", "//third_party:libzmq"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/simulink:zmq_services.cc"],
  hdrs = ["//simian/public/simulink:zmq_services.h"],
  include_prefix="applied/simian/public/simulink",
)

