package(default_visibility = ["//visibility:public"])

cc_library(
  name = "fmu_manager",
  tags = ["nowindows"],
  deps = ["//simian/public:proto_def", "//simian/public/proto:common_cc", "//simian/public/proto:fmu_config_cc", "@com_google_protobuf//:protobuf", "@fmi4cpp//:fmi4cpp", "@nlohmann_json//:json"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//simian/public/fmu:fmu_manager.cc"],
  hdrs = ["//simian/public/fmu:fmu_manager.h"],
  include_prefix="applied/simian/public/fmu",
)

