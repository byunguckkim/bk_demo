package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

cc_import(
  name = "global_proto_import_no_runfiles",
  hdrs = ["//simian/public/simulink/simulink_global_proto:global_simulink_proto.h"],
  shared_library = "//simian/public/simulink/simulink_global_proto:libglobalproto",
  deps = ["//simian/public:api_def"],
  include_prefix="applied/simian/public/simulink/simulink_global_proto",
)

