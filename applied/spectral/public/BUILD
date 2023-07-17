package(default_visibility = ["//visibility:public"])

cc_library(
  name = "depth_image_projector",
  deps = ["//spectral/public:image_projector"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  srcs = ["//spectral/public:depth_image_projector.cc"],
  hdrs = ["//spectral/public:depth_image_projector.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "extra_data_helpers",
  deps = ["//simian/public/proto/v2:io_cc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = [],
  hdrs = ["//spectral/public:extra_data_helpers.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "image_helpers",
  deps = ["//spectral/public:image_point", "//spectral/public:image_projector", "//spectral/public:ndarray_helpers", "//spectral/public:shared_memory_helper", "//spectral/public:utils", "//simian/public/proto:sensor_model_cc", "//simian/public/modules/transforms:spatial", "//simian/public/utils:optional", "@nlohmann_json//:json", "@opencv//:core", "@opencv//:imgcodecs", "@opencv//:imgproc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror", "-Wall"],
  srcs = ["//spectral/public:image_helpers.cc"],
  hdrs = ["//spectral/public:image_helpers.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "image_point",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:image_point.cc"],
  hdrs = ["//spectral/public:image_point.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "image_projector",
  deps = ["//spectral/public:image_point", "//simian/public/proto:sensor_model_cc", "//simian/public/modules/transforms:spatial", "@eigen//:eigen", "@opencv//:calib3d", "@opencv//:core"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:image_projector.cc", "//spectral/public:image_projector_impl.cc"],
  hdrs = ["//spectral/public:image_projector.h", "//spectral/public:image_projector_impl.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "label_visualization",
  tags = ["nowindows"],
  deps = ["//spectral/public:image_helpers", "//spectral/public:image_projector", "//spectral/public:utils", "//simian/public/proto:sensor_model_cc", "//simian/public/proto/sensor_sim:basic_types_cc", "//simian/public/proto/sensor_sim:keypoints_cc", "//simian/public/proto/sensor_sim:labels_cc", "//simian/public/modules/transforms:proto_conversion", "@opencv//:core", "@opencv//:imgproc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:label_visualization.cc"],
  hdrs = ["//spectral/public:label_visualization.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "lidar_helpers",
  tags = ["nowindows"],
  deps = ["//spectral/public:shared_memory_helper", "//spectral/public:utils", "//simian/public/proto:sensor_model_cc", "//simian/public/proto:spatial_cc", "@nlohmann_json//:json", "@opencv//:core", "@opencv//:imgcodecs", "@opencv//:imgproc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:lidar_helpers.cc"],
  hdrs = ["//spectral/public:lidar_helpers.h", "//spectral/public:lidar_point.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "shared_memory_helper",
  deps = ["//spectral/public:scoped_descriptor", "//simian/public/proto:sensor_model_cc", "//simian/public/proto:sim_data_cc", "@com_google_protobuf//:protobuf"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:mmap_region.cc", "//spectral/public:spectral_shared_memory_helper.cc"],
  hdrs = ["//spectral/public:mmap_region.h", "//spectral/public:spectral_shared_memory_helper.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "ndarray_helpers",
  deps = ["//spectral/public:shared_memory_helper", "//simian/public/proto:sensor_model_cc", "//simian/public/proto:sensor_output_cc", "//simian/public/proto/sensor_sim:basic_types_cc"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:ndarray_helpers.cc"],
  hdrs = ["//spectral/public:ndarray_helpers.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "radar_helpers",
  tags = ["nowindows"],
  deps = ["//spectral/public:shared_memory_helper", "//spectral/public:utils", "//simian/public/proto:sensor_model_cc", "@nlohmann_json//:json"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:radar_helpers.cc"],
  hdrs = ["//spectral/public:radar_helpers.h", "//spectral/public:radar_point.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "ros_helpers",
  tags = ["nowindows"],
  deps = ["//spectral/public:image_helpers", "//spectral/public:lidar_helpers", "//spectral/public:radar_helpers", "//spectral/public:shared_memory_helper", "//simian/public/proto:sensor_model_cc", "@opencv//:core", "@ros_common_msgs//:sensor_msgs_lib", "@roscpp_core//:roscore"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:ros_helpers.cc"],
  hdrs = ["//spectral/public:ros_helpers.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "scoped_descriptor",
  deps = [] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = [],
  hdrs = ["//spectral/public:scoped_descriptor.h"],
  include_prefix="applied/spectral/public",
)

cc_library(
  name = "utils",
  tags = ["nowindows"],
  deps = ["//simian/public:proto_def", "@com_google_protobuf//:protobuf", "@nlohmann_json//:json", "@zlib//:zlib"] + select({"@platforms//os:windows": ["@windows_dlls//:windows_dlls"], "//conditions:default": []}),
  copts = ["-Werror"],
  srcs = ["//spectral/public:utils.cc"],
  hdrs = ["//spectral/public:utils.h"],
  include_prefix="applied/spectral/public",
)

