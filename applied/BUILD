package(default_visibility = ["//visibility:public"])

cc_library(
    name = "stack_interface_c",
    hdrs = ["stack_interface_c_v2.h"],
    include_prefix = "applied/",
    deps = ["//simian/public:api_def"],
)

cc_library(
    name = "applied_cxx",
    deps = [
        ":applied_hdrs",
        ":simian_cxx",
    ],
)

cc_library(
    name = "applied_hdrs",
    hdrs = [
        "include/api.h",
        "include/protos.h",
        "include/spectral_utils.h",
        "include/transforms.h",
    ],
    include_prefix = "applied/",
)

cc_library(
    name = "simian_cxx",
    deps = [
        ":all_protos",
        ":modules_transforms_cxx",
        "//simian/public/osi3:osi3_cc",
    ],
)

alias(
    name = "ros_utils",
    actual = "//spectral/public:ros_helpers",
)

alias(
    name = "map_utils",
    actual = "//simian/public/modules/transforms:map_util",
)

alias(
    name = "simian_map_client",
    actual = "//simian/public/map:simian_map_client_cc",
)

alias(
    name = "fmu_manager",
    actual = "//simian/public/fmu:fmu_manager",
)

cc_library(
    name = "spectral_cxx",
    deps = [
        "//spectral/public:depth_image_projector",
        "//spectral/public:extra_data_helpers",
        "//spectral/public:image_helpers",
        "//spectral/public:image_point",
        "//spectral/public:image_projector",
        "//spectral/public:label_visualization",
        "//spectral/public:lidar_helpers",
        "//spectral/public:radar_helpers",
        "//spectral/public:scoped_descriptor",
        "//spectral/public:shared_memory_helper",
        "//spectral/public:utils",
    ],
)

cc_library(
    name = "modules_transforms_cxx",
    deps = [
        "//simian/public/modules/transforms:planar",
        "//simian/public/modules/transforms:proto_conversion",
        "//simian/public/modules/transforms:scalar",
        "//simian/public/modules/transforms:spatial",
        "//simian/public/modules/transforms:transform_forest",
    ],
)

cc_library(
    name = "all_protos",
    deps = [
        "//simian/public/proto:actor_cc",
        "//simian/public/proto:common_cc",
        "//simian/public/proto:composite_vehicle_cc",
        "//simian/public/proto:controller_cc",
        "//simian/public/proto:custom_actor_cc",
        "//simian/public/proto:customer_service_cc",
        "//simian/public/proto:damping_model_cc",
        "//simian/public/proto:drawing_cc",
        "//simian/public/proto:drives_cc",
        "//simian/public/proto:ego_cc",
        "//simian/public/proto:environment_cc",
        "//simian/public/proto:external_control_service_cc",
        "//simian/public/proto:fmu_config_cc",
        "//simian/public/proto:function_mapping_cc",
        "//simian/public/proto:geometry_cc",
        "//simian/public/proto:map_config_cc",
        "//simian/public/proto:motion_command_sequencer_cc",
        "//simian/public/proto:motion_model_cc",
        "//simian/public/proto:motion_state_modifier_cc",
        "//simian/public/proto:orbis_config_proto_cc",
        "//simian/public/proto:perception_cc",
        "//simian/public/proto:physics_engine_cc",
        "//simian/public/proto:planar_cc",
        "//simian/public/proto:plugin_cc",
        "//simian/public/proto:plugin_schema_cc",
        "//simian/public/proto:public_query_service_proto_cc",
        "//simian/public/proto:public_queue_service_proto_cc",
        "//simian/public/proto:public_triage_service_proto_cc",
        "//simian/public/proto:relative_position_cc",
        "//simian/public/proto:route_cc",
        "//simian/public/proto:run_results_cc",
        "//simian/public/proto:sensor_cc",
        "//simian/public/proto:sensor_model_cc",
        "//simian/public/proto:sensor_output_cc",
        "//simian/public/proto:shape_cc",
        "//simian/public/proto:signal_modifier_cc",
        "//simian/public/proto:sim_command_cc",
        "//simian/public/proto:sim_data_cc",
        "//simian/public/proto:simulation_service_cc",
        "//simian/public/proto:spatial_cc",
        "//simian/public/proto:spectral_assets_cc",
        "//simian/public/proto:spline_cc",
        "//simian/public/proto:stack_logs_cc",
        "//simian/public/proto:terrain_config_cc",
        "//simian/public/proto:traffic_light_cc",
        "//simian/public/proto:transfer_function_cc",
        "//simian/public/proto:triage_proto_cc",
        "//simian/public/proto:wind_cc",
        "//simian/public/proto:workspace_cc",
        "//simian/public/proto/map:map_cc",
        "//simian/public/proto/map:map_clear_area_cc",
        "//simian/public/proto/map:map_crosswalk_cc",
        "//simian/public/proto/map:map_enums_cc",
        "//simian/public/proto/map:map_junction_cc",
        "//simian/public/proto/map:map_lane_cc",
        "//simian/public/proto/map:map_lane_markings_cc",
        "//simian/public/proto/map:map_line_cc",
        "//simian/public/proto/map:map_parking_space_cc",
        "//simian/public/proto/map:map_region_cc",
        "//simian/public/proto/map:map_road_cc",
        "//simian/public/proto/map:map_semantic_cc",
        "//simian/public/proto/map:map_sidewalk_cc",
        "//simian/public/proto/map:map_sign_cc",
        "//simian/public/proto/map:map_signal_cc",
        "//simian/public/proto/map:map_speed_bump_cc",
        "//simian/public/proto/map:map_traversable_surface_cc",
        "//simian/public/proto/routing:error_code_proto_cc",
        "//simian/public/proto/routing:header_proto_cc",
        "//simian/public/proto/routing:routing_proto_cc",
        "//simian/public/proto/scenario:common_cc",
        "//simian/public/proto/scenario:scenario_digest_cc",
        "//simian/public/proto/scenario:sensor_config_cc",
        "//simian/public/proto/scenario:vehicle_config_cc",
        "//simian/public/proto/sensor_sim:asset_tags_cc",
        "//simian/public/proto/sensor_sim:basic_types_cc",
        "//simian/public/proto/sensor_sim:camera_model_cc",
        "//simian/public/proto/sensor_sim:keypoints_cc",
        "//simian/public/proto/sensor_sim:labels_cc",
        "//simian/public/proto/sensor_sim:model_output_cc",
        "//simian/public/proto/sensor_sim:model_spec_cc",
        "//simian/public/proto/sensor_sim:models_cc",
        "//simian/public/proto/sensor_sim:pedestrian_props_cc",
        "//simian/public/proto/sensor_sim:spectral_materials_cc",
        "//simian/public/proto/sensor_sim:spectral_rendering_cc",
        "//simian/public/proto/v2:customer_service_v2_cc",
        "//simian/public/proto/v2:io_cc",
    ],
)

filegroup(
    name = "public_files",
    srcs = [
        "BUILD",
        "WORKSPACE",
        "include/api.h",
        "include/protos.h",
        "include/spectral_utils.h",
        "include/transforms.h",
        "tools/applied_deps.bzl",
        "tools/applied_extra_deps.bzl",
        "tools/applied_opencv_deps.bzl",
        "tools/applied_proto_extra_deps.bzl",
        "tools/protos.bzl",
    ],
)


cc_library(
    name = "customer_stack_base",
    srcs = [
        "customer_stack_base.cc",
    ],
    hdrs = ["customer_stack_base.h"],
    include_prefix = "applied/",
    deps = [
        ":stack_interface_c",
        "//simian/public/utils:string_view",
        ":applied_cxx",
    ],
)
cc_library(
    name = "customer_interface_glue",
    srcs = ["customer_interface_glue.cc"],
    defines = select({
        "@//tools/build:windows": ["BUILDING_DLL"],
        "//conditions:default": [],
    }),
    deps = [
        ":customer_stack_base",
        ":stack_interface_c",
    ],
    alwayslink = True,
)
