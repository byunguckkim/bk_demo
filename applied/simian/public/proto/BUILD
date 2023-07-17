package(default_visibility = ["//visibility:public"])

load("@applied_release//:tools/protos.bzl", "applied_proto_library")

applied_proto_library(
  name = "access_control",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto:access_control.proto"],
)

applied_proto_library(
  name = "actor",
  deps = ["//simian/public/proto:common", "//simian/public/proto:composite_vehicle", "//simian/public/proto:spatial", "//simian/public/proto:spectral_assets", "//simian/public/proto/sensor_sim:basic_types", "//simian/public/proto/sensor_sim:fragment_description", "//simian/public/proto/sensor_sim:model_output", "@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:actor.proto"],
)

applied_proto_library(
  name = "actuation_config",
  deps = [],
  srcs = ["//simian/public/proto:actuation_config.proto"],
)

applied_proto_library(
  name = "common",
  deps = ["@com_google_protobuf//:descriptor_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:common.proto"],
)

applied_proto_library(
  name = "composite_vehicle",
  deps = ["//simian/public/proto:common", "//simian/public/proto:damping_model", "//simian/public/proto:function_mapping", "//simian/public/proto:planar", "//simian/public/proto:signal_modifier", "//simian/public/proto:spatial", "//simian/public/proto:spline", "//simian/public/proto:transfer_function"],
  srcs = ["//simian/public/proto:composite_vehicle.proto"],
)

applied_proto_library(
  name = "controller",
  deps = [],
  srcs = ["//simian/public/proto:controller.proto"],
)

applied_proto_library(
  name = "custom_actor",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:motion_model", "//simian/public/proto:planar", "//simian/public/proto:shape", "//simian/public/proto:spatial", "@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:custom_actor.proto"],
)

applied_proto_library(
  name = "customer_service",
  deps = ["//simian/public/proto:common", "//simian/public/proto:custom_actor", "//simian/public/proto:map_config", "//simian/public/proto:sim_data", "//simian/public/proto/scenario:scenario_digest", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:customer_service.proto"],
)

applied_proto_library(
  name = "damping_model",
  deps = [],
  srcs = ["//simian/public/proto:damping_model.proto"],
)

applied_proto_library(
  name = "dataset",
  deps = ["@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:dataset.proto"],
)

applied_proto_library(
  name = "drawing",
  deps = ["//simian/public/proto:common", "//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:basic_types", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:drawing.proto"],
)

applied_proto_library(
  name = "drives",
  deps = ["//simian/public/proto:ego", "//simian/public/proto:map_config", "//simian/public/proto:sensor_model", "//simian/public/proto:sim_data", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:drives.proto"],
)

applied_proto_library(
  name = "ego",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:motion_model", "//simian/public/proto:perception", "//simian/public/proto:sensor", "//simian/public/proto:spatial", "//simian/public/proto/scenario:common", "//simian/public/proto/sensor_sim:model_output"],
  srcs = ["//simian/public/proto:ego.proto"],
)

applied_proto_library(
  name = "environment",
  deps = ["//simian/public/proto:common", "//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:fragment_description", "//simian/public/proto/sensor_sim:spectral_materials", "//simian/public/proto/sensor_sim:spectral_rendering"],
  srcs = ["//simian/public/proto:environment.proto"],
)

applied_proto_library(
  name = "external_control_service",
  deps = ["//simian/public/proto:common", "//simian/public/proto:sim_data", "//simian/public/proto/v2:customer_service_v2", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:external_control_service.proto"],
)

applied_proto_library(
  name = "field_options_proto",
  deps = ["@com_google_protobuf//:descriptor_proto"],
  srcs = ["//simian/public/proto:field_options.proto"],
)

applied_proto_library(
  name = "fmu_config",
  deps = [],
  srcs = ["//simian/public/proto:fmu_config.proto"],
)

applied_proto_library(
  name = "function_mapping",
  deps = ["//simian/public/proto:transfer_function"],
  srcs = ["//simian/public/proto:function_mapping.proto"],
)

applied_proto_library(
  name = "geometry",
  deps = [],
  srcs = ["//simian/public/proto:geometry.proto"],
)

applied_proto_library(
  name = "helix_common",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:geometry", "//simian/public/proto/map:map_lane", "//simian/public/proto/scenario:common", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:helix_common.proto"],
)

applied_proto_library(
  name = "integration_definition_compiled",
  deps = [],
  srcs = ["//simian/public/proto:integration_definition_compiled.proto"],
)

applied_proto_library(
  name = "interactive_field",
  deps = ["//simian/public/proto:common", "//simian/public/proto:shape", "@com_google_protobuf//:empty_proto"],
  srcs = ["//simian/public/proto:interactive_field.proto"],
)

applied_proto_library(
  name = "kubernetes_config_proto",
  deps = [],
  srcs = ["//simian/public/proto:kubernetes_config.proto"],
)

applied_proto_library(
  name = "map_config",
  deps = ["//simian/public/proto:common", "//simian/public/proto:geometry", "//simian/public/proto:spatial", "//simian/public/proto:terrain_config", "//simian/public/proto/map:map_common", "//simian/public/proto/map:map_enums", "//simian/public/proto/sensor_sim:spectral_map_config"],
  srcs = ["//simian/public/proto:map_config.proto"],
)

applied_proto_library(
  name = "motion_command_sequencer",
  deps = ["//simian/public/proto:motion_model"],
  srcs = ["//simian/public/proto:motion_command_sequencer.proto"],
)

applied_proto_library(
  name = "motion_model",
  deps = ["//simian/public/proto:actuation_config", "//simian/public/proto:common", "//simian/public/proto:composite_vehicle", "//simian/public/proto:physics_engine", "//simian/public/proto:planar", "//simian/public/proto:signal_modifier", "//simian/public/proto:spatial", "//simian/public/proto:field_options_proto", "//simian/public/proto/scenario:common", "//simian/public/proto/scenario:vehiclesim_config", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:motion_model.proto"],
)

applied_proto_library(
  name = "motion_state_modifier",
  deps = ["//simian/public/proto:spatial", "//simian/public/proto/scenario:common"],
  srcs = ["//simian/public/proto:motion_state_modifier.proto"],
)

applied_proto_library(
  name = "orbis_config_proto",
  deps = ["//simian/public/proto:common", "//simian/public/proto:plugin", "@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:orbis_config.proto"],
)

applied_proto_library(
  name = "osc_import_config",
  deps = ["//simian/public/proto:actor"],
  srcs = ["//simian/public/proto:osc_import_config.proto"],
)

applied_proto_library(
  name = "perception",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:geometry", "//simian/public/proto:planar", "//simian/public/proto:spatial", "//simian/public/osi3:osi_lane", "//simian/public/osi3:osi_trafficlight", "//simian/public/osi3:osi_trafficsign", "//simian/public/proto/map:map", "//simian/public/proto/scenario:sensor_config", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:empty_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:perception.proto"],
)

applied_proto_library(
  name = "physics_engine",
  deps = ["//simian/public/proto:common", "//simian/public/proto:spatial", "//simian/public/proto/scenario:common"],
  srcs = ["//simian/public/proto:physics_engine.proto"],
)

applied_proto_library(
  name = "plan_item",
  deps = [],
  srcs = ["//simian/public/proto:plan_item.proto"],
)

applied_proto_library(
  name = "planar",
  deps = [],
  srcs = ["//simian/public/proto:planar.proto"],
)

applied_proto_library(
  name = "plugin",
  deps = ["//simian/public/proto:common", "//simian/public/proto:dataset", "//simian/public/proto:drawing", "//simian/public/proto:drives", "//simian/public/proto:planar", "//simian/public/proto:plugin_schema", "//simian/public/proto:public_triage_service_proto", "//simian/public/proto:run_results", "//simian/public/proto:sensor_model", "//simian/public/proto:sensor_output", "//simian/public/proto:sim_data", "//simian/public/proto:triage_proto", "//simian/public/proto:kubernetes_config_proto", "//simian/public/proto/scenario:scenario_digest", "//simian/public/proto/v2:io", "@com_google_protobuf//:duration_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:plugin.proto"],
)

applied_proto_library(
  name = "plugin_schema",
  deps = [],
  srcs = ["//simian/public/proto:plugin_schema.proto"],
)

applied_proto_library(
  name = "public_log_search_service_proto",
  deps = ["//simian/public/proto:common", "//simian/public/proto:public_query_service_proto"],
  srcs = ["//simian/public/proto:public_log_search_service.proto"],
)

applied_proto_library(
  name = "public_map_service_proto",
  deps = ["//simian/public/proto:common", "//simian/public/proto:geometry", "//simian/public/proto:map_config", "//simian/public/proto:spatial", "//simian/public/proto:terrain_config", "//simian/public/proto/map:map_lane", "//simian/public/proto/map:map_semantic", "//simian/public/proto/map:map_validation", "//simian/public/proto/routing:routing_proto", "//simian/public/proto/scenario:common"],
  srcs = ["//simian/public/proto:public_map_service.proto"],
  with_grpc=True,
)

applied_proto_library(
  name = "public_query_service_proto",
  deps = ["@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:public_query_service.proto"],
)

applied_proto_library(
  name = "public_queue_service_proto",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto:public_queue_service.proto"],
)

applied_proto_library(
  name = "public_triage_service_proto",
  deps = ["//simian/public/proto:common", "//simian/public/proto:plugin_schema", "//simian/public/proto:triage_proto", "//simian/public/proto/v2:io", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:public_triage_service.proto"],
)

applied_proto_library(
  name = "relative_position",
  deps = ["//simian/public/proto:common", "//simian/public/proto:planar", "//simian/public/proto:spatial", "//simian/public/proto/scenario:common", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:relative_position.proto"],
)

applied_proto_library(
  name = "route",
  deps = ["//simian/public/proto:common", "//simian/public/proto:planar", "//simian/public/proto:relative_position", "@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:route.proto"],
)

applied_proto_library(
  name = "run_results",
  deps = ["@com_google_protobuf//:struct_proto"],
  srcs = ["//simian/public/proto:run_results.proto"],
)

applied_proto_library(
  name = "sensor",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:spatial", "//simian/public/osi3:osi_lane", "//simian/public/osi3:osi_trafficlight"],
  srcs = ["//simian/public/proto:sensor.proto"],
)

applied_proto_library(
  name = "sensor_model",
  deps = ["//simian/public/proto:common", "//simian/public/proto:spatial", "//simian/public/proto:field_options_proto", "//simian/public/proto/sensor_sim:basic_types", "//simian/public/proto/sensor_sim:camera_model", "//simian/public/proto/sensor_sim:labels", "//simian/public/proto/sensor_sim:sensor_behavior", "@com_google_protobuf//:any_proto", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:sensor_model.proto"],
)

applied_proto_library(
  name = "sensor_output",
  deps = ["//simian/public/proto:sensor_model", "//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:basic_types", "//simian/public/proto/sensor_sim:camera_model", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:sensor_output.proto"],
)

applied_proto_library(
  name = "shape",
  deps = ["//simian/public/proto:planar"],
  srcs = ["//simian/public/proto:shape.proto"],
)

applied_proto_library(
  name = "signal_modifier",
  deps = [],
  srcs = ["//simian/public/proto:signal_modifier.proto"],
)

applied_proto_library(
  name = "sim_command",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:geometry", "//simian/public/proto:planar", "//simian/public/proto:route", "//simian/public/proto:sensor_model", "//simian/public/proto:shape", "//simian/public/proto:spatial", "//simian/public/proto/scenario:vehicle_config", "@com_google_protobuf//:empty_proto"],
  srcs = ["//simian/public/proto:sim_command.proto"],
)

applied_proto_library(
  name = "sim_data",
  deps = ["//simian/public/proto:actor", "//simian/public/proto:common", "//simian/public/proto:custom_actor", "//simian/public/proto:drawing", "//simian/public/proto:ego", "//simian/public/proto:motion_model", "//simian/public/proto:perception", "//simian/public/proto:route", "//simian/public/proto:sensor_model", "//simian/public/proto:sim_command", "//simian/public/proto:spatial", "//simian/public/proto:stack_logs", "//simian/public/proto:stack_state", "//simian/public/proto:traffic_light", "//simian/public/proto:triage_proto", "//simian/public/proto:sensor_output", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:sim_data.proto"],
)

applied_proto_library(
  name = "simulation_service",
  deps = ["//simian/public/proto:common", "//simian/public/proto:sim_command", "@com_google_protobuf//:duration_proto"],
  srcs = ["//simian/public/proto:simulation_service.proto"],
)

applied_proto_library(
  name = "simulink",
  deps = ["//simian/public/proto:helix_common", "//simian/public/proto:motion_model", "//simian/public/proto:perception", "//simian/public/proto:spatial", "//simian/public/proto:vehiclesim", "//simian/public/proto/scenario:vehiclesim_config", "//simian/public/proto/v2:customer_service_v2", "//simian/public/proto/v2:io", "@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:simulink.proto"],
)

applied_proto_library(
  name = "simulink_automator_service",
  deps = ["//simian/public/proto:common"],
  srcs = ["//simian/public/proto:simulink_automator_service.proto"],
)

applied_proto_library(
  name = "spatial",
  deps = ["//simian/public/proto:field_options_proto", "//simian/public/proto:geometry"],
  srcs = ["//simian/public/proto:spatial.proto"],
)

applied_proto_library(
  name = "spectral_assets",
  deps = ["//simian/public/proto:common", "//simian/public/proto:planar", "//simian/public/proto:spatial", "//simian/public/proto/sensor_sim:fragment_description", "//simian/public/proto/sensor_sim:pedestrian_props", "//simian/public/proto/sensor_sim:spectral_materials", "//simian/public/proto/sensor_sim:spectral_rendering"],
  srcs = ["//simian/public/proto:spectral_assets.proto"],
)

applied_proto_library(
  name = "spline",
  deps = [],
  srcs = ["//simian/public/proto:spline.proto"],
)

applied_proto_library(
  name = "stack_logs",
  deps = ["@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:stack_logs.proto"],
)

applied_proto_library(
  name = "stack_state",
  deps = [],
  srcs = ["//simian/public/proto:stack_state.proto"],
)

applied_proto_library(
  name = "terrain_config",
  deps = ["//simian/public/proto:common", "//simian/public/proto:field_options_proto", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto:terrain_config.proto"],
)

applied_proto_library(
  name = "traffic_light",
  deps = ["//simian/public/proto:common", "//simian/public/proto:spatial"],
  srcs = ["//simian/public/proto:traffic_light.proto"],
)

applied_proto_library(
  name = "transfer_function",
  deps = [],
  srcs = ["//simian/public/proto:transfer_function.proto"],
)

applied_proto_library(
  name = "triage_proto",
  deps = ["@com_google_protobuf//:struct_proto", "@com_google_protobuf//:timestamp_proto"],
  srcs = ["//simian/public/proto:triage.proto"],
)

applied_proto_library(
  name = "vehiclesim",
  deps = [],
  srcs = ["//simian/public/proto:vehiclesim.proto"],
)

applied_proto_library(
  name = "wind",
  deps = [],
  srcs = ["//simian/public/proto:wind.proto"],
)

applied_proto_library(
  name = "workspace",
  deps = [],
  srcs = ["//simian/public/proto:workspace.proto"],
)

