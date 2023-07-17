cmake_minimum_required(VERSION 3.8)

project(applied C CXX)

############# Define option definitions
# These options allow users to enable or disable the use of various libraries at build time.
# For example, running ./build_and_update.sh --use_map_utils will enable the use of libraries
# that the map utility functions depend on.
option(APPLIED_ENABLE_MAP_UTILS "Set to use map conversion utilities." OFF)
option(APPLIED_ENABLE_FMU_UTILS "Set to use FMU utilities." OFF)
option(APPLIED_ENABLE_SPECTRAL_UTILS "Set to use the Spectral utilities." OFF)
option(APPLIED_ENABLE_PUBLIC_MAP_SERVICE "Set to use Applied's map service." OFF)
option(APPLIED_ENABLE_SIMULINK_INTERFACE "Set to use Applied's Simulink interface utilities. This enables the stack to send data to/from Simulink in addition to processing the data in C++." OFF)
option(USE_ROS "Set to use ROS for message passing." OFF)
option(ROS_DISTRO "If using ROS, specify a supported distribution: [kinetic, melodic, noetic]." "")
option(APPLIED_DISABLE_GLUE_STACK_BASE "Set to exclude building the glue and stack base. You will need to ensure they are separately built and linked in at runtime if using this flag" OFF)
############# End option definitions

############# Locate third-party dependencies
find_package(Eigen3 REQUIRED)
find_package(Threads REQUIRED)

find_package(Protobuf REQUIRED)
set(PROTOBUF_PROTOC_EXECUTABLE $<TARGET_FILE:protobuf::protoc>)

if (${APPLIED_ENABLE_PUBLIC_MAP_SERVICE})
  find_package(gRPC REQUIRED)
  set(GRPC_CPP_PLUGIN $<TARGET_FILE:gRPC::grpc_cpp_plugin>)
endif()

set(CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/third_party/")
############# End locate third-party dependencies

# Accommodate catkin workspace directory structure separately
if (${USE_ROS})
  set(SOURCE_DIR "${CMAKE_SOURCE_DIR}/adp")
  set(BINARY_DIR "${CMAKE_BINARY_DIR}/adp")
else()
  set(SOURCE_DIR "${CMAKE_SOURCE_DIR}")
  set(BINARY_DIR "${CMAKE_BINARY_DIR}")
endif()

############# Protobuf generation
set(PROTO_SRCS)
set(PROTO_HDRS)

function (generate_proto PROTO_FIL)
  get_filename_component(ABS_FIL ${PROTO_FIL} ABSOLUTE)
  get_filename_component(FIL_WE ${PROTO_FIL} NAME_WE)
  file(RELATIVE_PATH REL_FIL_PATH ${SOURCE_DIR} ${PROTO_FIL})
  get_filename_component(REL_DIR ${REL_FIL_PATH} DIRECTORY)

  list(APPEND PROTO_SRCS "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.cc")
  list(APPEND PROTO_HDRS "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.h")

  if ((${APPLIED_ENABLE_PUBLIC_MAP_SERVICE}) AND ((${FIL_WE} STREQUAL "public_map_service") OR ((${APPLIED_ENABLE_SIMULINK_INTERFACE}) AND ${FIL_WE} STREQUAL "simulink_automator_service")))
    # NOTE: We handle the public_map_service and the simulink_automator_service protos separately so we can also
    # use the GRPC plugin to generate service classes along with the usual message classes.
    list(APPEND PROTO_SRCS "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.grpc.pb.cc")
    list(APPEND PROTO_HDRS "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.grpc.pb.h")
    add_custom_command(
        OUTPUT "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.cc"
                "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.h"
                "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.grpc.pb.cc"
                "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.grpc.pb.h"
        COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
        ARGS --grpc_out=${BINARY_DIR}
              --plugin=protoc-gen-grpc=${GRPC_CPP_PLUGIN}
              --cpp_out ${BINARY_DIR}
              -I ${SOURCE_DIR} ${SOURCE_DIR}/${REL_FIL_PATH}
        DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
        WORKING_DIRECTORY ${SOURCE_DIR}
        COMMENT "Running C++ protocol buffer compiler with gRPC plugin on ${PROTO_FIL}"
        VERBATIM )
  else()
    add_custom_command(
      OUTPUT "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.cc"
          "${BINARY_DIR}/${REL_DIR}/${FIL_WE}.pb.h"
      COMMAND  ${PROTOBUF_PROTOC_EXECUTABLE}
      ARGS --cpp_out ${BINARY_DIR}
            -I ${SOURCE_DIR} ${SOURCE_DIR}/${REL_FIL_PATH}
      DEPENDS ${ABS_FIL} ${PROTOBUF_PROTOC_EXECUTABLE}
      WORKING_DIRECTORY ${SOURCE_DIR}
      COMMENT "Running C++ protocol buffer compiler on ${PROTO_FIL}"
      VERBATIM )

  endif()

  set(PROTO_SRCS "${PROTO_SRCS}" PARENT_SCOPE)
  set(PROTO_HDRS "${PROTO_HDRS}" PARENT_SCOPE)
endfunction()

# NOTE: Usage of GLOB is discouraged as it can potentially lead to outdated builds. So we maintain a manual list of protos
# and perform a verification step against the GLOB result to ensure that this list is always up to date with the current
# protos present in this repository.
set(APPLIED_PROTOS
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_common.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_environment.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_groundtruth.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_lane.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_object.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_occupant.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_roadmarking.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_sentinel.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_trafficlight.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_trafficsign.proto
  ${PROJECT_SOURCE_DIR}/simian/public/osi3/osi_version.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/access_control.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/actor.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/actuation_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/common.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/composite_vehicle.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/controller.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/custom_actor.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/customer_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/damping_model.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/dataset.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/drawing.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/drives.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/ego.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/environment.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/external_control_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/field_options.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/fmu_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/function_mapping.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/geometry.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/helix_common.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/integration_definition_compiled.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/interactive_field.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/kubernetes_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_clear_area.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_common.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_crosswalk.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_enums.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_geometry.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_id.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_junction.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_lane.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_lane_markings.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_line.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_parking_space.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_region.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_road.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_road_marking.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_semantic.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_sidewalk.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_sign.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_signal.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_signal_control_states.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_speed_bump.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_traversable_surface.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/map_validation.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map/procedural_map.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/map_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/motion_command_sequencer.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/motion_model.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/motion_state_modifier.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/orbis_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/osc_import_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/perception.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/physics_engine.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/plan_item.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/planar.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/plugin.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/plugin_schema.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/public_log_search_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/public_map_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/public_query_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/public_queue_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/public_triage_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/relative_position.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/route.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/routing/error_code.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/routing/header.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/routing/routing.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/run_results.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/common.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/scenario_digest.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/sensor_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/simulink_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/vehicle_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/scenario/vehiclesim_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_model.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_output.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/asset_tags.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/basic_types.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/camera_model.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/fragment_description.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/image_mapping.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/imported_vehicle_description.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/keypoints.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/labels.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/materials/common_material_properties.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/materials/lane_line_material.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/materials/material_description.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/materials/object_material.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/materials/surface_material.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/mesh_description.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/model_output.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/model_spec.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/models.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/pedestrian_props.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/pedestrians/animations.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/pedestrians/body.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/pedestrians/clothing.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/pedestrians/spectral_pedestrian.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/sensor_behavior.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/spectral_map_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/spectral_materials.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sensor_sim/spectral_rendering.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/shape.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/signal_modifier.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sim_command.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/sim_data.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/simulation_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/simulink.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/simulink_automator_service.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/spatial.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/spectral_assets.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/spline.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/stack_logs.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/stack_state.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/terrain_config.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/traffic_light.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/transfer_function.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/triage.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/v2/customer_service_v2.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/v2/io.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/vehiclesim.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/wind.proto
  ${PROJECT_SOURCE_DIR}/simian/public/proto/workspace.proto
)

file(GLOB_RECURSE VALID_APPLIED_PROTOS "${PROJECT_SOURCE_DIR}/*.proto")

foreach (FIL ${APPLIED_PROTOS})
  # Verify proto exists
  if (${FIL} IN_LIST VALID_APPLIED_PROTOS)
    list(REMOVE_ITEM VALID_APPLIED_PROTOS ${FIL})
  else()
    message(SEND_ERROR "Failed to locate the following proto file: ${FIL}")
  endif()

  # Generate proto
  generate_proto(${FIL})
endforeach()

# Verify no proto was left out
if (VALID_APPLIED_PROTOS)
  message(SEND_ERROR "The following proto files exist in your directory, but were not compiled: ${VALID_APPLIED_PROTOS}")
endif()

set_source_files_properties(${PROTO_SRCS} PROPERTIES GENERATED TRUE)
############# End protobuf generation

############# Build applied_cxx static library
# Collect applied_cxx sources
set(SIMIAN_SRCS
  ${PROJECT_SOURCE_DIR}/simian/public/proto_def.cc
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/scalar.cc
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/spatial.cc
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/proto_conversion.cc
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/planar.cc
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/transform_forest.cc
)
if (${APPLIED_ENABLE_SIMULINK_INTERFACE})
  set(SIMIAN_SRCS
    ${SIMIAN_SRCS}
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/zmq_services.cc
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_bridge_manager.cc
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_automator/simulink_automator_core.cc
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/ip_addr_validator.cc
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_interface_helper.cc
  )
endif()

set(SIMIAN_HDRS
  ${PROJECT_SOURCE_DIR}/simian/public/api_def.h
  ${PROJECT_SOURCE_DIR}/simian/public/proto_def.h
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/scalar.h
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/spatial.h
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/proto_conversion.h
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/planar.h
  ${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/transform_forest.h
  ${PROJECT_SOURCE_DIR}/simian/public/utils/optional.h
  ${PROJECT_SOURCE_DIR}/simian/public/utils/string_view.h
)

if (${APPLIED_ENABLE_SIMULINK_INTERFACE})
  set(SIMIAN_HDRS
    ${SIMIAN_HDRS}
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/zmq_services.h
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_bridge_manager.h
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_automator/simulink_automator_core.h
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/ip_addr_validator.h
    ${PROJECT_SOURCE_DIR}/simian/public/simulink/simulink_interface_helper.h
  )
endif()

set(CUSTOMER_STACK_BASE
  "${PROJECT_SOURCE_DIR}/customer_stack_base.cc"
  "${PROJECT_SOURCE_DIR}/customer_stack_base.h"
)

# Build applied_cxx static library
add_library(applied_cxx STATIC
  ${SIMIAN_SRCS}
  ${SIMIAN_HDRS}
  ${PROTO_SRCS}
  ${PROTO_HDRS}
)
if (NOT ${APPLIED_DISABLE_GLUE_STACK_BASE})
  target_sources(applied_cxx PRIVATE ${CUSTOMER_STACK_BASE})
else ()
  message(STATUS "Skipped adding customer_stack_base{.h,.cc} to the 'applied_cxx' (alias 'applied::interface_cxx') target")
endif()

set_property(TARGET applied_cxx PROPERTY POSITION_INDEPENDENT_CODE ON)

target_link_libraries(applied_cxx PUBLIC protobuf::libprotobuf)
if (${APPLIED_ENABLE_PUBLIC_MAP_SERVICE})
  target_link_libraries(applied_cxx PUBLIC gRPC::grpc++)
endif()

if (${APPLIED_ENABLE_SIMULINK_INTERFACE})
  target_link_libraries(applied_cxx PUBLIC c zmq)
endif()

target_include_directories(applied_cxx
  PRIVATE
  ${SOURCE_DIR}
  ${BINARY_DIR}
  INTERFACE
  $<BUILD_INTERFACE:${SOURCE_DIR}>
  $<BUILD_INTERFACE:${BINARY_DIR}>
)

target_include_directories(applied_cxx
  SYSTEM PUBLIC
  ${EIGEN3_INCLUDE_DIR}
  ${Protobuf_INCLUDE_DIRS}
)

# Depending on the options set, add more sources and link more libraries to applied_cxx
set(OPTIONAL_SRCS)
set(OPTIONAL_HDRS)

if (${APPLIED_ENABLE_MAP_UTILS})
  find_package(GeographicLib REQUIRED)
  target_link_libraries(applied_cxx PRIVATE ${GeographicLib_LIBRARIES})
  target_include_directories(applied_cxx PUBLIC ${GeographicLib_INCLUDE_DIRS})
  target_compile_definitions(applied_cxx PUBLIC ${GeographicLib_DEFINITIONS})

  list(APPEND OPTIONAL_SRCS "${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/map_util.cc")
  list(APPEND OPTIONAL_HDRS "${PROJECT_SOURCE_DIR}/simian/public/modules/transforms/map_util.h")
endif()

if (${APPLIED_ENABLE_FMU_UTILS})
  if (${USE_ROS})
    message(SEND_ERROR "FMU usage currently not supported with ROS ${ROS_DISTRO}.")
  endif()
  find_package(fmi4cpp REQUIRED)
  set(FMI4CPP_LIB "/usr/local/lib/libfmi4cpp.so")
  set(FMI4CPP_INCLUDE_DIR "/usr/local/include/fmi4cpp/")
  target_link_libraries(applied_cxx PRIVATE ${FMI4CPP_LIB})
  target_include_directories(applied_cxx PUBLIC ${FMI4CPP_INCLUDE_DIR})

  list(APPEND OPTIONAL_SRCS "${PROJECT_SOURCE_DIR}/simian/public/fmu/fmu_manager.cc")
  list(APPEND OPTIONAL_HDRS "${PROJECT_SOURCE_DIR}/simian/public/fmu/fmu_manager.h")
endif()

if (${APPLIED_ENABLE_SPECTRAL_UTILS})
  find_package(OpenCV REQUIRED)
  target_link_libraries(applied_cxx PUBLIC ${OpenCV_LIBS})

  list(APPEND OPTIONAL_SRCS
    "${PROJECT_SOURCE_DIR}/spectral/public/mmap_region.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/radar_helpers.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/spectral_shared_memory_helper.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/utils.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/label_visualization.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/depth_image_projector.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_helpers.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_point.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_projector.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_projector_impl.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/lidar_helpers.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/ndarray_helpers.cc"
    "${PROJECT_SOURCE_DIR}/spectral/public/ros_helpers.cc"
  )


  list(APPEND OPTIONAL_HDRS
    "${PROJECT_SOURCE_DIR}/spectral/public/mmap_region.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/radar_helpers.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/spectral_shared_memory_helper.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/utils.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/label_visualization.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/depth_image_projector.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_helpers.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_point.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_projector.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/image_projector_impl.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/lidar_helpers.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/ndarray_helpers.h"
    "${PROJECT_SOURCE_DIR}/spectral/public/ros_helpers.h"
  )
endif()

if (${APPLIED_ENABLE_PUBLIC_MAP_SERVICE})
  list(APPEND OPTIONAL_SRCS "${PROJECT_SOURCE_DIR}/simian/public/map/simian_map_client.cc")
  list(APPEND OPTIONAL_HDRS "${PROJECT_SOURCE_DIR}/simian/public/map/simian_map_client.h")
endif()

if (${USE_ROS})
  list(APPEND OPTIONAL_SRCS "${PROJECT_SOURCE_DIR}/simian/public/ros_helpers/adapters.cc")
  list(APPEND OPTIONAL_HDRS "${PROJECT_SOURCE_DIR}/simian/public/ros_helpers/adapters.h")
  list(APPEND OPTIONAL_SRCS "${PROJECT_SOURCE_DIR}/simian/public/ros_helpers/bag_writer.cc")
  list(APPEND OPTIONAL_HDRS "${PROJECT_SOURCE_DIR}/simian/public/ros_helpers/bag_writer.h")
endif()

target_sources(applied_cxx PRIVATE ${OPTIONAL_SRCS} ${OPTIONAL_HDRS})

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
message("GCC Compiler, setting GCC specific compile flags")
target_compile_options(applied_cxx PRIVATE -Wall -Werror -Wextra -Wpedantic -Wshadow -g -DHAVE_PTHREAD -Wno-error=sign-compare -Wno-error=unused-function -Wno-unused-variable -Wno-deprecated-declarations -Wno-unused-parameter -Wno-ignored-qualifiers)
if (Eigen3_VERSION VERSION_LESS "3.3.4")
  target_compile_options(applied_cxx PUBLIC -Wno-error=ignored-attributes -Wno-error=int-in-bool-context)
endif()
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
message("MSVC Compiler, setting MSVC specific compile flags")
# Needs public visibility when adding map and public map utilities.
target_compile_options(applied_cxx PUBLIC -DHAVE_PTHREAD -D_USE_MATH_DEFINES)
endif()
add_library(applied::interface_cxx ALIAS applied_cxx)
############# End build applied_cxx static library

############# Build glue object library
# NOTE: The glue file presents a special case where compiling it into applied::interface_cxx drops the symbols that
# define the glue and causes the customer interface not to load. So we instead compile the glue as its own object library,
# applied::glue_cxx, so it is used directly as a source file when building the customer interface.
set(CUSTOMER_INTERFACE_GLUE "${PROJECT_SOURCE_DIR}/customer_interface_glue.cc")
set(STACK_INTERFACE_C_V2 "${PROJECT_SOURCE_DIR}/stack_interface_c_v2.h")

if (NOT ${APPLIED_DISABLE_GLUE_STACK_BASE})
  add_library(customer_interface_glue OBJECT
    ${CUSTOMER_INTERFACE_GLUE}
    ${STACK_INTERFACE_C_V2}
  )

  add_dependencies(customer_interface_glue applied_cxx)

  get_property(APPLIEDCXX_INCLUDE_DIRS TARGET applied_cxx PROPERTY INTERFACE_INCLUDE_DIRECTORIES)
  target_include_directories(customer_interface_glue PRIVATE
    ${SOURCE_DIR}
    ${BINARY_DIR}
    ${APPLIEDCXX_INCLUDE_DIRS}
  )
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU")
  target_compile_options(customer_interface_glue PUBLIC -Wno-deprecated-declarations)
  endif()
  add_library(applied::glue_cxx ALIAS customer_interface_glue)
else ()
  message(STATUS "Skipped adding customer_interface_glue.cc and stack_interface_c_v2.h to the \
                  'customer_interface_glue' (alias 'applied::glue_cxx') target")
endif()
############# End build glue object library

