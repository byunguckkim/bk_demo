// Include all available proto header files.

#pragma once

#include "applied/simian/public/osi3/osi_common.pb.h"
#include "applied/simian/public/osi3/osi_environment.pb.h"
#include "applied/simian/public/osi3/osi_groundtruth.pb.h"
#include "applied/simian/public/osi3/osi_lane.pb.h"
#include "applied/simian/public/osi3/osi_object.pb.h"
#include "applied/simian/public/osi3/osi_occupant.pb.h"
#include "applied/simian/public/osi3/osi_roadmarking.pb.h"
#include "applied/simian/public/osi3/osi_trafficlight.pb.h"
#include "applied/simian/public/osi3/osi_trafficsign.pb.h"
#include "applied/simian/public/osi3/osi_version.pb.h"
#include "applied/simian/public/proto/actor.pb.h"
#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/composite_vehicle.pb.h"
#include "applied/simian/public/proto/controller.pb.h"
#include "applied/simian/public/proto/custom_actor.pb.h"
#include "applied/simian/public/proto/customer_service.pb.h"
#include "applied/simian/public/proto/damping_model.pb.h"
#include "applied/simian/public/proto/drawing.pb.h"
#include "applied/simian/public/proto/drives.pb.h"
#include "applied/simian/public/proto/ego.pb.h"
#include "applied/simian/public/proto/environment.pb.h"
#include "applied/simian/public/proto/external_control_service.pb.h"
#include "applied/simian/public/proto/fmu_config.pb.h"
#include "applied/simian/public/proto/function_mapping.pb.h"
#include "applied/simian/public/proto/geometry.pb.h"
#include "applied/simian/public/proto/map/map.pb.h"
#include "applied/simian/public/proto/map/map_clear_area.pb.h"
#include "applied/simian/public/proto/map/map_common.pb.h"
#include "applied/simian/public/proto/map/map_crosswalk.pb.h"
#include "applied/simian/public/proto/map/map_enums.pb.h"
#include "applied/simian/public/proto/map/map_geometry.pb.h"
#include "applied/simian/public/proto/map/map_id.pb.h"
#include "applied/simian/public/proto/map/map_junction.pb.h"
#include "applied/simian/public/proto/map/map_lane.pb.h"
#include "applied/simian/public/proto/map/map_lane_markings.pb.h"
#include "applied/simian/public/proto/map/map_line.pb.h"
#include "applied/simian/public/proto/map/map_parking_space.pb.h"
#include "applied/simian/public/proto/map/map_region.pb.h"
#include "applied/simian/public/proto/map/map_road.pb.h"
#include "applied/simian/public/proto/map/map_semantic.pb.h"
#include "applied/simian/public/proto/map/map_sidewalk.pb.h"
#include "applied/simian/public/proto/map/map_sign.pb.h"
#include "applied/simian/public/proto/map/map_signal.pb.h"
#include "applied/simian/public/proto/map/map_speed_bump.pb.h"
#include "applied/simian/public/proto/map/map_traversable_surface.pb.h"
#include "applied/simian/public/proto/map/map_validation.pb.h"
#include "applied/simian/public/proto/map_config.pb.h"
#include "applied/simian/public/proto/motion_command_sequencer.pb.h"
#include "applied/simian/public/proto/motion_model.pb.h"
#include "applied/simian/public/proto/motion_state_modifier.pb.h"
#include "applied/simian/public/proto/orbis_config.pb.h"
#include "applied/simian/public/proto/perception.pb.h"
#include "applied/simian/public/proto/physics_engine.pb.h"
#include "applied/simian/public/proto/planar.pb.h"
#include "applied/simian/public/proto/plugin.pb.h"
#include "applied/simian/public/proto/public_map_service.pb.h"
#include "applied/simian/public/proto/public_query_service.pb.h"
#include "applied/simian/public/proto/public_queue_service.pb.h"
#include "applied/simian/public/proto/public_triage_service.pb.h"
#include "applied/simian/public/proto/relative_position.pb.h"
#include "applied/simian/public/proto/route.pb.h"
#include "applied/simian/public/proto/routing/error_code.pb.h"
#include "applied/simian/public/proto/routing/header.pb.h"
#include "applied/simian/public/proto/routing/routing.pb.h"
#include "applied/simian/public/proto/run_results.pb.h"
#include "applied/simian/public/proto/scenario/common.pb.h"
#include "applied/simian/public/proto/scenario/scenario_digest.pb.h"
#include "applied/simian/public/proto/scenario/sensor_config.pb.h"
#include "applied/simian/public/proto/scenario/vehicle_config.pb.h"
#include "applied/simian/public/proto/sensor.pb.h"
#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/sensor_output.pb.h"
#include "applied/simian/public/proto/sensor_sim/asset_tags.pb.h"
#include "applied/simian/public/proto/sensor_sim/basic_types.pb.h"
#include "applied/simian/public/proto/sensor_sim/labels.pb.h"
#include "applied/simian/public/proto/sensor_sim/model_output.pb.h"
#include "applied/simian/public/proto/sensor_sim/model_spec.pb.h"
#include "applied/simian/public/proto/sensor_sim/models.pb.h"
#include "applied/simian/public/proto/sensor_sim/pedestrian_props.pb.h"
#include "applied/simian/public/proto/shape.pb.h"
#include "applied/simian/public/proto/signal_modifier.pb.h"
#include "applied/simian/public/proto/sim_command.pb.h"
#include "applied/simian/public/proto/sim_data.pb.h"
#include "applied/simian/public/proto/simulation_service.pb.h"
#include "applied/simian/public/proto/simulink_automator_service.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"
#include "applied/simian/public/proto/spectral_assets.pb.h"
#include "applied/simian/public/proto/spline.pb.h"
#include "applied/simian/public/proto/stack_logs.pb.h"
#include "applied/simian/public/proto/terrain_config.pb.h"
#include "applied/simian/public/proto/traffic_light.pb.h"
#include "applied/simian/public/proto/transfer_function.pb.h"
#include "applied/simian/public/proto/triage.pb.h"
#include "applied/simian/public/proto/v2/customer_service_v2.pb.h"
#include "applied/simian/public/proto/v2/io.pb.h"
#include "applied/simian/public/proto/wind.pb.h"
#include "applied/simian/public/proto/workspace.pb.h"
