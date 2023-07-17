/*
 * NOTE: This file is relevant for the "Simplify Your ADP Update Workflow" interface migration
 * (tutorial linked below).
 * https://home.applied.co/tutorials/#/adp/02-simplify_your_ADP_update_workflow/simplify_your_ADP_update_workflow
 *
 * As part of this migration, we ask that you separate out your interface code from the public ADP
 * proto APIs and library source code. More details on what this separation will look like are
 * covered here:
 * https://home.applied.co/manual/adp/v1.32/#/integrating_with_adp/architecture/introduction?id=customer-interface
 */

// Copyright 2020 Applied Intuition, Inc.

#include <cstdint>
#include <functional>
#include <iostream>
#include <vector>

// ERROR is a macro in windows.h. We use a conflicting proto message named ERROR
// in this file so we need to undef the macro for Windows compatibility.
#if !defined(__linux__) && defined(ERROR)
#undef ERROR
#endif

#include "applied/stack_interface_c_v2.h"
#include "applied/customer_stack_base.h"

namespace {

simian_public::CustomerStackBase* AsADPCustomer(void* self) {
  return static_cast<simian_public::CustomerStackBase*>(self);
}

simian_public::simulator::v2::Channel ConvertChannel(const ADPChannel& adp_channel) {
  simian_public::simulator::v2::Channel result;
  result.set_name(adp_channel.name);
  result.set_type(static_cast<simian_public::simulator::v2::ChannelType>(adp_channel.type));
  return result;
}

}  // namespace

CustomerInterfaceVersion customer_interface_v2__get_version() {
  return CUSTOMER_INTERFACE_VERSION_2_3;
}

void* customer_interface_v2_3__create(const char* name, const ADPSendFunctions* func_ptrs) {
  // This memory will be freed in customer_interface_v2__destroy.
  return simian_public::CreateCustomerStack(name, *func_ptrs).release();
}

void customer_interface_v2__destroy(void* self) {
  // This memory was allocated in customer_interface_v2__create.
  delete AsADPCustomer(self);
}

int32_t customer_interface_v2__set_startup_options(
    // This is simian_public.simulator.CustomerStartupOptions
    void* self, uint8_t* startup_options, uint64_t startup_options_size) {
  // Deprecated in favor of customer_interface_v2_1__set_startup_options.
  return 0;
}

int32_t customer_interface_v2_1__set_startup_options(
    // This is simian_public.simulator.v2.InterfaceStartupOptions
    void* self, uint8_t* startup_options, uint64_t startup_options_size) {
  simian_public::simulator::v2::InterfaceStartupOptions options;
  if (!options.ParseFromArray(startup_options, static_cast<int>(startup_options_size))) {
    std::cout << "Customer interface failed to parse startup options!" << std::endl;
    std::cout.flush();
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return AsADPCustomer(self)->SetStartupOptions(options);
}

const char* customer_interface_v2__get_stack_version(void* self) {
  return AsADPCustomer(self)->GetStackVersion();
}

int32_t customer_interface_v2__get_default_rate(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->GetDefaultRate(ConvertChannel(*channel));
}

int64_t customer_interface_v2__get_default_period_ns(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->GetDefaultPeriodNs(ConvertChannel(*channel));
}

//////////////////////////////////////////////////
// Setup/teardown functions
//////////////////////////////////////////////////

int32_t customer_interface_v2__middleware_setup(void* self) {
  return AsADPCustomer(self)->MiddlewareSetup();
}

int32_t customer_interface_v2__middleware_teardown(void* self) {
  return AsADPCustomer(self)->MiddlewareTeardown();
}

int32_t customer_interface_v2__stack_setup(void* self) { return AsADPCustomer(self)->StackSetup(); }

int32_t customer_interface_v2__stack_teardown(void* self) {
  return AsADPCustomer(self)->StackTeardown();
}

int32_t customer_interface_v2__recording_setup(void* self, const char* recording_path) {
  return AsADPCustomer(self)->RecordingSetup(recording_path);
}

int32_t customer_interface_v2__recording_teardown(void* self) {
  return AsADPCustomer(self)->RecordingTeardown();
}

int32_t customer_interface_v2__visualization_setup(void* self) {
  return AsADPCustomer(self)->VisualizationSetup();
}

int32_t customer_interface_v2__visualization_teardown(void* self) {
  return AsADPCustomer(self)->VisualizationTeardown();
}

int32_t customer_interface_v2__initialize(void* self) { return AsADPCustomer(self)->Initialize(); }

int32_t customer_interface_v2__finalize(void* self) { return AsADPCustomer(self)->Finalize(); }

int32_t customer_interface_v2__simulation_summary(void* self, uint8_t* sim_summary,
                                                  uint64_t sim_summary_size) {
  simian_public::common::SimulationSummary summary;
  if (!summary.ParseFromArray(sim_summary, static_cast<int>(sim_summary_size))) {
    std::cout << "Customer interface failed to parse simulation summary" << std::endl;
    std::cout.flush();
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return AsADPCustomer(self)->SimulationSummary(summary);
}

//////////////////////////////////////////////////
// Stack data-related functions
//////////////////////////////////////////////////

int32_t customer_interface_v2__listen_setup(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->ChannelListenSetup(ConvertChannel(*channel));
}

int32_t customer_interface_v2__listen_teardown(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->ChannelListenTeardown(ConvertChannel(*channel));
}

int32_t customer_interface_v2__publish_setup(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->ChannelPublishSetup(ConvertChannel(*channel));
}

int32_t customer_interface_v2__publish_teardown(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->ChannelPublishTeardown(ConvertChannel(*channel));
}

int32_t customer_interface_v2__publish_send(void* self, const ADPChannel* channel) {
  return AsADPCustomer(self)->ChannelPublishSend(ConvertChannel(*channel));
}

//////////////////////////////////////////////////
// ADP-related functions
//////////////////////////////////////////////////

namespace {

template <typename Proto>
int32_t ConvertToADP(
    simian_public::CustomerStackBase* const customer_stack,
    const std::function<int32_t(simian_public::CustomerStackBase&, const std::string&, Proto*)>
        convert_func,
    const ADPChannel* channel, uint8_t** output, uint64_t* output_size) {
  Proto proto;
  const int32_t result = convert_func(*customer_stack, channel->name, &proto);
  if (result != 0) {
    return result;
  }
  *output_size = proto.ByteSize();
  *output = new uint8_t[*output_size];
  if (proto.SerializeToArray(*output, static_cast<int>(*output_size))) {
    return 0;
  } else {
    delete[] * output;
    *output = nullptr;
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
}

template <typename Proto>
int32_t ConvertFromADP(simian_public::CustomerStackBase* const customer_stack,
                       const std::function<int32_t(simian_public::CustomerStackBase&,
                                                   const std::string&, const Proto&)>
                           convert_func,
                       const ADPChannel* channel, const uint8_t* input, uint64_t input_size) {
  Proto proto;
  if (!proto.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return convert_func(*customer_stack, channel->name, proto);
}

}  // namespace

int32_t customer_interface_v2__convert__to_simian(void* self, const ADPChannel* channel,
                                                  uint8_t** output, uint64_t* output_size) {
  auto* const customer_stack = AsADPCustomer(self);
  switch (channel->type) {
    // Convert these channels for closed-loop simulation.
    case simian_public::simulator::v2::CONTROLS:
      return ConvertToADP<simian_public::motion_model::Input>(
          customer_stack, &simian_public::CustomerStackBase::ConvertControlsToADP, channel, output,
          output_size);
    case simian_public::simulator::v2::STACK_STATE:
      return ConvertToADP<simian_public::simulator::v2::StackState>(
          customer_stack, &simian_public::CustomerStackBase::ConvertStackStateToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::TRAJECTORY:
      return ConvertToADP<simian_public::spatial::Trajectory>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrajectoryToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::VEHICLE_DATA:
      return ConvertToADP<simian_public::simulator::v2::VehicleData>(
          customer_stack, &simian_public::CustomerStackBase::ConvertVehicleDataToADP, channel,
          output, output_size);
    // Convert this channel for open-loop simulation.
    case simian_public::simulator::v2::POSE:
      return ConvertToADP<simian_public::simulator::v2::Pose>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPoseToADP, channel, output,
          output_size);

    // Convert object-level data for visualization and to compute metrics.
    case simian_public::simulator::v2::ACTORS:
      return ConvertToADP<simian_public::perception::PerceptionChannel::ActorSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertActorSensorToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::LANE_SENSOR:
      return ConvertToADP<simian_public::perception::PerceptionChannel::LaneSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertLaneSensorToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::TRAFFIC_LIGHTS:
      return ConvertToADP<simian_public::perception::PerceptionChannel::TrafficLightSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrafficLightSensorToADP,
          channel, output, output_size);
    case simian_public::simulator::v2::TRAFFIC_SIGN_SENSOR:
      return ConvertToADP<simian_public::perception::PerceptionChannel::TrafficSignSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrafficSignSensorToADP, channel,
          output, output_size);

    // Convert raw sensor data for visualization.
    case simian_public::simulator::v2::LIDAR:
      return ConvertToADP<simian_public::sensor_model::SensorOutput::LidarCloud>(
          customer_stack, &simian_public::CustomerStackBase::ConvertLidarSensorToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::RADAR:
      return ConvertToADP<simian_public::sensor_model::SensorOutput::RadarTrack>(
          customer_stack, &simian_public::CustomerStackBase::ConvertRadarSensorToADP, channel,
          output, output_size);
    case simian_public::simulator::v2::CAMERA:
      return ConvertToADP<simian_public::sensor_model::SensorOutput::CameraImage>(
          customer_stack, &simian_public::CustomerStackBase::ConvertCameraSensorToADP, channel,
          output, output_size);

    case simian_public::simulator::v2::CUSTOM:
      return customer_stack->HandleCustomChannel(channel->name);
  }
  std::cout << "Unsupported channel requested in convert__to_simian. Name " << channel->name
            << " and type " << channel->type << std::endl;
  return -1;
}

int32_t customer_interface_v2__convert__to_simian_free(void* self, uint8_t* output,
                                                       uint64_t output_size) {
  // This memory was allocated in customer_interface_v2__convert__to_simian.
  delete[] output;
  return 0;
}

int32_t customer_interface_v2__convert__from_simian(void* self, const ADPChannel* channel,
                                                    const uint8_t* input, uint64_t input_size) {
  auto* const customer_stack = AsADPCustomer(self);
  switch (channel->type) {
    case simian_public::simulator::v2::TIME:
      return ConvertFromADP<google::protobuf::Timestamp>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTimeFromADP, channel, input,
          input_size);
    case simian_public::simulator::v2::POSE:
      return ConvertFromADP<simian_public::simulator::v2::Pose>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPoseFromADP, channel, input,
          input_size);
    case simian_public::simulator::v2::MOTION_FEEDBACK:
      return ConvertFromADP<simian_public::motion_model::Feedback>(
          customer_stack, &simian_public::CustomerStackBase::ConvertMotionFeedbackFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::CONTROLS:
      return ConvertFromADP<simian_public::motion_model::Input>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPredictedControlFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::TRIGGER:
      return ConvertFromADP<simian_public::simulator::v2::Trigger>(
          customer_stack, &simian_public::CustomerStackBase::ConvertEgoTriggersFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::TRIP_AGENT:
      return ConvertFromADP<simian_public::common::TripAgentOutput>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTripAgentFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::STACK_STATE:
      return ConvertFromADP<simian_public::simulator::v2::StackState>(
          customer_stack, &simian_public::CustomerStackBase::ConvertStackStateFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::VEHICLE_DATA:
      return ConvertFromADP<simian_public::simulator::v2::VehicleData>(
          customer_stack, &simian_public::CustomerStackBase::ConvertVehicleDataFromADP, channel,
          input, input_size);

    // Sensors
    case simian_public::simulator::v2::ACTORS:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::ActorSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertActorSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::LANE_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::LaneSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertLaneSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::TRAFFIC_LIGHTS:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::TrafficLightSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrafficLightSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::LOCALIZATION_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::LocalizationSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertLocalizationSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::PLANAR_LIDAR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::PlanarLidarSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPlanarLidarSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::PLANAR_OCCUPANCY_GRID:
      return ConvertFromADP<
          simian_public::perception::PerceptionChannel::PlanarOccupancyGridSensor>(
          customer_stack,
          &simian_public::CustomerStackBase::ConvertPlanarOccupancyGridSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::OCCLUSION_GRID:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::OcclusionGridSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertOcclusionGridSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::FREE_SPACE_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::FreeSpaceSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertFreeSpaceSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::TRAFFIC_LIGHT_BLOCKS:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::TrafficLightBlockSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrafficLightBlockSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::TRAFFIC_SIGN_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::TrafficSignSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTrafficSignSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::IMU_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::IMUSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertImuSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::WHEEL_SPEED_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::WheelSpeedSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertWheelSpeedSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::MAP_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::MapSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertMapSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::TERRAIN_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::TerrainSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertTerrainSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::POLAR_OBSTACLE_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::PolarObstacleSensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPolarObstacleSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::AGENT_TRAJECTORY_SENSOR:
      return ConvertFromADP<simian_public::perception::PerceptionChannel::AgentTrajectorySensor>(
          customer_stack, &simian_public::CustomerStackBase::ConvertAgentTrajectorySensorFromADP,
          channel, input, input_size);

    // Spectral
    case simian_public::simulator::v2::LIDAR:
      return ConvertFromADP<simian_public::sensor_model::SensorOutput::LidarCloud>(
          customer_stack, &simian_public::CustomerStackBase::ConvertLidarSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::RADAR:
      return ConvertFromADP<simian_public::sensor_model::SensorOutput::RadarTrack>(
          customer_stack, &simian_public::CustomerStackBase::ConvertRadarSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::CAMERA:
      return ConvertFromADP<simian_public::sensor_model::SensorOutput::CameraImage>(
          customer_stack, &simian_public::CustomerStackBase::ConvertCameraSensorFromADP, channel,
          input, input_size);
    case simian_public::simulator::v2::ULTRASOUND:
      return ConvertFromADP<simian_public::sensor_model::SensorOutput::Range>(
          customer_stack, &simian_public::CustomerStackBase::ConvertUltrasoundSensorFromADP,
          channel, input, input_size);
    case simian_public::simulator::v2::PERCEPTION_SENSOR:
      return ConvertFromADP<simian_public::sensor_output::SensorOutputList>(
          customer_stack, &simian_public::CustomerStackBase::ConvertPerceptionSensorFromADP,
          channel, input, input_size);
  }

  std::cout << "Unsupported channel requested in convert__from_simian. Name " << channel->name
            << " and type " << channel->type << std::endl;
  return -1;
}

//////////////////////////////////////////////////
// Log(stream)-related functions
//////////////////////////////////////////////////

int32_t customer_interface_v2_1__log__fetch(void* const self, const uint8_t* const input,
                                            const uint64_t input_size, uint8_t** const output_buf,
                                            uint64_t* const output_size) {
  simian_public::simulator::v2::LogFetchOptions options;
  simian_public::simulator::v2::LogFetchOutput output;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  const int32_t result = AsADPCustomer(self)->LogFetch(options, &output);
  if (result != 0) {
    *output_buf = nullptr;
    return result;
  }
  *output_size = output.ByteSize();
  *output_buf = new uint8_t[*output_size];
  if (!output.SerializeToArray(*output_buf, static_cast<int>(*output_size))) {
    delete[] * output_buf;
    *output_buf = nullptr;
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return 0;
}

int32_t customer_interface_v2_1__log__fetch_free(void* self, uint8_t* const output,
                                                 uint64_t output_size) {
  std::ignore = self;
  std::ignore = output_size;
  // This memory was allocated in customer_interface_v2_1__log__fetch.
  delete[] output;
  return 0;
}

int32_t customer_interface_v2__log__open(void* self, const uint8_t* input, size_t input_size) {
  simian_public::simulator::v2::LogOpenOptions options;
  simian_public::simulator::v2::LogOpenOutput unused_output;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return AsADPCustomer(self)->LogOpen(options, &unused_output);
}

int32_t customer_interface_v2_2__log__open(void* const self, const uint8_t* const input,
                                           const uint64_t input_size, uint8_t** const output_buf,
                                           uint64_t* const output_size) {
  simian_public::simulator::v2::LogOpenOptions options;
  simian_public::simulator::v2::LogOpenOutput output;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  const int32_t result = AsADPCustomer(self)->LogOpen(options, &output);
  if (result != 0) {
    *output_buf = nullptr;
    return result;
  }
  *output_size = output.ByteSize();
  *output_buf = new uint8_t[*output_size];
  if (!output.SerializeToArray(*output_buf, static_cast<int>(*output_size))) {
    delete[] * output_buf;
    *output_buf = nullptr;
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return 0;
}

int32_t customer_interface_v2_2__log__open_free(void* self, uint8_t* output, uint64_t output_size) {
  std::ignore = self;
  std::ignore = output_size;
  // This memory was allocated in customer_interface_v2_2__log__open.
  delete[] output;
  return 0;
}

int32_t customer_interface_v2__log__close(void* self, const uint8_t* input, size_t input_size) {
  simian_public::simulator::v2::LogCloseOptions opts;
  if (!opts.ParseFromArray(input, static_cast<int>(input_size))) {
    return -1;
  }
  return AsADPCustomer(self)->LogClose(opts);
}

int32_t customer_interface_v2_1__log__read(void* self, const uint8_t* input, uint64_t input_size,
                                           uint8_t** output_buf, uint64_t* output_size) {
  simian_public::simulator::v2::LogReadOptions options;
  simian_public::simulator::v2::LogReadOutput output;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  const int32_t result = AsADPCustomer(self)->LogRead(options, &output);
  if (result != 0) {
    *output_buf = nullptr;
    return result;
  }
  *output_size = output.ByteSize();
  *output_buf = new uint8_t[*output_size];
  if (!output.SerializeToArray(*output_buf, static_cast<int>(*output_size))) {
    delete[] * output_buf;
    *output_buf = nullptr;
    return -1;
  }
  return 0;
}

int32_t customer_interface_v2_1__log__read_free(void* self, uint8_t* output, uint64_t output_size) {
  std::ignore = self;
  std::ignore = output_size;
  // This memory was allocated in customer_interface_v2_1__log__read.
  delete[] output;
  return 0;
}

int32_t customer_interface_v2_4__log__rewind(void* self, const uint8_t* input, uint64_t input_size,
                                             uint8_t** output_buf, uint64_t* output_size) {
  simian_public::simulator::v2::LogRewindOptions options;
  simian_public::simulator::v2::LogRewindOutput output;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  const int32_t result = AsADPCustomer(self)->LogRewind(options, &output);
  if (result != 0) {
    *output_buf = nullptr;
    return result;
  }
  *output_size = output.ByteSize();
  *output_buf = new uint8_t[*output_size];
  if (!output.SerializeToArray(*output_buf, static_cast<int>(*output_size))) {
    delete[] * output_buf;
    *output_buf = nullptr;
    return -1;
  }
  return 0;
}

int32_t customer_interface_v2_4__log__rewind_free(void* self, uint8_t* output,
                                                  uint64_t output_size) {
  std::ignore = self;
  std::ignore = output_size;
  // This memory was allocated in customer_interface_v2_4__log__rewind.
  delete[] output;
  return 0;
}

int32_t customer_interface_v2_1__patch(void* self, const ADPChannel* channel, const uint8_t* input,
                                       uint64_t input_size) {
  simian_public::simulator::v2::PatchOptions options;
  if (!options.ParseFromArray(input, static_cast<int>(input_size))) {
    return simian_public::CustomerStackBase::kProtoSerializationError;
  }
  return AsADPCustomer(self)->Patch(ConvertChannel(*channel), options);
}
