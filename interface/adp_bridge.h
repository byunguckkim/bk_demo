#pragma once

/*
  A customer's AV stack gets integrated into Simian by implementing the
  ADPBridge class.

  This class gets instantiated inside a grpc server that runs inside the
  customer docker container. The Simian simulation engine, which runs
  in the Applied docker, calls that grpc server as appropriate during
  a simulation run. The class can override a number of methods to
  extend or customize some of the generic behavior, and there are a
  few methods that must be provided.

  The other constants and methods start to matter for specific
  initialization and finalization needs.
*/

#include <cstdint>
#include <iostream>
#include <fstream>
#include <memory>
#include <string>
#include "applied/customer_stack_base.h"
#include "interface/log_reader.h"
#include "applied/spectral/public/spectral_shared_memory_helper.h"

class ADPBridge : public simian_public::CustomerStackBase {
 public:
  using CustomerStackBase::CustomerStackBase;

  const char* GetStackVersion() override;

  int32_t SetStartupOptions(
      const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) override;

  int32_t StackSetup() override;

  int32_t StackTeardown() override;

  int32_t Initialize() override;

  int32_t Finalize() override;

  int32_t RecordingSetup(const nonstd::string_view recording_path) override;

  int32_t RecordingTeardown() override;

  int32_t MiddlewareSetup() override;

  int32_t MiddlewareTeardown() override;

  int32_t ChannelListenSetup(const simian_public::simulator::v2::Channel& channel) override;

  int32_t ChannelListenTeardown(const simian_public::simulator::v2::Channel& channel) override;

  int32_t ChannelPublishSetup(const simian_public::simulator::v2::Channel& channel) override;

  int32_t ChannelPublishSend(const simian_public::simulator::v2::Channel& channel) override;

  int32_t ChannelPublishTeardown(const simian_public::simulator::v2::Channel& channel) override;

  int32_t GetDefaultRate(const simian_public::simulator::v2::Channel& channel) override;

  int32_t LogFetch(const simian_public::simulator::v2::LogFetchOptions& options,
                  simian_public::simulator::v2::LogFetchOutput* output) override;

  int32_t LogOpen(const simian_public::simulator::v2::LogOpenOptions& options,
                  simian_public::simulator::v2::LogOpenOutput* output) override;

  int32_t LogRead(const simian_public::simulator::v2::LogReadOptions& options,
                  simian_public::simulator::v2::LogReadOutput* output) override;

  int32_t LogClose(const simian_public::simulator::v2::LogCloseOptions& options) override;

  int32_t ConvertTimeFromADP(const nonstd::string_view channel_name,
                                const google::protobuf::Timestamp& time) override;

  int32_t ConvertPoseFromADP(const nonstd::string_view channel_name,
                                const simian_public::simulator::v2::Pose& pose) override;

  int32_t ConvertActorSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor) override;

  int32_t ConvertLocalizationSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::LocalizationSensor& localization_sensor) override;

  int32_t ConvertRadarSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::sensor_model::SensorOutput::RadarTrack& radar_track) override;

  int32_t ConvertLidarSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud) override;

  int32_t ConvertCameraSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::sensor_model::SensorOutput::CameraImage& camera_image) override;

  int32_t ConvertPerceptionSensorFromADP(
      const nonstd::string_view channel_name,
      const simian_public::sensor_output::SensorOutputList& sensor_output_list) override;


  int32_t ConvertPredictedControlFromADP(
      const nonstd::string_view channel_name,
      const simian_public::motion_model::Input& predicted_controls) override;

  int32_t ConvertMotionFeedbackFromADP(
      const nonstd::string_view channel_name,
      const simian_public::motion_model::Feedback& motion_feedback) override;

  int32_t ConvertEgoTriggersFromADP(
      const nonstd::string_view channel_name,
      const simian_public::simulator::v2::Trigger& ego_triggers) override;

  int32_t ConvertTripAgentFromADP(
      const nonstd::string_view channel_name,
      const simian_public::common::TripAgentOutput& trip_agent) override;

  int32_t ConvertControlsToADP(const nonstd::string_view channel_name,
                                  simian_public::motion_model::Input* ego_input) override;

  int32_t ConvertStackStateToADP(const nonstd::string_view channel_name,
                                    simian_public::simulator::v2::StackState* stack_state) override;

 private:
  ::applied::spectral::SMHelper shmem_helper_;
  std::unique_ptr<LogReader> log_reader_;
  Mailbox mailbox_;
  simian_public::motion_model::Input predicted_controls_;
  const std::string kStackVersion_ = "bk_demo_bridge_v1";

  // Configuration data from the startup options.
  google::protobuf::Value scenario_extra_data_;
  std::string record_stack_data_;

  // Output stream to write logs from the stack to disk
  std::ofstream output_log_;

  // The current simulation time (different from wall time), used for
  // determining which stack logs apply to which sim times.
  google::protobuf::Timestamp curr_sim_time_;
  // Whether or not the simulation is paused. (These stack interface calls will only occur in the case when updateWhilePaused: True).
  // Used to enable live edit mode (interactive sim) with ego behaviors.
  bool sim_paused_ = false;

  // The UTM zone of the map that the scenario is run in.
  uint8_t utm_zone_;
  bool utm_zone_north_;
  // The name of the map that the simulation is being run on.
  std::string map_key_;

  // Whether to engage the planner to send back controls (false), or
  // if we are using Simian pre-defined "ego behaviors" to control the
  // ego without having the planner in the loop (true).
  bool use_ego_behavior_ = true;

};