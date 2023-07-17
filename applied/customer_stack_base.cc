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

#include "applied/customer_stack_base.h"

#include <functional>

#include "applied/stack_interface_c_v2.h"

namespace simian_public {

CustomerStackBase::CustomerStackBase(const std::string& ego_name, const ADPSendFunctions& func_ptrs)
    : ego_name_(ego_name), send_func_ptrs_(func_ptrs) {}

int32_t CustomerStackBase::MiddlewareSetup() { return 0; }

int32_t CustomerStackBase::StackSetup() { return 0; }

int32_t CustomerStackBase::RecordingSetup(const nonstd::string_view recording_path) { return 0; }

int32_t CustomerStackBase::VisualizationSetup() { return 0; }

int32_t CustomerStackBase::GetDefaultRate(const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int64_t CustomerStackBase::GetDefaultPeriodNs(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelListenSetup(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishSetup(
    const simian_public::simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::LogFetch(const simian_public::simulator::v2::LogFetchOptions& options,
                                    simian_public::simulator::v2::LogFetchOutput* output) {
  return 0;
}

int32_t CustomerStackBase::LogOpen(const simian_public::simulator::v2::LogOpenOptions& opts,
                                   simian_public::simulator::v2::LogOpenOutput* output) {
  return 0;
}

int32_t CustomerStackBase::LogRewind(const simian_public::simulator::v2::LogRewindOptions& options,
                                     simian_public::simulator::v2::LogRewindOutput* output) {
  return 0;
}

int32_t CustomerStackBase::Initialize() { return 0; }

int32_t CustomerStackBase::SetStartupOptions(
    const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) {
  return 0;
}

int32_t CustomerStackBase::ConvertControlsToADP(const nonstd::string_view channel_name,
                                                simian_public::motion_model::Input* ego_input) {
  return 0;
}

int32_t CustomerStackBase::ConvertStackStateToADP(
    const nonstd::string_view channel_name, simian_public::simulator::v2::StackState* stack_state) {
  stack_state->set_stack_state(simian_public::sim_data::SimulatorInput::NOT_READY);
  return 0;
}

int32_t CustomerStackBase::ConvertPoseToADP(const nonstd::string_view channel_name,
                                            simulator::v2::Pose* pose) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrajectoryToADP(const nonstd::string_view channel_name,
                                                  simian_public::spatial::Trajectory* trajectory) {
  return 0;
}

int32_t CustomerStackBase::ConvertVehicleDataToADP(
    const nonstd::string_view channel_name,
    simian_public::simulator::v2::VehicleData* vehicle_data) {
  return 0;
}

int32_t CustomerStackBase::ConvertActorSensorToADP(
    const nonstd::string_view channel_name,
    perception::PerceptionChannel::ActorSensor* actor_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLaneSensorToADP(
    const nonstd::string_view channel_name,
    perception::PerceptionChannel::LaneSensor* lane_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficLightSensorToADP(
    const nonstd::string_view channel_name,
    perception::PerceptionChannel::TrafficLightSensor* traffic_light_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficSignSensorToADP(
    const nonstd::string_view channel_name,
    perception::PerceptionChannel::TrafficSignSensor* traffic_sign_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLidarSensorToADP(
    const nonstd::string_view channel_name, sensor_model::SensorOutput::LidarCloud* lidar_cloud) {
  return 0;
}

int32_t CustomerStackBase::ConvertRadarSensorToADP(
    const nonstd::string_view channel_name, sensor_model::SensorOutput::RadarTrack* radar_track) {
  return 0;
}

int32_t CustomerStackBase::ConvertCameraSensorToADP(
    const nonstd::string_view channel_name, sensor_model::SensorOutput::CameraImage* camera_image) {
  return 0;
}

int32_t CustomerStackBase::HandleCustomChannel(const nonstd::string_view channel_name) { return 0; }

int32_t CustomerStackBase::ConvertTimeFromADP(const nonstd::string_view channel_name,
                                              const google::protobuf::Timestamp& time) {
  return 0;
}

int32_t CustomerStackBase::ConvertPoseFromADP(const nonstd::string_view channel_name,
                                              const simulator::v2::Pose& pose) {
  return 0;
}

int32_t CustomerStackBase::ConvertMotionFeedbackFromADP(
    const nonstd::string_view channel_name, const motion_model::Feedback& motion_feedback) {
  return 0;
}

int32_t CustomerStackBase::ConvertPredictedControlFromADP(
    const nonstd::string_view channel_name, const motion_model::Input& predicted_controls) {
  return 0;
}

int32_t CustomerStackBase::ConvertEgoTriggersFromADP(const nonstd::string_view channel_name,
                                                     const simulator::v2::Trigger& ego_triggers) {
  return 0;
}

int32_t CustomerStackBase::ConvertTripAgentFromADP(const nonstd::string_view channel_name,
                                                   const common::TripAgentOutput& trip_agent) {
  return 0;
}

int32_t CustomerStackBase::ConvertStackStateFromADP(const nonstd::string_view channel_name,
                                                    const simulator::v2::StackState& stack_state) {
  return 0;
}

int32_t CustomerStackBase::ConvertVehicleDataFromADP(
    const nonstd::string_view channel_name, const simulator::v2::VehicleData& vehicle_data) {
  return 0;
}

int32_t CustomerStackBase::ConvertActorSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::ActorSensor& actor_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLaneSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::LaneSensor& lane_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficLightSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLocalizationSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::LocalizationSensor& localization_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertPlanarLidarSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::PlanarLidarSensor& planar_lidar_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertPlanarOccupancyGridSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::PlanarOccupancyGridSensor& planar_occupancy_grid_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertOcclusionGridSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::OcclusionGridSensor& occlusion_grid_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertFreeSpaceSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::FreeSpaceSensor& free_space_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficLightBlockSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::TrafficLightBlockSensor& traffic_light_block_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTrafficSignSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertImuSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::IMUSensor& imu_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertWheelSpeedSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertMapSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::MapSensor& map_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertTerrainSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::TerrainSensor& terrain_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertPolarObstacleSensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::PolarObstacleSensor& polar_obstacle_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertAgentTrajectorySensorFromADP(
    const nonstd::string_view channel_name,
    const perception::PerceptionChannel::AgentTrajectorySensor& agent_trajectory_sensor) {
  return 0;
}

int32_t CustomerStackBase::ConvertLidarSensorFromADP(
    const nonstd::string_view channel_name,
    const sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  return 0;
}

int32_t CustomerStackBase::ConvertRadarSensorFromADP(
    const nonstd::string_view channel_name,
    const sensor_model::SensorOutput::RadarTrack& radar_track) {
  return 0;
}

int32_t CustomerStackBase::ConvertCameraSensorFromADP(
    const nonstd::string_view channel_name,
    const sensor_model::SensorOutput::CameraImage& camera_image) {
  return 0;
}

int32_t CustomerStackBase::ConvertUltrasoundSensorFromADP(
    const nonstd::string_view channel_name,
    const sensor_model::SensorOutput::Range& ultrasound_range) {
  return 0;
}

int32_t CustomerStackBase::ConvertPerceptionSensorFromADP(
    const nonstd::string_view channel_name,
    const sensor_output::SensorOutputList& sensor_output_list) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishSend(const simulator::v2::Channel& channel) { return 0; }

int32_t CustomerStackBase::LogRead(const simulator::v2::LogReadOptions& options,
                                   simulator::v2::LogReadOutput* output) {
  return 0;
}

int32_t CustomerStackBase::Patch(const simulator::v2::Channel& channel,
                                 const simulator::v2::PatchOptions& patch_options) {
  return 0;
}

int32_t CustomerStackBase::SimulationSummary(
    const simian_public::common::SimulationSummary& summary) {
  return 0;
}

int32_t CustomerStackBase::Finalize() { return 0; }

int32_t CustomerStackBase::LogClose(const simian_public::simulator::v2::LogCloseOptions& options) {
  return 0;
}

int32_t CustomerStackBase::ChannelPublishTeardown(const simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::ChannelListenTeardown(const simulator::v2::Channel& channel) {
  return 0;
}

int32_t CustomerStackBase::VisualizationTeardown() { return 0; }

int32_t CustomerStackBase::RecordingTeardown() { return 0; }

int32_t CustomerStackBase::StackTeardown() { return 0; }

int32_t CustomerStackBase::MiddlewareTeardown() { return 0; }

namespace {

template <typename Proto>
int32_t SendToADP(const CustomerStackBase* self, const Proto& proto,
                  const std::function<int32_t(const void* self, const uint8_t* input,
                                              uint64_t input_size)>& send_func) {
  std::vector<uint8_t> buffer(proto.ByteSize());
  if (!proto.SerializeToArray(buffer.data(), buffer.size())) {
    return CustomerStackBase::kProtoSerializationError;
  }
  return send_func(self, buffer.data(), buffer.size());
}

}  // namespace

int32_t CustomerStackBase::SendStackLog(
    const simian_public::stack_logs::StackLogLine& log_line) const {
  return SendToADP(this, log_line, send_func_ptrs_.send_stack_log);
}

int32_t CustomerStackBase::SendDrawing(const simian_public::drawing::Drawing& drawing) const {
  return SendToADP(this, drawing, send_func_ptrs_.send_drawing);
}

int32_t CustomerStackBase::SendDataPoint(const simian_public::common::DataPoint& data_point) const {
  return SendToADP(this, data_point, send_func_ptrs_.send_data_point);
}

int32_t CustomerStackBase::SendTimestampedDataPoint(
    const simian_public::common::TimestampedDataPoint& timestamped_data_point) const {
  return SendToADP(this, timestamped_data_point, send_func_ptrs_.send_timestamped_data_point);
}

int32_t CustomerStackBase::SendCustomDataPointMetadata(
    const simian_public::common::CustomDataPointMetadata& metadata) const {
  return SendToADP(this, metadata, send_func_ptrs_.send_custom_data_point_metadata);
}

int32_t CustomerStackBase::SendSimCommand(
    const simian_public::sim_command::SimCommand& sim_command) const {
  return SendToADP(this, sim_command, send_func_ptrs_.send_sim_command);
}

int32_t CustomerStackBase::SendObserverEvent(
    const simian_public::common::ObserverEvent& event) const {
  return SendToADP(this, event, send_func_ptrs_.send_observer_event);
}

int32_t CustomerStackBase::SendMessage(const simian_public::common::Message& message) const {
  return SendToADP(this, message, send_func_ptrs_.send_message);
}

int32_t CustomerStackBase::SendTimestampedStruct(
    const simian_public::common::TimestampedStruct& timestamped_struct) const {
  return SendToADP(this, timestamped_struct, send_func_ptrs_.send_timestamped_struct);
}

int32_t CustomerStackBase::SendSimulationCustomField(
    const simian_public::common::CustomField& custom_field) const {
  return SendToADP(this, custom_field, send_func_ptrs_.send_simulation_custom_field);
}

int32_t CustomerStackBase::SendLogCustomField(
    const simian_public::common::CustomField& custom_field) const {
  return SendToADP(this, custom_field, send_func_ptrs_.send_log_custom_field);
}

int32_t CustomerStackBase::SendTriageEvent(const public_triage::TriageEvent& triage_event) const {
  return SendToADP(this, triage_event, send_func_ptrs_.send_triage_event);
}

}  // namespace simian_public
