#pragma once

#include <iostream>
#include "applied/simian/public/proto/simulink.pb.h"
#include "applied/simian/public/simulink/simulink_bridge_manager.h"
#include "applied/simian/public/utils/string_view.h"

constexpr int kDefaultRate = 100;

namespace simian_public {

// The SimianInputType enum represents the various types of motion commands
// that can be used for the specified motion model in the scenario digest in Simian.
typedef enum SimianInputType {
  SimianInputTypeNotSet_e,

  // Normalized Drive-By-Wire (Steering / Throttle / Brake) for the Kinematic / Coastdown Bicycle
  // model.
  SimianInputNormalizedDBW_e = 100,

  // OverrideState / OverrideState3d for External / External3d models.
  SimianInputOverrideState3d_e = 101,
  SimianInputOverrideState_e = 102,

  // Composite Vehicle Feedback for Composite Vehicle model.
  SimianInputCompositeVehicle_e = 103,
} SimianInputType_e;

class SimulinkInterfaceHelper {
  bool is_initialized_ = false;
  bool is_using_vehiclesim_ego_behaviors_ = false;

  /* VehicleSim features */
  simian_public::proto::scenario::vehiclesim_config::CompiledVehiclesimConfig vehiclesim_config_;
  simian_public::simulink::VehiclesimModel vehiclesim_controls_;
  bool is_first_tick_ = true;

  /* Drive constant features */
  std::string motion_feedback_drive_constant_channel_name_;
  bool drive_constant_ = false;
  bool drive_constant_name_flag_ = false;
  SimianInputType_e simian_input_type_;
  std::unordered_map<std::string, simian_public::motion_model::Feedback> feedback_;
  std::optional<simian_public::composite_vehicle::CompositeVehicleSpec> compositeVehicleSpec_ =
      std::nullopt;

  std::unique_ptr<applied::SimulinkManager> simulink_manager_;
  std::unique_ptr<simian_public::simulink::ADPInput> adp_input_;
  std::unique_ptr<simian_public::simulink::ADPOutput> adp_output_;

  void ParseStartupOptions(
      const simian_public::simulator::v2::InterfaceStartupOptions& startup_options);

  void ParseScenarioExtraData(const google::protobuf::Value& scenario_extra_data);

  void ParseSimulinkOptions(const google::protobuf::Struct& extra_data);

  void ParseScenarioDigest(const simian_public::scenario::ScenarioDigest& digest);

  void ParseDigestMotionModel(const simian_public::motion_model::Description& motion_model);

  void ParseDriveEgoOptions(const google::protobuf::Struct& simulink_options);

  void ParseDigestVehicleSimConfig(const simian_public::scenario::ScenarioDigest& digest);

 public:
  SimulinkInterfaceHelper() {
    adp_input_ = std::make_unique<simian_public::simulink::ADPInput>();
    adp_output_ = std::make_unique<simian_public::simulink::ADPOutput>();
  }

  // Get the modifiable ADPInput and Output messages for the data in your interface.
  simian_public::simulink::SimulinkInterfaceData& GetADPInputInterfaceData() {
    return *adp_input_->mutable_simulink_interface_data();
  }
  simian_public::simulink::SimulinkInterfaceData& GetADPOutputInterfaceData() {
    return *adp_output_->mutable_simulink_interface_data();
  }

  // Simulink Interface Setup and Teardown Functions. In order to use the
  // Simulink Interface, you need to define an instance of `SimulinkInterfaceHelper` and call the
  // interface setup and teardown functions from the customer interface (adp_bridge.cc).
  // Usage:
  // 1. SetStartupOptions(startup_options) must be called from the SetStartupOptions() or
  // the
  //    StackSetup() function in adp_bridge.cc.
  // 2. Initialize() must be called from the Initialize() function in adp_bridge.cc.
  // 3. ChannelPublishSend() must be called under the TIME channel in ChannelPublishSend
  //    inside adp_bridge.cc. Note that the only strict requirement is that
  //    ChannelPublishSend() is called at least once, however it is recommended that it is
  //    called at the same rate as the TIME channel.
  // 4. Finalize() must be called from the Finalize() function in adp_bridge.cc.
  // 5. GetDefaultRate() must be called inside GetDefaultRate() to return the rate
  // corresponding to the Simulink
  //    interface (default 100 Hz) instead of the default rate of the customer interface (10 Hz).

  int32_t SetStartupOptions(simian_public::simulator::v2::InterfaceStartupOptions startup_options);

  int32_t Initialize();

  int32_t ChannelPublishSend();

  int32_t Finalize();

  int32_t GetDefaultRate(const simian_public::simulator::v2::Channel& channel) {
    return kDefaultRate;
  }

  // The SendToSimulink functions enable the decoder blocks in the Applied Simulink
  // library. In order to populate the fields of the decoder blocks (such as `pose.px` for
  // Localization sensor), the `Send<Sensor>ToSimulink` (such as
  // `SendLocalizationSensorToSimulink`) function must be called inside
  // `Convert<Sensor>FromADP` (such as `ConvertLocalizationSensorFromADP`) from adp_bridge.cc.
  // Similar to the other customer interface `ConvertTo/FromADP` functions, you can check the status
  // of success of the `SendToSimulink` functions using the return value (0 indicates
  // success).
  int32_t SendPoseToSimulink(const nonstd::string_view channel_name,
                             const simian_public::simulator::v2::Pose& pose);

  int32_t SendEgoTriggersToSimulink(const nonstd::string_view channel_name,
                                    const simian_public::simulator::v2::Trigger& ego_triggers);

  int32_t SendWheelSpeedSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor);

  int32_t SendMotionFeedbackToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::motion_model::Feedback& motion_feedback);

  int32_t SendMapSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::MapSensor& map_sensor);

  int32_t SendActorSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor);

  int32_t SendPredictedControlToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::motion_model::Input& predicted_controls);

  int32_t SendLaneSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::LaneSensor& lane_sensor);

  int32_t SendTrafficLightSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor);

  int32_t SendTrafficSignSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor);

  int32_t SendLocalizationSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::LocalizationSensor& localization_sensor);

  int32_t SendTerrainSensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::TerrainSensor& terrain_sensor);

  int32_t SendAgentTrajectorySensorToSimulink(
      const nonstd::string_view channel_name,
      const simian_public::perception::PerceptionChannel::AgentTrajectorySensor&
          agent_trajectory_sensor);

  // The ReceiveFromSimulink functions enable the encoder blocks in the `Applied Simulink
  // Library`. In order to populate the fields of the encoder blocks (such as
  // normalized_dbw.steering for `NormDBW Encoder`), the `ReceiveControlsFromSimulink` function
  // must be called inside `ConvertControlsToADP` function from adp_bridge.cc. Similar to the other
  // customer interface `ConvertTo/FromADP` functions, you can check the status of success of the
  // `ReceiveFromSimulink` functions using the return value (0 indicates success).
  int32_t ReceiveControlsFromSimulink(const nonstd::string_view channel_name,
                                      simian_public::motion_model::Input* ego_input);
};

}  // namespace simian_public
