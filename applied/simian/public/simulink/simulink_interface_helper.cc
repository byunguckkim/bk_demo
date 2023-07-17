#include "applied/simian/public/simulink/simulink_interface_helper.h"
#include <fstream>
#include "applied/simian/public/simulink/ip_addr_validator.h"

namespace simian_public {

void SimulinkInterfaceHelper::ParseStartupOptions(
    const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) {
  ParseScenarioExtraData(startup_options.scenario_extra_data());
  ParseScenarioDigest(startup_options.scenario_digest());
}

void SimulinkInterfaceHelper::ParseScenarioExtraData(
    const google::protobuf::Value& scenario_extra_data) {
  // Parsing example extra_data in scenario description
  if (scenario_extra_data.kind_case() == google::protobuf::Value::kStructValue) {
    const google::protobuf::Struct& extra_data = scenario_extra_data.struct_value();
    ParseSimulinkOptions(extra_data);
  }
}

void SimulinkInterfaceHelper::ParseScenarioDigest(
    const simian_public::scenario::ScenarioDigest& digest) {
  ParseDigestMotionModel(digest.motion_model());
  ParseDigestVehicleSimConfig(digest);
}

void SimulinkInterfaceHelper::ParseDigestVehicleSimConfig(
    const simian_public::scenario::ScenarioDigest& digest) {
  if (digest.has_vehiclesim_config() &&
      digest.vehiclesim_config().vehicle_model_name().length() > 0) {
    is_using_vehiclesim_ego_behaviors_ = true;
    vehiclesim_config_.CopyFrom(digest.vehiclesim_config());
  }
}

void SimulinkInterfaceHelper::ParseSimulinkOptions(const google::protobuf::Struct& extra_data) {
  const auto& extra_data_fields = extra_data.fields();
  if (extra_data_fields.count("simulink") != 0 &&
      extra_data_fields.at("simulink").kind_case() == google::protobuf::Value::kStructValue) {
    const google::protobuf::Struct& simulink_options =
        extra_data_fields.at("simulink").struct_value();
    ParseDriveEgoOptions(simulink_options);
  }
}

void SimulinkInterfaceHelper::ParseDriveEgoOptions(
    const google::protobuf::Struct& simulink_options) {
  const auto& fields = simulink_options.fields();
  // Enables Driving the Ego at Constant Velocity even if it is damped.
  if (fields.count("drive_constant") != 0 &&
      fields.at("drive_constant").kind_case() == google::protobuf::Value::kBoolValue) {
    drive_constant_ = fields.at("drive_constant").bool_value();
    std::cout << "drive_constant is: " << drive_constant_ << std::endl;
  }
}

void SimulinkInterfaceHelper::ParseDigestMotionModel(
    const simian_public::motion_model::Description& motion_model) {
  if (motion_model.has_composite_vehicle()) {
    std::cout << "Motion model is a kinematic bicycle. Looking for a brake output." << std::endl;

    if (drive_constant_) {
      if (!motion_model.composite_vehicle().brake().has_brake_linear() &&
          !motion_model.composite_vehicle().brake().has_simple_wheel_lock()) {
        std::stringstream ss;
        ss << "This composite motion spec is not supported with drive_constant flag. Brake spec "
              "must either be linear or the simple wheel lock spec";
        throw std::runtime_error(ss.str());
      }
      if (!motion_model.composite_vehicle().powertrain().has_powertrain_linear()) {
        std::stringstream ss;
        ss << "This composite motion spec is not supported. Powertrain spec must be linear";
        throw std::runtime_error(ss.str());
      }
      compositeVehicleSpec_ = std::make_optional(motion_model.composite_vehicle());
    }
    simian_input_type_ = simian_public::SimianInputCompositeVehicle_e;
  }
}

int32_t SimulinkInterfaceHelper::SetStartupOptions(
    simian_public::simulator::v2::InterfaceStartupOptions startup_options) {
  std::fstream fin;
  std::string line;

  fin.open("/etc/simulink_ip.conf", std::fstream::in);

  if (fin) {
    std::string maybe_simulink_addr;
    std::getline(fin, maybe_simulink_addr);
    if (isValidIPv4(maybe_simulink_addr)) {
      if (startup_options.scenario_digest()
              .simulink_config()
              .network_configuration()
              .simulink_ip_address() == "127.0.0.1") {
        startup_options.mutable_scenario_digest()
            ->mutable_simulink_config()
            ->mutable_network_configuration()
            ->set_simulink_ip_address(maybe_simulink_addr);
      }
    }
  }

  ParseStartupOptions(startup_options);

  simulink_manager_ = std::make_unique<applied::SimulinkManager>(
      startup_options.scenario_digest().simulink_config());
  simian_public::common::CommonResponse resp = simulink_manager_->Init();
  if (resp.status() == simian_public::common::CommonResponse::EXCEPTION) {
    std::cerr << resp.exception().exception_message() << std::endl;
    return 1;
  }
  adp_output_->mutable_startup_options()->MergeFrom(startup_options);
  adp_input_->Clear();

  simian_public::common::CommonResponse response =
      simulink_manager_->ReceiveFromSimulink(*adp_input_.get());
  adp_input_->Clear();

  return 0;
}

int32_t SimulinkInterfaceHelper::Initialize() {
  if (is_initialized_) {
    return 0;
  }
  return ChannelPublishSend();
}

int32_t SimulinkInterfaceHelper::ChannelPublishSend() {
  if (is_using_vehiclesim_ego_behaviors_) {
    for (int i = 0; i < adp_output_->predicted_controls_channels_size(); i++) {
      adp_output_->mutable_predicted_controls_channels(i)
          ->mutable_vehiclesim_channel_data()
          ->MergeFrom(vehiclesim_controls_);
    }
    vehiclesim_controls_.Clear();
  }

  simian_public::common::CommonResponse resp;
  resp = simulink_manager_->SendToSimulink(*adp_output_);
  if (resp.status() == simian_public::common::CommonResponse::EXCEPTION) {
    std::cerr << resp.exception().exception_message() << std::endl;
    return -1;
  }
  adp_output_->Clear();
  adp_input_->Clear();

  resp = simulink_manager_->ReceiveFromSimulink(*adp_input_);
  if (resp.status() == simian_public::common::CommonResponse::EXCEPTION) {
    std::cerr << resp.exception().exception_message() << std::endl;
    return -1;
  }
  is_initialized_ = true;

  return 0;
}

// SendToSimulink functions.

int32_t SimulinkInterfaceHelper::SendEgoTriggersToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::simulator::v2::Trigger& ego_triggers) {
  simian_public::simulink::TriggerChannelData* trigger_data = adp_output_->add_trigger_channels();
  *trigger_data->mutable_channel_metadata()->mutable_name() = channel_name;
  trigger_data->mutable_channel_data()->MergeFrom(ego_triggers);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendWheelSpeedSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor) {
  simian_public::simulink::WheelSpeedChannelData* wheel_speed_data =
      adp_output_->add_wheel_speed_channels();
  *wheel_speed_data->mutable_channel_metadata()->mutable_name() = channel_name;
  wheel_speed_data->mutable_channel_data()->MergeFrom(wheel_speed_sensor);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendPoseToSimulink(
    const nonstd::string_view channel_name, const simian_public::simulator::v2::Pose& pose) {
  if (is_first_tick_) {
    if (is_using_vehiclesim_ego_behaviors_) {
      vehiclesim_controls_.mutable_initial_ego_state()->CopyFrom(pose.sections(0).state());
      adp_output_->mutable_vehiclesim_config()->CopyFrom(vehiclesim_config_);
      for (const auto& [var_name, var_value] : vehiclesim_config_.variable_overrides()) {
        (*vehiclesim_controls_.mutable_variable_overrides())[var_name] = var_value;
      }
      is_first_tick_ = false;
    }
  }
  simian_public::simulink::PoseChannelData* pose_data = adp_output_->add_pose_channels();
  *pose_data->mutable_channel_metadata()->mutable_name() = channel_name;
  pose_data->mutable_channel_data()->MergeFrom(pose);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendPredictedControlToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::motion_model::Input& predicted_controls) {
  simian_public::simulink::ControlsChannelData* pred_controls_data =
      adp_output_->add_predicted_controls_channels();
  *pred_controls_data->mutable_channel_metadata()->mutable_name() = channel_name;

  pred_controls_data->mutable_channel_data()->MergeFrom(predicted_controls);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendMapSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::MapSensor& map_sensor) {
  simian_public::simulink::MapSensorData* map_data = adp_output_->add_map_sensor_channels();
  *map_data->mutable_channel_metadata()->mutable_name() = channel_name;
  map_data->mutable_channel_data()->MergeFrom(map_sensor);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendActorSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor) {
  simian_public::simulink::ActorSensorChannelData* actor_data =
      adp_output_->add_actor_sensor_channels();
  *actor_data->mutable_channel_metadata()->mutable_name() = channel_name;
  actor_data->mutable_channel_data()->MergeFrom(actor_sensor);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendMotionFeedbackToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::motion_model::Feedback& motion_feedback) {
  feedback_[std::string(channel_name)].CopyFrom(motion_feedback);

  if (!drive_constant_name_flag_) {
    motion_feedback_drive_constant_channel_name_ = channel_name;
    drive_constant_name_flag_ = true;
  }

  simian_public::simulink::MotionFeedbackData* feedback_data = adp_output_->add_feedback_channels();
  *feedback_data->mutable_channel_metadata()->mutable_name() = channel_name;
  feedback_data->mutable_channel_data()->MergeFrom(motion_feedback);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendLaneSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::LaneSensor& lane_sensor) {
  simian_public::simulink::LaneSensorChannelData* lane_data =
      adp_output_->add_lane_sensor_channels();
  *lane_data->mutable_channel_metadata()->mutable_name() = channel_name;
  lane_data->mutable_channel_data()->MergeFrom(lane_sensor);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendTrafficLightSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor) {
  simian_public::simulink::TrafficLightChannelData* traffic_light_channel_data =
      adp_output_->add_traffic_light_channels();
  *traffic_light_channel_data->mutable_channel_metadata()->mutable_name() = channel_name;
  traffic_light_channel_data->mutable_channel_data()->MergeFrom(traffic_light_sensor);

  return 0;
}

int32_t SimulinkInterfaceHelper::SendTrafficSignSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor) {
  simian_public::simulink::TrafficSignChannelData* traffic_sign_channel_data =
      adp_output_->add_traffic_sign_channels();
  *traffic_sign_channel_data->mutable_channel_metadata()->mutable_name() = channel_name;
  traffic_sign_channel_data->mutable_channel_data()->MergeFrom(traffic_sign_sensor);

  return 0;
}

int32_t SimulinkInterfaceHelper::SendLocalizationSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::LocalizationSensor& localization_sensor) {
  simian_public::simulink::LocalizationSensorChannelData* localization_channel_data =
      adp_output_->add_localization_sensor_channels();
  *localization_channel_data->mutable_channel_metadata()->mutable_name() = channel_name;
  localization_channel_data->mutable_channel_data()->MergeFrom(localization_sensor);

  return 0;
}

int32_t SimulinkInterfaceHelper::SendTerrainSensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::TerrainSensor& terrain_sensor) {
  simian_public::simulink::TerrainSensorChannelData* terrain_data =
      adp_output_->add_terrain_sensor_channels();
  *terrain_data->mutable_channel_metadata()->mutable_name() = channel_name;
  terrain_data->mutable_channel_data()->MergeFrom(terrain_sensor);
  return 0;
}

int32_t SimulinkInterfaceHelper::SendAgentTrajectorySensorToSimulink(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::AgentTrajectorySensor&
        agent_trajectory_sensor) {
  simian_public::simulink::AgentTrajectoryChannelData* agent_trajectory_sensor_data =
      adp_output_->add_agent_trajectory_channels();
  *agent_trajectory_sensor_data->mutable_channel_metadata()->mutable_name() = channel_name;
  agent_trajectory_sensor_data->mutable_channel_data()->MergeFrom(agent_trajectory_sensor);
  return 0;
}

// ReceiveFromSimulink Functions.

int32_t SimulinkInterfaceHelper::ReceiveControlsFromSimulink(
    const nonstd::string_view channel_name, simian_public::motion_model::Input* ego_input) {
  for (const simian_public::simulink::ControlsChannelData& control_channel :
       adp_input_->controls_channels()) {
    if (control_channel.channel_metadata().name() == channel_name) {
      ego_input->CopyFrom(control_channel.channel_data());
      if (drive_constant_ && drive_constant_name_flag_ &&
          simian_input_type_ == SimianInputCompositeVehicle_e) {
        std::cout << "Driving the Ego at constant velocity" << std::endl;

        double powertrain_damping_force =
            feedback_[motion_feedback_drive_constant_channel_name_].has_no_feedback()
                ? 0
                : feedback_[motion_feedback_drive_constant_channel_name_]
                      .composite_vehicle()
                      .powertrain_feedback()
                      .powertrain_damping_force()
                      .value();
        double vehicle_mass =
            compositeVehicleSpec_.value().powertrain().powertrain_linear().vehicle_mass();
        double max_acceleration =
            compositeVehicleSpec_.value().powertrain().powertrain_linear().max_acceleration();
        double new_normalized_throttle = 0;
        if (vehicle_mass != 0 && max_acceleration != 0) {
          new_normalized_throttle = powertrain_damping_force / vehicle_mass / max_acceleration;
        }
        ego_input->mutable_composite_vehicle_command()
            ->mutable_powertrain_input()
            ->set_normalized_throttle(new_normalized_throttle);
      }
      break;
    }
  }
  return 0;
}

int32_t SimulinkInterfaceHelper::Finalize() {
  simulink_manager_->Terminate();
  return 0;
}

}  // namespace simian_public
