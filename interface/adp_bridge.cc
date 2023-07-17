/*
 *  See latest full documentation on V2 interface here:
 *  https://home.applied.co/manual/adp/v1.32/#/integrating_with_adp/multi_rate_interface/implementing_the_multi_rate_interface
 *
 * At sim start time, the functions are called in this order:
 *   1. Constructor()
 *   2. SetStartupOptions()
 *   3. GetStackVersion()
 *   4. MiddlewareSetup()
 *   5. RecordingSetup()
 *   6. StackSetup()
 *   For Each Channel:
 *   7. GetDefaultRate()
 *   8. ListenSetup() / PublishSetup() // Depending on channel
 *   9. ConvertFromADP() + PublishSend() // For all channel that source from Simian
 *   10. Initialize()
 *
 * Once the sim starts, the following functions are called until sim end
 *   1. ConvertToADP()
 *   2. ConvertFromADP() + PublishSend() // For Simian-source channels
 *
 * At the end of the sim, the following functions are called in this order:
 *   1. Finalize()
 *   2. SimulationSummary()
 *   For Each Channel:
 *   3. ListenTeardown() / PublishTeardown() // Depending on channel
 *   4. StackTeardown()
 *   5. VisualizationTeardown()
 *   6. RecordingTeardown()
 *   7. MiddlewareTeardown()
 *
 *
 * To convert a channel output from Simian to be published in your custom message format, do the following:
 *   1. Create an instance variable of the publisher in adp_bridge.h (i.e. such as ros::Publisher).
 *   2. Setup the publisher in the corresponding switch case in the PublishSetup function in adp_bridge.cc
 *   3. Teardown the publisher in the corresponding switch case in the PublishTeardown function in adp_bridge.cc
 *   4. Create an instance variable of your middleware message itself in mailbox.h under the Mailbox struct.
 *      Be sure to include your custom message header in the mailbox.h file first before instantiating.
 *   5. In the PublishSend function in adp_bridge.cc, have your publisher instance variable publish
 *      the value of the mailbox_'s custom middleware message.
 *   6. In ConvertCHANNELFromADP, translate the Simian Google Protobuf message into your custom
 *      message format, and store it in the corresponding middleware message attached to the mailbox_.
 *      The proto message definition can be found in applied/simian/public/proto.
 *   7. Run a simulation and verify within the customer interface docker container that the message is being
 *      published. You can exec into the docker container by running `docker/into.sh`. For specifically
 *      ROS middlewares, you need to be sure to run `source /catkin_ws/devel/setup.bash` in order to be able to
 *      echo the custom topic.
 *
 */
#include "interface/adp_bridge.h"

#include <iomanip>
#include <iostream>
#include <google/protobuf/util/time_util.h>

#include "applied/stack_interface_c_v2.h"
#include "interface/filesystem.h"

#include "applied/simian/public/modules/transforms/spatial.h"

#include "applied/spectral/public/image_helpers.h"
#include "applied/spectral/public/lidar_helpers.h"
#include "applied/spectral/public/radar_helpers.h"

namespace {

// If true, setup and teardown lifecycle functions will print a message when they run.
constexpr const bool kVerboseSetup = false;
// If true, per-tick functions (e.g. ConvertFromSimian) will print a message when they run.
constexpr const bool kVerboseTick = false;

uint64_t sim_time_ms(const google::protobuf::Timestamp& timestamp) {
  return timestamp.seconds() * 1000 + timestamp.nanos() / (1000 * 1000);
}

std::string get_timestamped_file_name(const google::protobuf::Timestamp& sim_time, nonstd::string_view suffix) {
  std::stringstream file_name;
  file_name << std::setfill('0') << std::setw(8)
              << sim_time_ms(sim_time) << suffix;
  return file_name.str();
}

} // namespace

//////////////////////////////////////////////////
// Setup/teardown functions
//////////////////////////////////////////////////

std::unique_ptr<simian_public::CustomerStackBase> simian_public::CreateCustomerStack(
    const std::string& name, const ADPSendFunctions& func_ptrs) {
  return std::make_unique<ADPBridge>(name, func_ptrs);
}

const char* ADPBridge::GetStackVersion() { return kStackVersion_.c_str(); }

// Use startup_options->scenario_extra_data to tweak how your stack is started
int32_t ADPBridge::SetStartupOptions(
    const simian_public::simulator::v2::InterfaceStartupOptions& startup_options) {
  scenario_extra_data_ = startup_options.scenario_extra_data();
  record_stack_data_ = startup_options.record_stack_data();
  // Parsing example extra_data in scenario description
  if (startup_options.scenario_extra_data().kind_case() == google::protobuf::Value::kStructValue) {
    const google::protobuf::Struct& extra_data =
        startup_options.scenario_extra_data().struct_value();
    // iterate through the extra_data fields
    const auto& fields = extra_data.fields();
    if (fields.count("example_number") != 0 &&
        fields.at("example_number").kind_case() == google::protobuf::Value::kNumberValue) {
      const double example_number = fields.at("example_number").number_value();
      std::cout << "EXAMPLE: Extra Data Value 'example_number' is " << example_number << std::endl;
    }
    if (fields.count("example_string") != 0 &&
        fields.at("example_string").kind_case() == google::protobuf::Value::kStringValue) {
      const std::string example_string = fields.at("example_string").string_value();
      std::cout << "EXAMPLE: Extra Data Value 'example_string' is " << example_string << std::endl;
    }
    if (fields.count("example_dictionary") != 0 &&
        fields.at("example_dictionary").kind_case() == google::protobuf::Value::kStructValue) {
      const auto& struct_fields = fields.at("example_dictionary").struct_value().fields();
      google::protobuf::Value value1 = struct_fields.at("value1");
      if (value1.kind_case() == google::protobuf::Value::kStringValue) {
        const std::string val_str = value1.string_value();
        std::cout << "EXAMPLE: Extra Data Value 'example_dictionary[value1]' is " << val_str
                  << std::endl;
      }
      std::cout << "EXAMPLE: Extra Data Value 'example_dictionary' has been parsed!" << std::endl;
    }

    // If we are using ego behavior, then the stack is not engaged and we are
    // running an open loop simulation, not a closed loop simulation.
    // This allows us to pass in whether or not to use ego behavior by specifing
    // the following in the scenario yaml.
    // extra_data:
    //   use_ego_behavior: true
    if (fields.count("use_ego_behavior") != 0 &&
        fields.at("use_ego_behavior").kind_case() == google::protobuf::Value::kBoolValue) {
      use_ego_behavior_ = fields.at("use_ego_behavior").bool_value();
    }
    std::cout << "Use Ego Behavior: " << use_ego_behavior_ << std::endl;
  }

  // Extract the UTM zone from the scenario YAML file.
  // Note that startup_options is an InterfaceStartupOptions proto message (defined in io.proto).
  // startup_options.map_config() is a MapConfig message (defined in map_config.proto).
  // startup_options.map_config().transforms() is a MapTransforms message (defined in map_config.proto).
  // and finally, startup_options.map_config().transforms().utm() is a UTMProjection message (defined in common.proto)
  utm_zone_ = startup_options.map_config().transforms().utm().zone();
  utm_zone_north_ = startup_options.map_config().transforms().utm().north();
  std::cout << "UTM Zone of the map is " << utm_zone_ << " with north value of " << utm_zone_north_ << std::endl;
  // Extract the map key name from the scenario.
  map_key_ = startup_options.map_key();
  std::cout << "Running on map " << map_key_ << std::endl;

  return 0;
}

// Setup your AV stack
int32_t ADPBridge::StackSetup() {
  if (kVerboseSetup) std::cout << "StackSetup()" << std::endl;

  return 0;
}

int32_t ADPBridge::StackTeardown() {
  if (kVerboseSetup) std::cout << "StackTeardown()" << std::endl;
  return 0;
}

int32_t ADPBridge::Initialize() {
  if (kVerboseSetup) std::cout << "Initialize()" << std::endl;
  return 0;
}

int32_t ADPBridge::Finalize() {
  if (kVerboseSetup) std::cout << "Finalize()" << std::endl;
  return 0;
}

int32_t ADPBridge::RecordingSetup(const nonstd::string_view recording_path) {
  // Example code snippet to populate files in the customer output folder.
  // Once you have written files to this folder, you should be able to see the output path
  // by viewing the result, clicking on the info pane in the right hand sidebar, scrolling
  // down to the bottom, and looking for the "Stack Output Path" heading.
  // If the simulation was run in Applied's cloud environment, you will find a
  // "Download Customer Output" button to download these log files.
  // See the following documentation for more info:
  // https://home.applied.co/manual/adp/v1.37/#/protobuf_documentation/io?id=interfacestartupoptions

  std::cout << "Saving stack logs to recording_path: " << recording_path << std::endl;
  // Filesystem don't support turning string_view into fs::paths, so we convert to string.
  std::string recording_path_str(recording_path);
  fs::path customer_output_directory_location_in_container(recording_path_str);
  fs::path custom_data_file_path =
      customer_output_directory_location_in_container / "customer_data.txt";
  // create_directories() returns whether or not directories were created
  (void)fs::create_directories(custom_data_file_path.parent_path());

  // Output some text to this file to know we are sending data.
  std::ofstream custom_data_file(custom_data_file_path);
  custom_data_file << "Custom Data File Generated From Customer Interface Bridge\n";
  custom_data_file.close();
  return 0;
}

int32_t ADPBridge::RecordingTeardown() {
  // Example closing output log here
  // output_log_.close();
  return 0;
}

int32_t ADPBridge::MiddlewareSetup() {
  if (kVerboseSetup) std::cout << "MiddlewareSetup()" << std::endl;
  return 0;
}

int32_t ADPBridge::MiddlewareTeardown() {
  if (kVerboseSetup) std::cout << "MiddlewareTeardown()" << std::endl;
  return 0;
}
int32_t ADPBridge::ChannelListenSetup(const simian_public::simulator::v2::Channel& channel) {
  if (channel.type() == simian_public::simulator::v2::CONTROLS) {
    if (kVerboseSetup) std::cout << "Listening to Controls" << std::endl;
    return 0;
  } else if (channel.type() == simian_public::simulator::v2::STACK_STATE) {
    if (kVerboseSetup) std::cout << "Listening to Stack state" << std::endl;
    return 0;
  }
  return -1;
}

int32_t ADPBridge::ChannelListenTeardown(const simian_public::simulator::v2::Channel& channel) {
  if (channel.type() == simian_public::simulator::v2::CONTROLS) {
    if (kVerboseSetup) std::cout << "No longer listening to Controls" << std::endl;
    return 0;
  } else if (channel.type() == simian_public::simulator::v2::STACK_STATE) {
    if (kVerboseSetup) std::cout << "No longer listening to Stack state" << std::endl;
    return 0;
  }
  return -1;
}

// This is where you'll be initializing your instance variables that handling publishing
// your custom message format to whatever middleware topics are necessary to be fed as inputs
// into your stack.
int32_t ADPBridge::ChannelPublishSetup(const simian_public::simulator::v2::Channel& channel) {
  switch (channel.type()) {
    case simian_public::simulator::v2::TIME:
      if (kVerboseSetup) std::cout << "Setting up publishing Time" << std::endl;
      return 0;
    case simian_public::simulator::v2::POSE:
      if (kVerboseSetup) std::cout << "Setting up publishing Pose" << std::endl;
      return 0;
    case simian_public::simulator::v2::ACTORS:
      if (channel.name() == "simian_perceived_actors") {
        if (kVerboseSetup) std::cout << "Setting up publishing perceived actors" << std::endl;
        return 0;
      } else if (channel.name() == "simian_ground_truth_actors") {
        if (kVerboseSetup) std::cout << "Setting up publishing ground truth actors" << std::endl;
        return 0;
      }
      return -1;
    case simian_public::simulator::v2::CONTROLS:
      if (kVerboseSetup) std::cout << "Setting up publishing behavior predicted control" << std::endl;
      return 0;
    case simian_public::simulator::v2::MOTION_FEEDBACK:
      if (kVerboseSetup) std::cout << "Setting up publishing motion feedback" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIGGER:
      if (kVerboseSetup) std::cout << "Setting up publishing trigger" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIP_AGENT:
      if (kVerboseSetup) std::cout << "Setting up publishing trip agent" << std::endl;
      return 0;
    case simian_public::simulator::v2::CAMERA:
      return 0;
    case simian_public::simulator::v2::LIDAR:
      return 0;
    case simian_public::simulator::v2::RADAR:
      return 0;
    case simian_public::simulator::v2::PERCEPTION_SENSOR:
      return 0;
    default:
      std::cout << "Ignoring publish setup for unsupported channel '" << channel.name() << "'"
                << std::endl;
      return 0;
  }
}

// This is where you'll be actually publishing your custom messages to your stack.
int32_t ADPBridge::ChannelPublishSend(const simian_public::simulator::v2::Channel& channel) {
  if (kVerboseTick) std::cout << "PublishSend()" << std::endl;

  switch (channel.type()) {
    case simian_public::simulator::v2::TIME:
      if (kVerboseTick) std::cout << "Publishing to Time" << std::endl;
      return 0;
    case simian_public::simulator::v2::POSE:
      if (kVerboseTick) std::cout << "Publishing to Pose" << std::endl;
      return 0;
    case simian_public::simulator::v2::ACTORS:
      if (channel.name() == "simian_perceived_actors") {
        if (kVerboseTick) std::cout << "Publishing to perceived actors" << std::endl;
        return 0;
      } else if (channel.name() == "simian_ground_truth_actors") {
        if (kVerboseTick) std::cout << "Publishing to ground truth actors" << std::endl;
        return 0;
      }
      return -1;
    case simian_public::simulator::v2::CONTROLS:
      if (kVerboseTick) std::cout << "Publishing to behavior predicted control" << std::endl;
      return 0;
    case simian_public::simulator::v2::MOTION_FEEDBACK:
      if (kVerboseTick) std::cout << "Publishing to motion feedback" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIGGER:
      if (kVerboseTick) std::cout << "Publishing to trigger" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIP_AGENT:
      if (kVerboseTick) std::cout << "Publishing to trip agent" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRAFFIC_LIGHTS:
      if (kVerboseTick) std::cout << "Publishing traffic lights" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRAFFIC_SIGN_SENSOR:
      if (kVerboseTick) std::cout << "Publishing traffic signs" << std::endl;
      return 0;
    case simian_public::simulator::v2::WIND_SENSOR:
      if (kVerboseTick) std::cout << "Publishing wind" << std::endl;
      return 0;
    case simian_public::simulator::v2::CAMERA:
      return 0;
    case simian_public::simulator::v2::LIDAR:
      return 0;
    case simian_public::simulator::v2::RADAR:
      return 0;
    case simian_public::simulator::v2::PERCEPTION_SENSOR:
      return 0;
    default:
      return 0;
  }
}

int32_t ADPBridge::ChannelPublishTeardown(const simian_public::simulator::v2::Channel& channel) {
  if (kVerboseSetup) std::cout << "PublishTeardown()" << std::endl;

  switch (channel.type()) {
    case simian_public::simulator::v2::TIME:
      if (kVerboseSetup) std::cout << "No longer publishing Time" << std::endl;
      return 0;
    case simian_public::simulator::v2::POSE:
      if (kVerboseSetup) std::cout << "No longer publishing Pose" << std::endl;
      return 0;
    case simian_public::simulator::v2::ACTORS:
      if (channel.name() == "simian_perceived_actors") {
        if (kVerboseSetup) std::cout << "No longer publishing perceived actors" << std::endl;
        return 0;
      } else if (channel.name() == "simian_ground_truth_actors") {
        if (kVerboseSetup) std::cout << "No longer publishing ground truth actors" << std::endl;
        return 0;
      }
      return -1;
    case simian_public::simulator::v2::CONTROLS:
      if (kVerboseSetup) std::cout << "No longer publishing behavior predicted control" << std::endl;
      return 0;
    case simian_public::simulator::v2::MOTION_FEEDBACK:
      if (kVerboseSetup) std::cout << "No longer publishing motion feedback" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIGGER:
      if (kVerboseSetup) std::cout << "No longer publishing trigger" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRIP_AGENT:
      if (kVerboseSetup) std::cout << "No longer publishing trip agent" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRAFFIC_LIGHTS:
      if (kVerboseSetup) std::cout << "No longer publishing traffic lights" << std::endl;
      return 0;
    case simian_public::simulator::v2::TRAFFIC_SIGN_SENSOR:
      if (kVerboseSetup) std::cout << "No longer publishing traffic signs" << std::endl;
      return 0;
    case simian_public::simulator::v2::CAMERA:
      return 0;
    case simian_public::simulator::v2::LIDAR:
      return 0;
    case simian_public::simulator::v2::RADAR:
      return 0;
    case simian_public::simulator::v2::PERCEPTION_SENSOR:
      return 0;
    default:
      std::cout << "Ignoring publish teardown for unsupported channel '" << channel.name() << "'"
                << std::endl;
      return 0;
  }
}

int32_t ADPBridge::GetDefaultRate(const simian_public::simulator::v2::Channel& channel) {
  if (kVerboseSetup) std::cout << "GetDefaultRate()" << std::endl;

  // The list of all potential channel types can be found in
  // proto/v2/io.proto file in the ChannelType message.
  // To set a new channel's frequency, please refer to that proto
  // file and create a new case statement below.
  switch (channel.type()) {
    case simian_public::simulator::v2::TIME:
    case simian_public::simulator::v2::POSE:
    case simian_public::simulator::v2::ACTORS:
    case simian_public::simulator::v2::STACK_STATE:
    case simian_public::simulator::v2::CONTROLS:
    case simian_public::simulator::v2::MOTION_FEEDBACK:
    case simian_public::simulator::v2::TRIGGER:
    case simian_public::simulator::v2::TRIP_AGENT:
      return 10;
    default:
      return 10;
  }

  return -1;
}

//////////////////////////////////////////////////
// Log(stream)-related functions
//////////////////////////////////////////////////

int32_t ADPBridge::LogFetch(const simian_public::simulator::v2::LogFetchOptions& options,
                            simian_public::simulator::v2::LogFetchOutput* output) {
  log_reader_ = std::make_unique<LogReader>(scenario_extra_data_, &mailbox_);
  return log_reader_->Fetch(options, output) ? 0 : -1;
}

int32_t ADPBridge::LogOpen(const simian_public::simulator::v2::LogOpenOptions& options,
                           simian_public::simulator::v2::LogOpenOutput* output) {
  return log_reader_->Open(options, output) ? 0 : -1;
}

int32_t ADPBridge::LogRead(const simian_public::simulator::v2::LogReadOptions& options,
                              simian_public::simulator::v2::LogReadOutput* output) {
  return log_reader_->Read(options, output) ? 0 : -1;
}

int32_t ADPBridge::LogClose(const simian_public::simulator::v2::LogCloseOptions& options) {
  bool log_close_successful = log_reader_->Close(options);
  log_reader_.reset();
  return log_close_successful ? 0 : -1;
}

//////////////////////////////////////////////////
// Core loop functions
//////////////////////////////////////////////////

// Messages will come from ADP as Google Protobuf structs.
// It is the job of the ConvertFromADP functions to convert these Protobuf messages
// into your necessary format needed as inputs for your stack.
// You should be storing instances of your custom message format in the mailbox.h file,
// and then storing the converted result in the `mailbox_` singleton, to be published
// later during the PublishSend function.

int32_t ADPBridge::ConvertTimeFromADP(const nonstd::string_view channel_name,
                                            const google::protobuf::Timestamp& time) {
  if (kVerboseTick) std::cout << "ConvertTimeFromADP()" << std::endl;
  // Check when we have different sim times for enabling interactive sim workflows.
  sim_paused_ = ((
    time.seconds() == curr_sim_time_.seconds() &&
    time.nanos() == curr_sim_time_.nanos()) &&
    (time.seconds() > 0 ||
    time.nanos() > 0));
  curr_sim_time_.CopyFrom(time);

  return 0;
}

int32_t ADPBridge::ConvertPoseFromADP(const nonstd::string_view channel_name,
                                      const simian_public::simulator::v2::Pose& pose) {
  if (kVerboseTick) std::cout << "ConvertPoseFromADP()" << std::endl;

  // Ego position in the world frame.
  const double ego_pose_px = pose.sections(0).state().pose().px();
  const double ego_pose_py = pose.sections(0).state().pose().py();
  const double ego_pose_pz = pose.sections(0).state().pose().pz();

  // Ego translational velocity observed in the world frame and expressed in the section frame.
  // For more accurate velocity info please add a localization sensor.
  const double ego_vel_tx = pose.sections(0).state().velocity().tx();
  const double ego_vel_ty = pose.sections(0).state().velocity().ty();
  const double ego_vel_tz = pose.sections(0).state().velocity().tz();

  // Ego rotational velocity observed in the world frame and expressed in the section frame.
  // Roll rate.
  const double ego_vel_rx = pose.sections(0).state().velocity().rx();
  // Pitch rate.
  const double ego_vel_ry = pose.sections(0).state().velocity().ry();
  // Yaw rate.
  const double ego_vel_rz = pose.sections(0).state().velocity().rz();

  // Ego translational acceleration observed in the world frame and expressed in the section frame.
  // For more accurate acceleration info please add a localization sensor.
  const double ego_accel_tx = pose.sections(0).state().acceleration().tx();
  const double ego_accel_ty = pose.sections(0).state().acceleration().ty();
  const double ego_accel_tz = pose.sections(0).state().acceleration().tz();

  // Ego rotational acceleration observed in the world frame and expressed in the section frame.
  // Roll accleration.
  const double ego_accel_rx = pose.sections(0).state().acceleration().rx();
  // Pitch acceleration.
  const double ego_accel_ry = pose.sections(0).state().acceleration().ry();
  // Yaw acceleration.
  const double ego_accel_rz = pose.sections(0).state().acceleration().rz();

  simian_public::drawing::Drawing example_drawing;
  example_drawing.set_name("example drawing");
  auto* color = example_drawing.mutable_color();
  color->set_red(255);
  color->set_green(0);
  color->set_blue(0);
  color->set_alpha(0.5);
  example_drawing.mutable_spline()->set_line_width(3.0);
  auto* point1 = example_drawing.mutable_spline()->add_points();
  // The Pose messsage definition and other Simian google protobuf
  // message definitions can be viewed in `applied/simian/public/proto`.
  // The Pose definition specifically lives in `spatial.proto`.
  point1->set_x(ego_pose_px);
  point1->set_y(ego_pose_py);
  point1->set_z(ego_pose_pz);
  auto* point2 = example_drawing.mutable_spline()->add_points();
  point2->set_x(ego_pose_px + 10);
  point2->set_y(ego_pose_py);
  point2->set_z(ego_pose_pz);
  SendDrawing(example_drawing);

  // Example of how we extract the orientation of the ego vehicle (roll, pitch, yaw)
  // from the ADP quaternion protobuf format stored on the pose.
  // In this case, the 2D heading of the ego vehicle maps to the yaw value
  // that is returned by this function call. Note that these converted values are in radians.
  applied::Quaternion pose_quaternion{
    pose.sections(0).state().pose().qw(),
    pose.sections(0).state().pose().qx(),
    pose.sections(0).state().pose().qy(),
    pose.sections(0).state().pose().qz()
  };
  // conversion[2] represents the yaw value, in radians.
  applied::Vector3d conversion = applied::UnitQuaternionToRollPitchYaw(pose_quaternion);
  if (kVerboseTick) std::cout << "Roll: " << conversion[0] << ", Pitch: " << conversion[1] << ", Yaw: " << conversion[2] << std::endl;

  // Example of sending a stack log, which will show up in a separate tab
  // in the frontend separate from stdout and stderr. This makes it easier
  // to filter for specific logs / events instead of putting everything in stdout.
  // For reference, a traditional std::cout will redirect to stdout.
  simian_public::stack_logs::StackLogLine log;
  log.mutable_time()->CopyFrom(curr_sim_time_);
  log.set_source_file("adp_bridge.cc");
  std::ostringstream msgStream;
  msgStream << "Ego Pose x: " << ego_pose_px << ", y: " << ego_pose_py << ", z: " << ego_pose_pz;
  log.set_msg(msgStream.str());
  log.set_thread_name(google::protobuf::util::TimeUtil::ToString(curr_sim_time_));
  log.set_log_level(simian_public::stack_logs::StackLogLine::WARN);
  SendStackLog(log);

  return 0;
}

int32_t ADPBridge::ConvertActorSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::ActorSensor& actor_sensor) {
  if (kVerboseTick) std::cout << "ConvertActorSensorFromADP(): " << channel_name << std::endl;
  return 0;
}


int32_t ADPBridge::ConvertLocalizationSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::perception::PerceptionChannel::LocalizationSensor& localization_sensor) {
  if (kVerboseTick) std::cout << "ConvertLocalizationSensorFromADP(): " << channel_name << std::endl;

  // Position in the specified reporting frame.
  // If the reporting_frame is SENSOR, then the pose is nearly identity.
  const double pose_px = localization_sensor.pose().px();
  const double pose_py = localization_sensor.pose().py();
  const double pose_pz = localization_sensor.pose().pz();

  // Translational velocity observed in the world frame and expressed in the specified reporting frame.
  // Units: [m/s]
  const double vel_tx = localization_sensor.velocity().tx();
  const double vel_ty = localization_sensor.velocity().ty();
  const double vel_tz = localization_sensor.velocity().tz();

  // Rotational velocity observed in the world frame and expressed in the specified frame.
  // Units: [rad/s]
  const double vel_rx = localization_sensor.velocity().rx();
  const double vel_ry = localization_sensor.velocity().ry();
  const double vel_rz = localization_sensor.velocity().rz();

  // Translational acceleration observed in the world frame and expressed in the specified frame.
  // Units: [m/s^2]
  const double accel_tx = localization_sensor.acceleration().tx();
  const double accel_ty = localization_sensor.acceleration().ty();
  const double accel_tz = localization_sensor.acceleration().tz();

  // Rotational acceleration observed in the world frame and expressed in the specified frame.
  // Units: [rad/s^2]
  const double ego_accel_rx = localization_sensor.acceleration().rx();
  const double ego_accel_ry = localization_sensor.acceleration().ry();
  const double ego_accel_rz = localization_sensor.acceleration().rz();

  return 0;
}

int32_t ADPBridge::ConvertRadarSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::sensor_model::SensorOutput::RadarTrack& radar_track) {
  if (kVerboseTick) std::cout << "ConvertRadarSensorFromADP(): " << channel_name << std::endl;
  std::stringstream output_path;
  output_path << record_stack_data_ << "/radar_outputs/" << radar_track.metadata().sensor_name();
  const std::string save_directory(output_path.str());
  output_path << "/" << get_timestamped_file_name(radar_track.metadata().sensor_timestamp(), ".json.gz");
  // create_directories() returns whether or not directories were created, which is false
  // for the second image, onward.
  (void) fs::create_directories(save_directory);
  std::string err_msg =
      ::applied::SaveRadarOutputAsUnrolledJson(radar_track, ::applied::RadarPoint::ALL, output_path.str());
  if (!err_msg.empty()) {
    std::cout << "error saving radar output: " + err_msg << std::endl;
    return -1;
  }
  return 0;
}

int32_t ADPBridge::ConvertLidarSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::sensor_model::SensorOutput::LidarCloud& lidar_cloud) {
  if (kVerboseTick) std::cout << "ConvertLidarSensorFromADP(): " << channel_name << std::endl;
  std::stringstream output_path;
  output_path << record_stack_data_ << "/lidar_outputs/" << lidar_cloud.metadata().sensor_name();
  const std::string save_directory(output_path.str());
  output_path << "/" << get_timestamped_file_name(lidar_cloud.metadata().sensor_timestamp(), ".png");
  // create_directories() returns whether or not directories were created, which is false
  // for the second image, onward.
  (void) fs::create_directories(save_directory);
  std::string err_msg =
      ::applied::SaveLidarOutputAsImage(lidar_cloud, ::applied::LidarImageParams(), output_path.str());
  if (!err_msg.empty()) {
    std::cout << "error saving lidar output to PNG: " + err_msg << std::endl;
    return -1;
  }
  return 0;
}

int32_t ADPBridge::ConvertCameraSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::sensor_model::SensorOutput::CameraImage& camera_image) {
  if (kVerboseTick) std::cout << "ConvertCameraSensorFromADP(): " << channel_name << std::endl;
  std::stringstream output_path;
  output_path << record_stack_data_ << "/images/" << camera_image.metadata().sensor_name();
  const std::string save_directory(output_path.str());
  output_path << "/" << get_timestamped_file_name(camera_image.metadata().sensor_timestamp(), ".png");
  ::applied::spectral::DataPointer data_pointer = shmem_helper_.ReadDataPointer(camera_image);
  if ((data_pointer.size == 0ul) || (data_pointer.memory_address == nullptr)) {
    std::cout << "error reading image: message from adp is missing image data" << std::endl;
    return -1;
  }
  std::error_code error_code{};
  // create_directories() returns whether or not directories were created, which is false
  // for the second image, onward.
  (void)fs::create_directories(save_directory, error_code);
  if (error_code) {
    std::cout << "error writing image: failed to create directory " << save_directory << ": "
              << error_code.message() << std::endl;
    return -1;
  }
  const std::string err_msg = ::applied::WritePng(data_pointer, camera_image, output_path.str());
  if (!err_msg.empty()) {
    std::cout << "error writing image: " << err_msg << std::endl;
    return -1;
  }
  return 0;
}

int32_t ADPBridge::ConvertPerceptionSensorFromADP(
    const nonstd::string_view channel_name,
    const simian_public::sensor_output::SensorOutputList& sensor_output_list) {
  return 0;
}

int32_t ADPBridge::ConvertPredictedControlFromADP(
    const nonstd::string_view channel_name, const simian_public::motion_model::Input& predicted_controls) {
  if (kVerboseTick) std::cout << "ConvertPredictedControlFromADP()" << std::endl;
  // If we get a new valid ego behavior command and the sim isn't paused, then copy it over.
  // Else use the previous ego behaviors command (like in the case for interactive sim).
  bool copy_ego_behaviors = (
    use_ego_behavior_ == true &&
    sim_paused_ == false &&
    predicted_controls.motion_command_case() != 0
  );
  if (copy_ego_behaviors) {
    predicted_controls_.CopyFrom(predicted_controls);
  }
  return 0;
}

int32_t ADPBridge::ConvertMotionFeedbackFromADP(
    const nonstd::string_view channel_name, const simian_public::motion_model::Feedback& motion_feedback) {
  if (kVerboseTick) std::cout << "ConvertMotionFeedbackFromADP()" << channel_name << std::endl;
  return 0;
}

int32_t ADPBridge::ConvertEgoTriggersFromADP(
    const nonstd::string_view channel_name, const simian_public::simulator::v2::Trigger& ego_triggers) {
  if (kVerboseTick) std::cout << "ConvertEgoTriggersFromADP()" << channel_name << std::endl;
  return 0;
}

int32_t ADPBridge::ConvertTripAgentFromADP(
    const nonstd::string_view channel_name, const simian_public::common::TripAgentOutput& trip_agent) {
  if (kVerboseTick) std::cout << "ConvertTripAgentFromADP()" << channel_name << std::endl;
  return 0;
}

// It is the job of the ConvertToADP functions to convert your stack outputs,
// in other words the control commands that your stack is generating, and store them
// in the specified pointer input variables. Completing these below functions will enable
// "closed loop" simulation, since this means that your stack is able to both
// accept the world state from Simian and compute the next control command for Simian to simulate.

int32_t ADPBridge::ConvertControlsToADP(const nonstd::string_view channel_name,
                                              simian_public::motion_model::Input* ego_input) {
  if (kVerboseTick) std::cout << "ConvertControlsToADP()" << channel_name << std::endl;

  if (use_ego_behavior_ == true && predicted_controls_.motion_command_case() == 0) {
    std::cout << "No behavior specified in the scenario, please specify "
              << "one under agents->ego->behaviors" << std::endl;
    return -1;
  }

  // This allows the customer interface bridge to forward any Simian ego behaviors
  // back to Simian to enable open loop simulation.
  if(use_ego_behavior_ == true) {
    ego_input->CopyFrom(predicted_controls_);
    return 0;
  }

  // TODO(customer): When closing the loop, the customer must pass their
  // controls commands into the ego_input pointer in order for Simian
  // to see the stack's commands. The motion model that Simian expects
  // will be defined under the `vehicle` section in the scenario yaml, and
  // the protobuf message definition for the corresponding motion model
  // can be found in motion_model.proto.
  return -1;
}

int32_t ADPBridge::ConvertStackStateToADP(
    const nonstd::string_view channel_name, simian_public::simulator::v2::StackState* stack_state) {
  if (kVerboseTick) std::cout << "ConvertStackStateToADP()" << channel_name << std::endl;

  stack_state->set_stack_state(simian_public::sim_data::SimulatorInput::ENGAGED);
  return 0;
}