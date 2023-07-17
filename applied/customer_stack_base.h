#pragma once

/*
 * A customer's AV stack gets integrated into Simian by inheriting from the
 * CustomerStackBase class and implementing the necessary methods.
 * This class gets instantiated inside a grpc server that runs inside the
 * customer docker container. The Simian simulation engine, which runs
 * in the Applied docker, calls that grpc server as appropriate during
 * a simulation run. The class can override a number of methods to
 * extend or customize some of the generic behavior.
 */

#include <cstdint>
#include <memory>
#include <string>

// RGB is a macro when using MSVC. We use a conflicting proto message named RGB
// from common.proto so we need to undef the macro for Windows compatibility.
#if !defined(__linux__) && defined(RGB)
#undef RGB
#endif
#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/drawing.pb.h"
#include "applied/simian/public/proto/motion_model.pb.h"
#include "applied/simian/public/proto/perception.pb.h"
#include "applied/simian/public/proto/route.pb.h"
#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/sensor_output.pb.h"
#include "applied/simian/public/proto/sim_command.pb.h"
#include "applied/simian/public/proto/spatial.pb.h"
#include "applied/simian/public/proto/stack_logs.pb.h"
#include "applied/simian/public/proto/v2/io.pb.h"
#include "applied/stack_interface_c_v2.h"
#include "applied/simian/public/utils/string_view.h"

// SendMessage is a macro in windows.h. Messes up when calling SendMessage from
// Customer Stack Base
#if !defined(__linux__) && defined(SendMessage)
#undef SendMessage
#endif

namespace simian_public {

class CustomerStackBase {
 public:
  CustomerStackBase(const std::string& ego_name, const ADPSendFunctions& func_ptrs);
  virtual ~CustomerStackBase() = default;

  CustomerStackBase(const CustomerStackBase&) = delete;
  CustomerStackBase& operator=(const CustomerStackBase&) = delete;

  /********************************************************
   * INITIALIZATION
   * These functions are called in the order provided here.
   ********************************************************/

  /**
   * @brief Returns a version string for your customer stack.
   * @return A version identifier which will make it clear what customer
   *         stack a simulation was running with.
   */
  virtual const char* GetStackVersion() { return nullptr; }

  /**
   * @brief Process the startup options (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t SetStartupOptions(const simulator::v2::InterfaceStartupOptions& startup_options);

  /**
   * @brief Bring up your middleware (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t MiddlewareSetup();

  /**
   * @brief Set up your AV stack (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t StackSetup();

  /**
   * @brief Bring up customer log recording (optional).
   *        Note that if you do not intend to implement recording, you should
   *        start Simian with the sim flag --no_customer_record.
   * @param recording_path path to write logs to. Note that you need to create
   *        this directory first.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t RecordingSetup(const nonstd::string_view recording_path);

  /**
   * @brief Bring up your live visualization (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t VisualizationSetup();

  /**
   * @brief Return the default rate for the given channel. Simian will call
   *        this function once for every channel. For every channel, you need
   *        to provide either a rate or a period.
   * @param channel Channel description for which to return the rate.
   * @return Channel rate in Hertz. If you want to provide the channel
   *         period instead, return 0.
   */
  virtual int32_t GetDefaultRate(const simulator::v2::Channel& channel);

  /**
   * @brief Return the default channel period in nanoseconds. Simian will call
   *        this function once for every channel. For every channel, you need
   *        to provide either a rate or a period.
   * @param channel Channel description for which to return the period.
   * @return Channel period in nanoseconds, i.e. 1e9 is once per second. If you
   *         want to provide the channel rate instead, return 0.
   */
  virtual int64_t GetDefaultPeriodNs(const simulator::v2::Channel& channel);

  /**
   * @brief Bring up any listener for the given channel (optional). Simian will
   *        call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelListenSetup(const simulator::v2::Channel& channel);

  /**
   * @brief Initialize any publisher for the given channel (optional). Simian
   *        will call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishSetup(const simulator::v2::Channel& channel);

  /**
   * @brief Fetch a drive log (optional) with the given filename.
   *        Optionally cache the log for faster retrieval in subsequent
   *        simulation runs.
   * @param options Input information for fetching the log.
   * @param output Output containing information about the log being fetched.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogFetch(const simian_public::simulator::v2::LogFetchOptions& options,
                           simian_public::simulator::v2::LogFetchOutput* output);

  /**
   * @brief Open a drive log (optional) with the given filename, to the given
   *        slot, listening to the given channel, which should be held whenever
   *        a log_read() call is made.
   * @param options Options for opening the log.
   * @param output Output containing information about the log being opened.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogOpen(const simian_public::simulator::v2::LogOpenOptions& options,
                          simian_public::simulator::v2::LogOpenOutput* output);

  /**
   * @brief Rewinds the log to a previous timestamp i.e. performs any needed preparation for the
   *        next LogRead call, which may jump back in time from the last LogRead call. The nexto
   *        LogRead call's `offset` will be at the provided `target_sim_time` or later.
   * @param options Parameters for rewinding the log.
   * @param output Output containing information about the log rewind operation.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogRewind(const simian_public::simulator::v2::LogRewindOptions& options,
                            simian_public::simulator::v2::LogRewindOutput* output);

  /**
   * @brief Optional initialization after transmitting the initial simulation state at t=0.
   * This function is called after ListenSetup/PublishSetup, ConvertFromADP, and (where applicable)
   * PublishSend was called for all channels.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t Initialize();

  /****************************************************************
   * CORE LOOP - INPUT TO THE SIMULATOR
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************************************************/

  /**
   * @brief Provide motion model input to the simulator.
   * @param channel_name Name of CONTROLS channel.
   * @param ego_input Motion model input for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertControlsToADP(const nonstd::string_view channel_name,
                                       motion_model::Input* ego_input);

  /**
   * @brief Provide stack state to the simulator.
   * @param channel_name Name of STACK_STATE channel.
   * @param stack_state Stack state for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertStackStateToADP(const nonstd::string_view channel_name,
                                         simulator::v2::StackState* stack_state);

  /**
   * @brief Provide trajectory input to the simulator.
   * @param channel_name Name of TRAJECTORY channel.
   * @param trajectory Trajectory input for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrajectoryToADP(const nonstd::string_view channel_name,
                                         simian_public::spatial::Trajectory* trajectory);

  /**
   * @brief Provide simulated or logged ego's pose to the simulator.
   * @param channel_name Name of POSE channel.
   * @param pose Pose for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPoseToADP(const nonstd::string_view channel_name,
                                   simulator::v2::Pose* pose);

  /**
   * @brief Provide data on the vehicle to the simulator.
   * @param channel_name Name of VEHICLE_DATA channel.
   * @param vehicle_data Data for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertVehicleDataToADP(const nonstd::string_view channel_name,
                                          simulator::v2::VehicleData* vehicle_data);

  /**
   * @brief Provide actor sensor detections or labels to the simulator.
   * @param channel_name Name of ACTORS channel.
   * @param actor_sensor Actor sensor for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertActorSensorToADP(const nonstd::string_view channel_name,
                                          perception::PerceptionChannel::ActorSensor* actor_sensor);

  /**
   * @brief Provide lane sensor detections to the simulator.
   * @param channel_name Name of LANE_SENSOR channel.
   * @param lane_sensor Lane sensor for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLaneSensorToADP(const nonstd::string_view channel_name,
                                         perception::PerceptionChannel::LaneSensor* lane_sensor);

  /**
   * @brief Provide traffic light sensor detections to the simulator.
   * @param channel_name Name of TRAFFIC_LIGHTS channel.
   * @param traffic_light_sensor Traffic light sensor for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficLightSensorToADP(
      const nonstd::string_view channel_name,
      perception::PerceptionChannel::TrafficLightSensor* traffic_light_sensor);

  /**
   * @brief Provide traffic sign sensor detections to the simulator.
   * @param channel_name Name of TRAFFIC_SIGN_SENSOR channel.
   * @param traffic_sign_sensor Traffic sign sensor for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficSignSensorToADP(
      const nonstd::string_view channel_name,
      perception::PerceptionChannel::TrafficSignSensor* traffic_sign_sensor);

  /**
   * @brief Provide a lidar point cloud to the simulator.
   * @param channel_name Name of LIDAR channel.
   * @param lidar_cloud Lidar cloud for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLidarSensorToADP(const nonstd::string_view channel_name,
                                          sensor_model::SensorOutput::LidarCloud* lidar_cloud);

  /**
   * @brief Provide a radar track to the simulator.
   * @param channel_name Name of RADAR channel.
   * @param radar_track Radar track for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertRadarSensorToADP(const nonstd::string_view channel_name,
                                          sensor_model::SensorOutput::RadarTrack* radar_track);

  /**
   * @brief Provide a camera image to the simulator.
   * @param channel_name Name of CAMERA channel.
   * @param camera_image Camera image for the simulator.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertCameraSensorToADP(const nonstd::string_view channel_name,
                                           sensor_model::SensorOutput::CameraImage* camera_image);

  /**
   * @brief Used for running code at a particular rate, handled like a channel.
   * @param channel_name Name of the CUSTOM channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t HandleCustomChannel(const nonstd::string_view channel_name);

  /****************************************************************
   * CORE LOOP - OUTPUT FROM THE SIMULATOR
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************************************************/

  /**
   * @brief Convert timestamp to stack format.
   * @param channel_name Name of TIME Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTimeFromADP(const nonstd::string_view channel_name,
                                     const google::protobuf::Timestamp& time);

  /**
   * @brief Convert ego vehicle pose to stack format. You should prefer output
   *        from the localization sensor for more exact data.
   * @param channel_name Name of POSE Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPoseFromADP(const nonstd::string_view channel_name,
                                     const simulator::v2::Pose& pose);

  /**
   * @brief Convert motion feedback message to stack format.
   * @param channel_name Name of MOTION_FEEDBACK Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertMotionFeedbackFromADP(const nonstd::string_view channel_name,
                                               const motion_model::Feedback& motion_feedback);

  /**
   * @brief Convert Simian's predicted controls for the ego to stack format.
   * @param channel_name Name of CONTROLS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPredictedControlFromADP(const nonstd::string_view channel_name,
                                                 const motion_model::Input& predicted_controls);

  /**
   * @brief Convert ego trigger message to stack format. Note that actor
   *        triggers are reported through the ACTORS channel.
   * @param channel_name Name of MOTION_FEEDBACK Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertEgoTriggersFromADP(const nonstd::string_view channel_name,
                                            const simulator::v2::Trigger& ego_triggers);

  /**
   * @brief Convert the ego's trip agent command message to stack format.
   * @param channel_name Name of TRIP_AGENT Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTripAgentFromADP(const nonstd::string_view channel_name,
                                          const common::TripAgentOutput& trip_agent);

  /**
   * @brief Convert the ego vehicle's stack state to stack format.
   * @param channel_name Name of STACK_STATE Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertStackStateFromADP(const nonstd::string_view channel_name,
                                           const simulator::v2::StackState& stack_state);

  /**
   * @brief Convert vehicle data to stack format.
   * @param channel_name Name of VEHICLE_DATA channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertVehicleDataFromADP(const nonstd::string_view channel_name,
                                            const simulator::v2::VehicleData& vehicle_data);

  /****************************
   * CORE LOOP - SENSOR OUTPUT
   * These functions are called in the order in which the channels
   * are defined in the scenario.
   ****************************/

  /**
   * @brief Convert the actor sensor output to stack format.
   * @param channel_name Name of ACTORS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertActorSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::ActorSensor& actor_sensor);

  /**
   * @brief Convert the lane sensor output to stack format.
   * @param channel_name Name of LANE_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLaneSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::LaneSensor& lane_sensor);

  /**
   * @brief Convert the traffic light sensor output to stack format.
   * @param channel_name Name of TRAFFIC_LIGHTS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficLightSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::TrafficLightSensor& traffic_light_sensor);

  /**
   * @brief Convert the localization sensor output to stack format.
   * @param channel_name Name of LOCALIZATION_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLocalizationSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::LocalizationSensor& localization_sensor);

  /**
   * @brief Convert the planar Lidar sensor output to stack format.
   * @param channel_name Name of PLANAR_LIDAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPlanarLidarSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::PlanarLidarSensor& planar_lidar_sensor);

  /**
   * @brief Convert the planar occupancy grid sensor output to stack format.
   * @param channel_name Name of PLANAR_OCCUPANCY_GRID Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPlanarOccupancyGridSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::PlanarOccupancyGridSensor& planar_occupancy_grid_sensor);

  /**
   * @brief Convert the occlusion grid sensor output to stack format.
   * @param channel_name Name of OCCLUSION_GRID Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertOcclusionGridSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::OcclusionGridSensor& occlusion_grid_sensor);

  /**
   * @brief Convert the free space sensor output to stack format.
   * @param channel_name Name of FREE_SPACE_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertFreeSpaceSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::FreeSpaceSensor& free_space_sensor);

  /**
   * @brief Convert the traffic light block sensor output to stack format.
   * @param channel_name Name of TRAFFIC_LIGHT_BLOCKS Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficLightBlockSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::TrafficLightBlockSensor& traffic_light_block_sensor);

  /**
   * @brief Convert the traffic sign sensor output to stack format.
   * @param channel_name Name of TRAFFIC_SIGN_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTrafficSignSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::TrafficSignSensor& traffic_sign_sensor);

  /**
   * @brief Convert the IMU sensor output to stack format.
   * @param channel_name Name of IMU_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertImuSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::IMUSensor& imu_sensor);

  /**
   * @brief Convert the wheel speed sensor output to stack format.
   * @param channel_name Name of WHEEL_SPEED_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertWheelSpeedSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::WheelSpeedSensor& wheel_speed_sensor);

  /**
   * @brief Convert the map sensor output to stack format.
   * @param channel_name Name of MAP_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertMapSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::MapSensor& map_sensor);

  /**
   * @brief Convert the terrain sensor output to stack format.
   * @param channel_name Name of TERRAIN_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertTerrainSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::TerrainSensor& terrain_sensor);

  /**
   * @brief Convert the polar obstacle sensor output to stack format.
   * @param channel_name Name of POLAR_OBSTACLE_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPolarObstacleSensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::PolarObstacleSensor& polar_obstacle_sensor);

  /**
   * @brief Convert the agent trajectory sensor output to stack format.
   * @param channel_name Name of AGENT_TRAJECTORY_SENSOR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertAgentTrajectorySensorFromADP(
      const nonstd::string_view channel_name,
      const perception::PerceptionChannel::AgentTrajectorySensor& agent_trajectory_sensor);

  /**
   * @brief Convert the Lidar output to stack format.
   * @param channel_name Name of LIDAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertLidarSensorFromADP(
      const nonstd::string_view channel_name,
      const sensor_model::SensorOutput::LidarCloud& lidar_cloud);

  /**
   * @brief Convert the radar output to stack format.
   * @param channel_name Name of RADAR Simian channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertRadarSensorFromADP(
      const nonstd::string_view channel_name,
      const sensor_model::SensorOutput::RadarTrack& radar_track);

  /**
   * @brief Convert the camera output to stack format.
   * @param channel_name Name of CAMERA channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertCameraSensorFromADP(
      const nonstd::string_view channel_name,
      const sensor_model::SensorOutput::CameraImage& camera_image);

  /**
   * @brief Convert the ultrasound range sensor to stack format.
   * @param channel_name Name of ULTRASOUND channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertUltrasoundSensorFromADP(
      const nonstd::string_view channel_name,
      const sensor_model::SensorOutput::Range& ultrasound_range);

  /**
   * @brief Convert the outputs of a Spectral sensor model to stack format.
   * @param channel_name Name of PERCEPTION_SENSOR channel.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ConvertPerceptionSensorFromADP(
      const nonstd::string_view channel_name,
      const sensor_output::SensorOutputList& sensor_output_list);

  /***********************************
   * CORE LOOP - ADDITIONAL FUNCTIONS
   ***********************************/

  /**
   * @brief Publish currently held messages for the given channel.
   * @param channel Channel information for which to publish messages.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishSend(const simulator::v2::Channel& channel);

  /************************
   * LOG-READING FUNCTIONS
   ************************/

  /**
   * @brief Read up until the provided offset into the bag, optionally returning
   *        early. Any data seen meant for the stack should be sent directly,
   *        preferably after updating the stack's time. This is equivalent to
   *        updating the TIME channel, calling publish_send() on it and the
   *        channel received from the log, without the Simian version of those
   *        channels.
   * @param options Options for reading from the log.
   * @param output Log data.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogRead(const simulator::v2::LogReadOptions& options,
                          simulator::v2::LogReadOutput* output);

  /**
   * @brief When a message in a log needs to be patched, after the relevant
   *        log__read() and convert__to_simian() calls, patch() will be
   *        called with the changes necessary for the given channel.
   * @param channel Channel to apply patch to.
   * @param options Patch options, containing patch operations to execute.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t Patch(const simulator::v2::Channel& channel,
                        const simulator::v2::PatchOptions& patch_options);

  /***************
   * FINALIZATION
   ***************/

  /**
   * @brief Before anything is torn down, execute any early or simulation-wide finalization.
   * This is executed before the simulation summary, before plugins are finalized, and before the
   * teardown.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t Finalize();

  /**
   * @brief Process the simulation summary (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t SimulationSummary(const simian_public::common::SimulationSummary& summary);

  /**
   * @brief Close and clean up the log in the given slot (optional).
   * @param options Options for closing the log.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t LogClose(const simulator::v2::LogCloseOptions& options);

  /**
   * @brief Clean up any publishers created for the given channel (optional).
   *        Simian will call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelPublishTeardown(const simulator::v2::Channel& channel);

  /**
   * @brief Clean up any listener for the given channel (optional). Simian will
   *        call this function once for every channel.
   * @param channel Channel description.
   * @return 0 for success, other values for failure.
   */
  virtual int32_t ChannelListenTeardown(const simulator::v2::Channel& channel);

  /**
   * @brief Clean up live visualization (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t VisualizationTeardown();

  /**
   * @brief Clean up customer log recording (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t RecordingTeardown();

  /**
   * @brief Clean up your AV stack (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t StackTeardown();

  /**
   * @brief Clean up your middleware (optional).
   * @return 0 for success, other values for failure.
   */
  virtual int32_t MiddlewareTeardown();

  /***************************************************
   * HELPER FUNCTIONS TO BE CALLED FROM THE INTERFACE
   * Do not override, simply call these functions.
   ***************************************************/

  /**
   * @brief Send a stack log line to Simian.
   * @param log_line Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendStackLog(const simian_public::stack_logs::StackLogLine& log_line) const;

  /**
   * @brief Send a drawing to Simian.
   * @param drawing Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendDrawing(const simian_public::drawing::Drawing& drawing) const;

  /**
   * @brief Send a data point to Simian.
   * @param data_point Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendDataPoint(const simian_public::common::DataPoint& data_point) const;

  /**
   * @brief Send a timestamped data point to Simian.
   * @param timestamped_data_point Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendTimestampedDataPoint(
      const simian_public::common::TimestampedDataPoint& timestamped_data_point) const;

  /**
   * @brief Send custom data point metadata to Simian.
   * @param metadata Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendCustomDataPointMetadata(
      const simian_public::common::CustomDataPointMetadata& metadata) const;

  /**
   * @brief Send a simulation command to Simian.
   * @param sim_command Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendSimCommand(const simian_public::sim_command::SimCommand& sim_command) const;

  /**
   * @brief Send an observer event to Simian.
   * @param event Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendObserverEvent(const simian_public::common::ObserverEvent& event) const;

  /**
   * @brief Send a message to Simian.
   * @param message Message payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendMessage(const simian_public::common::Message& message) const;

  /**
   * @brief Send time-series struct data to ADP.
   * @param timestamped_struct Struct payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendTimestampedStruct(
      const simian_public::common::TimestampedStruct& timestamped_struct) const;

  /**
   * @brief Send a simulation custom field to ADP.
   * @param custom_field CustomField payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendSimulationCustomField(const simian_public::common::CustomField& custom_field) const;

  /**
   * @brief Send a log custom field to ADP.
   * @param custom_field CustomField payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendLogCustomField(const simian_public::common::CustomField& custom_field) const;

  /**
   * @brief Send a triage event to ADP.
   * @param triage_event TriageEvent payload.
   * @return 0 for success, other values for failure.
   */
  int32_t SendTriageEvent(const public_triage::TriageEvent& triage_event) const;

  constexpr static int32_t kProtoSerializationError = -1000;

 protected:
  std::string ego_name_;
  ADPSendFunctions send_func_ptrs_;
};

/**
 * @brief Implement this function to create your customer stack. Typically this will simply be:
 * std::unique_ptr<simian_public::CustomerStackBase> simian_public::CreateCustomerStack(
 *      const std::string& name, const ADPSendFunctions& func_ptrs) {
 *   return std::make_unique<YourCustomerStack>(name, func_ptrs);
 * }
 */
std::unique_ptr<CustomerStackBase> CreateCustomerStack(const std::string& name,
                                                       const ADPSendFunctions& func_ptrs);

}  // namespace simian_public
