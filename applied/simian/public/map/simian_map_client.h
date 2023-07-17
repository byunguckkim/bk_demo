#pragma once

#include "grpcpp/grpcpp.h"
#include "applied/simian/public/proto/public_map_service.grpc.pb.h"
#include "applied/simian/public/proto/public_map_service.pb.h"
#include "applied/simian/public/proto/terrain_config.pb.h"

namespace simian_public {
namespace map {

class SimianMapClient {
 public:
  // Inputs:
  //   connection_string: server address (i.e. startup_options_.map_service_address()).
  //   map_key: Name of the map to be queried, most likely startup_options_.map_key().
  SimianMapClient(std::string connection_string, std::string map_key, bool retry = false);

  // Return the map_config based on the initialized map_key.
  virtual simian_public::map_config::MapConfig GetMapConfig() const;

  virtual public_map_service::SearchRouteResponse SearchRoute(
      const google::protobuf::RepeatedPtrField<simian_public::common::PointENU>& points,
      const bool enable_change_lane_in_result = false) const;
  virtual public_map_service::GetNearestLanePointResponse GetNearestLanePoint(
      const simian_public::common::PointENU* point);
  virtual public_map_service::GetLaneByIdResponse GetLaneById(std::string lane_id);
  // Projects along the lane for the given distance from the given point (in meters).
  // Defers to the lane split preference if there are multiple choices for the successor lane.
  virtual public_map_service::GetLaneProjectionResponse GetLaneProjection(
      const simian_public::common::PointENU* point, const double distance,
      simian_public::scenario::LaneSplitPreference::SplitPreference split_preference,
      const bool allow_cycles, const bool backwards);
  // Find all lanes that satisfy the curvature (with unit 1/m) and filter conditions.
  virtual public_map_service::GetLanesWithCurvatureResponse GetLanesWithCurvature(
      const double lower_bound, const double upper_bound, const double min_dist_between_points,
      const double min_length_of_lane, const bool has_single_neighbor_forward_lane,
      const bool has_single_successor, const double min_ratio_points_in_lane);
  // Given UTM x and y values, return the closest lane center point and any lane points
  // above, below or intersecting it on the map, their headings, 2d distances from
  // the given point (in meters), and the lane_ids of those points.
  virtual public_map_service::GetNearestLanePointsAcrossTerrainResponse
  GetNearestLanePointsAcrossTerrain(const simian_public::common::PointENU* point);

  // Find the intersection point between rays and the terrain.
  // The first call to this function for a given terrain_config requires
  // initialization that may take a few seconds on large maps.
  // Consider invoking this function once during initialization using
  // a deadline of approximately 5 seconds. In this initialization, use rays
  // that are representative of the rays you'll use during the simulation.
  virtual public_map_service::TerrainRaycastResponse TerrainRaycast(
      const simian_public::common::TerrainConfig& terrain_config,
      const google::protobuf::RepeatedPtrField<public_map_service::Ray>& rays) const;

  // Set the duration of time to wait for gRPC calls to complete. Default: 50ms.
  virtual void SetDeadline(std::chrono::milliseconds deadline);

  virtual const std::string& map_key() const { return map_key_; }

  virtual ~SimianMapClient() = default;

 protected:
  // Protected constructor for use by mock.
  SimianMapClient() = default;

 private:
  void LogErrors(std::string rpc_name, const grpc::Status& grpc_status,
                 const public_map_service::CommonResponse& common_response) const;
  std::string map_key_;
  std::unique_ptr<public_map_service::SimianMapService::Stub> stub_;
  std::chrono::milliseconds deadline_ = std::chrono::milliseconds(1000);
  bool retry_;
  static constexpr int retry_max = 10;
};

}  // namespace map
}  // namespace simian_public
