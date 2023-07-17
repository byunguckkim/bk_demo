#include "applied/simian/public/map/simian_map_client.h"

#include <stdlib.h>
#include <time.h>
#include <iostream>
#include <random>

#ifdef _WIN32
#include <windows.h>
#undef ERROR
#else
#include <unistd.h>
#endif

namespace {

void msleep(int ms) {
#ifdef _WIN32
  Sleep(ms);
#else
  usleep(ms * 1000);
#endif
}

int get_sleep_time_milliseconds(int retry) {
  // calculating in milliseconds
  std::random_device rd;
  std::mt19937 gen(rd());
  std::uniform_real_distribution<> distrib(0, 1);

  const float kMinSleeping = 50.f;
  const float kMaxSleeping = 5000.f;

  float backoff = std::min(kMinSleeping * (float)std::pow(2, retry), kMaxSleeping);
  float jitter = backoff * distrib(gen);
  float sleep_time = (backoff + jitter) / 2.;
  return (int)sleep_time;
}

}  // namespace

namespace simian_public {
namespace map {
SimianMapClient::SimianMapClient(std::string connection_string, std::string map_key, bool retry)
    : map_key_(std::move(map_key)), retry_(retry) {
  grpc::ChannelArguments map_channel_args = grpc::ChannelArguments();
  map_channel_args.SetMaxSendMessageSize(200 * 1024 * 1024);
  map_channel_args.SetMaxReceiveMessageSize(200 * 1024 * 1024);

  auto map_channel = grpc::CreateCustomChannel(
      connection_string, grpc::InsecureChannelCredentials(), map_channel_args);
  stub_ = std::move(public_map_service::SimianMapService::NewStub(map_channel));
  auto deadline_string = std::getenv("SIMIAN_MAP_GRPC_DEADLINE");
  if (deadline_string != nullptr && std::atoi(deadline_string) != 0) {
    deadline_ = std::chrono::milliseconds(std::atoi(deadline_string));
  }
}

simian_public::map_config::MapConfig SimianMapClient::GetMapConfig() const {
  public_map_service::GetMapConfigRequest request;
  request.set_map_key(map_key_);

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::GetMapConfigResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->GetMapConfig(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "GetMapConfig errored!" << std::endl;
        return response.map_config();
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->GetMapConfig(&context, request, &response);
  }

  LogErrors("GetMapConfig", status, response.common());

  return response.map_config();
}

public_map_service::SearchRouteResponse SimianMapClient::SearchRoute(
    const google::protobuf::RepeatedPtrField<simian_public::common::PointENU>& points,
    const bool enable_change_lane_in_result) const {
  public_map_service::SearchRouteRequest request;
  request.set_map_key(map_key_);
  request.mutable_map_layer()->set_selection(public_map_service::MapLayer_Selection_HDMAP);
  request.mutable_request()->set_enable_change_lane_in_result(enable_change_lane_in_result);
  for (simian_public::common::PointENU point : points) {
    auto waypoint = request.mutable_request()->add_waypoint();
    waypoint->mutable_pose()->Swap(&point);
  }

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::SearchRouteResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->SearchRoute(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "SearchRoute errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->SearchRoute(&context, request, &response);
  }

  LogErrors("SearchRoute", status, response.common());

  return response;
}

public_map_service::GetNearestLanePointResponse SimianMapClient::GetNearestLanePoint(
    const simian_public::common::PointENU* point) {
  public_map_service::GetNearestLanePointRequest request;
  request.set_map_key(map_key_);
  request.mutable_map_layer()->set_selection(public_map_service::MapLayer_Selection_HDMAP);
  request.mutable_point()->set_x(point->x());
  request.mutable_point()->set_y(point->y());

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::GetNearestLanePointResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->GetNearestLanePoint(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "GetNearestLanePoint errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->GetNearestLanePoint(&context, request, &response);
  }

  LogErrors("GetNearestLanePoint", status, response.common());

  return response;
}

public_map_service::GetLaneByIdResponse SimianMapClient::GetLaneById(std::string lane_id) {
  public_map_service::GetLaneByIdRequest request;
  request.set_map_key(map_key_);
  request.mutable_map_layer()->set_selection(public_map_service::MapLayer_Selection_HDMAP);
  request.set_lane_id(lane_id);
  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);
  public_map_service::GetLaneByIdResponse response;
  grpc::Status status = stub_->GetLaneById(&context, request, &response);

  LogErrors("GetLaneById", status, response.common());

  return response;
}

public_map_service::GetLaneProjectionResponse SimianMapClient::GetLaneProjection(
    const simian_public::common::PointENU* point, const double distance,
    simian_public::scenario::LaneSplitPreference::SplitPreference split_preference,
    const bool allow_cycles, const bool backwards) {
  public_map_service::GetLaneProjectionRequest request;
  request.set_map_key(map_key_);
  request.mutable_map_layer()->set_selection(public_map_service::MapLayer_Selection_HDMAP);
  request.set_x(point->x());
  request.set_y(point->y());
  request.set_distance(distance);
  request.set_split_preference(split_preference);
  request.set_allow_cycles(allow_cycles);
  request.set_backwards(backwards);

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::GetLaneProjectionResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->GetLaneProjection(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "GetLaneProjection errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->GetLaneProjection(&context, request, &response);
  }

  LogErrors("GetLaneProjection", status, response.common());
  return response;
}

public_map_service::GetLanesWithCurvatureResponse SimianMapClient::GetLanesWithCurvature(
    const double lower_bound, const double upper_bound, const double min_dist_between_points,
    const double min_length_of_lane, const bool has_single_neighbor_forward_lane,
    const bool has_single_successor, const double min_ratio_points_in_lane) {
  public_map_service::GetLanesWithCurvatureRequest request;
  request.set_map_key(map_key_);
  request.set_lower_bound(lower_bound);
  request.set_upper_bound(upper_bound);
  request.mutable_filters()->set_min_dist_between_points(min_dist_between_points);
  request.mutable_filters()->set_min_length_of_lane(min_length_of_lane);
  request.mutable_filters()->set_has_single_neighbor_forward_lane(has_single_neighbor_forward_lane);
  request.mutable_filters()->set_has_single_successor(has_single_successor);
  request.mutable_filters()->set_min_ratio_points_in_lane(min_ratio_points_in_lane);

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::GetLanesWithCurvatureResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->GetLanesWithCurvature(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "GetLanesWithCurvature errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->GetLanesWithCurvature(&context, request, &response);
  }

  LogErrors("GetLanesWithCurvature", status, response.common());
  return response;
}

public_map_service::GetNearestLanePointsAcrossTerrainResponse
SimianMapClient::GetNearestLanePointsAcrossTerrain(const simian_public::common::PointENU* point) {
  public_map_service::GetNearestLanePointsAcrossTerrainRequest request;
  request.set_map_key(map_key_);
  request.mutable_map_layer()->set_selection(public_map_service::MapLayer_Selection_HDMAP);
  request.mutable_point()->set_x(point->x());
  request.mutable_point()->set_y(point->y());

  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::GetNearestLanePointsAcrossTerrainResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->GetNearestLanePointsAcrossTerrain(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "GetNearestLanePointsAcrossTerrain errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->GetNearestLanePointsAcrossTerrain(&context, request, &response);
  }

  LogErrors("GetNearestLanePointsAcrossTerrain", status, response.common());
  return response;
}

public_map_service::TerrainRaycastResponse SimianMapClient::TerrainRaycast(
    const simian_public::common::TerrainConfig& terrain_config,
    const google::protobuf::RepeatedPtrField<public_map_service::Ray>& rays) const {
  public_map_service::TerrainRaycastRequest request;
  request.set_map_key(map_key_);
  request.mutable_terrain_config()->CopyFrom(terrain_config);
  for (auto ray : rays) {
    auto proto_ray = request.add_ray();
    proto_ray->CopyFrom(ray);
  }
  grpc::ClientContext context;
  context.set_wait_for_ready(true);
  context.set_deadline(std::chrono::system_clock::now() + deadline_);

  public_map_service::TerrainRaycastResponse response;
  response.mutable_common()->set_status(public_map_service::CommonResponse::TRY_AGAIN);
  grpc::Status status;
  if (retry_) {
    int retries = 0;
    while (response.common().status() == public_map_service::CommonResponse::TRY_AGAIN) {
      status = stub_->TerrainRaycast(&context, request, &response);
      if (response.common().status() == public_map_service::CommonResponse::ERROR) {
        std::cerr << "TerrainRaycast errored!" << std::endl;
        return response;
      }
      msleep(get_sleep_time_milliseconds(retries));
      retries++;
    }
  } else {
    status = stub_->TerrainRaycast(&context, request, &response);
  }

  LogErrors("TerrainRaycast", status, response.common());

  return response;
}

void SimianMapClient::LogErrors(std::string rpc_name, const grpc::Status& grpc_status,
                                const public_map_service::CommonResponse& common_response) const {
  if (!grpc_status.ok()) {
    std::cerr << rpc_name + " failed with error code "
              << grpc_status.error_code() + ": " + grpc_status.error_message() << std::endl;
    return;
  }
  if (common_response.status() != public_map_service::CommonResponse::SUCCESS) {
    std::string status = public_map_service::CommonResponse_Status_Name(common_response.status());
    std::cerr << rpc_name + " failed with status " + status + ": " + common_response.error_message()
              << std::endl;
  }
}

void SimianMapClient::SetDeadline(std::chrono::milliseconds deadline) {
  deadline_ = std::move(deadline);
}

}  // namespace map
}  // namespace simian_public
