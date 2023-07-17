// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/label_visualization.h"

#include <fstream>
#include <vector>

#include "applied/simian/public/modules/transforms/proto_conversion.h"
#include "applied/spectral/public/image_helpers.h"
#include "applied/spectral/public/image_point.h"

namespace applied {
using ::google::protobuf::RepeatedPtrField;
namespace {

cv::Point rounded_cv_point(const ImagePoint& ip) {
  return {static_cast<int>(round(ip.x)), static_cast<int>(round(ip.y))};
}

}  // namespace

void Draw2DBox(const simian_public::sensor_sim::basic_types::ImageBoundingBox& box,
               const LabelVisualizationParams::DrawingParams& params, cv::Mat& modified) {
  const cv::Point top_left(box.min().col(), box.min().row());
  const cv::Point bottom_right(box.max().col(), box.max().row());
  cv::rectangle(modified, top_left, bottom_right, params.color, params.thickness, params.line_type);
}

bool Draw3DBox(const simian_public::spatial::Pose& pose,
               const simian_public::spatial::Point& extent,
               const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
               const LabelVisualizationParams::DrawingParams& params,
               const ::applied::ImageProjector& image_projector, cv::Mat& modified) {
  static bool warning_printed = false;
  if (!warning_printed) {
    std::cout << "WARNING:  This overload of Draw3DBox is deprecated and will be removed soon.  "
                 "Please use the other overload or use Draw3DBoxFromCentroid."
              << std::endl;
    warning_printed = true;
  }
  const applied::Pose3d pose_T_centroid{Vector3d(0.0, 0.0, extent.z() / 2.0), IdentityQuaternion()};
  Pose3d world_T_pose;
  if (!FromProto(pose, &world_T_pose)) {
    return false;
  }

  simian_public::spatial::Pose assumed_centroid;
  ToProto(world_T_pose * pose_T_centroid, &assumed_centroid);
  return Draw3DBoxFromCentroid(assumed_centroid, extent, reference_frame, params, image_projector,
                               modified);
}

namespace {
bool Draw3DBoxFromCentroidPose3D(const Pose3d& centroid,
                                 const simian_public::spatial::Point& extent,
                                 const LabelVisualizationParams::DrawingParams& params,
                                 const ::applied::ImageProjector& image_projector,
                                 cv::Mat& modified) {
  const double half_x = extent.x() / 2.0;
  const double half_y = extent.y() / 2.0;
  const double half_z = extent.z() / 2.0;

  std::vector<ImagePoint> image_points;
  {
    const std::vector<Vector3d> bbox_points{
        {half_x, half_y, half_z},    {half_x, -half_y, half_z},  {half_x, -half_y, -half_z},
        {half_x, half_y, -half_z},   {-half_x, half_y, half_z},  {-half_x, -half_y, half_z},
        {-half_x, -half_y, -half_z}, {-half_x, half_y, -half_z},
    };
    image_points.resize(bbox_points.size());
    for (size_t i = 0; i < bbox_points.size(); ++i) {
      if (!image_projector.Project(centroid * bbox_points[i], image_points[i])) {
        // Some point didn't project correctly, impossible to draw a complete 3d box.
        return false;
      }
    }
  }

  constexpr size_t kFrontTopLeftCornerIdx = 0;
  std::vector<ImagePoint> corner_points_in_image;
  {
    // Mark the front top left corner in a different color to show orientation
    const Vector3d corner{half_x, half_y, half_z};
    const std::vector<Vector3d> corner_points{
        corner + 0.2 * (Vector3d(half_x, half_y, 0) - corner),
        corner + 0.2 * (Vector3d(half_x, 0, half_z) - corner),
        corner + 0.2 * (Vector3d(0, half_y, half_z) - corner),
    };
    corner_points_in_image.resize(corner_points.size());
    for (size_t i = 0; i < corner_points.size(); ++i) {
      if (!image_projector.Project(centroid * corner_points[i], corner_points_in_image[i])) {
        // Some point didn't project correctly, impossible to draw a complete 3d box.
        return false;
      }
    }
  }
  static const std::array<std::pair<int, int>, 12> connectivity{
      // front face
      std::make_pair(0, 1),
      std::make_pair(1, 2),
      std::make_pair(2, 3),
      std::make_pair(3, 0),
      // back face
      std::make_pair(4, 5),
      std::make_pair(5, 6),
      std::make_pair(6, 7),
      std::make_pair(7, 4),
      // connections between the corners
      std::make_pair(0, 4),
      std::make_pair(1, 5),
      std::make_pair(2, 6),
      std::make_pair(3, 7),
  };
  for (const auto& ab : connectivity) {
    cv::line(modified, rounded_cv_point(image_points[ab.first]),
             rounded_cv_point(image_points[ab.second]), params.color, params.thickness,
             params.line_type);
  }
  for (const applied::ImagePoint& corner_point : corner_points_in_image) {
    cv::line(modified, rounded_cv_point(image_points[kFrontTopLeftCornerIdx]),
             rounded_cv_point(corner_point), params.contrasting_color, params.thickness,
             params.line_type);
  }
  return true;
}
}  // namespace

bool Draw3DBox(const simian_public::spatial::Pose& centroid,
               const simian_public::spatial::Point& extent,
               const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
               const simian_public::sensor_model::Metadata& metadata,
               const LabelVisualizationParams::DrawingParams& params,
               const ::applied::ImageProjector& image_projector, cv::Mat& modified) {
  switch (reference_frame.frame_case()) {
    case simian_public::sensor_sim::labels::FrameDescription::FRAME_NOT_SET:
      return false;
    case simian_public::sensor_sim::labels::FrameDescription::kSensorFrame:
      return Draw3DBoxFromCentroid(centroid, extent, reference_frame, params, image_projector,
                                   modified);
    case simian_public::sensor_sim::labels::FrameDescription::kWorldFrame: {
      Pose3d world_T_sensor;
      if (!FromProto(metadata.world_t_sensor(), &world_T_sensor)) {
        return false;
      }
      Pose3d centroid_pose3d;
      if (!FromProto(centroid, &centroid_pose3d)) {
        return false;
      }
      return Draw3DBoxFromCentroidPose3D(world_T_sensor.Inverse() * centroid_pose3d, extent, params,
                                         image_projector, modified);
    }
    case simian_public::sensor_sim::labels::FrameDescription::kEgoFrame:
      std::cout << "Ego frame not supported" << std::endl;
      return false;
  }
  throw std::runtime_error("Unexpected reference frame");
  return false;
}

bool Draw3DBoxFromCentroid(
    const simian_public::spatial::Pose& centroid, const simian_public::spatial::Point& extent,
    const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
    const LabelVisualizationParams::DrawingParams& params,
    const ::applied::ImageProjector& image_projector, cv::Mat& modified) {
  if (!reference_frame.has_sensor_frame()) {
    return false;
  }
  Pose3d actor_in_camera;
  if (!FromProto(centroid, &actor_in_camera)) {
    return false;
  }
  return Draw3DBoxFromCentroidPose3D(actor_in_camera, extent, params, image_projector, modified);
}

std::string DrawKeypoints(
    const simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    const simian_public::sensor_sim::labels::ActorOutput actor_output,
    const LabelVisualizationParams::DrawingParams& params,
    const ::applied::ImageProjector& image_projector, cv::Mat& modified) {
  if (!actor_output.has_reference_frame() || !actor_output.reference_frame().has_sensor_frame()) {
    return "";  // Can't draw keypoints when ActorOutput is in another reference frame, but don't
                // error either.
  }
  std::vector<ImageKeypoint> out_keypoints{};
  std::string err_msg = GetLabelKeypointsOnImage(camera_image, out_keypoints);
  if (!err_msg.empty()) {
    return err_msg;
  }

  for (ImageKeypoint keypoint : out_keypoints) {
    cv::circle(modified, rounded_cv_point(keypoint.image_point), 5, params.color);  // radius of 5
  }
  return "";
}

ImageModifier VisualizeBoundingBoxes(LabelVisualizationParams params) {
  return [params](const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                  cv::Mat& modified_image) -> std::string {
    if (!camera_image.has_label_snapshot()) {
      return std::string("");
    }
    // Construct image projector for 3d bbox and for actor keypoints
    const Pose3d identity_pose;
    auto projector_with_error = ImageProjector::Make(identity_pose, camera_image);
    ImageProjector& image_projector = projector_with_error.first;
    if (!projector_with_error.second.empty()) {
      return "error making projector to visualize bounding boxes and/or keypoints: " +
             projector_with_error.second;
    }

    for (const auto& label_output : camera_image.label_snapshot().labels()) {
      // Visualize 3D bounding boxes
      if (params.enable_3d_bounding_boxes) {
        if (label_output.has_bounding_box_3ds()) {
          // For 3D bounding boxes directly requested under label_request
          for (const auto& bbox : label_output.bounding_box_3ds().boxes()) {
            Draw3DBox(bbox.centroid(), bbox.extent(), bbox.reference_frame(),
                      camera_image.metadata(), params.bounding_box_3d_params, image_projector,
                      modified_image);
          }
        }
        if (label_output.has_actor_outputs()) {
          // For 3D bounding boxes requested as a field under actoroutputfields
          for (const auto& actor : label_output.actor_outputs().actors()) {
            for (const auto& bbox_3d : actor.bboxes_3d()) {
              Draw3DBox(bbox_3d.centroid(), bbox_3d.extent(), actor.reference_frame(),
                        camera_image.metadata(), params.bounding_box_3d_params, image_projector,
                        modified_image);
            }
          }
        }
        if (label_output.has_road_markings()) {
          for (const auto& road_marking : label_output.road_markings().markings()) {
            for (const auto& box : road_marking.bounding_boxes_3d()) {
              Draw3DBox(box.centroid(), box.extent(), box.reference_frame(),
                        camera_image.metadata(), params.bounding_box_3d_params, image_projector,
                        modified_image);
            }
          }
        }
      }

      if (params.enable_occluded_bounding_boxes) {
        if (label_output.has_bounding_box_2ds()) {
          // For 2D bounding boxes directly requested under label_request
          for (const auto& bbox : label_output.bounding_box_2ds().boxes()) {
            Draw2DBox(bbox.bounds_ignoring_occlusion(), params.occluded_bounding_box_params,
                      modified_image);
          }
        } else if (label_output.has_actor_outputs()) {
          // For 2D bounding boxes requested as a field under actoroutputfields
          for (const auto& actor : label_output.actor_outputs().actors()) {
            for (const auto& bbox_2d : actor.bboxes_2d()) {
              Draw2DBox(bbox_2d.bounds_ignoring_occlusion(), params.occluded_bounding_box_params,
                        modified_image);
            }
          }
        }
      }
      if (params.enable_visible_bounding_boxes) {
        // For 2D bounding boxes directly requested under label_request
        if (label_output.has_bounding_box_2ds()) {
          for (const auto& bbox : label_output.bounding_box_2ds().boxes()) {
            Draw2DBox(bbox.visible_bounds(), params.visible_bounding_box_params, modified_image);
          }
        } else if (label_output.has_actor_outputs()) {
          // For 2D bounding boxes requested as a field under actoroutputfields
          for (const auto& actor : label_output.actor_outputs().actors()) {
            for (const auto& bbox_2d : actor.bboxes_2d()) {
              Draw2DBox(bbox_2d.visible_bounds(), params.visible_bounding_box_params,
                        modified_image);
            }
          }
        }
      }
      if (params.enable_decal_bounding_boxes) {
        for (const auto& decal : label_output.decal_bounding_box_2ds().decals()) {
          Draw2DBox(decal.full_decal_visible_bounds(), params.full_decal_bounding_box_params,
                    modified_image);
          for (const auto& bbox : decal.character_bounding_boxes()) {
            Draw2DBox(bbox.visible_bounds(), params.decal_character_bounding_box_params,
                      modified_image);
          }
        }
      }

      if (params.enable_lane_markings_2d && label_output.has_lane_markings_2d()) {
        for (const auto& segment : label_output.lane_markings_2d().segments()) {
          cv::Scalar color;
          int thickness;
          int line_type;
          if (segment.road_marking_type() ==
              simian_public::hdmap::RoadMarkingTypeEnum_RoadMarkingType_CROSSWALK) {
            color = params.crosswalk_markings_2d_params.color;
            line_type = params.crosswalk_markings_2d_params.line_type;
            thickness = params.crosswalk_markings_2d_params.thickness;
          } else if (segment.road_marking_type() ==
                     simian_public::hdmap::RoadMarkingTypeEnum_RoadMarkingType_STOP_BAR) {
            color = params.stopline_markings_2d_params.color;
            line_type = params.stopline_markings_2d_params.line_type;
            thickness = params.stopline_markings_2d_params.thickness;
          } else {
            color = params.lane_markings_2d_params.color;
            line_type = params.lane_markings_2d_params.line_type;
            thickness = params.lane_markings_2d_params.thickness;
          }
          for (size_t i = 0; i + 1 < static_cast<size_t>(segment.polygon_points().size()); ++i) {
            const auto& pt0 = segment.polygon_points()[i];
            const auto& pt1 = segment.polygon_points()[i + 1];
            cv::line(modified_image, {int(std::floor(pt0.col())), int(std::floor(pt0.row()))},
                     {int(std::floor(pt1.col())), int(std::floor(pt1.row()))}, color, thickness,
                     line_type);
          }
          // If we're drawing around the outside of the lane marking, we need to close the loop.
          if (segment.polygon_points().size() > 1 &&
              label_output.lane_markings_2d().trace_perimeter()) {
            const auto& pt_front = segment.polygon_points()[0];
            const auto& pt_back = segment.polygon_points()[segment.polygon_points().size() - 1];
            cv::line(modified_image,
                     {int(std::floor(pt_front.col())), int(std::floor(pt_front.row()))},
                     {int(std::floor(pt_back.col())), int(std::floor(pt_back.row()))}, color,
                     thickness, line_type);
          }
        }
      }

      if (params.enable_parking_spaces && label_output.has_parking_space()) {
        for (const auto& polygon : label_output.parking_space().parking_polygons()) {
          for (size_t i = 0; i + 1 < static_cast<size_t>(polygon.polygon_points().size()); ++i) {
            const auto& pt0 = polygon.polygon_points()[i];
            const auto& pt1 = polygon.polygon_points()[i + 1];
            cv::line(modified_image, {int(std::floor(pt0.col())), int(std::floor(pt0.row()))},
                     {int(std::floor(pt1.col())), int(std::floor(pt1.row()))},
                     params.parking_spaces_params.color, params.parking_spaces_params.thickness,
                     params.parking_spaces_params.line_type);
          }
        }
      }
      if (params.enable_free_space && label_output.has_free_space()) {
        for (const auto& segment : label_output.free_space().boundary()) {
          const auto& free_space_params =
              (segment.is_hole()) ? params.free_space_hole_params : params.free_space_params;
          for (size_t i = 0; i + 1 < static_cast<size_t>(segment.polygon_points().size()); ++i) {
            const auto& pt0 = segment.polygon_points()[i];
            const auto& pt1 = segment.polygon_points()[i + 1];
            cv::line(modified_image, {int(std::floor(pt0.col())), int(std::floor(pt0.row()))},
                     {int(std::floor(pt1.col())), int(std::floor(pt1.row()))},
                     free_space_params.color, free_space_params.thickness,
                     free_space_params.line_type);
          }
          // If we're drawing around the outside of the lane marking, we need to close the loop.
          if (segment.polygon_points().size() > 1) {
            const auto& pt_front = segment.polygon_points()[0];
            const auto& pt_back = segment.polygon_points()[segment.polygon_points().size() - 1];
            cv::line(
                modified_image, {int(std::floor(pt_front.col())), int(std::floor(pt_front.row()))},
                {int(std::floor(pt_back.col())), int(std::floor(pt_back.row()))},
                free_space_params.color, free_space_params.thickness, free_space_params.line_type);
          }
        }
      }
      if (params.enable_curbwall && label_output.has_curbwall_polygons()) {
        for (const auto& polygon : label_output.curbwall_polygons().polygons()) {
          for (size_t i = 0; i + 1 < static_cast<size_t>(polygon.polygon_points().size()); ++i) {
            const auto& pt0 = polygon.polygon_points()[i];
            const auto& pt1 = polygon.polygon_points()[i + 1];
            cv::line(modified_image, {int(std::floor(pt0.col())), int(std::floor(pt0.row()))},
                     {int(std::floor(pt1.col())), int(std::floor(pt1.row()))},
                     params.curbwall_params.color, params.curbwall_params.thickness,
                     params.curbwall_params.line_type);
          }
          // Close the loop.
          if (polygon.polygon_points().size() > 1) {
            const auto& pt_front = polygon.polygon_points()[0];
            const auto& pt_back = polygon.polygon_points()[polygon.polygon_points().size() - 1];
            cv::line(modified_image,
                     {int(std::floor(pt_front.col())), int(std::floor(pt_front.row()))},
                     {int(std::floor(pt_back.col())), int(std::floor(pt_back.row()))},
                     params.curbwall_params.color, params.curbwall_params.thickness,
                     params.curbwall_params.line_type);
          }
        }
      }
      if (params.enable_actor_keypoints && label_output.has_actor_outputs()) {
        for (const auto& actor : label_output.actor_outputs().actors()) {
          const std::string err_msg = DrawKeypoints(
              camera_image, actor, params.actor_keypoints_params, image_projector, modified_image);
          if (!err_msg.empty()) {
            return "error drawing keypoints: " + err_msg;
          }
        }
      }
    }
    return std::string();
  };
}

}  // namespace applied
