// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/proto/sensor_sim/basic_types.pb.h"
#include "applied/simian/public/proto/sensor_sim/keypoints.pb.h"
#include "applied/simian/public/proto/sensor_sim/labels.pb.h"
#include "applied/spectral/public/image_helpers.h"
#include "applied/spectral/public/image_projector.h"

namespace applied {

struct LabelVisualizationParams {
  struct DrawingParams {
    cv::Scalar color{235, 64, 52};
    int thickness = 1;
    int line_type = cv::LINE_AA;
    cv::Scalar contrasting_color{64, 235, 52};
  };

  // enable_visible_bounding_boxes is true iff we should draw a box around the
  // visible pixels of each actor in the image.
  bool enable_visible_bounding_boxes = true;
  DrawingParams visible_bounding_box_params{cv::Scalar{106, 221, 251}, 1, cv::LINE_AA};
  // enable_occluded_bounding_boxes is true iff we should draw a box around
  // the visible and occluded pixels of an actor in an image.
  bool enable_occluded_bounding_boxes = true;
  DrawingParams occluded_bounding_box_params{cv::Scalar{84, 124, 226}, 1, cv::LINE_AA};

  // enable_3d_bounding_boxes is true iff we should draw the 3d bounding box
  // of each actor in the image.
  // the visible and occluded pixels of an actor in an image.
  bool enable_3d_bounding_boxes = true;
  DrawingParams bounding_box_3d_params;

  // Set to true if lane_markings_2d labels should be visualized if present.  This must also be set
  // to true if crosswalk or stopline markings should be visualized.
  bool enable_lane_markings_2d = true;
  DrawingParams lane_markings_2d_params{{0, 128, 0}, 2, cv::LINE_AA};
  DrawingParams crosswalk_markings_2d_params{{64, 128, 0}, 2, cv::LINE_AA};
  DrawingParams stopline_markings_2d_params{{0, 128, 64}, 2, cv::LINE_AA};

  bool enable_parking_spaces = true;
  DrawingParams parking_spaces_params{{200, 100, 50}, 2, cv::LINE_AA};

  // Set to true if free_space labels should be visualized if present.
  bool enable_free_space = true;
  DrawingParams free_space_params{{128, 255, 0}, 2, cv::LINE_AA};
  DrawingParams free_space_hole_params{{255, 0, 255}, 2, cv::LINE_AA};

  bool enable_actor_keypoints = true;
  DrawingParams actor_keypoints_params{{235, 64, 52}, 2, cv::LINE_AA};

  bool enable_decal_bounding_boxes = true;
  DrawingParams full_decal_bounding_box_params{cv::Scalar{53, 221, 251}, 1, cv::LINE_AA};
  DrawingParams decal_character_bounding_box_params{cv::Scalar{251, 221, 53}, 1, cv::LINE_AA};

  bool enable_curbwall = true;
  DrawingParams curbwall_params{{200, 0, 150}, 2, cv::LINE_AA};
};

void Draw2DBox(const simian_public::sensor_sim::basic_types::ImageBoundingBox& box,
               const LabelVisualizationParams::DrawingParams& params, cv::Mat& modified);

// Draw a 3D bounding box around a given centroid.  Works in "sensor" and "world" reference frames.
// If the "world" frame is used, the `world_T_sensor` field in the sensor metadata is used to find
// the bounds in the sensor frame.
bool Draw3DBox(const simian_public::spatial::Pose& centroid,
               const simian_public::spatial::Point& extent,
               const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
               const simian_public::sensor_model::Metadata& metadata,
               const LabelVisualizationParams::DrawingParams& params,
               const ::applied::ImageProjector& image_projector, cv::Mat& modified);

// This draws a 3D bounding box from a given pose which is assumed to be center of the
// bottom face of the bounding box.
// If drawing 3d bounding box labels, prefer to instead use Draw3DBoxFromCentroid because
// the reported actor pose from Spectral may not actually be this center, bottom face (i.e. for
// trailer-like actors).
// This overload of Draw3DBox is deprecated and will be removed soon.
bool Draw3DBox(const simian_public::spatial::Pose& pose,
               const simian_public::spatial::Point& extent,
               const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
               const LabelVisualizationParams::DrawingParams& params,
               const ::applied::ImageProjector& image_projector, cv::Mat& modified);

// Draw a 3D bounding box from the bounding boxes centroid.
bool Draw3DBoxFromCentroid(
    const simian_public::spatial::Pose& centroid, const simian_public::spatial::Point& extent,
    const simian_public::sensor_sim::labels::FrameDescription& reference_frame,
    const LabelVisualizationParams::DrawingParams& params,
    const ::applied::ImageProjector& image_projector, cv::Mat& modified);

std::string DrawKeypoints(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    const google::protobuf::RepeatedPtrField<simian_public::sensor_sim::keypoints::ActorKeypoint>&
        keypoints,
    const LabelVisualizationParams::DrawingParams& params,
    const ::applied::ImageProjector& image_projector, cv::Mat& modified);

// VisualizeBoundingBoxes returns an image modifier that may be used with
// routines from image_helpers.h to draw bounding boxes from labels to an
// image:
//
//   ::applied::VisualizationParams default_params;
//   err_msg = applied::WritePng(data_pointer, camera_image, output_path,
//                      {VisualizeBoundingBoxes(default params)};
ImageModifier VisualizeBoundingBoxes(LabelVisualizationParams params);

}  // namespace applied
