// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <functional>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
#include <optional>
#include <string>
#include <vector>

#include "applied/simian/public/proto/sensor_model.pb.h"
#include "applied/simian/public/utils/optional.h"
#include "applied/spectral/public/image_point.h"
#include "applied/spectral/public/spectral_shared_memory_helper.h"

namespace applied {

/**
 * @brief Lets the user modify an image before compression, and returns a non-empty error message to
 * abort the process of saving a compressed image.
 */
using ImageModifier = std::function<std::string(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    cv::Mat& modified_image)>;

/**
 * @struct WriteImageParams
 * @brief Collection of parameters used to determine how the image is written.
 *
 * Currently contains a list of image modifier functions and an OpenCV color conversion code to
 * apply to the final written image. Codes can be found at
 * https://docs.opencv.org/3.4/d8/d01/group__imgproc__color__conversions.html
 */
struct WriteImageParams {
  std::vector<ImageModifier> modifiers;
  int cv_color_conversion_code = cv::COLOR_BGRA2BGR;
};

/**
 * @brief Write a PNG with image data from a CameraImage to the given output_path.
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the DataPointer's image properties.
 * @param output_path Full-path output destination, include the filename. Must end with ".png".
 *                    The parent directory of `output_path` must already exist for this to succeed.
 * @param write_image_params A WriteImageParams struct containing params that control how the image
 * is written.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WritePng(const ::applied::spectral::DataPointer& data_pointer,
                     const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path, const WriteImageParams& write_image_params);

std::string WritePng(const simian_public::sensor_output::NDArray& ndarray,
                     const std::string& output_path);

/**
 * @brief Write a PNG with image data from a CameraImage to the given output_path.
 *
 * @param camera_image The camera image describing the image and image properties.
 * @param output_path Full-path output destination, include the filename. Must end with ".png".
 *                    The parent directory of `output_path` must already exist for this to succeed.
 * @param modifiers The list of modifier functions to execute on the image before compression.
 *                  Modifier functions are called sequentially, and the first non-empty string
 *                  returned will abort writing the PNG.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WritePng(const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path,
                     const std::vector<ImageModifier>& modifiers = {});

/**
 * @brief Write a PNG with image data from the given shared-memory DataPointer to the given
 * output_path.
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the DataPointer's image properties.
 * @param output_path Full-path output destination, include the filename. Must end with ".png".
 *                    The parent directory of `output_path` must already exist for this to succeed.
 * @param modifiers The list of modifier functions to execute on the image before compression.
 *                  Modifier functions are called sequentially, and the first non-empty string
 *                  returned will abort writing the PNG.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WritePng(const ::applied::spectral::DataPointer& data_pointer,
                     const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path,
                     const std::vector<ImageModifier>& modifiers = {});

/**
 * @brief Save the raw bytes of a PNG from the given shared-memory DataPointer to the given buffer.
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the DataPointer's image properties.
 * @param modified_buffer The output buffer to write to.
 * @param write_image_params A WriteImageParams struct containing params that control how the image
 * is written.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SavePngToBuffer(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const WriteImageParams& write_image_params);

/**
 * @brief Save the raw bytes of a PNG from the given CameraImage to the given buffer.
 *
 * @param camera_image The camera image describing the image and image properties.
 * @param modified_buffer The output buffer to write to.
 * @param modifiers The list of modifier functions to execute on the image before compression.
 *                  Modifier functions are called sequentially, and the first non-empty string
 *                  returned will abort writing the PNG.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SavePngToBuffer(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const std::vector<ImageModifier>& modifiers = {});

/**
 * @brief Save the raw bytes of a PNG from the given shared-memory DataPointer to the given buffer.
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the DataPointer's image properties.
 * @param modified_buffer The output buffer to write to.
 * @param modifiers The list of modifier functions to execute on the image before compression.
 *                  Modifier functions are called sequentially, and the first non-empty string
 *                  returned will abort writing the PNG.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string SavePngToBuffer(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const std::vector<ImageModifier>& modifiers = {});

struct ImageKeypoint {
  int actor_id;
  ::simian_public::sensor_sim::keypoints::ActorKeypoint::KeypointType keypoint_type;
  ImagePoint image_point;
};

std::string GetLabelKeypointsOnImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<ImageKeypoint>& out_keypoints);

std::string GetLabelKeypointsOnImageAsJson(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    nlohmann::json& out_json);
/**
 * @brief Calculate how many bytes each channel of the pixel (RGBA) will occupy.
 *
 * @returns a number of bytes (e.g. 1, 2, or 4)
 */
size_t GetBytesPerPixelChannel(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image);

/**
 * @brief Converts the raw BGRA array pointed to by a DataPointer and described by a CameraImage
 * into a BGR cv::Mat of uint8_t or uint16_t suitable for use with various OpenCV image compression
 * functions (which generally expect integral data formats).
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the image and image properties.
 * @returns Empty string on success, non-empty string on failure.
 */
std::pair<cv::Mat, std::string> PrepareImageData(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image);

/**
 * @brief Converts the raw BGRA array pointed to by a DataPointer and described by a CameraImage
 * into a BGR cv::Mat of uint8_t or uint16_t suitable for use with various OpenCV image compression
 * functions (which generally expect integral data formats).
 *
 * @param data_pointer The shared-memory DataPointer to read image data from.
 * @param camera_image The camera image describing the image and image properties.
 * @param write_image_params A WriteImageParams struct containing params that control how the image
 * is written.
 * @returns Empty string on success, non-empty string on failure.
 */
std::pair<cv::Mat, std::string> PrepareImageData(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    const WriteImageParams& write_image_params);

std::string SaveCameraDecalLabelsAsJson(
    const simian_public::sensor_model::SensorOutput_CameraImage& camera_image,
    const std::string& output_path);
namespace impl {
// WARN implementation details, may change without notice
nonstd::optional<size_t> GetDepthByteDepth(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image);
}  // namespace impl
}  // namespace applied
