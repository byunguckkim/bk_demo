#include "applied/spectral/public/image_helpers.h"

#include <google/protobuf/util/json_util.h>
#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <vector>

#include "applied/simian/public/proto/sensor_output.pb.h"
#include "applied/simian/public/modules/transforms/spatial.h"
#include "applied/spectral/public/image_projector.h"
#include "applied/spectral/public/ndarray_helpers.h"
#include "applied/spectral/public/spectral_shared_memory_helper.h"
#include "applied/spectral/public/utils.h"

namespace {
using ::google::protobuf::RepeatedPtrField;

bool EndsWith(std::string str, std::string suffix, bool ignore_case) {
  if (ignore_case) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);
  }
  return str.length() >= suffix.length() && str.substr(str.length() - suffix.length()) == suffix;
}
}  // namespace
namespace applied {

std::string WritePng(const simian_public::sensor_output::NDArray& ndarray,
                     const std::string& output_path) {
  const auto BLUE = simian_public::sensor_output::NDArray::Field::BLUE;
  const auto GREEN = simian_public::sensor_output::NDArray::Field::GREEN;
  const auto RED = simian_public::sensor_output::NDArray::Field::RED;
  auto make_result = NDArrayRepackedBuffer::Make(ndarray, {BLUE, GREEN, RED});
  if (!make_result.second.empty()) return make_result.second;
  NDArrayRepackedBuffer& repacked_buffer = *make_result.first;
#pragma pack(push, 1)
  struct ReducedElement {
    float blue;
    float green;
    float red;
  };
#pragma pack(pop)
  const ReducedElement* reduced_elements =
      reinterpret_cast<const ReducedElement*>(repacked_buffer.PackedData());
  cv::Mat camera_image(ndarray.shape(1), ndarray.shape(0), CV_32FC3);
  float* p = reinterpret_cast<float*>(camera_image.data);
  for (uint32_t i = 0; i < ndarray.shape(1) * ndarray.shape(0); i++) {
    const ReducedElement& reduced_element = reduced_elements[i];
    p[0] = reduced_element.blue;
    p[1] = reduced_element.green;
    p[2] = reduced_element.red;
    p += 3;
  }
  camera_image *= 255.0;
  cv::imwrite(output_path, camera_image, std::vector<int>());
  return "";
}

std::string WritePng(const ::applied::spectral::DataPointer& data_pointer,
                     const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path, const WriteImageParams& write_image_params) {
  if (!EndsWith(output_path, ".png", true)) {
    return "expected output_path to end with .png: " + output_path;
  }
  auto image_data_result = PrepareImageData(data_pointer, camera_image, write_image_params);
  cv::Mat& prepared_data = image_data_result.first;
  const std::string& prepare_image_err_msg = image_data_result.second;
  if (!prepare_image_err_msg.empty()) {
    return "error preparing image data: " + prepare_image_err_msg;
  }
  for (const ImageModifier& modifier_func : write_image_params.modifiers) {
    const std::string modifier_err_msg = modifier_func(camera_image, prepared_data);
    if (!modifier_err_msg.empty()) {
      return "error modifying image data: " + modifier_err_msg;
    }
  }
  static const std::vector<int> use_default_png_params;
  const bool success = cv::imwrite(output_path, prepared_data, use_default_png_params);
  if (!success) {
    // Typically this is because the parent directory hasn't been created yet.
    return "error writing png to: " + output_path;
  }
  return "";
}

std::string WritePng(const ::applied::spectral::DataPointer& data_pointer,
                     const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path, const std::vector<ImageModifier>& modifiers) {
  WriteImageParams writing_params;
  writing_params.modifiers = modifiers;
  return WritePng(data_pointer, camera_image, output_path, writing_params);
}

std::string WritePng(const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
                     const std::string& output_path, const std::vector<ImageModifier>& modifiers) {
  return WritePng(spectral::SMHelperSingleton::GetInstance().ReadDataPointer(camera_image),
                  camera_image, output_path, modifiers);
}

std::string SavePngToBuffer(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const WriteImageParams& write_image_params) {
  auto image_data_result = PrepareImageData(data_pointer, camera_image, write_image_params);
  cv::Mat& prepared_data = image_data_result.first;
  std::string& prepare_image_err_msg = image_data_result.second;
  if (!prepare_image_err_msg.empty()) {
    return "error preparing image data: " + prepare_image_err_msg;
  }
  for (const ImageModifier& modifier_func : write_image_params.modifiers) {
    const std::string modifier_err_msg = modifier_func(camera_image, prepared_data);
    if (!modifier_err_msg.empty()) {
      return "error modifying image data: " + modifier_err_msg;
    }
  }
  static const std::vector<int> use_default_png_params;
  const bool success = cv::imencode(".png", prepared_data, modified_buffer, use_default_png_params);
  if (!success) {
    return "error compressing PNG to buffer";
  }
  return "";
}

std::string SavePngToBuffer(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const std::vector<ImageModifier>& modifiers) {
  WriteImageParams writing_params;
  writing_params.modifiers = modifiers;
  return SavePngToBuffer(data_pointer, camera_image, modified_buffer, writing_params);
}

std::string SavePngToBuffer(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<uint8_t>& modified_buffer, const std::vector<ImageModifier>& modifiers) {
  return SavePngToBuffer(spectral::SMHelperSingleton::GetInstance().ReadDataPointer(camera_image),
                         camera_image, modified_buffer, modifiers);
}

std::string GetLabelKeypointsOnImage(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    std::vector<ImageKeypoint>& out_keypoints) {
  const Pose3d identity_pose;
  auto image_projector_result = ImageProjector::Make(identity_pose, camera_image);
  ImageProjector& image_projector = image_projector_result.first;
  const std::string& image_projector_err_msg = image_projector_result.second;
  if (!image_projector_err_msg.empty()) {
    return image_projector_err_msg;
  }

  for (const auto& label_output : camera_image.label_snapshot().labels()) {
    if (!label_output.has_actor_outputs()) {
      continue;
    }

    for (const auto& actor : label_output.actor_outputs().actors()) {
      // TODO(#113442):  Support world frame.
      if (!actor.has_reference_frame() || !actor.reference_frame().has_sensor_frame()) {
        continue;
      }

      for (const auto& keypoint : actor.actor_keypoints()) {
        // make into vec, then project that vec
        Vector3d original_point = {keypoint.point().x(), keypoint.point().y(),
                                   keypoint.point().z()};
        // Create a new ImageKeypoint
        ImageKeypoint img_keypoint;
        img_keypoint.actor_id = actor.id();
        img_keypoint.keypoint_type = keypoint.keypoint_type();
        if (!image_projector.Project(original_point, img_keypoint.image_point)) {
          // This point didn't project in image frame
          continue;
        }
        // If within frame, add this to a list
        out_keypoints.push_back(img_keypoint);
      }
    }
  }
  return "";
}

std::string GetLabelKeypointsOnImageAsJson(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    nlohmann::json& out_json) {
  std::vector<ImageKeypoint> out_keypoints{};
  const std::string err_msg = GetLabelKeypointsOnImage(camera_image, out_keypoints);
  if (!err_msg.empty()) {
    return err_msg;
  }

  for (ImageKeypoint keypoint : out_keypoints) {
    (out_json)["actor_keypoints"].push_back(
        {{"actor_id", keypoint.actor_id},
         {"reference_frame", "pixel_frame"},
         {"type", keypoint.keypoint_type},
         {"image_point", {{"col", keypoint.image_point.x}, {"row", keypoint.image_point.y}}}});
  }

  return "";
}

size_t GetBytesPerPixelChannel(
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image) {
  return ceil(camera_image.image().color_depth() / 8.0);
}

std::pair<cv::Mat, std::string> PrepareImageData(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image) {
  WriteImageParams writing_params;
  return PrepareImageData(data_pointer, camera_image, writing_params);
}

std::pair<cv::Mat, std::string> PrepareImageData(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image,
    const WriteImageParams& write_image_params) {
  const int channels = camera_image.image_shape().channels();
  if (channels != 4 && camera_image.image_shape().channels() != 1) {
    return {{}, "expected 1 or 4 channels of image data, but got: " + std::to_string(channels)};
  }
  size_t bytes_per_channel = GetBytesPerPixelChannel(camera_image);
  bool is_uint = camera_image.image().data_type() ==
                 simian_public::sensor_model::CameraSystemParams_DataType_UINT;
  const size_t expected_size = camera_image.image_shape().height() *
                               camera_image.image_shape().width() *
                               camera_image.image_shape().channels() * bytes_per_channel;

  if (data_pointer.size != expected_size) {
    nonstd::optional<size_t> depth_size = impl::GetDepthByteDepth(data_pointer, camera_image);

    if (depth_size) {
      // If size matches expected depth bit depth, use that
      bytes_per_channel = *depth_size;
      // Depth currently doesn't support floating point
      is_uint = true;
    } else {
      return {{},
              "expected " + std::to_string(expected_size) + " bytes of image data, but got " +
                  std::to_string(data_pointer.size)};
    }
  }

  int channel_data_type = CV_8U;
  if (is_uint && bytes_per_channel == 1) {
    channel_data_type = CV_8U;
  } else if (is_uint && bytes_per_channel == 2) {
    channel_data_type = CV_16U;
  } else if (!is_uint && bytes_per_channel == 4) {
    channel_data_type = CV_32F;
  } else {
    return {{},
            "unsupported channel format: " + std::to_string(camera_image.image().color_depth()) +
                " color depth " + (is_uint ? "(uint)" : "(float)")};
  }

  const cv::Mat orig_data(camera_image.image_shape().height(), camera_image.image_shape().width(),
                          CV_MAKETYPE(channel_data_type, channels),
                          (void*)data_pointer.memory_address);
  static const std::vector<int> use_default_png_params;
  cv::Mat uint_data;
  if (is_uint && (channel_data_type == CV_16U || channel_data_type == CV_8U)) {
    uint_data = orig_data;
  } else {
    // Scale to a 16 bit uint since OpenCV will save only uint types.
    // Original data is in [0, 1] so we need to scale it up.
    orig_data.convertTo(uint_data, CV_16U, (std::numeric_limits<uint16_t>::max)());
  }
  if (channels == 1) {
    return {orig_data, ""};
  }
  // Apply CV color conversion.
  cv::Mat result;
  cv::cvtColor(uint_data, result, write_image_params.cv_color_conversion_code);
  return {result, ""};
}

std::string SaveCameraDecalLabelsAsJson(
    const simian_public::sensor_model::SensorOutput_CameraImage& camera_image,
    const std::string& output_path) {
  // Generate JSON
  std::string decal_json;
  google::protobuf::util::JsonPrintOptions options;
  options.add_whitespace = true;
  options.preserve_proto_field_names = true;
  for (const auto& label : camera_image.label_snapshot().labels()) {
    if (label.has_decal_bounding_box_2ds()) {
      google::protobuf::util::MessageToJsonString(label.decal_bounding_box_2ds(), &decal_json,
                                                  options);
    }
  }

  // Write JSON to disk
  const std::string err_msg = ::applied::utils::WriteToFile(output_path, decal_json);
  if (!err_msg.empty()) {
    return "error wrting json for decal labels: " + err_msg;
  }

  return "";
}

namespace impl {
nonstd::optional<size_t> GetDepthByteDepth(
    const ::applied::spectral::DataPointer& data_pointer,
    const ::simian_public::sensor_model::SensorOutput::CameraImage& camera_image) {
  if (!camera_image.has_metadata()) {
    return {};
  }
  const auto& metadata = camera_image.metadata();
  if (!metadata.has_sensor_description()) {
    return {};
  }
  const auto& sensor_description = metadata.sensor_description();
  if (!sensor_description.has_camera_model()) {
    return {};
  }
  const auto& camera_model = sensor_description.camera_model();
  if (!camera_model.has_standard_params()) {
    return {};
  }
  const auto& standard_params = camera_model.standard_params();
  if (!standard_params.has_system_params()) {
    return {};
  }
  const auto& system_params = standard_params.system_params();
  if (!system_params.has_depth_params()) {
    return {};
  }
  const auto& depth_params = system_params.depth_params();

  const size_t bit_depth = depth_params.bit_depth();
  const size_t byte_depth = (bit_depth + 7) / 8;
  const size_t expected_size = camera_image.image_shape().height() *
                               camera_image.image_shape().width() *
                               camera_image.image_shape().channels() * byte_depth;

  if (data_pointer.size != expected_size) {
    return {};
  }

  return byte_depth;
}
}  // namespace impl

}  // namespace applied
