// Copyright (C) 2022 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#include <cstring>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

#include "applied/simian/public/proto/sensor_output.pb.h"

namespace applied {

class NDArrayElement;  // Forward Declaration

/**
 * @brief Helper class to access and decode entries inside an existing NDArray message
 *        from Spectral.
 */
class NDArrayHelper {
 public:
  using NDArray = simian_public::sensor_output::NDArray;

  using DataType = simian_public::sensor_output::DataType;

  /**
   * @brief Returns the size, in bytes, of the argument data type.
   */
  static size_t GetDataTypeSize(const DataType::Enum data_type);

  /**
   * @brief Returns the field value as requested type.
   *        Returns zero initialized OutType if invalid Enum.
   */
  template <typename OutType>
  static OutType GetDataTypeAs(const DataType::Enum data_type, const char* field_ptr) {
    switch (data_type) {
      case simian_public::sensor_output::DataType::INT8:
        return static_cast<OutType>(*reinterpret_cast<const int8_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::UINT8:
        return static_cast<OutType>(*reinterpret_cast<const uint8_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::INT16:
        return static_cast<OutType>(*reinterpret_cast<const int16_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::UINT16:
        return static_cast<OutType>(*reinterpret_cast<const uint16_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::INT32:
        return static_cast<OutType>(*reinterpret_cast<const int32_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::UINT32:
        return static_cast<OutType>(*reinterpret_cast<const uint32_t*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::FLOAT32:
        return static_cast<OutType>(*reinterpret_cast<const float*>(field_ptr));
        break;
      case simian_public::sensor_output::DataType::FLOAT64:
        return static_cast<OutType>(*reinterpret_cast<const double*>(field_ptr));
        break;
      default:
        return OutType{};
        break;
    }
  }

  /**
   * @param ndarray the NDArray proto message from to parse.
   */
  NDArrayHelper(const NDArray& ndarray);

  /**
   * @brief Returns the total number of elements inside the NDArray.
   */
  size_t GetNumElements() const;

  /**
   * @brief Returns the total number of bytes for a single element in the NDArray.
   */
  size_t GetElementSize() const;

  /**
   * @brief Returns the size of each dimension of the NDArray.
   */
  std::vector<size_t> GetShape() const;

  /**
   * @brief Returns pointer to the source data buffer.
   */
  const char* Data() const { return data_ptr_; }

  /**
   * @brief Returns a mapping between FieldTypes and the Fields present inside each
   *        element of this NDArray.
   */
  std::unordered_map<NDArray::Field::FieldType, NDArray::Field> GetFields() const;

  /**
   * @brief Returns a list of Fields which are present inside an element of this
   *        this NDArray, sorted in ascending byte offset order.
   */
  std::vector<NDArray::Field> GetSortedFields() const;

  /**
   * @brief Outputs the NDArray::Field message for a given field type inside this NDArray.
   *
   * @param field_type The field type to look-up.
   * @param out_field The NDArray::Field message to write to.
   *
   * @returns Empty string on success, non-empty string on failure with a failure message.
   */
  std::string GetField(const NDArray::Field::FieldType field_type, NDArray::Field& out_field) const;

  /**
   * @brief Returns true if the argument field is present and all members match the corresponding
   * field inside the NDArray element, false otherwise.
   */
  bool HasField(const NDArray::Field& field) const;

  /**
   * @brief Checks if each field is present in the NDArray, and all field members match the
   * corresponding field inside the NDArray element.
   *
   * @param fields The fields to compare against.
   *
   * @return Empty string if all fields are present, non-empty string containing a message with the
   * first field that is not found or does not match.
   */
  std::string HasFields(std::vector<NDArray::Field> fields) const;

  /**
   * @brief Returns true if the argument field type is present inside the NDArray element, false
   * otherwise.
   */
  bool HasFieldType(const NDArray::Field::FieldType field_type) const;

  /**
   * @brief Checks if each field type in the given list is present in the NDArray element.
   *
   * @param field_types The field types to look-up.
   *
   * @return Empty string if all fields are present, non-empty string containing a message with the
   * first field type not found.
   */
  std::string HasFieldTypes(std::vector<NDArray::Field::FieldType> field_types) const;

  /**
   * @brief Sequentially executes a function on each element inside the NDarray.
   *
   * @param func A function to invoke on an NDArrayElement which returns an empty std::string
   *             on success, and a non-empty string on failure.
   *
   * @returns Empty string on success, non-empty string on failure with a failure message.
   *
   * @example auto func = [&](const applied::NDArrayElement& element) {
   *              //...
   *              // Do work on element
   *              //...
   *              return "";
   *          }
   *          NDarray ndarray;
   *          applied::NDArrayHelper helper(ndarray)
   *          std::string err_msg = helper.ForEachElement(func);
   */
  std::string ForEachElement(std::function<std::string(const NDArrayElement&)> func);

  /**
   * @brief Generates a human readable string showing the NDArrayElement format as C++ struct
   *
   * @example std::cout << ndarray_helper.PrettyFormattedString() << std::endl:
   *          example output:
   *            struct NDArrayElement {
   *               UINT8    x;          // Byte offset 0
   *               UINT16   y;          // Byte offset 1
   *               UINT32   z;          // Byte offset 3
   *               FLOAT32  intensity;  // Byte offset 7
   *            }
   */
  std::string PrettyFormattedString() const;

 private:
  friend class NDArrayElement;

  const NDArray& ndarray_;

  // Total number of entries/elements inside this NDArray.
  size_t num_elements_;

  // Shape (each element represents a dimension, value of element is size of that dimension).
  std::vector<size_t> shape_;

  // Pointer to packed byte array of NDArray data.
  const char* data_ptr_;
  size_t data_size_;

  // Sorted list of fields in ascending byte offset order.
  std::vector<NDArray::Field> sorted_fields_;

  // Mapping between NDArray field type and field info for this NDArray.
  std::unordered_map<NDArray::Field::FieldType, NDArray::Field> fields_map_;

  // Length of the longest field name.
  size_t max_field_name_length_;
};

/**
 * @brief Represents a single element/element inside an NDArray.
 * @note See NDArrayHelper for interacting with NDArrayElement instances.
 */
class NDArrayElement {
 public:
  using NDArray = simian_public::sensor_output::NDArray;

  /**
   * @note NDArrayEntries should only be created by NDArrayHelper for a given NDArray.
   */
  NDArrayElement() = delete;

  /**
   * @brief Returns a pointer the packed byte data for the NDArray element.
   * @returns const char* to data.
   */
  const char* Data() const;

  /**
   * @brief Returns the size in bytes of the packed byte data for the NDArray element.
   */
  size_t Size() const;

  /**
   * @brief Retrieves the data associated with a specific field.
   *
   * @param field_type The field which to retrieve data for.
   * @param out The output to write the data of the field to.
   *
   * @returns Empty string on success, non-empty string on failure with a failure message.
   */
  template <typename OutputTypeT>
  std::string GetFieldData(const NDArray::Field::FieldType field_type, OutputTypeT& out) const;

  /**
   * @brief Outputs the NDArray::Field message for a given field type inside this NDArray.
   *
   * @param field_type The field type to look-up.
   * @param out_field The NDArray::Field message to write to.
   *
   * @returns Empty string on success, non-empty string on failure with a failure message.
   */
  std::string GetField(const NDArray::Field::FieldType field_type, NDArray::Field& out_field) const;

  /**
   * @brief Returns true if the argument field is present and all members match the corresponding
   * field inside the NDArray element, false otherwise.
   */
  bool HasField(const NDArray::Field& field) const;

  /**
   * @brief Checks if each field is present in the NDArray, and all field members match the
   * corresponding field inside the NDArray element.
   *
   * @param fields The fields to compare against.
   *
   * @return Empty string if all fields are present, non-empty string containing a message with the
   * first field that is not found or does not match.
   */
  std::string HasFields(std::vector<NDArray::Field> fields) const;

  /**
   * @brief Returns true if the argument field type is present inside the NDArray element, false
   * otherwise.
   */
  bool HasFieldType(const NDArray::Field::FieldType field_type) const;

  /**
   * @brief Checks if each field type in the given list is present in the NDArray element.
   *
   * @param field_types The field types to look-up.
   *
   * @return Empty string if all fields are present, non-empty string containing a message with the
   * first field type not found.
   */
  std::string HasFieldTypes(std::vector<NDArray::Field::FieldType> field_types) const;

  /**
   * @brief Returns a human-readable string representation of the fields and values inside this
   * element.
   */
  std::string PrettyFormattedString() const;

  friend std::ostream& operator<<(std::ostream& os, const NDArrayElement& element) {
    return os << element.PrettyFormattedString();
  }

 private:
  friend class NDArrayHelper;

  /**
   * @brief Constructor for NDArrayHelper (and only NDArrayHelper) to create entries.
   *
   * @param data_ptr Pointer to first byte of this element's data.
   * @param ndarray Reference to the NDArrayHelper class owning this element.
   */
  NDArrayElement(const char* data_ptr, const NDArrayHelper& ndarray_helper);

  const char* data_ptr_{nullptr};
  const NDArrayHelper& ndarray_helper_;
};

template <typename OutputTypeT>
std::string NDArrayElement::GetFieldData(const NDArray::Field::FieldType field_type,
                                         OutputTypeT& out) const {
  // Check if this field is present inside this NDArray/NDArrayElement.
  if (!HasFieldType(field_type)) {
    return "Field \'" + NDArray_Field_FieldType_Name(field_type) + " is not present this NDArray";
  }

  NDArray::Field field;
  std::string result = ndarray_helper_.GetField(field_type, field);
  if (!result.empty()) {
    return "Could not retrieve field information for field \'" +
           NDArray_Field_FieldType_Name(field_type) + "\': " + result;
  }

  // Check if the output type is correct for the field data type
  size_t data_type_size = NDArrayHelper::GetDataTypeSize(field.data_type());
  if (sizeof(OutputTypeT) != data_type_size) {
    return "Output type size (" + std::to_string(sizeof(OutputTypeT)) +
           ") does not match Field data type size (" + std::to_string(data_type_size) +
           ") for field data type " +
           simian_public::sensor_output::DataType_Enum_Name(field.data_type());
  }

  std::memcpy(&out, data_ptr_ + field.offset(), data_type_size);
  return "";
}

/**
 * @brief: Helper object to iterate quickly over the elements of an NDArray with a user-defined
 * function requiring particular fields.
 */
class NDArrayRepackedBuffer {
 public:
  using NDArray = simian_public::sensor_output::NDArray;
  static std::pair<std::unique_ptr<NDArrayRepackedBuffer>, std::string> Make(
      const NDArray& ndarray, const std::vector<NDArray::Field::FieldType>& field_types);

  /**
   * @brief Sequentially executes a function on the fields originally specified in the call
   *        to `Make`, for each element of the NDArray.
   *
   * @param func A function to invoke on a packed segment of bytes representing the fields
   *             originally specified in the call to `Make`, in the order in which they were
   *             specified in the call to `Make`.  `func` should return a `std::string` value
   *             which will specify the error if there is one.
   *
   * @return Empty string on success; error message on error.
   *
   * @example
   *   auto make_result = applied::NDArrayRepackedBuffer::Make(
   *                      ndarray, {NDArray::Field::INTENSITY, NDArray::Field::POSITION_Y});
   *   auto& for_each = *make_result.value();
   *
   *    #pragma pack(push, 1)
   *    struct ReducedElement {
   *      float intensity;
   *      uint16_t position_y;
   *    };
   *    #pragma pack(pop)
   *    std::vector<ReducedElement> reduced_elements(ndarray.shape(0));
   *    ReducedElement* p = reduced_elements.data();
   *    auto copy_fields_func = [&p](const char* packed_data) -> applied::MaybeError {
   *      memcpy(p, packed_data, sizeof(ReducedElement));
   *      p++;
   *      return "";
   *    };
   *    std::string foreach_errmsg = for_each.ForEachElement(test_func);
   *
   * This will populate the `reduced_elements` vector with elements with the desired intensity and
   * position_y.
   *
   * Note that the `#pragma pack` stuff above is *necessary* for this example -- without it the
   * bytes might be copied to incorrect locations.
   */
  std::string ForEachElement(std::function<std::string(const char*)> func) const;

  /**
   * @brief Expose the packed data buffer as raw bytes.
   * @return Pointer to the packed data buffer.
   * @example
   *   auto make_result = applied::NDArrayRepackedBuffer::Make(
   *                      ndarray, {NDArray::Field::INTENSITY, NDArray::Field::POSITION_Y});
   *  auto& repacked_buffer = *make_result.first;
   *
   * #pragma pack(push, 1)
   *   struct ReducedElement {
   *     float intensity;
   *     uint16_t position_y;
   *   };
   * #pragma pack(pop)
   *
   *   const ReducedElement* reduced_element_array =
   *       reinterpret_cast<const ReducedElement*>(repacked_buffer.PackedData());
   *
   *   // Now access the third element as reduced_element_array[2]
   */
  const char* PackedData() const { return packed_data_.get(); }

 private:
  NDArrayRepackedBuffer(size_t num_fields, const std::vector<size_t>& offsets,
                        const std::vector<size_t>& sizes, size_t stride, size_t data_size,
                        const char* data_ptr, size_t total_size);
  // Number of NDArrayElement objects in data.
  const size_t num_elements_;
  // Total bytes in a single NDArrayElement, restricted to the requested field_types.
  const size_t packed_stride_;
  // Underlying data, restricted to the fields requested.
  std::unique_ptr<char[]> packed_data_;
};

}  // namespace applied
