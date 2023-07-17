// Copyright (C) 2022 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include <algorithm>
#include <cmath>
#include <functional>
#include <iomanip>
#include <sstream>
#include <string>

#include "applied/spectral/public/ndarray_helpers.h"

#include "applied/spectral/public/spectral_shared_memory_helper.h"

using namespace simian_public::sensor_output;

namespace applied {

size_t NDArrayHelper::GetDataTypeSize(const DataType::Enum data_type) {
  switch (data_type) {
    case DataType::INT8:
    case DataType::UINT8:
      return 1;
    case DataType::INT16:
    case DataType::UINT16:
      return 2;
    case DataType::INT32:
    case DataType::UINT32:
    case DataType::FLOAT32:
      return 4;
    case DataType::FLOAT64:
      return 8;
    case DataType::NOT_SET:
    default:
      return 0;
  }
}

NDArrayHelper::NDArrayHelper(const NDArray& ndarray)
    : ndarray_{ndarray}, num_elements_{}, data_ptr_{nullptr}, data_size_{0} {
  // Resolve shared memory pointer if necessary
  if (ndarray_.sensor_data_output().has_shmem_pointer()) {
    const auto data_pointer = spectral::SMHelperSingleton::GetInstance().ReadDataPointer(
        ndarray_.sensor_data_output().shmem_pointer());
    data_ptr_ = data_pointer.memory_address;
    data_size_ = data_pointer.size;
  } else {
    data_ptr_ =
        ndarray_.sensor_data_output().data().data();  // First data for string in SensorDataOutput,
                                                      // second for char* inside std::string.
    data_size_ = ndarray_.sensor_data_output().data().size();
  }

  num_elements_ = static_cast<size_t>(data_size_ / ndarray_.element_stride());

  shape_ = std::vector<size_t>(ndarray_.shape().begin(), ndarray_.shape().end());

  sorted_fields_ = std::vector<NDArray::Field>(ndarray_.fields().begin(), ndarray_.fields().end());
  std::sort(
      sorted_fields_.begin(), sorted_fields_.end(),
      [](const NDArray_Field& f1, const NDArray_Field& f2) { return f1.offset() < f2.offset(); });

  // Convert the list of fields to a map for faster look-up
  for (const auto& field : sorted_fields_) {
    fields_map_[field.field_type()] = field;
  }

  // For pretty formatting, find the length of longest field name so we can set the field name
  // columns length appropriately.
  auto it_longest_named_field =
      std::max_element(sorted_fields_.begin(), sorted_fields_.end(),
                       [](const NDArray::Field& f1, const NDArray::Field& f2) {
                         return NDArray_Field_FieldType_Name(f1.field_type()).size() <
                                NDArray_Field_FieldType_Name(f2.field_type()).size();
                       });
  max_field_name_length_ =
      NDArray_Field_FieldType_Name(it_longest_named_field->field_type()).size();
}

size_t NDArrayHelper::GetNumElements() const { return num_elements_; }

size_t NDArrayHelper::GetElementSize() const { return ndarray_.element_stride(); }

std::vector<size_t> NDArrayHelper::GetShape() const { return shape_; }

std::unordered_map<NDArray::Field::FieldType, NDArray::Field> NDArrayHelper::GetFields() const {
  return fields_map_;
}

std::vector<NDArray::Field> NDArrayHelper::GetSortedFields() const { return sorted_fields_; }

std::string NDArrayHelper::GetField(const NDArray::Field::FieldType field_type,
                                    NDArray::Field& out_field) const {
  if (!HasFieldType(field_type)) {
    return "Field \'" + NDArray_Field_FieldType_Name(field_type) +
           "\' is not present in this NDArray.";
  }

  out_field = fields_map_.at(field_type);
  return "";
}

bool NDArrayHelper::HasField(const NDArray::Field& field) const {
  NDArray::Field this_field;
  std::string result = GetField(field.field_type(), this_field);
  if (!result.empty()) {
    return false;
  }
  return (this_field.offset() == field.offset() && this_field.data_type() == field.data_type());
}

std::string NDArrayHelper::HasFields(std::vector<NDArray::Field> fields) const {
  if (fields.empty()) {
    return "";
  }

  for (const auto& field : fields) {
    if (!HasField(field)) {
      return "Field (\'" + NDArray_Field_FieldType_Name(field.field_type()) +
             "\', offset=" + std::to_string(field.offset()) + ", " +
             DataType_Enum_Name(field.data_type()) +
             ") does not match any fields present in this NDArray.";
    }
  }
  return "";
}

bool NDArrayHelper::HasFieldType(const NDArray::Field::FieldType field_type) const {
  return (fields_map_.find(field_type) != fields_map_.end());
}

std::string NDArrayHelper::HasFieldTypes(std::vector<NDArray::Field::FieldType> field_types) const {
  if (field_types.empty()) {
    return "";
  }

  for (const auto& field_type : field_types) {
    if (!HasFieldType(field_type)) {
      return "Field \'" + NDArray_Field_FieldType_Name(field_type) +
             "\' is not present in this NDArray.";
    }
  }
  return "";
}

std::string NDArrayHelper::PrettyFormattedString() const {
  std::stringstream ss;

  // Print the keys using the sorted vector in a C++ struct format
  ss << "struct NDArrayElement { " << std::endl;
  for (const NDArray::Field& field : sorted_fields_) {
    // Data Type
    ss << "  ";
    ss << std::left << std::setw(8) << DataType_Enum_Name(field.data_type());

    // Field name
    ss << std::left << std::setw(max_field_name_length_ + 1)
       << NDArray_Field_FieldType_Name(field.field_type()) + ";";

    // Byte offset comment
    ss << " // Byte offset " << field.offset() << std::endl;
  }
  ss << "}; " << std::endl;

  return ss.str();
}

std::string NDArrayHelper::ForEachElement(std::function<std::string(const NDArrayElement&)> func) {
  size_t byte_offset = 0;
  for (size_t i = 0; i < this->GetNumElements(); i++) {
    NDArrayElement element(data_ptr_ + byte_offset, *this);
    byte_offset += this->GetElementSize();
    std::string err_msg = func(element);
    if (!err_msg.empty()) {
      return err_msg;
    }
  }
  return "";
}

NDArrayElement::NDArrayElement(const char* data_ptr, const NDArrayHelper& ndarray_helper)
    : data_ptr_{data_ptr}, ndarray_helper_{ndarray_helper} {}

const char* NDArrayElement::Data() const { return data_ptr_; }

size_t NDArrayElement::Size() const { return ndarray_helper_.GetElementSize(); }

bool NDArrayElement::HasField(const NDArray::Field& field) const {
  return ndarray_helper_.HasField(field);
}

std::string NDArrayElement::HasFields(std::vector<NDArray::Field> fields) const {
  return ndarray_helper_.HasFields(fields);
}

bool NDArrayElement::HasFieldType(const NDArray::Field::FieldType field_type) const {
  return ndarray_helper_.HasFieldType(field_type);
}

std::string NDArrayElement::HasFieldTypes(
    std::vector<NDArray::Field::FieldType> field_types) const {
  return ndarray_helper_.HasFieldTypes(field_types);
}

namespace {
std::string NDArrayFieldDataToString(const void* data,
                                     simian_public::sensor_output::DataType::Enum data_type) {
  std::stringstream ss;
  switch (data_type) {
    case DataType::INT8:
      ss << (static_cast<int>(*(reinterpret_cast<const int8_t*>(data))));
      break;
    case DataType::UINT8:
      ss << (static_cast<unsigned int>(*(reinterpret_cast<const uint8_t*>(data))));
      break;
    case DataType::INT16:
      ss << (*(reinterpret_cast<const int16_t*>(data)));
      break;
    case DataType::UINT16:
      ss << (*(reinterpret_cast<const uint16_t*>(data)));
      break;
    case DataType::INT32:
      ss << (*(reinterpret_cast<const int32_t*>(data)));
      break;
    case DataType::UINT32:
      ss << (*(reinterpret_cast<const uint32_t*>(data)));
      break;
    case DataType::FLOAT32:
      ss << (*(reinterpret_cast<const float*>(data)));
      break;
    case DataType::FLOAT64:
      ss << (*(reinterpret_cast<const double*>(data)));
      break;
    case DataType::NOT_SET:
    default:
      return "<Unsupported Data Type>";
  }
  return ss.str();
}
}  // namespace

std::string NDArrayElement::PrettyFormattedString() const {
  std::stringstream ss;

  constexpr size_t kPrecision = 10;
  size_t width = std::max(ndarray_helper_.max_field_name_length_ + 1, kPrecision);

  for (const NDArray::Field& field : ndarray_helper_.sorted_fields_) {
    ss << "  ";

    // Field name
    ss << std::left << std::setw(width)
       << simian_public::sensor_output::NDArray_Field_FieldType_Name(field.field_type());

    // Field Value
    ss << std::setw(width) << std::setprecision(kPrecision)
       << NDArrayFieldDataToString(data_ptr_ + field.offset(), field.data_type());

    // Field Data Type
    ss << std::right << std::setw(width) << "(" << DataType_Enum_Name(field.data_type()) << ")";

    ss << std::endl;
  }

  return ss.str();
}

std::pair<std::unique_ptr<NDArrayRepackedBuffer>, std::string> NDArrayRepackedBuffer::Make(
    const NDArray& ndarray, const std::vector<NDArray::Field::FieldType>& field_types) {
  NDArrayHelper helper(ndarray);
  std::string has_field_types = helper.HasFieldTypes(field_types);
  if (!has_field_types.empty()) {
    return {nullptr, "NDArrayRepackedBuffer construction failed: " + has_field_types};
  }
  std::vector<size_t> offsets;
  std::vector<size_t> sizes;
  size_t total_size = 0;
  for (const auto& field_type : field_types) {
    NDArray::Field field;
    helper.GetField(field_type, field);
    offsets.push_back(field.offset());
    sizes.push_back(helper.GetDataTypeSize(field.data_type()));
    total_size += sizes.back();
  }
  if (offsets.size() != field_types.size() || sizes.size() != field_types.size()) {
    return {nullptr, "Internal error in NDArrayRepackedBuffer construction"};
  }

  return {std::unique_ptr<NDArrayRepackedBuffer>(new NDArrayRepackedBuffer(
              field_types.size(), offsets, sizes, helper.GetElementSize(),
              helper.GetNumElements() * helper.GetElementSize(), helper.Data(), total_size)),
          ""};
}

NDArrayRepackedBuffer::NDArrayRepackedBuffer(size_t num_fields, const std::vector<size_t>& offsets,
                                             const std::vector<size_t>& sizes, size_t stride,
                                             size_t data_size, const char* data_ptr,
                                             size_t total_size)
    : num_elements_(data_size / stride), packed_stride_(total_size) {
  packed_data_ = std::make_unique<char[]>(num_elements_ * packed_stride_);
  char* packed_data_ptr = packed_data_.get();
  for (size_t i = 0; i < num_elements_; i++) {
    const char* element_start = data_ptr + i * stride;
    for (size_t j = 0; j < offsets.size(); j++) {
      size_t cur_size = sizes[j];
      memcpy(packed_data_ptr, element_start + offsets[j], cur_size);
      packed_data_ptr += cur_size;
    }
  }
}

std::string NDArrayRepackedBuffer::ForEachElement(
    std::function<std::string(const char*)> func) const {
  for (size_t i = 0; i < num_elements_; i++) {
    // Pass packed fields to user function.
    auto result = func(packed_data_.get() + i * packed_stride_);
    if (!result.empty()) return result;
  }
  return "";
}

}  // namespace applied
