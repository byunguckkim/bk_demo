#include "applied/simian/public/fmu/fmu_manager.h"

#include <math.h>
#include <fstream>
#include <iostream>

#include "fmi4cpp/fmi2/xml/scalar_variable.hpp"
#include "google/protobuf/util/json_util.h"

#include "applied/simian/public/proto_def.h"

namespace {

FMUManager::FieldType FmuTypeNameToFieldType(const std::string& type_name) {
  if (type_name == fmi4cpp::fmi2::REAL_TYPE) {
    return FMUManager::FieldType::REAL_TYPE;
  } else {
    return FMUManager::FieldType::UNKNOWN_TYPE;
  }
}

}  // namespace

FMUManager::FMUManager(const std::string& config_path, const std::string& fmu_path)
    : kFmuConfigPath_(config_path), kFmuPath_(fmu_path) {}

// Load the FMU config and initialize the FMU.
simian_public::common::CommonResponse FMUManager::Init() {
  simian_public::common::CommonResponse response;
  std::ifstream fmu_config_in(kFmuConfigPath_);
  if (!fmu_config_in.is_open()) {
    response.mutable_exception()->set_exception_message("Error while opening FMU config. Path: " +
                                                        kFmuConfigPath_);
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  std::stringstream buffer;
  buffer << fmu_config_in.rdbuf();
  fmu_config_in.close();

  const google::protobuf::util::JsonParseOptions opt;
  const auto status = google::protobuf::util::JsonStringToMessage(buffer.str(), &fmu_config_);
  if (!status.ok()) {
    response.mutable_exception()->set_exception_message("Could not parse FMU config JSON: " +
                                                        get_protobuf_status_message(status));
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  fmi4cpp::fmi2::fmu fmu(kFmuPath_);

  if (!fmu.supports_cs()) {
    response.mutable_exception()->set_exception_message("FMU does not support cosimulation!");
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  cs_fmu_ = fmu.as_cs_fmu();

  // Smart pointer to a cs_model_description instance
  cs_md_ = cs_fmu_->get_model_description();

  const auto add_var_refs = [this](const simian_public::fmu::FMUConfig::ChannelMapping& channel) {
    simian_public::common::CommonResponse result;
    // Iterate through proto fields in each channel
    for (const auto& field_mapping : channel.fields()) {
      simian_public::common::CommonResponse curr_result =
          AddVariableRefToMap(field_mapping.fmu_field_name());
      if (curr_result.status() != simian_public::common::CommonResponse::SUCCESS) {
        return curr_result;
      }
    }
    result.set_status(simian_public::common::CommonResponse::SUCCESS);
    return result;
  };

  // Iterate through input and output channel sets
  for (const auto& in_channel : fmu_config_.from_adp_channels()) {
    simian_public::common::CommonResponse result = add_var_refs(in_channel);
    if (result.status() != simian_public::common::CommonResponse::SUCCESS) {
      return result;
    }
  }
  for (const auto& out_channel : fmu_config_.to_adp_channels()) {
    simian_public::common::CommonResponse result = add_var_refs(out_channel);
    if (result.status() != simian_public::common::CommonResponse::SUCCESS) {
      return result;
    }
  }

  // Loads the FMU libraries, must be Linux-compatible
  cs_slave_ = cs_fmu_->new_instance();

  // Starts the FMU processes
  cs_slave_->setup_experiment();
  cs_slave_->enter_initialization_mode();
  cs_slave_->exit_initialization_mode();

  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}

simian_public::common::CommonResponse FMUManager::AddVariableRefToMap(
    const std::string& fmu_field_name) {
  simian_public::common::CommonResponse response;

  // Ensure FMU field name is not a duplicate.
  if (fmu_vr_.find(fmu_field_name) != fmu_vr_.end()) {
    response.mutable_exception()->set_exception_message("Duplicate field name: " + fmu_field_name);
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  // Ensure FMU signal name exists.
  try {
    const fmi4cpp::fmi2::scalar_variable& fmu_var = cs_md_->get_variable_by_name(fmu_field_name);

    // Ensure field type is supported.
    const FieldType field_type = FmuTypeNameToFieldType(fmu_var.type_name());
    switch (field_type) {
      case FieldType::REAL_TYPE:
        fmu_vr_[fmu_field_name] = {fmu_var.value_reference, field_type};
        break;
      default:
        response.mutable_exception()->set_exception_message("Field type not supported for field " +
                                                            fmu_field_name);
        response.set_status(simian_public::common::CommonResponse::EXCEPTION);
        return response;
    }
    response.set_status(simian_public::common::CommonResponse::SUCCESS);
    return response;
  } catch (const std::runtime_error& error) {
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    response.mutable_exception()->set_exception_message("No such variable with name \'" +
                                                        fmu_field_name + "\'");
    return response;
  }
}

simian_public::common::CommonResponse FMUManager::ConvertFromADP(
    const std::string& channel_name, const google::protobuf::Message& message) {
  simian_public::common::CommonResponse response;
  // Serialize proto to json to make field values accessible by JSON pointer
  std::string message_json_str;
  const auto status = google::protobuf::util::MessageToJsonString(message, &message_json_str);
  if (!status.ok()) {
    response.mutable_exception()->set_exception_message("Conversion to JSON failed: " +
                                                        get_protobuf_status_message(status));
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }
  nlohmann::json message_json = nlohmann::json::parse(message_json_str);

  // Find the correct input channel field mapping
  for (const auto& channel : fmu_config_.from_adp_channels()) {
    if (channel_name != channel.channel_name()) {
      continue;
    }
    for (const auto& field : channel.fields()) {
      const std::string& json_ptr = field.adp_field_name();
      const std::string& fmu_field_name = field.fmu_field_name();
      // Use JSON pointer to get value from JSON-ized proto
      nlohmann::json_pointer<nlohmann::json> val_pointer(json_ptr);

      // Write value to FMU input
      bool input_valid = false;
      if (message_json.contains(val_pointer)) {
        input_valid = true;
      } else {
        // If no value exists, then write proto default 0 to FMU input
        std::cerr << "Error: Could not find value at " << val_pointer << ", writing 0 instead."
                  << std::endl;
      }

      const FieldType field_type = fmu_vr_[fmu_field_name].second;
      switch (field_type) {
        case FieldType::REAL_TYPE: {
          bool result = false;
          if (input_valid) {
            result =
                cs_slave_->write_real(fmu_vr_[fmu_field_name].first, message_json[val_pointer]);
          } else {
            result = cs_slave_->write_real(fmu_vr_[fmu_field_name].first, 0);
          }
          if (!result) {
            response.mutable_exception()->set_exception_message(
                "write_real failed for FMU field: " + fmu_field_name);
            response.set_status(simian_public::common::CommonResponse::EXCEPTION);
            return response;
          }
          break;
        }
        default:
          response.mutable_exception()->set_exception_message(
              "Field type not supported for field: " + fmu_field_name);
          response.set_status(simian_public::common::CommonResponse::EXCEPTION);
          return response;
      }
    }
  }

  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}

simian_public::common::CommonResponse FMUManager::ConvertToADP(const std::string& channel_name,
                                                               google::protobuf::Message* message) {
  simian_public::common::CommonResponse response;
  if (message == nullptr) {
    response.mutable_exception()->set_exception_message("message is NULL");
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  // Find correct output channel field mapping
  for (const auto& channel : fmu_config_.to_adp_channels()) {
    if (channel_name != channel.channel_name()) {
      continue;
    }
    nlohmann::json output_json;
    for (const auto& field : channel.fields()) {
      const std::string& fmu_field_name = field.fmu_field_name();
      const FieldType field_type = fmu_vr_[fmu_field_name].second;
      switch (field_type) {
        case FieldType::REAL_TYPE: {
          double output_val;
          bool result = cs_slave_->read_real(fmu_vr_[fmu_field_name].first, output_val);
          if (!result) {
            response.mutable_exception()->set_exception_message("read_real failed for FMU field: " +
                                                                fmu_field_name);
            response.set_status(simian_public::common::CommonResponse::EXCEPTION);
            return response;
          }
          output_json[field.adp_field_name()] = output_val;
          break;
        }
        default:
          response.mutable_exception()->set_exception_message(
              "Field type not supported for field " + fmu_field_name);
          response.set_status(simian_public::common::CommonResponse::EXCEPTION);
          return response;
      }
    }
    // Unflatten JSON with output signals to get parseable JSON->proto
    nlohmann::json full_out = output_json.unflatten();
    const auto status = google::protobuf::util::JsonStringToMessage(full_out.dump(), message);
    if (!status.ok()) {
      response.mutable_exception()->set_exception_message("JSON parsing failed: " +
                                                          get_protobuf_status_message(status));
      response.set_status(simian_public::common::CommonResponse::EXCEPTION);
      return response;
    }
  }

  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}

simian_public::common::CommonResponse FMUManager::Step(const double step_size) {
  simian_public::common::CommonResponse response;
  // Return an error if FMU step forward fails
  if (!cs_slave_->step(step_size)) {
    response.mutable_exception()->set_exception_message("Error! step() returned with status: " +
                                                        to_string(cs_slave_->last_status()));
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }

  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}

simian_public::common::CommonResponse FMUManager::WriteRealValue(const std::string& fmu_field_name,
                                                                 const double value) {
  simian_public::common::CommonResponse response;
  int vr;
  try {
    vr = cs_md_->get_variable_by_name(fmu_field_name).value_reference;
  } catch (const std::runtime_error& error) {
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    response.mutable_exception()->set_exception_message("No such variable with name \'" +
                                                        fmu_field_name + "\'");
    return response;
  }

  const bool result = cs_slave_->write_real(vr, value);
  if (!result) {
    response.mutable_exception()->set_exception_message("write_real failed for FMU field: " +
                                                        fmu_field_name);
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }
  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}

simian_public::common::CommonResponse FMUManager::ReadRealValue(const std::string& fmu_field_name,
                                                                double* output_val) {
  simian_public::common::CommonResponse response;
  int vr;
  try {
    vr = cs_md_->get_variable_by_name(fmu_field_name).value_reference;
  } catch (const std::runtime_error& error) {
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    response.mutable_exception()->set_exception_message("No such variable with name \'" +
                                                        fmu_field_name + "\'");
    return response;
  }

  const bool result = cs_slave_->read_real(vr, *output_val);
  if (!result) {
    response.mutable_exception()->set_exception_message("read_real failed for FMU field: " +
                                                        fmu_field_name);
    response.set_status(simian_public::common::CommonResponse::EXCEPTION);
    return response;
  }
  response.set_status(simian_public::common::CommonResponse::SUCCESS);
  return response;
}
