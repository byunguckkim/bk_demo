#pragma once

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <unordered_map>

// RGB is a macro when using MSVC. We use a conflicting proto message named RGB
// from common.proto so we need to undef the macro for Windows compatibility.
#if !defined(__linux__) && defined(RGB)
#undef RGB
#endif
#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/fmu_config.pb.h"

#include "fmi4cpp/fmi4cpp.hpp"
#include "google/protobuf/util/json_util.h"
#include "nlohmann/json.hpp"

using simian_public::common::CommonResponse;

class FMUManager {
 public:
  enum class FieldType {
    REAL_TYPE,
    UNKNOWN_TYPE,
  };
  explicit FMUManager(const std::string& config_path, const std::string& fmu_path);

  CommonResponse Init();

  CommonResponse ConvertFromADP(const std::string& channel_name,
                                const google::protobuf::Message& message);

  CommonResponse ConvertToADP(const std::string& channel_name, google::protobuf::Message* message);

  CommonResponse Step(double step_size);

  // Precondition: fmu_field_name should exist in the model description and have type REAL
  CommonResponse WriteRealValue(const std::string& fmu_field_name, const double value);

  // Precondition: fmu_field_name should exist in the model description and have type REAL
  CommonResponse ReadRealValue(const std::string& fmu_field_name, double* output_val);

 private:
  CommonResponse AddVariableRefToMap(const std::string& fmu_field_name);

  // FMU config management
  const std::string kFmuConfigPath_;
  simian_public::fmu::FMUConfig fmu_config_;

  // FMU state management
  const std::string kFmuPath_;
  std::unique_ptr<fmi4cpp::fmi2::cs_fmu> cs_fmu_;
  std::shared_ptr<const fmi4cpp::fmi2::cs_model_description> cs_md_;
  std::unique_ptr<fmi4cpp::fmi2::cs_slave, std::default_delete<fmi4cpp::fmi2::cs_slave>> cs_slave_;
  // Keep a mapping of the FMU field names to variable references and field types
  std::unordered_map<std::string, std::pair<int, FieldType>> fmu_vr_;
};
