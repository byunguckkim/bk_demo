#pragma once

#include <variant>
#include "applied/simian/public/api_def.h"
#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/simulink.pb.h"

namespace applied {

class CustomerSimulinkData {
  // Populate any fields that you wish to send to Simulink.
};

class GlobalSimulinkProto {
  simian_public::simulink::ADPInput* adp_input_ptr = nullptr;
  simian_public::simulink::ADPOutput* adp_output_ptr = nullptr;

  CustomerSimulinkData* customer_simulink_data_ptr = nullptr;

  std::unordered_map<std::string, std::variant<double*, int*>> vehiclesim_variables;

 public:
  simian_public::simulink::ADPInput* GetADPInputPtr() { return adp_input_ptr; }
  simian_public::simulink::ADPOutput* GetADPOutputPtr() { return adp_output_ptr; }

  CustomerSimulinkData* GetCustomerSimulinkDataPtr() { return customer_simulink_data_ptr; }

  std::unordered_map<std::string, std::variant<double*, int*>>& GetVehicleSimVariables() {
    return vehiclesim_variables;
  }

  void InitializeGlobalSimulinkProto(simian_public::simulink::ADPInput& adp_input,
                                     simian_public::simulink::ADPOutput& adp_output,
                                     CustomerSimulinkData& customer_simulink_data);

  void DestroyGlobalSimulinkProto();
};

GlobalSimulinkProto* GetGlobalSimulinkProtoPtr();
void SetGlobalSimulinkProtoPtr(GlobalSimulinkProto& global_simulink_proto);
void ClearGlobalSimulinkProtoPtr(GlobalSimulinkProto& global_simulink_proto);

}  // namespace applied
