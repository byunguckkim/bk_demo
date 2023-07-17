#include "applied/simian/public/simulink/simulink_global_proto/global_simulink_proto.h"

namespace applied {

void GlobalSimulinkProto::InitializeGlobalSimulinkProto(
    simian_public::simulink::ADPInput& adp_input, simian_public::simulink::ADPOutput& adp_output,
    CustomerSimulinkData& customer_simulink_data) {
  adp_input_ptr = &adp_input;
  adp_output_ptr = &adp_output;
  customer_simulink_data_ptr = &customer_simulink_data;
  vehiclesim_variables.clear();
}

void GlobalSimulinkProto::DestroyGlobalSimulinkProto() {
  adp_input_ptr = nullptr;
  adp_output_ptr = nullptr;
  customer_simulink_data_ptr = nullptr;
  vehiclesim_variables.clear();
}

GlobalSimulinkProto* simulink_global_proto_ptr = nullptr;

GlobalSimulinkProto* GetGlobalSimulinkProtoPtr() { return simulink_global_proto_ptr; }

void SetGlobalSimulinkProtoPtr(GlobalSimulinkProto& global_simulink_proto) {
  simulink_global_proto_ptr = &global_simulink_proto;
}

void ClearGlobalSimulinkProtoPtr(GlobalSimulinkProto& global_simulink_proto) {
  global_simulink_proto.DestroyGlobalSimulinkProto();
  simulink_global_proto_ptr = nullptr;
}

}  // namespace applied
