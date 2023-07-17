#pragma once

#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/scenario/simulink_config.pb.h"
#include "applied/simian/public/proto/simulink.pb.h"
#include "applied/simian/public/simulink/simulink_automator/simulink_automator_core.h"
#include "applied/simian/public/simulink/zmq_services.h"

namespace applied {

class SimulinkManager {
 public:
  explicit SimulinkManager(
      const simian_public::proto::scenario::simulink_config::SimulinkConfig& simulink_config);

  simian_public::common::CommonResponse Init();

  simian_public::common::CommonResponse SendToSimulink(
      const simian_public::simulink::ADPOutput& adp_output);

  simian_public::common::CommonResponse ReceiveFromSimulink(
      simian_public::simulink::ADPInput& out_adp_input);

  simian_public::common::CommonResponse Terminate();

 private:
  // Simulink config management
  const simian_public::proto::scenario::simulink_config::SimulinkConfig simulink_config_;

  // Simulink state management
  std::unique_ptr<zmq_services::ZmqClient> client_endpoint_;

  std::unique_ptr<SimulinkAutomatorCore> simulink_automator_;

  AutomaticMatlabConfigOptions automator_options_;
  bool stopped_by_simulink_ = false;
  bool simulink_manager_did_fail_ = false;
};
}  // namespace applied
