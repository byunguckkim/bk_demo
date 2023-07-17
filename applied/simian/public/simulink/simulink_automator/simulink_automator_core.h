#pragma once

/*
 * Helix Core
 *
 *
 */

#include <google/protobuf/util/message_differencer.h>
#include <grpcpp/grpcpp.h>
#include <math.h>
#include <map>
#include <string>
#include <unordered_set>

#include "applied/simian/public/proto/common.pb.h"
#include "applied/simian/public/proto/simulink_automator_service.grpc.pb.h"
#include "applied/simian/public/proto/simulink_automator_service.pb.h"

struct AutomaticMatlabConfigOptions {
  std::string matlab_root;
  std::string model_path;
  std::vector<std::string> model_dependencies;
  std::vector<std::string> model_init_scripts;
  std::vector<std::string> model_post_init_scripts;
  std::vector<std::string> matlab_paths;
};

class SimulinkAutomatorCore {
 public:
  void InitializeCore(std::string& automator_ip, int automator_port,
                      AutomaticMatlabConfigOptions& options);
  void StartSimulinkModel();
  void FinalizeCore();
  void CheckHeartBeat();
  void CreateStub(std::string& automator_ip, int automator_port);

 private:
  std::unique_ptr<simulink_automator::SimulinkAutomator::Stub> simulink_automator_stub_;
};
