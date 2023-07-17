/*
 * Function implementations for Helix Core
 */

#include <iostream>
#include <sstream>

#include "simulink_automator_core.h"

#define INITIALIZE_TIMEOUT 240
#define FINALIZE_TIMEOUT 240
#define HEARTBEAT_TIMEOUT 1
#define START_MODEL_TIMEOUT 240

void SimulinkAutomatorCore::FinalizeCore() {
  simulink_automator::FinalizeRequest request;
  simulink_automator::FinalizeResponse response;

  grpc::ClientContext context;
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(FINALIZE_TIMEOUT);
  context.set_deadline(deadline);
  std::cout << "Starting Simulink Model..." << std::endl;
  grpc::Status status = simulink_automator_stub_->Finalize(&context, request, &response);

  if (!status.ok()) {
    if (status.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: Could Not Reset Simulink Model Due to Communication Issue.";
      throw std::runtime_error(ss.str());
    } else {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: " << status.error_message();
      throw std::runtime_error(ss.str());
    }
  }

  std::cout << "START MATLAB STDOUT: " << std::endl;
  std::cout << "--------------------------------------------------------------------------------"
            << std::endl;
  std::cout << response.matlab_output().stdout_content() << std::endl;
  std::cout << "--------------------------------------------------------------------------------"
            << std::endl;
  std::cout << "END MATLAB STDOUT" << std::endl;

  std::cerr << "START MATLAB STDERR: " << std::endl;
  std::cerr << "--------------------------------------------------------------------------------"
            << std::endl;
  std::cerr << response.matlab_output().stderr_content() << std::endl;
  std::cerr << "--------------------------------------------------------------------------------"
            << std::endl;
  std::cerr << "END MATLAB STDERR" << std::endl;
}

void SimulinkAutomatorCore::StartSimulinkModel() {
  simulink_automator::StartSimulinkModelRequest request;
  simulink_automator::StartSimulinkModelResponse response;

  grpc::ClientContext context;
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(START_MODEL_TIMEOUT);
  context.set_deadline(deadline);
  std::cout << "Starting Simulink Model..." << std::endl;
  grpc::Status status = simulink_automator_stub_->StartSimulinkModel(&context, request, &response);

  if (!status.ok()) {
    if (status.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: Could not start Simulink Model due to communication "
            "timeout. "
         << "Has the Simulink Automator service been started?";
      throw std::runtime_error(ss.str());
    } else {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: " << status.error_message();
      throw std::runtime_error(ss.str());
    }
  }

  std::cout << "Successfully Started Simulink Model" << std::endl;
}

void SimulinkAutomatorCore::CreateStub(std::string& automator_ip, int automator_port) {
  std::cout << "Creating channel..." << std::endl;
  grpc::ChannelArguments channel_args = grpc::ChannelArguments();
  channel_args.SetInt(GRPC_ARG_ENABLE_HTTP_PROXY, 0);
  std::shared_ptr<grpc::Channel> channel =
      grpc::CreateCustomChannel(automator_ip + ":" + std::to_string(automator_port),
                                grpc::InsecureChannelCredentials(), channel_args);
  std::cout << "Channel created." << std::endl;
  std::cout << "Creating stub..." << std::endl;
  simulink_automator_stub_ = simulink_automator::SimulinkAutomator::NewStub(channel);
  std::cout << "Stub created." << std::endl;
}

void SimulinkAutomatorCore::CheckHeartBeat() {
  grpc::ClientContext context;
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(HEARTBEAT_TIMEOUT);
  context.set_deadline(deadline);
  std::cout << "Pinging Server" << std::endl;

  simulink_automator::HeartBeatRequest request = simulink_automator::HeartBeatRequest();
  simulink_automator::HeartBeatResponse response = simulink_automator::HeartBeatResponse();
  grpc::Status status = simulink_automator_stub_->HeartBeat(&context, request, &response);

  if (!status.ok()) {
    if (status.error_code() == grpc::StatusCode::DEADLINE_EXCEEDED) {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: Could not initialize Simulink Automator due to "
            "communication timeout. "
         << "Has the Simulink Automator service been started?";
      throw std::runtime_error(ss.str());
    } else {
      std::stringstream ss;
      ss << "SIMULINK AUTOMATOR ERROR: " << status.error_message();
      throw std::runtime_error(ss.str());
    }
  }
}

void SimulinkAutomatorCore::InitializeCore(std::string& automator_ip, int automator_port,
                                           AutomaticMatlabConfigOptions& options) {
  CreateStub(automator_ip, automator_port);
  CheckHeartBeat();

  simulink_automator::InitializeResponse response;

  simulink_automator::InitializeRequest request;
  request.set_matlab_workspace(options.matlab_root);
  request.set_abs_model_path(options.model_path);

  for (std::string& dep : options.model_dependencies) {
    request.add_model_dependencies(dep);
  }

  for (std::string& init_script : options.model_init_scripts) {
    request.add_model_init_scripts(init_script);
  }

  for (std::string& init_script : options.model_post_init_scripts) {
    request.add_model_post_init_scripts(init_script);
  }

  for (std::string& matlab_path : options.matlab_paths) {
    request.add_matlab_paths(matlab_path);
  }

  grpc::ClientContext context;
  std::chrono::system_clock::time_point deadline =
      std::chrono::system_clock::now() + std::chrono::seconds(INITIALIZE_TIMEOUT);
  context.set_deadline(deadline);
  std::cout << "Initializing Simulink Automator and Loading Matlab Models...." << std::endl;

  grpc::Status status = simulink_automator_stub_->Initialize(&context, request, &response);

  if (!status.ok()) {
    std::stringstream ss;
    ss << "SIMULINK AUTOMATOR ERROR: " << status.error_message();
    throw std::runtime_error(ss.str());
  }

  std::cout << "Successfully Initialized Simulink Automator and Loaded Matlab Models! "
            << std::endl;
}
