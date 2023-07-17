

#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

#include "applied/simian/public/simulink/simulink_bridge_manager.h"
#include "applied/simian/public/simulink/simulink_version.h"

namespace applied {

using simian_public::common::CommonResponse;

SimulinkManager::SimulinkManager(
    const simian_public::proto::scenario::simulink_config::SimulinkConfig& simulink_config)
    : simulink_config_(simulink_config) {}

CommonResponse SimulinkManager::Init() {
  CommonResponse resp;
  std::string ip_addr = simulink_config_.network_configuration().simulink_ip_address();
  uint32_t block_manager_port = simulink_config_.network_configuration().manager_port();
  uint32_t automator_port = simulink_config_.network_configuration().automator_port();

  client_endpoint_ = std::make_unique<zmq_services::ZmqClient>();
  zmq_services::Status status = client_endpoint_->TryInitialize(ip_addr, block_manager_port);

  bool debug = simulink_config_.verbose();
  if (debug) {
    std::cout << "Simulink Config" << std::endl;
    simulink_config_.PrintDebugString();
  }
  if (!status.IsSuccess()) {
    resp.set_status(CommonResponse::EXCEPTION);
    resp.mutable_exception()->set_exception_message(status.message);
    return resp;
  }

  client_endpoint_->Purge();

  if (simulink_config_.has_automation_configuration()) {
    simulink_automator_ = std::make_unique<SimulinkAutomatorCore>();
    automator_options_.matlab_root =
        simulink_config_.automation_configuration().matlab_root_absolute();
    automator_options_.model_path =
        simulink_config_.automation_configuration().model_absolute_path();

    for (const std::string& model_dep :
         simulink_config_.automation_configuration().model_deps_absolute()) {
      automator_options_.model_dependencies.push_back(model_dep);
    }
    for (const std::string& model_script :
         simulink_config_.automation_configuration().model_init_scripts_filenames()) {
      automator_options_.model_init_scripts.push_back(model_script);
    }
    for (const std::string& model_script :
         simulink_config_.automation_configuration().model_post_init_scripts_filenames()) {
      automator_options_.model_post_init_scripts.push_back(model_script);
    }
    for (const std::string& matlab_path :
         simulink_config_.automation_configuration().matlab_absolute_paths()) {
      automator_options_.matlab_paths.push_back(matlab_path);
    }

    simulink_automator_->InitializeCore(ip_addr, automator_port, automator_options_);
    simulink_automator_->StartSimulinkModel();
  }

  int simulink_start_timeout =
      simulink_config_.timeout_configuration().simulink_start_timeout_duration().seconds();
  std::cout << "Waiting " << simulink_start_timeout << " seconds for Simulink to start..."
            << std::endl;
  std::cout << "Please press 'Run' on your desired Simulink model." << std::endl;

  auto start = std::chrono::steady_clock::now();
  auto end = start + std::chrono::seconds(simulink_start_timeout);

  int send_timeout_mus =
      (simulink_config_.timeout_configuration().adp_send_to_simulink_timeout_duration().nanos() +
       1e9 * simulink_config_.timeout_configuration()
                 .adp_send_to_simulink_timeout_duration()
                 .seconds()) /
      1000;
  int recv_timeout_mus = (simulink_config_.timeout_configuration()
                              .adp_receive_from_simulink_timeout_duration()
                              .nanos() +
                          1e9 * simulink_config_.timeout_configuration()
                                    .adp_receive_from_simulink_timeout_duration()
                                    .seconds()) /
                         1000;
  bool ready = false;
  simian_public::simulink::ADPInitRequest init_request;
  while ((std::chrono::steady_clock::now() - end).count() < 0) {
    std::string unused_error_msg = "";
    // 1s timeout
    bool success = zmq_services::ReceiveRequest<zmq_services::ZmqClient,
                                                simian_public::simulink::ADPInitRequest>(
        *client_endpoint_.get(), init_request, unused_error_msg, recv_timeout_mus, debug = debug);

    if (success) {
      if (debug) {
        std::cout << "Received Data from Simulink" << std::endl;
      }
      client_endpoint_->Purge();
      ready = true;
      break;
    }
    // Sleep for 1s waiting.
    std::this_thread::sleep_for(std::chrono::seconds(1));
    if (debug) {
      std::cout << "Sleeping for 1s..." << std::endl;
    }
  }

  if (!ready) {
    resp.set_status(CommonResponse::EXCEPTION);
    resp.mutable_exception()->set_exception_message(
        "ERROR: Did not receive ready signal from Simulink within timeout. Please make sure you've "
        "specified the correct simulink_ip_address, and pressed 'Run' on the Simulink model prior "
        "to the timeout.");
    std::string error_msg;
    // Send request to stop the Simulink model.
    bool success =
        zmq_services::SendRequest<zmq_services::ZmqClient, simian_public::common::CommonResponse>(
            *client_endpoint_.get(), resp, error_msg, send_timeout_mus, 10, debug = debug);
    if (!success) {
      resp.mutable_exception()->set_exception_message(error_msg);
    }
    return resp;
  }

  if (init_request.version().major_version() != MAJOR_VERSION_SIMULINK_TOOLBOX ||
      init_request.version().minor_version() != MINOR_VERSION_SIMULINK_TOOLBOX) {
    resp.set_status(CommonResponse::EXCEPTION);
    std::stringstream error_msg_stream;
    error_msg_stream << "Requested version and toolbox version do not match (we require major and "
                        "minor versions to match). Toolbox version: "
                     << init_request.version().major_version() << "."
                     << init_request.version().minor_version() << "."
                     << init_request.version().patch_version()
                     << ". Bridge Version: " << MAJOR_VERSION_SIMULINK_TOOLBOX << "."
                     << MINOR_VERSION_SIMULINK_TOOLBOX << "." << PATCH_VERSION_SIMULINK_TOOLBOX
                     << "." << std::endl;
    resp.mutable_exception()->set_exception_message(error_msg_stream.str());
    std::string error_msg;
    bool success =
        zmq_services::SendRequest<zmq_services::ZmqClient, simian_public::common::CommonResponse>(
            *client_endpoint_.get(), resp, error_msg, send_timeout_mus, 10, debug = debug);
    if (!success) {
      resp.mutable_exception()->set_exception_message(error_msg);
      simulink_manager_did_fail_ = true;
    }
    return resp;
  }

  if (debug) {
    std::cout << "Successfully received init data. Sending handshake back..." << std::endl;
  }

  resp.set_status(CommonResponse::SUCCESS);
  std::string error_msg;
  bool success =
      zmq_services::SendRequest<zmq_services::ZmqClient, simian_public::common::CommonResponse>(
          *client_endpoint_.get(), resp, error_msg, send_timeout_mus, 10, debug = debug);
  if (!success) {
    resp.set_status(CommonResponse::EXCEPTION);
    resp.mutable_exception()->set_exception_message(error_msg);
    simulink_manager_did_fail_ = true;
  }
  return resp;
}

CommonResponse SimulinkManager::SendToSimulink(
    const simian_public::simulink::ADPOutput& adp_output) {
  CommonResponse resp;

  // Timeout in microseconds
  int send_timeout_mus =
      (simulink_config_.timeout_configuration().adp_send_to_simulink_timeout_duration().nanos() +
       1e9 * simulink_config_.timeout_configuration()
                 .adp_send_to_simulink_timeout_duration()
                 .seconds()) /
      1000;
  std::string error_msg;
  bool debug = simulink_config_.verbose();

  if (debug) {
    std::cout << "Sending Data..." << std::endl;
    adp_output.PrintDebugString();
  }
  client_endpoint_->Purge();

  bool success =
      zmq_services::SendRequest<zmq_services::ZmqClient, simian_public::simulink::ADPOutput>(
          *client_endpoint_.get(), adp_output, error_msg, send_timeout_mus, 10, debug);

  if (!success) {
    resp.set_status(CommonResponse::EXCEPTION);
    resp.mutable_exception()->set_exception_message(error_msg);
    simulink_manager_did_fail_ = true;
    return resp;
  }

  resp.set_status(CommonResponse::SUCCESS);
  return resp;
}

CommonResponse SimulinkManager::ReceiveFromSimulink(simian_public::simulink::ADPInput& adp_input) {
  CommonResponse resp;
  // Timeout in microseconds
  int recv_timeout_mus = (simulink_config_.timeout_configuration()
                              .adp_receive_from_simulink_timeout_duration()
                              .nanos() +
                          1e9 * simulink_config_.timeout_configuration()
                                    .adp_receive_from_simulink_timeout_duration()
                                    .seconds()) /
                         1000;
  std::string error_msg;
  bool debug = simulink_config_.verbose();

  bool success =
      zmq_services::ReceiveRequest<zmq_services::ZmqClient, simian_public::simulink::ADPInput>(
          *client_endpoint_.get(), adp_input, error_msg, recv_timeout_mus, debug);
  client_endpoint_->Purge();

  if (!success) {
    resp.set_status(CommonResponse::EXCEPTION);
    resp.mutable_exception()->set_exception_message(error_msg);
    simulink_manager_did_fail_ = true;
    return resp;
  }

  if (adp_input.has_sim_command()) {
    if (adp_input.sim_command().command() ==
        simian_public::simulink::ADPSimulinkSimCommand::STOP_SUCCESSFULLY) {
      resp.set_status(CommonResponse::EXCEPTION);
      resp.mutable_exception()->set_exception_message(
          "Error: Simulink Terminated before Simian. Ensure that your simulink model timeouts are "
          "greater than or equal to the Simian sim end time.");
      stopped_by_simulink_ = true;
      return resp;
    } else if (adp_input.sim_command().command() ==
               simian_public::simulink::ADPSimulinkSimCommand::ERROR) {
      resp.set_status(CommonResponse::EXCEPTION);
      resp.mutable_exception()->set_exception_message(
          "Error: Simulink Terminated with Error Status. Check Simulink Logs for More Detail.");
      stopped_by_simulink_ = true;
      return resp;
    } else if (adp_input.sim_command().command() ==
               simian_public::simulink::ADPSimulinkSimCommand::PAUSE) {
      std::cout << "Received Pause Signal from Simulink...Waiting for Continue" << std::endl;
      while (true) {
        success = zmq_services::ReceiveRequest<zmq_services::ZmqClient,
                                               simian_public::simulink::ADPInput>(
            *client_endpoint_.get(), adp_input, error_msg, recv_timeout_mus, debug);
        client_endpoint_->Purge();
        if (!success) {
          // Upon timeout continue to try and receive data.
          continue;
        }
        if (adp_input.has_sim_command()) {
          // We check for any sim_commands that can potentially terminate the Simulink/Simian
          // co-simulation. If there are any such interrupts sent, we terminate the simulation.
          if (adp_input.sim_command().command() ==
              simian_public::simulink::ADPSimulinkSimCommand::STOP_SUCCESSFULLY) {
            resp.set_status(CommonResponse::EXCEPTION);
            resp.mutable_exception()->set_exception_message(
                "ERROR: Simulink Terminated before Simian.");
            stopped_by_simulink_ = true;
            return resp;
          } else if (adp_input.sim_command().command() ==
                     simian_public::simulink::ADPSimulinkSimCommand::ERROR) {
            resp.set_status(CommonResponse::EXCEPTION);
            resp.mutable_exception()->set_exception_message(
                "Error: Simulink Terminated with Error Status. Check Simulink Logs for More "
                "Detail.");
            stopped_by_simulink_ = true;
            return resp;
          }
        }
        if (!(adp_input.controls_channels_size() > 0)) {
          // If the Simulink Model uses a solver that runs at a finer step size than the sample
          // time of the manager, it is possible that the `Continue` signal is received at these
          // `transitionary` steps, in which cases we want to receive data till we receive a valid
          // input back to Simian. The valid input is typically sent at a rate configured by the ADP
          // Manager block.
          continue;
        }
        break;
      }
    } else {
      resp.set_status(CommonResponse::EXCEPTION);
      resp.mutable_exception()->set_exception_message(
          "Error: Received unknown response form Simulink.");
      return resp;
    }
  }

  if (debug) {
    std::cout << "Received Data..." << std::endl;
    adp_input.PrintDebugString();
  }

  resp.set_status(CommonResponse::SUCCESS);
  return resp;
}

CommonResponse SimulinkManager::Terminate() {
  CommonResponse resp;
  resp.set_status(CommonResponse::SUCCESS);
  if (!stopped_by_simulink_) {
    // Send Request to Terminate Simulink Model

    simian_public::simulink::ADPSimulinkSimCommand::SimulinkADPCommand command =
        simulink_manager_did_fail_
            ? simian_public::simulink::ADPSimulinkSimCommand::ERROR
            : simian_public::simulink::ADPSimulinkSimCommand::STOP_SUCCESSFULLY;
    simian_public::simulink::ADPOutput stop_request;
    stop_request.mutable_sim_command()->set_command(command);
    resp = SendToSimulink(stop_request);
  }

  if (simulink_config_.has_automation_configuration()) {
    simulink_automator_->FinalizeCore();
  }

  return resp;
}
}  // namespace applied
