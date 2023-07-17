
#include <stdlib.h>
#include <chrono>
#include <memory>
#include <string>
// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt
#include "zmq_services.h"

#include <sys/types.h>
#include <cstdint>

#ifdef __linux__
#include <err.h>
#include <errno.h>
#include <string.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstdlib>
#else
#include <winsock.h>
#include <algorithm>
#define EXIT_FAILURE 1
#define errx(exit_code, fmt, ...) \
  while (true) {                  \
    printf((fmt), (__VA_ARGS__)); \
    exit((exit_code));            \
  }
#pragma comment(lib, "ws2_32.lib")

#endif

#include <iostream>
#include <sstream>

#define REQUEST_BYTES_MAX 64000
#define SLEEP_DURATION 10
namespace {
zmq::context_t global_context{1};

const zmq_services::Status SendRawBytesImpl(zmq::socket_t& sock, zmq::message_t& request) {
  try {
    bool stat = sock.send(request, ZMQ_NOBLOCK);
    return stat ? zmq_services::Status::Success() : zmq_services::Status::TryAgain();
  } catch (zmq::error_t& err) {
    return zmq_services::Status::Error(std::string(err.what()));
  }
}

const zmq_services::Status RawBytesReceiveImpl(zmq::socket_t& sock, zmq::message_t& request,
                                               int& bytes_read) {
  std::optional<size_t> recv_bytes = sock.recv(request);
  if (recv_bytes.has_value()) {
    bytes_read = recv_bytes.value();
  } else {
    return zmq_services::Status::TryAgain();
  }
  return zmq_services::Status::Success();
}

}  // namespace

namespace zmq_services {

ZmqServer::ZmqServer() : ctx_(global_context) {
  sock_ = zmq::socket_t(global_context, ZMQ_PAIR);
  sock_.setsockopt(ZMQ_LINGER, 0);
}

ZmqClient::ZmqClient() : ctx_(global_context) {
  sock_ = zmq::socket_t(global_context, ZMQ_PAIR);
  sock_.setsockopt(ZMQ_LINGER, 0);
}

ZmqServer::~ZmqServer() {
  std::cout << "ZmqServer::~ZmqServer() closing connection." << std::endl;
  sock_.close();
}

Status ZmqServer::Receive(zmq::message_t& request, int& num_bytes_read) {
  return RawBytesReceiveImpl(sock_, request, num_bytes_read);
}

Status ZmqClient::Receive(zmq::message_t& request, int& num_bytes_read) {
  return RawBytesReceiveImpl(sock_, request, num_bytes_read);
}

Status ZmqServer::Purge() {
  zmq::message_t request(REQUEST_BYTES_MAX);
  zmq::pollitem_t items[] = {{(void*)GetSock(), 0, ZMQ_POLLIN, 0}};

  int num_bytes_rxed = 0;
  // Flushes buffer
  zmq::poll(&(items[0]), 1, 0);
  while (items[0].revents & ZMQ_POLLIN) {
    Receive(request, num_bytes_rxed);
    zmq::poll(&(items[0]), 1, 0);
  }
  return Status::Success();
}

Status ZmqClient::Purge() {
  zmq::message_t request(REQUEST_BYTES_MAX);
  int num_bytes_rxed = 0;
  zmq::pollitem_t items[] = {{(void*)GetSock(), 0, ZMQ_POLLIN, 0}};

  // Flushes buffer
  zmq::poll(&(items[0]), 1, 0);
  while (items[0].revents & ZMQ_POLLIN) {
    Receive(request, num_bytes_rxed);
    zmq::poll(&(items[0]), 1, 0);
  }
  return Status::Success();
}

const Status ZmqServer::Initialize(const std::string& port) {
  const std::string transport_type = "tcp";
  const std::string host = "*";
  std::string bind_str = get_connect_str(transport_type, host, port);
  try {
    sock_.bind(bind_str);
  } catch (zmq::error_t& err) {
    return Status::Error(std::string(err.what()));
  }
  return Status::Success();
}

ZmqClient::~ZmqClient() {
  std::cout << "ZmqClient::~ZmqClient() closing connection." << std::endl;
  sock_.close();
}

const Status ZmqClient::TrySend(zmq::message_t& request) {
  return SendRawBytesImpl(sock_, request);
}

const Status ZmqServer::TrySend(zmq::message_t& request) {
  return SendRawBytesImpl(sock_, request);
}

const Status ZmqClient::Initialize(const std::string& host, const std::string& port) {
  const std::string transport_type = "tcp";
  std::string connect_str = get_connect_str(transport_type, host, port);
  try {
    sock_.connect(connect_str);
  } catch (zmq::error_t& err) {
    return Status::Error(std::string(err.what()));
  }
  return Status::Success();
}

Status ZmqClient::TryInitialize(const std::string& server_address, int port_num) {
  std::ostringstream port_oss;
  port_oss << port_num;
  return Initialize(server_address, port_oss.str());
}

void ZmqClient::InitializeOrDie(const std::string& server_address, int port_num) {
  const Status status = TryInitialize(server_address, port_num);
  if (!status.IsSuccess()) {
    errx(EXIT_FAILURE, "ZmqClient::InitializeOrDie(%d): %s", port_num, status.message.c_str());
  }
}

void ZmqServer::InitializeOrDie(int port_num) {
  const Status status = TryInitialize(port_num);
  if (!status.IsSuccess()) {
    errx(EXIT_FAILURE, "ZmqServer::IniitalizeOrDie(%d): %s", port_num, status.message.c_str());
  }
}

Status ZmqServer::TryInitialize(int port_num) {
  std::ostringstream port_oss;
  port_oss << port_num;
  return Initialize(port_oss.str());
}

void Status::DieIfError(const std::string& context) const {
  if (IsError()) {
    errx(EXIT_FAILURE, "%s: %s", context.c_str(), message.c_str());
  }
}

void Status::DieIfNotSuccess(const std::string& context) const {
  if (!IsSuccess()) {
    errx(EXIT_FAILURE, "%s: %s", context.c_str(), message.c_str());
  }
}

std::string get_connect_str(const std::string transport_type, const std::string endpoint,
                            const std::string port) {
  std::string connect_str = transport_type + "://" + endpoint + ":" + port;
  return connect_str;
}
}  // namespace zmq_services
