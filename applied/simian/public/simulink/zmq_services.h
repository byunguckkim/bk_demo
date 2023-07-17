// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt
#pragma once

#include <chrono>
#include <cstdint>
#include <functional>
#include <iostream>
#include <string>

#include "applied/simian/public/simulink/compatibility.h"
#include "zmq.hpp"

constexpr int kMaxDataReceive = 256000;
namespace zmq_services {

enum StatusCode { kStatusError = 0, kStatusSuccess = 1, kStatusTryAgain = 2 };

struct Status {
  Status(StatusCode _code, const std::string& _message) : code(_code), message(_message) {}

  static Status Error(const std::string& message) { return Status(kStatusError, message); }
  static Status Success() { return Status(kStatusSuccess, ""); }
  static Status TryAgain() { return Status(kStatusTryAgain, "try again"); }

  bool IsError() const { return kStatusError == code; }
  bool IsSuccess() const { return kStatusSuccess == code; }
  bool IsTryAgain() const { return kStatusTryAgain == code; }

  void DieIfError(const std::string& context) const;
  void DieIfNotSuccess(const std::string& context) const;

  StatusCode code;
  std::string message;
};

class ZmqServer {
 public:
  ZmqServer();
  ~ZmqServer();
  ZmqServer(const ZmqServer&) = delete;
  ZmqServer& operator=(const ZmqServer&) = delete;

  void InitializeOrDie(int port_num);
  Status TryInitialize(int port_num);

  const Status Initialize(const std::string& port);

  Status Receive(zmq::message_t& request, int& num_bytes_read);

  const Status TrySend(zmq::message_t& request);
  Status Purge();
  zmq::socket_t& GetSock() { return sock_; }

 private:
  zmq::context_t& ctx_;
  zmq::socket_t sock_;
};

class ZmqClient {
 public:
  ZmqClient();
  ~ZmqClient();

  ZmqClient(const ZmqClient&) = delete;
  ZmqClient& operator=(const ZmqClient&) = delete;

  void InitializeOrDie(const std::string& server_address, int port_num);
  Status TryInitialize(const std::string& server_address, int port_num);

  const Status Initialize(const std::string& host, const std::string& port);

  Status Receive(zmq::message_t& request, int& num_bytes_read);

  const Status TrySend(zmq::message_t& request);
  Status Purge();
  zmq::socket_t& GetSock() { return sock_; }

 private:
  zmq::socket_t sock_;
  zmq::context_t& ctx_;
};

template <typename ClientOrServer, typename ProtoRequest>
bool SendRequest(ClientOrServer& endpoint, const ProtoRequest& send_proto,
                 std::string& out_error_msg, const useconds_t send_timeout,
                 const useconds_t sleep_duration, bool debug = false) {
  std::string in_proto_str;
  in_proto_str.clear();
  in_proto_str.resize(send_proto.ByteSizeLong());

  if (debug) {
    std::cout << "Sending Proto:  " << send_proto.DebugString() << std::endl;
  }
  bool success = send_proto.SerializeToString(&in_proto_str);
  if (!success) {
    out_error_msg = "ERROR: Could not serialize proto";
    return success;
  }
  int bytes_sent = 0;
  success = SendOrTimeout(endpoint, send_timeout, sleep_duration, (uint8_t*)in_proto_str.c_str(),
                          in_proto_str.size(), bytes_sent);
  if (debug) {
    std::cout << "Bytes Sent: " << bytes_sent << std::endl;
  }
  if (!success) {
    out_error_msg = "ERROR: Could not send data";
    return success;
  }
  return success;
}

template <typename ClientOrServer, typename ProtoRequest>
bool ReceiveRequest(ClientOrServer& endpoint, ProtoRequest& out_recv_proto,
                    std::string& out_error_msg, const useconds_t recv_timeout, bool debug = false) {
  int bytes_rxed;
  uint8_t data[kMaxDataReceive];
  bool success = ReceiveOrTimeout(endpoint, recv_timeout, data, sizeof(data), bytes_rxed);
  if (debug) {
    std::cout << "Bytes Rxed: " << bytes_rxed << std::endl;
  }
  if (!success) {
    out_error_msg = "ERROR: Could not receive data";
    return success;
  }

  std::string proto_data((char*)data, bytes_rxed);
  success = out_recv_proto.ParseFromString(proto_data);

  if (debug) {
    std::cout << "Data Rxed: " << out_recv_proto.DebugString() << std::endl;
  }
  if (!success) {
    out_error_msg = "ERROR: Could not parse received data into proto";
    return success;
  }
  return success;
}

template <typename ClientOrServer, typename ProtoResponse, typename ProtoRequest>
bool SubmitRequest(ClientOrServer& endpoint, const ProtoRequest& send_proto,
                   ProtoResponse& out_recv_proto, std::string& out_error_msg,
                   const useconds_t send_timeout = 1000000, const useconds_t recv_timeout = 1000000,
                   const useconds_t sleep_duration = 10, bool debug = false) {
  endpoint.Purge();
  bool success =
      SendRequest(endpoint, send_proto, out_error_msg, send_timeout, sleep_duration, debug);
  if (!success) {
    return success;
  }
  success = ReceiveRequest(endpoint, out_recv_proto, out_error_msg, recv_timeout, debug);

  return success;
}

template <typename ClientOrServer, typename ProtoResponse, typename ProtoRequest>
bool RespondToRequest(ClientOrServer& endpoint,
                      std::function<bool(ProtoRequest&, ProtoResponse&)> req_func,
                      std::string& out_error_msg, const useconds_t send_timeout = 1000000,
                      const useconds_t recv_timeout = 1000000, const useconds_t sleep_duration = 10,
                      bool debug = false) {
  ProtoRequest req_proto;
  bool success = ReceiveRequest(endpoint, req_proto, out_error_msg, recv_timeout, debug);

  if (!success) {
    return success;
  }
  ProtoResponse resp_proto;
  success = req_func(req_proto, resp_proto);
  if (!success) {
    out_error_msg = "ERROR: Error in computing the proto response";
    return success;
  }
  if (debug) {
    std::cout << "Sending Proto: " << resp_proto.DebugString() << std::endl;
  }
  success = SendRequest(endpoint, resp_proto, out_error_msg, send_timeout, sleep_duration, debug);
  return success;
}

template <typename ClientOrServer>
bool ReceiveOrTimeout(ClientOrServer& endpoint, useconds_t max_wait, uint8_t* data, int data_len,
                      int& num_bytes_rxed) {
  zmq::message_t request(data_len);
  zmq::pollitem_t items[] = {{(void*)endpoint.GetSock(), 0, ZMQ_POLLIN, 0}};
  typedef std::chrono::high_resolution_clock Clock;
  auto start_time = Clock::now();
  while (std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_time).count() <
         max_wait) {
    zmq::poll(&(items[0]), 1, 0);
    if (items[0].revents & ZMQ_POLLIN) {
      endpoint.Receive(request, num_bytes_rxed);
      std::memcpy(data, request.data(), num_bytes_rxed);
      return true;
    }
  }
  return false;
}

template <typename ClientOrServer>
bool SendOrTimeout(ClientOrServer& endpoint, useconds_t max_wait, useconds_t sleep_duration,
                   const uint8_t* data, int data_len, int& num_bytes_sent) {
  Status status = Status::TryAgain();
  zmq::message_t request(data, data_len);
  typedef std::chrono::high_resolution_clock Clock;
  auto start_time = Clock::now();
  while (std::chrono::duration_cast<std::chrono::microseconds>(Clock::now() - start_time).count() <
         max_wait) {
    status = endpoint.TrySend(request);
    if (status.IsSuccess()) {
      num_bytes_sent = data_len;
      return true;
    }
    if (status.IsError()) {
      num_bytes_sent = 0;
      return false;
    }
    usleep(sleep_duration);
  }
  return false;
}

std::string get_connect_str(const std::string transport_type, const std::string endpoint,
                            const std::string port);

}  // namespace zmq_services
