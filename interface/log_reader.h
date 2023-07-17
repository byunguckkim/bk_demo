#pragma once

#include <google/protobuf/struct.pb.h>
#include <google/protobuf/util/time_util.h>
#include <optional>
#include <string>

// Simian dependencies
#include "applied/simian/public/proto/v2/io.pb.h"

// Local dependencies
#include "interface/filesystem.h"
#include "interface/mailbox.h"

class LogReader {
 public:
  explicit LogReader(const google::protobuf::Value& extra_data, Mailbox* mailbox);
  bool Fetch(const simian_public::simulator::v2::LogFetchOptions& options,
             simian_public::simulator::v2::LogFetchOutput* output);
  bool Open(const simian_public::simulator::v2::LogOpenOptions& options,
            simian_public::simulator::v2::LogOpenOutput* output);
  bool Read(const simian_public::simulator::v2::LogReadOptions& options,
            simian_public::simulator::v2::LogReadOutput* output);
  bool Close(const simian_public::simulator::v2::LogCloseOptions& options);

 private:
  bool ReadInitialPose(simian_public::simulator::v2::LogReadOutput* output);
  // Directory inside the bridge container in which fetched logs will be cached.
  // Should point to a directory that persists across container restarts.
  fs::path log_cache_dir_ = "/simian/logs";

  Mailbox* mailbox_;
  std::string bag_path_;
};
