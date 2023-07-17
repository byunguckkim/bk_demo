#include "log_reader.h"

#include <iostream>
#include <optional>

static constexpr double kNanosecondsPerSecond = 1e9;

LogReader::LogReader(const google::protobuf::Value& extra_data, Mailbox* mailbox)
    : mailbox_(mailbox) {
  // An example of configuring state (the log cache directory) via `extra_data` in the scenario YAML
  // file.
  const auto& fields = extra_data.struct_value().fields();
  if (fields.count("log_cache_directory")) {
    log_cache_dir_ = fields.at("log_cache_directory").string_value();
    std::cout << "Log cache directory manually overridden to " << log_cache_dir_
              << " via `extra_data`." << std::endl;
  }
}

// Fetch will attempt to obtain your log from the path provided for your log in the scenario and
// store it in the given `cache_directory`.
bool LogReader::Fetch(const simian_public::simulator::v2::LogFetchOptions& options,
                      simian_public::simulator::v2::LogFetchOutput* output) {
  fs::path log_src = options.path();
  fs::path log_filename = log_src.filename();

  // First, check the cache directory to see if we've already fetched this log.
  fs::path log_dest = log_cache_dir_ / log_filename;

  if (fs::exists(log_dest)) {
    std::cout << "INFO: Log found in cache directory at '" << log_dest << "'!" << std::endl;
    output->set_updated_path(log_dest.string());
    output->mutable_cache_info()->mutable_cache_hit();
    return true;
  }

  std::cout << "INFO: Log missing, trying to retrieve '" << log_src << "' and copy into '"
            << log_dest << "'." << std::endl;

  // Start measuring download time
  std::chrono::system_clock::time_point download_start_time =
      std::chrono::high_resolution_clock::now();

  // TODO: Customer to implement a script or helper function to download, copy, or move
  // the log into the given destination in the cache directory from cloud storage, a NAS
  // system, mounted directory etc. and call it here...
  int ret_code =
      system(("/path/to/your/fetch/script.sh " + options.path() + " " + log_dest.string()).c_str());
  if (ret_code) {
    std::cerr << "ERROR: Failed to fetch log. Command to fetch log had non-zero return code "
              << ret_code << std::endl;
    return false;
  }

  if (!fs::exists(log_dest)) {
    std::cerr << "ERROR: Log not available after attempted retrieval. "
              << "Please check if there's an issue with fetching the file." << std::endl;
    return false;
  }

  std::cout << "INFO: Log was fetched successfully!" << std::endl;
  std::chrono::system_clock::time_point download_end_time =
      std::chrono::high_resolution_clock::now();
  auto download_duration =
      std::chrono::duration_cast<std::chrono::nanoseconds>(download_end_time - download_start_time);
  int duration_seconds = download_duration.count() / kNanosecondsPerSecond;
  output->mutable_cache_info()->mutable_download_duration()->set_seconds(duration_seconds);
  output->mutable_cache_info()->mutable_download_duration()->set_nanos(
      download_duration.count() - duration_seconds * kNanosecondsPerSecond);
  output->set_updated_path(log_dest.string());
  return true;
}

bool LogReader::Open(const simian_public::simulator::v2::LogOpenOptions& options,
                     simian_public::simulator::v2::LogOpenOutput* output) {
  // Use `log_fetch_output.value()` as the path at which to access your log when opening it!
  return true;
}

bool LogReader::Read(const simian_public::simulator::v2::LogReadOptions& options,
                     simian_public::simulator::v2::LogReadOutput* output) {
  if (options.offset().seconds() == 0 && options.offset().nanos() == 0) {
    std::cout << "Reading initial pose from log into memory." << std::endl;
    return ReadInitialPose(output);
  }
  return true;
}

bool LogReader::ReadInitialPose(simian_public::simulator::v2::LogReadOutput* output) {
  // TODO: Customer to implement functionality to read initial pose from the log and save
  // it in the `mailbox_` here...
  output->set_data_remaining(true);
  output->add_seen_channel_names("simian_pose");
  return true;
}

bool LogReader::Close(const simian_public::simulator::v2::LogCloseOptions& options) { return true; }
