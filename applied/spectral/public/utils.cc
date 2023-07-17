// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#include "applied/spectral/public/utils.h"

#include <algorithm>
#include <fstream>
#include <string>

#include <google/protobuf/io/gzip_stream.h>
#include <google/protobuf/io/printer.h>
#include <google/protobuf/io/zero_copy_stream_impl.h>
#include <google/protobuf/util/json_util.h>
#include <nlohmann/json.hpp>

#include "applied/simian/public/proto_def.h"

namespace applied {
namespace utils {

bool EndsWith(std::string str, std::string suffix, bool ignore_case) {
  if (ignore_case) {
    std::transform(str.begin(), str.end(), str.begin(), ::tolower);
    std::transform(suffix.begin(), suffix.end(), suffix.begin(), ::tolower);
  }
  return str.length() >= suffix.length() && str.substr(str.length() - suffix.length()) == suffix;
}

std::string GetProtoAsJSON(const google::protobuf::Message& proto, std::string& out_json_str,
                           bool populate_defaults) {
  google::protobuf::util::JsonOptions options;
  options.add_whitespace = true;
  options.always_print_primitive_fields = populate_defaults;
  google::protobuf::util::Status status =
      google::protobuf::util::MessageToJsonString(proto, &out_json_str, options);
  if (!status.ok()) {
    return "error getting json for proto message: " + get_protobuf_status_message(status);
  }
  return "";
}

std::string WriteToFile(const fs::path& file_path, const std::string& str) {
  std::ofstream file;
  file.open(file_path);
  if (!file.good()) {
    return "failed to open file for writing: " + file_path.string();
  }
  if (!EndsWith(file_path.string(), ".gz")) {
    file << str;
    file.flush();
    if (!file.good()) {
      return "failed to write file: " + file_path.string();
    }
    return "";
  }

  google::protobuf::io::OstreamOutputStream osos(&file);
  google::protobuf::io::GzipOutputStream gzos(&osos,
                                              google::protobuf::io::GzipOutputStream::Options());
  google::protobuf::io::Printer printer(&gzos, '$');
  printer.Print(str.c_str());
  return "";
}

std::string WriteToFile(const fs::path& file_path, const nlohmann::json& json_object) {
  if (!EndsWith(file_path.string(), ".gz") && !EndsWith(file_path.string(), ".json")) {
    return "output path must end with .json or .gz";
  }

  return WriteToFile(file_path, json_object.dump(1));
}

std::string WriteToFile(const fs::path& file_path, const google::protobuf::Message& proto) {
  std::string json_str;
  std::string err_msg = GetProtoAsJSON(proto, json_str);
  if (!err_msg.empty()) {
    return "error writing json for proto message: " + err_msg;
  }

  err_msg = WriteToFile(file_path, json_str);
  if (!err_msg.empty()) {
    return "error writing json for proto message: " + err_msg;
  }

  return "";
}

}  // namespace utils
}  // namespace applied
