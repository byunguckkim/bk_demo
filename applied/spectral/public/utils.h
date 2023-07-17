// Copyright (C) 2021 Applied Intuition, Inc. All rights reserved.
// This source code file is distributed under and subject to the LICENSE in
// license.txt

#pragma once

#if __has_include(<filesystem>)
#include <filesystem>
namespace fs = std::filesystem;
#elif __has_include(<experimental/filesystem>)
// Ubuntu Bionic's default toolchain doesn't have <filesystem>.
#include <experimental/filesystem>
namespace fs = std::experimental::filesystem;
#else
error "Missing the <filesystem> header."
#endif

#include <nlohmann/json_fwd.hpp>
#include <string>

namespace google {
namespace protobuf {
class Message;
}  // namespace protobuf
}  // namespace google

namespace applied {
namespace utils {

// When we have C++20 we can use the builtin .endswith()
bool EndsWith(std::string str, std::string suffix, bool ignore_case = false);

/**
 * @brief Converst a proto message to a JSON object represented as an std::string.
 *
 * @param proto Source LabelSnapshot.
 * @param out_str std::string to write output to.
 * @param populate_defaults populate JSON with proto primitive defaults.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string GetProtoAsJSON(const google::protobuf::Message& proto, std::string& out_str,
                           bool populate_defaults = false);

/**
 * @brief Writes a string to a designated file.
 *
 * @param file_path Full-path output destination, including the filename ending with extension.
 *                    If ending with ".gz", the save file will be gzip compressed.
 *                    E.g. "/dir/filename.gz".
 * @param str String data to write to disk.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WriteToFile(const fs::path& file_path, const std::string& str);

/**
 * @brief Writes a JSON object to a designated file.
 *
 * @param file_path Full-path output destination, including the filename ending with ".json".
 *                    If ending with ".gz", the save file will be gzip compressed.
 *                    E.g. "/dir/filename.json.gz".
 * @param json JSON object to write
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WriteToFile(const fs::path& file_path, const nlohmann::json& json_object);

/**
 * @brief Writes a proto message to a designated file using JSON representation.
 *
 * @param file_path Full-path output destination, including the filename ending with ".json".
 *                    If ending with ".gz", the save file will be gzip compressed.
 *                    E.g. "/dir/filename.json.gz".
 * @param proto Proto message to write.
 *
 * @returns Empty string on success, non-empty string on failure.
 */
std::string WriteToFile(const fs::path& file_path, const google::protobuf::Message& proto);

}  // namespace utils
}  // namespace applied
