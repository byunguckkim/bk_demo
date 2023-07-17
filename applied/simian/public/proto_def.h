#pragma once

// Protobuf introduced status.message() in version 3.7.0 to replace status.error_message().
// Since the public header (port_def.inc) that tells us the version number being used was
// also only introduced in 3.7.0, we use this workaround to determine whether to use `message`
// or `error_message`.
#if __has_include(<google/protobuf/port_def.inc>)
#define PROTOBUF_STATUS_MESSAGE 1
#endif  // __has_include(...)

#include <google/protobuf/stubs/status.h>
#include <string>

std::string get_protobuf_status_message(const google::protobuf::util::Status& status);
