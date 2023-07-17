#include "applied/simian/public/proto_def.h"

std::string get_protobuf_status_message(const google::protobuf::util::Status& status) {
#if defined(PROTOBUF_STATUS_MESSAGE)
  return status.message().as_string();
#else
  return status.error_message().as_string();
#endif
}
