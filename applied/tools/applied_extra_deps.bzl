load("@com_google_protobuf//:protobuf_deps.bzl", "protobuf_deps")
load("@rules_proto_grpc//:repositories.bzl", "rules_proto_grpc_repos", "rules_proto_grpc_toolchains")
load("@rules_proto_grpc//cpp:repositories.bzl", rules_proto_grpc_cpp_repos = "cpp_repos")

def applied_extra_deps():
    protobuf_deps()

    rules_proto_grpc_toolchains()
    rules_proto_grpc_repos()
    rules_proto_grpc_cpp_repos()
