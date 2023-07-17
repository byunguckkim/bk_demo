load("@rules_proto//proto:repositories.bzl", "rules_proto_dependencies", "rules_proto_toolchains")

def applied_protos_extra_deps():
    rules_proto_dependencies()
    rules_proto_toolchains()
