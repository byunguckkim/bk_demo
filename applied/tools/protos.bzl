load("@rules_proto_grpc//cpp:defs.bzl", "cpp_grpc_library", "cpp_proto_library")

def applied_proto_library(
        name,
        srcs = [],
        src = None,
        deps = [],
        with_grpc = False,
        **kwargs):
    if srcs and src or (not srcs and not src):
        fail("Must specify srcs or src, not both or neither")
    if not srcs:
        srcs = [src]

    native.proto_library(
        name = name,
        srcs = srcs,
        deps = deps,
        import_prefix = "applied/",
        **kwargs
    )
    cc_deps = _protobuf_deps(deps, "_cc")
    applied_cc_proto_library(
        name = name + "_cc",
        src = name,
        deps = cc_deps,
    )

    if with_grpc:
        applied_cc_grpc_library(
            name = name + "_cc_grpc",
            src = name,
            deps = [name + "_cc"],
        )

def applied_cc_proto_library(name, src, deps = []):
    cpp_proto_library(
        name = name,
        protos = [src],
        deps = deps,
        include_prefix = "applied/",
    )

def applied_cc_grpc_library(name, src, deps = []):
    cpp_grpc_library(
        name = name,
        protos = [src],
        deps = deps,
        include_prefix = "applied/",
    )

def _protobuf_deps(deps, suffix):
    replaced_deps = []
    for dep in deps:
        if not dep.startswith("@com_google_protobuf"):
            replaced_deps.append(dep + suffix)
    return depset(replaced_deps).to_list()
