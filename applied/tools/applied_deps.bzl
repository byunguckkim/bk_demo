# Load dependencies needed to compile the applied library as a 3rd-party consumer.
load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

def _maybe_http_archive(name, **kwargs):
    if not native.existing_rule(name):
        http_archive(name = name, **kwargs)

def applied_deps():
    _maybe_http_archive(
        name = "com_google_protobuf",
        strip_prefix = "protobuf-3.14.0",
        sha256 = "d0f5f605d0d656007ce6c8b5a82df3037e1d8fe8b121ed42e536f569dec16113",
        urls = [
            "https://github.com/protocolbuffers/protobuf/archive/v3.14.0.tar.gz",
        ],
    )

    _maybe_http_archive(
        name = "rules_proto_grpc",
        sha256 = "507e38c8d95c7efa4f3b1c0595a8e8f139c885cb41a76cab7e20e4e67ae87731",
        strip_prefix = "rules_proto_grpc-4.1.1",
        urls = ["https://github.com/rules-proto-grpc/rules_proto_grpc/archive/4.1.1.tar.gz"],
    )

    _maybe_http_archive(
        name = "com_github_mjbots_bazel_deps",
        url = "https://github.com/mjbots/bazel_deps/archive/457a92bc426f23146d4edeb72fb96fd214684426.zip",
        strip_prefix = "bazel_deps-457a92bc426f23146d4edeb72fb96fd214684426",
        sha256 = "520cbe8cc5e960ba39f079e9001b8f992e4614cab08720b27afa47a65fe8566b",
    )

    _maybe_http_archive(
        name = "com_github_grpc_grpc",
        strip_prefix = "grpc-1.35.0",
        sha256 = "27dd2fc5c9809ddcde8eb6fa1fa278a3486566dfc28335fca13eb8df8bd3b958",
        urls = [
            "https://github.com/grpc/grpc/archive/v1.35.0.tar.gz",
        ],
    )

    _maybe_http_archive(
        name = "eigen",
        build_file = "@applied_release//:third_party/eigen.BUILD",
        sha256 = "04f8a4fa4afedaae721c1a1c756afeea20d3cdef0ce3293982cf1c518f178502",
        strip_prefix = "eigen-eigen-b9cd8366d4e8",
        urls = [
            "https://s3-us-west-2.amazonaws.com/buildkite-public/eigen-3.2.10.tar.gz",
        ],
    )

    _maybe_http_archive(
        name = "nlohmann_json",
        build_file = "@applied_release//:third_party/nlohmann_json.BUILD",
        sha256 = "8590fbcc2346a3eefc341935765dd57598022ada1081b425678f0da9a939a3c0",
        urls = [
            "https://github.com/nlohmann/json/releases/download/v3.8.0/include.zip",
        ],
    )

    _maybe_http_archive(
        name = "geographiclib",
        build_file = "@applied_release//:third_party/geographiclib.BUILD",
        strip_prefix = "GeographicLib-1.49",
        sha256 = "aec0ab52b6b9c9445d9d0a77e3af52257e21d6e74e94d8c2cb8fa6f11815ee2b",
        urls = [
            "https://s3-us-west-2.amazonaws.com/buildkite-public/GeographicLib-1.49.tar.gz",
            "http://pilotfiber.dl.sourceforge.net/project/geographiclib/distrib/GeographicLib-1.49.tar.gz",
        ],
        patches = [
            "@applied_release//:third_party/geographiclib.cpp17.patch",
        ],
    )
