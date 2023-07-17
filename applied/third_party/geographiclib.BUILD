package(default_visibility = ["//visibility:public"])

licenses(["notice"])

cc_library(
    name = "geographiclib",
    srcs = glob(["src/*.cpp"]),
    hdrs = glob([
        "include/GeographicLib/*.hpp",
        "include/GeographicLib/*.h",
    ]),
    includes = ["include"],
)

exports_files(["LICENSE.txt"])
