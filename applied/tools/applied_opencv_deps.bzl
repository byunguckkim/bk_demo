load("@com_github_mjbots_bazel_deps//tools/workspace/libpng:repository.bzl", "libpng_repository")
load("@com_github_mjbots_bazel_deps//tools/workspace/libjpeg:repository.bzl", "libjpeg_repository")
load("@com_github_mjbots_bazel_deps//tools/workspace/opencv:repository.bzl", "opencv_repository")

def applied_opencv_deps():
    libpng_repository("libpng")
    libjpeg_repository("libjpeg")
    opencv_repository("opencv", version = "3.4.2")
