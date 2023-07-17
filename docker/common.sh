#!/bin/bash

export CONTAINER_NAME="bk_demo_bridge"
export IMAGE_NAME_PREFIX="bk_demo_prebuilt"
export PREBUILT_IMAGE_TAG="latest"
export IMAGE_NAME="${IMAGE_NAME_PREFIX}:${PREBUILT_IMAGE_TAG}"

run_in_dev_container() {
  docker exec -u $USER -it $CONTAINER_NAME bash -ic "$@"
}

is_running_in_docker() {
  grep -q docker /proc/self/cgroup || test -f /.dockerenv
}

fatal_error() {
  echo "$@" >&2
  exit 1
}