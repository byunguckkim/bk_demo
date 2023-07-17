#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "$0")"; pwd -P)"
source "$ROOT_DIR/docker/common.sh"

# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083

if ! docker ps --format "{{.Names}}" | grep "^$CONTAINER_NAME$" > /dev/null 2>&1; then
# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
  echo "Bringing up container"
  "$ROOT_DIR/docker/start.sh"
fi

echo "Rebuilding customer interface files..."
# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
run_in_dev_container "mkdir -p /simian/build/ &&\
  cd /simian/build &&\
  cmake -DCMAKE_BUILD_TYPE=RelWithDebInfo ../ &&\
  make -j12  &&\
  ln -sf /simian/build/customer_interface.so /simian/customer_interface.so"
# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
echo "Finished building and updating customer interface."