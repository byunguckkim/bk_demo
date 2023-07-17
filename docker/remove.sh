#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." || exit; pwd -P)"
source "$ROOT_DIR/docker/common.sh"

echo "Stopping docker container $CONTAINER_NAME"
docker stop $CONTAINER_NAME
