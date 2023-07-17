#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." || exit; pwd -P)"
source "$ROOT_DIR/docker/common.sh"

docker exec \
    -u "$USER" \
    -it $CONTAINER_NAME \
    /bin/bash
