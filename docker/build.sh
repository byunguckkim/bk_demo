#!/bin/bash

ROOT_DIR="$(cd "$(dirname "$0")/.." || exit; pwd -P)"
source "${ROOT_DIR}/docker/common.sh"

DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

UNIQUE_TAG="$(date +'%m-%d-%y')-$(git describe --always)"
UNIQUE_IMAGE_NAME="${IMAGE_NAME_PREFIX}:${UNIQUE_TAG}"

DOCKER_BUILDKIT=1 DOCKER_BUILDKIT=1 docker build -f "${DIR}/Dockerfile" -t "$UNIQUE_IMAGE_NAME" "$ROOT_DIR"
docker tag "$UNIQUE_IMAGE_NAME" "${IMAGE_NAME_PREFIX}:latest"
