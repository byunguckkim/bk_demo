#!/bin/bash

set -e

ROOT_DIR="$(cd "$(dirname "$0")/.."; pwd -P)"
source "$ROOT_DIR/docker/common.sh"

while [ $# -gt 0 ]
do
    case "$1" in
    --help)
        fatal_error "Use this script to start a development docker"
        ;;
    *)
        fatal_error "Unknown option: $1"
        ;;
    esac
    shift
done

DOCKER_USER_ID=$(id -u)
DOCKER_GROUP=$(id -g -n)
DOCKER_GROUP_ID=$(id -g)
DOCKER_DOCKER_GID=$(getent group docker | cut -d: -f3)

EXTRA_ARGS=(
  --name "${CONTAINER_NAME}"
  # Creates the docker network on host, as opposed to being between containers.
  # Note that this is a very lax condition to handle the case where customers
  # run their stack on the host machine, and should be changed if the customer
  # runs their stack within a container. To create a more restrictive bridge docker
  # network, follow the steps listed here:
  # https://docs.docker.com/engine/reference/commandline/network_create/
  --net=host
# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
  # Mount this repository as a shared volume and use it as the working dir
  -v $ROOT_DIR:/simian/
# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
  -w /simian
  # Share the X socket so we can run GUI apps in the container
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw
  # When simulations are run with --sensor_shared_memory=all in the simulation flags
  # Spectral will send sensor data to the customer interface in a file under /tmp/spectral.
  -v /tmp/spectral/:/tmp/spectral/:ro
  -e "DISPLAY=$DISPLAY"
  # X will need us to be the same user
  -e "USER=$USER"
)

# Developers can interject here to add their customizations
# For example you could modify EXTRA_ARGS to mount directories to get your
# IDE's binaries+config into the container.
[ -e "$ROOT_DIR/.docker_start_config" ] && . "$ROOT_DIR/.docker_start_config"

docker run -id --rm "${EXTRA_ARGS[@]}" $IMAGE_NAME

if [ "${USER}" != "root" ]; then
  docker exec \
    -e "DOCKER_USER=$USER" \
    -e "DOCKER_USER_ID=$DOCKER_USER_ID" \
    -e "DOCKER_GROUP=$DOCKER_GROUP" \
    -e "DOCKER_GROUP_ID=$DOCKER_GROUP_ID" \
    -e "DOCKER_DOCKER_GID=$DOCKER_DOCKER_GID" \
    "$CONTAINER_NAME" bash -c "/simian/docker/adduser.sh"
fi


# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083

# Developers can interject here to customize the docker container after
# it has been started (e.g. creating config files, onetime setup steps).
if [ -e "$ROOT_DIR/.docker_post_start" ] ; then
  docker exec \
      "$CONTAINER_NAME" bash -c "/simian/.docker_post_start"
fi