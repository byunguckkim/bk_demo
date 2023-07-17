#!/bin/bash

# Create the user and its main group:
DOCKER_GROUP_NAME=$(echo "$DOCKER_GROUP" | sed -e 's/ /_/g')
addgroup --force-badname --gid "$DOCKER_GROUP_ID" "$DOCKER_GROUP_NAME"
adduser --disabled-password --force-badname --gecos '' "$DOCKER_USER" \
    --uid "$DOCKER_USER_ID" --gid "$DOCKER_GROUP_ID" 2>/dev/null
usermod -aG sudo "$DOCKER_USER"
# Add user to the 'docker' group:
if getent group docker > /dev/null; then
  groupmod --gid "$DOCKER_DOCKER_GID" docker
else
  addgroup --gid "$DOCKER_DOCKER_GID" docker
fi
adduser --force-badname "$DOCKER_USER" docker

echo '%sudo ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers
cp -r /etc/skel/ "/home/${DOCKER_USER}"

pushd "/home/${DOCKER_USER}"
chown "${DOCKER_USER}:${DOCKER_GROUP_NAME}" .
chown -R "${DOCKER_USER}:${DOCKER_GROUP_NAME}" $(ls -A . | grep -v '.cache' | grep -v '.yarn')
popd

# shellcheck disable=SC1054,SC1056,SC1072,SC1073,SC1083
