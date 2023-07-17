#!/bin/bash

set -e

if [ $# -eq 0 ] || [ "$1" != "-y" ]; then
  echo "This script will install all necessary dependencies to build to customer interface on your machine."
  echo "That is usually not required since the recommended way is to build and run the interface "
  echo "in a Docker container."
  echo "If you intend to use the provided Docker container, it is not necessary to execute this script."
  echo "In that case, please start with 'docker/build.sh' and then './build_and_update.sh'."
  echo
  echo "If you do intend to build the customer interface on your host machine, execute this script again with '-y'."
  exit 1
fi

if [ "$(id -u)" -ne 0 ]; then
  echo "Since this script installs packages and dependencies, you need to run it as root, e.g. with 'sudo'."
  exit 1
fi

PROTOBUF_PATCH=docker/protobuf_position_independent_code.patch
if [ ! -f "${PROTOBUF_PATCH}" ]; then
  echo "Could not find ${PROTOBUF_PATCH}"
  echo "Please execute this script from the root directory of the customer interface."
  exit 1
fi

TMPDIR="$(mktemp -d)"

echo "*** Installing dependencies ***"
apt-get update
DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
    build-essential \
    ca-certificates \
    clang \
    cmake \
    geographiclib-tools \
    git \
    libeigen3-dev \
    libgeographic-dev \
    libgflags-dev \
    libopencv-dev \
    unzip \
    wget


echo "*** Installing protobuf ***"
pushd "${TMPDIR}"
GIT_SSL_NO_VERIFY=true git clone --branch v3.15.2 --depth 1 --recurse https://github.com/protocolbuffers/protobuf.git
mkdir -p "${TMPDIR}/protobuf/cmake/build"
pushd "${TMPDIR}/protobuf/cmake/build"
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_BUILD_TYPE=Release -DCMAKE_POSITION_INDEPENDENT_CODE=TRUE ..
make -j$(nproc) install
popd
rm -rf "${TMPDIR}/protobuf"
popd

echo "*** Installing nlohmann/json ***"
mkdir -p "${TMPDIR}/json-install"
pushd "${TMPDIR}/json-install"
wget https://github.com/nlohmann/json/releases/download/v3.10.2/include.zip
unzip include.zip
mkdir -p /usr/local/include
mv include/nlohmann /usr/local/include/
popd
rm -rf "${TMPDIR}/json-install"

rm -rf "${TMPDIR}"