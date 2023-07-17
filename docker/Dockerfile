
FROM ubuntu:bionic

RUN apt-get update && \
    DEBIAN_FRONTEND=noninteractive apt-get install -y --no-install-recommends \
      bash-completion \
      sudo \
      vim


RUN mkdir /tmp/install
RUN mkdir /tmp/install/docker
COPY install_build_dependencies.sh /tmp/install
COPY docker/protobuf_position_independent_code.patch /tmp/install/docker
WORKDIR /tmp/install
RUN ./install_build_dependencies.sh -y
RUN rm -rf /tmp/install



  
RUN mkdir /simian
WORKDIR /simian
ENTRYPOINT /bin/bash