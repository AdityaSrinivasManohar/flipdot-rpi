FROM ubuntu:24.04

RUN apt-get update && \
    apt-get install -y \
    git \
    nano \
    curl \
    libxml2 libxml2-dev zlib1g-dev \ 
    python3 \
    python3-pip \
    python3-venv && \
    apt-get clean

RUN curl -L https://github.com/bazelbuild/bazelisk/releases/download/v1.27.0/bazelisk-linux-arm64 -o bazelisk && \
    mv bazelisk /usr/local/bin/bazelisk && \
    chmod +x /usr/local/bin/bazelisk && \
    ln -s /usr/local/bin/bazelisk /usr/local/bin/bazel

COPY .devcontainer/bazel_completion.bash /etc/bash_completion.d/bazel_completion.bash

COPY .devcontainer/.bashrc /root/.bashrc
