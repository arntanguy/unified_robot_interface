# Devcontainer for URI on Ubuntu 24.04 (noble) + ROS Jazzy: mc_rtc itself,
# installed from the mc-rtc "head" apt repository with ROS support, and URI's
# own extra dependencies: zenoh-c / zenoh-cpp (zenohcxx), FlatBuffers,
# Protobuf, and GTest.
ARG ROS_DISTRO=jazzy
# Must match the channel in zenoh-c's rust-toolchain.toml.
ARG RUST_VERSION=1.93.0
ARG ZENOH_C_VERSION=1.9.0
ARG ZENOH_CPP_VERSION=1.9.0
ARG FLATBUFFERS_VERSION=25.12.19

ARG MC_RTC_APT_REFRESH=2026-10-30

# Shared install prefix for this project's from-source dependencies.
ARG EXTRA_DEPS_PREFIX=/home/vscode/.local/unified_robot_interface-deps

FROM rust:${RUST_VERSION}-slim-bookworm AS rust

# ── Common base: ROS image + build tools + the `vscode` user ───────────────
FROM ros:${ROS_DISTRO}-ros-base AS base
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
      build-essential \
      ca-certificates \
      ccache \
      cmake \
      curl \
      git \
      libcap2-bin \
      pkg-config \
      sudo \
      apt-transport-https \
    && rm -rf /var/lib/apt/lists/* \
    && usermod -l vscode -d /home/vscode -m -s /bin/bash ubuntu \
    && groupmod -n vscode ubuntu \
    && echo 'vscode ALL=(ALL) NOPASSWD:ALL' > /etc/sudoers.d/vscode \
    && chmod 0440 /etc/sudoers.d/vscode

# ── FlatBuffers: build a current release from source ─────────────────────────
# Ubuntu 24.04's 2.0.8 lacks flatbuffers_generate_headers().
FROM base AS flatbuffers
ARG FLATBUFFERS_VERSION
ARG EXTRA_DEPS_PREFIX
RUN git clone --branch v${FLATBUFFERS_VERSION} --depth 1 https://github.com/google/flatbuffers.git /tmp/flatbuffers \
    && cmake -S /tmp/flatbuffers -B /tmp/flatbuffers/build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_INSTALL_PREFIX=${EXTRA_DEPS_PREFIX} \
        -DFLATBUFFERS_BUILD_TESTS=OFF \
    && cmake --build /tmp/flatbuffers/build --parallel $(nproc) \
    && cmake --install /tmp/flatbuffers/build

# ── zenoh-c + zenoh-cpp ──────────────────────────────────────────────────────
FROM base AS zenoh
ARG ZENOH_C_VERSION
ARG ZENOH_CPP_VERSION
ARG EXTRA_DEPS_PREFIX
ENV RUSTUP_HOME=/usr/local/rustup \
    CARGO_HOME=/usr/local/cargo \
    PATH=/usr/local/cargo/bin:${PATH}
COPY --from=rust /usr/local/rustup /usr/local/rustup
COPY --from=rust /usr/local/cargo /usr/local/cargo

# zenoh-c: built from source to enable shared memory.
RUN --mount=type=cache,target=/usr/local/cargo/registry \
    --mount=type=cache,target=/usr/local/cargo/git \
    git clone --branch ${ZENOH_C_VERSION} --depth 1 https://github.com/eclipse-zenoh/zenoh-c.git /tmp/zenoh-c \
    && cmake -S /tmp/zenoh-c -B /tmp/zenoh-c/build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_INSTALL_PREFIX=${EXTRA_DEPS_PREFIX} \
        -DZENOHC_BUILD_WITH_SHARED_MEMORY=ON \
    && cmake --build /tmp/zenoh-c/build --parallel $(nproc) \
    && cmake --install /tmp/zenoh-c/build

# zenoh-cpp (zenohcxx): header-only C++ bindings over zenoh-c
RUN git clone --branch ${ZENOH_CPP_VERSION} --depth 1 https://github.com/eclipse-zenoh/zenoh-cpp.git /tmp/zenoh-cpp \
    && cmake -S /tmp/zenoh-cpp -B /tmp/zenoh-cpp/build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_INSTALL_PREFIX=${EXTRA_DEPS_PREFIX} \
        -DCMAKE_PREFIX_PATH=${EXTRA_DEPS_PREFIX} \
        -DZENOHCXX_ZENOHC=ON \
        -DZENOHCXX_ZENOHPICO=OFF \
        -DZENOHCXX_EXAMPLES=OFF \
    && cmake --build /tmp/zenoh-cpp/build --parallel $(nproc) \
    && cmake --install /tmp/zenoh-cpp/build

# ── devcontainer: all dependencies, URI itself is built by post-create.sh ───
FROM base AS devcontainer
ARG EXTRA_DEPS_PREFIX
ARG ROS_DISTRO

LABEL org.opencontainers.image.source="https://github.com/isri-aist/unified_robot_interface" \
      org.opencontainers.image.description="Devcontainer for unified_robot_interface: Ubuntu 24.04, ROS Jazzy, mc_rtc (head), zenoh-c/zenoh-cpp, FlatBuffers" \
      org.opencontainers.image.title="unified_robot_interface-devcontainer"

USER root

# ── Extra system dependencies specific to URI ──────────────────
# Note: FlatBuffers is NOT installed via apt here, see the flatbuffers stage.
RUN --mount=type=cache,target=/var/cache/apt \
    apt-get update && apt-get install -y --no-install-recommends \
      protobuf-compiler \
      libprotobuf-dev \
      libgtest-dev \
      libnanomsg-dev \
      libltdl-dev \
      libboost-program-options-dev \
      libboost-thread-dev \
      openssh-server \
      xauth \
      x11-apps \
      mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# ── mc_rtc from the mc-rtc "head" apt repository, with ROS support ──────────
RUN --mount=type=cache,target=/var/cache/apt \
    echo "mc_rtc apt refresh: ${MC_RTC_APT_REFRESH}" \
    && curl -1sLf https://dl.cloudsmith.io/public/mc-rtc/head/setup.deb.sh | bash \
    && apt-get install -y --no-install-recommends \
      ros-${ROS_DISTRO}-mc-rtc-ros-compat \
      ros-${ROS_DISTRO}-mc-rtc-msgs \
      libmc-rtc-dev \
      mc-rtc-utils \
    && rm -rf /var/lib/apt/lists/*

RUN mkdir -p /run/sshd /etc/ssh/sshd_config.d \
    && chmod 0755 /run/sshd \
    && printf '%s\n' \
         'X11Forwarding yes' \
         'X11UseLocalhost yes' \
         'XAuthLocation /usr/bin/xauth' \
       > /etc/ssh/sshd_config.d/60-x11-forwarding.conf \
    && sshd -t

USER vscode

RUN mkdir -p ${EXTRA_DEPS_PREFIX}
COPY --from=flatbuffers --chown=vscode:vscode ${EXTRA_DEPS_PREFIX} ${EXTRA_DEPS_PREFIX}
COPY --from=zenoh --chown=vscode:vscode ${EXTRA_DEPS_PREFIX} ${EXTRA_DEPS_PREFIX}

# `uri` has cap_sys_nice, which makes the loader ignore LD_LIBRARY_PATH.
RUN printf '%s\n' \
      /opt/ros/${ROS_DISTRO}/lib \
      /opt/ros/${ROS_DISTRO}/lib/x86_64-linux-gnu \
      ${EXTRA_DEPS_PREFIX}/lib \
    | sudo tee /etc/ld.so.conf.d/unified_robot_interface.conf > /dev/null \
    && sudo ldconfig

ENV EXTRA_DEPS_PREFIX=${EXTRA_DEPS_PREFIX}
ENV CMAKE_PREFIX_PATH="${EXTRA_DEPS_PREFIX}:${CMAKE_PREFIX_PATH}"
ENV LD_LIBRARY_PATH="${EXTRA_DEPS_PREFIX}/lib:${LD_LIBRARY_PATH}"

WORKDIR /home/vscode/unified_robot_interface

# ── uri: the devcontainer + URI built and installed in /opt/uri ─────────────
# Base image for driver images (e.g. ros2_control_driver): build the driver
# with -DCMAKE_INSTALL_PREFIX=${URI_PREFIX} so `uri interface` finds it in
# ${URI_PREFIX}/lib/robot_interface. Needs the repository root as build
# context. Run containers with `--cap-add=sys_nice`: `uri` has that file
# capability, and exec fails without it in the container's bounding set.
FROM devcontainer AS uri
ARG ROS_DISTRO

LABEL org.opencontainers.image.source="https://github.com/isri-aist/unified_robot_interface" \
      org.opencontainers.image.description="unified_robot_interface (uri) installed in /opt/uri, on Ubuntu 24.04, ROS Jazzy, mc_rtc (head)" \
      org.opencontainers.image.title="unified_robot_interface"

ENV URI_PREFIX=/opt/uri
USER root
COPY . /tmp/uri-src
RUN . /opt/ros/${ROS_DISTRO}/setup.sh \
    && cmake -S /tmp/uri-src -B /tmp/uri-build \
        -DCMAKE_BUILD_TYPE=RelWithDebInfo \
        -DCMAKE_PREFIX_PATH=${EXTRA_DEPS_PREFIX} \
        -DCMAKE_INSTALL_PREFIX=${URI_PREFIX} \
    && cmake --build /tmp/uri-build --parallel $(nproc) \
    && cmake --install /tmp/uri-build \
    && rm -rf /tmp/uri-src /tmp/uri-build \
    && echo ${URI_PREFIX}/lib >> /etc/ld.so.conf.d/unified_robot_interface.conf \
    && ldconfig \
    && getcap ${URI_PREFIX}/bin/uri | grep -q cap_sys_nice

ENV PATH="${URI_PREFIX}/bin:${PATH}" \
    CMAKE_PREFIX_PATH="${URI_PREFIX}:${CMAKE_PREFIX_PATH}"
USER vscode
WORKDIR /home/vscode
