# syntax=docker/dockerfile:1

############################################################
# Stage 1: build librealsense for the T265 in isolation
# This layer is cached on its own, so changing the apt package
# list below does NOT trigger a librealsense rebuild anymore.
# v2.53.1 is the last librealsense release that supports the T265.
############################################################
FROM ros:humble-ros-base-jammy AS realsense_builder

ARG DEBIAN_FRONTEND=noninteractive
ARG LIBREALSENSE_VERSION=v2.53.1

RUN apt-get update && apt-get install -y --no-install-recommends \
    build-essential \
    cmake \
    git \
    ca-certificates \
    pkg-config \
    libusb-1.0-0-dev \
    libudev-dev \
    libssl-dev \
 && rm -rf /var/lib/apt/lists/*

# Shallow clone + build + install in one layer, source is thrown away with the stage
RUN git clone --depth 1 --branch ${LIBREALSENSE_VERSION} \
        https://github.com/IntelRealSense/librealsense.git /tmp/librealsense \
 && cmake -S /tmp/librealsense -B /tmp/librealsense/build \
        -DCMAKE_BUILD_TYPE=Release \
        -DFORCE_RSUSB_BACKEND=ON \
        -DBUILD_PYTHON_BINDINGS=OFF \
        -DBUILD_EXAMPLES=OFF \
        -DBUILD_GRAPHICAL_EXAMPLES=OFF \
        -DBUILD_TOOLS=OFF \
        -DBUILD_UNIT_TESTS=OFF \
        -DCHECK_FOR_UPDATES=OFF \
 && cmake --build /tmp/librealsense/build -j"$(nproc)" \
 && DESTDIR=/rs_install cmake --install /tmp/librealsense/build


############################################################
# Stage 2: WCR runtime / development image
############################################################
FROM ros:humble-ros-base-jammy

# ARG instead of ENV: only applies during the build, not inside the running container
ARG DEBIAN_FRONTEND=noninteractive
ENV TZ=Etc/UTC
ENV RCUTILS_COLORIZED_OUTPUT=1
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# ------------------------------------------------------------
# Base dependencies (rarely change)
# --no-install-recommends keeps the image noticeably smaller
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    apt-utils \
    sudo \
    nano \
    git \
    wget \
    curl \
    ca-certificates \
    build-essential \
    cmake \
    pkg-config \
    libusb-1.0-0-dev \
    libudev-dev \
    libssl-dev \
    libeigen3-dev \
    libboost-all-dev \
    libatlas-base-dev \
    libceres-dev \
    libgoogle-glog-dev \
    libgflags-dev \
    libsuitesparse-dev \
    libyaml-cpp-dev \
    libpcl-dev \
    libopencv-dev \
    python3-pip \
    python3-dev \
    python3-numpy \
    python3-matplotlib \
    python3-psutil \
    python3-tk \
    python3-colcon-common-extensions \
    python3-vcstool \
    python3-rosdep \
    ros-humble-xacro \
    ros-humble-robot-state-publisher \
    ros-humble-ros2-control \
    ros-humble-ros2-controllers \
    ros-humble-rmw-fastrtps-cpp \
    ros-humble-rmw-cyclonedds-cpp \
    ros-humble-rosbag2 \
    ros-humble-rosbag2-transport \
    ros-humble-rosbag2-storage-default-plugins \
    ros-humble-rosbag2-storage-mcap \
    ros-humble-rviz2 \
    ros-humble-rqt \
    ros-humble-rqt-image-view \
    ros-humble-cv-bridge \
    ros-humble-diagnostic-updater \
    ros-humble-image-transport \
    ros-humble-camera-info-manager \
    ros-humble-launch-xml \
    ros-humble-launch-yaml \
    ros-humble-tf2 \
    ros-humble-tf2-ros \
    ros-humble-tf2-eigen \
    ros-humble-tf2-geometry-msgs \
    ros-humble-tf-transformations \
    ros-humble-nav-msgs \
    ros-humble-sensor-msgs \
    ros-humble-geometry-msgs \
    ros-humble-std-msgs \
    ros-humble-visualization-msgs \
    ros-humble-pcl-ros \
    ros-humble-pcl-conversions \
    ros-humble-vrpn-mocap \
    x11-apps \
    mesa-utils \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libxkbcommon-x11-0 \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# librealsense (T265) from the builder stage
# ------------------------------------------------------------
COPY --from=realsense_builder /rs_install/usr/local/ /usr/local/
RUN ldconfig

# ------------------------------------------------------------
# Extra dependencies - add new packages HERE
# Only this layer and the ones below it rebuild, so it's fast
# ------------------------------------------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
    libasio-dev \
    ros-humble-udp-msgs \
    ros-humble-ackermann-msgs \
    ros-humble-example-interfaces \
    ros-humble-rclcpp-components \
    ros-humble-rclcpp-lifecycle \
    ros-humble-lifecycle-msgs \
    ros-humble-rosidl-default-generators \
    ros-humble-ament-cmake-auto \
    ros-humble-ament-cmake-gtest \
    ros-humble-ament-lint-auto \
    ros-humble-ament-lint-common \
    ros-humble-joy \
    ros-humble-teleop-twist-joy \
 && rm -rf /var/lib/apt/lists/*

# ------------------------------------------------------------
# Entrypoint
# ------------------------------------------------------------
COPY --chmod=755 entrypoint.sh /entrypoint.sh

# ------------------------------------------------------------
# User setup (UID/GID match the host user, so files colcon
# creates in the mounted workspaces are owned by you, not root)
# ------------------------------------------------------------
ARG USERNAME=wcr_docker
ARG USER_UID=1000
ARG USER_GID=1000

RUN groupadd --gid ${USER_GID} ${USERNAME} \
 && useradd --uid ${USER_UID} --gid ${USER_GID} -m -s /bin/bash \
        -G dialout,plugdev,video ${USERNAME} \
 && echo "${USERNAME} ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/${USERNAME} \
 && chmod 0440 /etc/sudoers.d/${USERNAME}

# ------------------------------------------------------------
# Shell setup - written once to /etc/bash.bashrc, applies to
# every user (root and wcr_docker) and every `docker exec` shell
# Workspaces are expected to be mounted at /dxl_ws and /vins_ws
# ------------------------------------------------------------
RUN { \
      echo 'source /opt/ros/humble/setup.bash'; \
      echo '[ -f /dxl_ws/install/setup.bash ] && source /dxl_ws/install/setup.bash'; \
      echo '[ -f /vins_ws/install/setup.bash ] && source /vins_ws/install/setup.bash'; \
    } >> /etc/bash.bashrc

# rosdep init is already done in the ros:humble base image;
# update the cache for the user that will actually run rosdep
USER ${USERNAME}
RUN rosdep update --rosdistro humble

WORKDIR /home/${USERNAME}

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
