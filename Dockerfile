FROM ros:humble-ros-core

# Install build dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    python3-colcon-common-extensions \
    libeigen3-dev \
    libgoogle-glog-dev \
    && rm -rf /var/lib/apt/lists/*

# Setup workspace
WORKDIR /ws/src

# Copy project
COPY . /ws/src/OmniSLAM

# Build
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && \
    colcon build --symlink-install --packages-select omnislam_core

# Default entrypoint
CMD ["/bin/bash", "-c", ". /opt/ros/humble/setup.sh && . install/setup.bash && ros2 pkg list | grep omnislam"]
