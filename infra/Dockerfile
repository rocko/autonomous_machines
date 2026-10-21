# ROS 2 Jazzy on Ubuntu 24.04.
FROM ros:jazzy-ros-base

# Install tools required to register the official Gazebo repository.
RUN apt-get update && apt-get install -y --no-install-recommends \
    curl \
    ca-certificates \
    lsb-release \
    gnupg \
    xvfb \
    x11-utils \
    x11vnc \
    novnc \
    websockify \
    libgl1-mesa-dri \
    mesa-utils \
    && rm -rf /var/lib/apt/lists/*

# Register the official Open Source Robotics Foundation
# repository containing Gazebo Harmonic.
#
# The signing key is installed into a dedicated keyring.
# The repository is restricted to packages signed by that key.
RUN curl -fsSL https://packages.osrfoundation.org/gazebo.gpg \
    -o /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] https://packages.osrfoundation.org/gazebo/ubuntu-stable $(lsb_release -cs) main" \
    > /etc/apt/sources.list.d/gazebo-stable.list

# Install Gazebo Harmonic, ROS integration and development tools.
RUN apt-get update && apt-get install -y --no-install-recommends \
    gz-harmonic \
    ros-jazzy-demo-nodes-cpp \
    ros-jazzy-ros-gz \
    python3-colcon-common-extensions \
    python3-pytest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /workspace

CMD ["sleep", "infinity"]