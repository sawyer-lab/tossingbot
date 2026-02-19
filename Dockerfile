FROM ros:noetic-perception

# Set standard TERM for consistent terminal behavior
ENV TERM=xterm-256color

RUN apt update && apt install -y --no-install-recommends \
    curl \
    git \
    tmux \
    vim-nox \
    gdb \
    wget \
    unzip \
    fontconfig \
    && rm -rf /var/lib/apt/lists/*

# Install system libraries required by OpenCV (and Open3D)
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# Install Python 3 pip and development tools (uses system Python 3.8)
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Create python symlink for compatibility
RUN ln -s /usr/bin/python3 /usr/bin/python

# Install only essential ROS Python packages
RUN python3 -m pip install \
    rospkg \
    catkin_pkg \
    "numpy>=1.20.0" \
    "scipy>=1.5.0" \
    pyyaml \
    empy

# Install robot API dependencies from requirements.txt
# (Flask, ZMQ, SocketIO - communication packages only)
COPY ros/robot_api/requirements.txt /tmp/robot_api_requirements.txt
RUN python3 -m pip install --no-cache-dir -r /tmp/robot_api_requirements.txt

# Install ros_numpy from GitHub (not available on PyPI)
RUN python3 -m pip install --no-cache-dir \
    git+https://github.com/eric-wieser/ros_numpy.git

# Install ROS packages for robot connectivity
RUN apt update && apt install --no-install-recommends -y \
    build-essential \
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-tf2-sensor-msgs \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-actionlib \
    ros-${ROS_DISTRO}-actionlib-msgs \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-rospy-message-converter \
    ros-${ROS_DISTRO}-openni2-launch \
    ros-${ROS_DISTRO}-mocap-optitrack \
    ros-${ROS_DISTRO}-moveit \
    ros-${ROS_DISTRO}-joint-state-publisher-gui \
    && rm -rf /var/lib/apt/lists/*


RUN apt update && apt install --no-install-recommends -y \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# Gazebo and simulation packages
RUN apt update && apt install --no-install-recommends -y \
    gazebo11 \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-kdl-parser \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-xacro \
    pcl-tools \
    && rm -rf /var/lib/apt/lists/*

# Network debugging tools for robot connection testing
RUN apt update && apt install -y \
    iputils-ping \
    net-tools \
    avahi-utils \
    avahi-daemon \
    libnss-mdns \
    dnsutils \
    iproute2 \
    sudo \
    && rm -rf /var/lib/apt/lists/*

ARG USER=kid
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $USER
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $USER

# Configure passwordless sudo for user
RUN echo "$USER ALL=(ALL) NOPASSWD:ALL" > /etc/sudoers.d/$USER && \
    chmod 0440 /etc/sudoers.d/$USER

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc

USER kid
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws
RUN /ros_entrypoint.sh catkin_make

RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool init .
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_robot.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_simulator.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sns_ik.git -b melodic-devel

RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool merge sawyer_simulator/sawyer_simulator.rosinstall
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool update

# ==============================================================================
# NETWORK CONFIGURATION - HARDCODED
# ==============================================================================
RUN cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/.

# Configure intera.sh for the real robot connection.
# these values match the physical robot setup (192.168.1.101/100).
RUN sed -i 's/ros_version=".*"/ros_version="noetic"/g' ~/ros_ws/intera.sh && \
    sed -i 's/your_ip="192.168.XXX.XXX"/your_ip="192.168.1.101"/g' ~/ros_ws/intera.sh && \
    sed -i 's/my_computer/rog/g' ~/ros_ws/intera.sh && \
    sed -i 's/robot_hostname="robot_hostname.local"/robot_hostname="192.168.1.100"/g' ~/ros_ws/intera.sh
# ==============================================================================

RUN /ros_entrypoint.sh rosdep update

# Set GAZEBO_MODEL_PATH as environment variable (available to all processes)
# Use absolute path to ensure it's always correct
ENV GAZEBO_MODEL_PATH=/home/kid/ros_ws/src/custom/environments/models

RUN echo "export GAZEBO_MODEL_PATH=/home/kid/ros_ws/src/custom/environments/models:\$GAZEBO_MODEL_PATH" >> ~/.bashrc
RUN echo "if [ -f ~/ros_ws/devel/setup.bash ]; then source ~/ros_ws/devel/setup.bash; fi" >> ~/.bashrc

# ==============================================================================
# ROBOT API - Production mode (copy into image)
# ==============================================================================
# For development: mount robot_api with docker-compose (see docker-compose.yml)
