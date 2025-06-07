FROM ros:melodic-perception

LABEL maintainer="Robotics Team"
LABEL description="ROS Melodic environment for robotics development with Sawyer robot"
LABEL version="1.0"

# Install basic utilities and development tools
# https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup
# https://sdk.rethinkrobotics.com/intera/Workstation_Setup
RUN apt update && apt install -y --no-install-recommends \
    # Basic utilities
    curl \
    git \
    tmux \
    vim-nox \
    # Development tools
    build-essential \
    python-argparse \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-vcstools \
    python-wstool \
    # ROS packages
    ros-${ROS_DISTRO}-control-msgs \
    ros-${ROS_DISTRO}-xacro \
    ros-${ROS_DISTRO}-tf2-ros \
    ros-${ROS_DISTRO}-rviz \
    ros-${ROS_DISTRO}-cv-bridge \
    ros-${ROS_DISTRO}-actionlib \
    ros-${ROS_DISTRO}-actionlib-msgs \
    ros-${ROS_DISTRO}-dynamic-reconfigure \
    ros-${ROS_DISTRO}-trajectory-msgs \
    ros-${ROS_DISTRO}-rospy-message-converter \
    ros-${ROS_DISTRO}-openni2-launch \
    ros-${ROS_DISTRO}-mocap-optitrack \
    ros-${ROS_DISTRO}-desktop-full \
    # Gazebo and related packages
    # From https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial
    gazebo9 \
    ros-${ROS_DISTRO}-qt-build \
    ros-${ROS_DISTRO}-gazebo-ros-control \
    ros-${ROS_DISTRO}-gazebo-ros-pkgs \
    ros-${ROS_DISTRO}-ros-control \
    ros-${ROS_DISTRO}-control-toolbox \
    ros-${ROS_DISTRO}-realtime-tools \
    ros-${ROS_DISTRO}-ros-controllers \
    ros-${ROS_DISTRO}-tf-conversions \
    ros-${ROS_DISTRO}-kdl-parser \
    && rm -rf /var/lib/apt/lists/*

# Configure NVIDIA Prime for GPU support
RUN echo 'export __NV_PRIME_RENDER_OFFLOAD=1' >> /etc/bash.bashrc && \
    echo 'export __GLX_VENDOR_LIBRARY_NAME=nvidia' >> /etc/bash.bashrc && \
    echo 'export __VK_LAYER_NV_optimus=NVIDIA_only' >> /etc/bash.bashrc

# Set up user environment
ARG USER=kid
ARG UID=1000
ARG GID=1000
# Create user with specified UID and GID
RUN groupadd -g $GID -o $USER && \
    useradd -m -u $UID -g $GID -o -s /bin/bash $USER && \
    echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc && \
    echo "if [ -f /home/$USER/ros_ws/devel/setup.bash ]; then source /home/$USER/ros_ws/devel/setup.bash; fi" >> /home/$USER/.bashrc

# Switch to the created user
USER ${USER}

# Set up ROS workspace and install required packages
WORKDIR /home/${USER}
RUN mkdir -p /home/${USER}/ros_ws/src && \
    cd /home/${USER}/ros_ws && \
    /ros_entrypoint.sh catkin_make

# Link required packages to the workspace
WORKDIR /home/${USER}/ros_ws
RUN ln -s /cinves_perception ~/ros_ws/src/. && \
    ln -s /turtle_pkg ~/ros_ws/src/. && \
    ln -s /perception_pkg ~/ros_ws/src/. && \
    ln -s /bagfiles ~/ros_ws/src/.

# Create and build packages
RUN cd ~/ros_ws/src && \
    /ros_entrypoint.sh catkin_create_pkg test_keny rospy roscpp && \
    cd .. && \
    /ros_entrypoint.sh catkin_make && \
    cd src && \
    /ros_entrypoint.sh wstool init .

# Clone required repositories
RUN cd ~/ros_ws/src && \
    git clone https://github.com/RethinkRobotics/sawyer_robot.git -b release-5.2.0 && \
    git clone https://github.com/RethinkRobotics/sawyer_simulator.git -b release-5.2.0 && \
    git clone https://github.com/RethinkRobotics-opensource/sns_ik.git -b melodic-devel && \
    /ros_entrypoint.sh wstool merge sawyer_simulator/sawyer_simulator.rosinstall && \
    /ros_entrypoint.sh wstool update

# Build the workspace
RUN cd ~/ros_ws && \
    /ros_entrypoint.sh catkin_make && \
    cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/.

# Configure robot connection
ARG HOST_HOSTNAME=`hostname`
ARG HOST_IP=172.17.0.1
ARG ROBOT_HOSTNAME=021607CP00070.local
RUN sed -i 's/ros_version=".*"/ros_version="melodic"/g' ~/ros_ws/intera.sh && \
    sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh && \
    sed -i "s/my_computer/${HOST_HOSTNAME}/g" ~/ros_ws/intera.sh && \
    sed -i "s/robot_hostname.local/${ROBOT_HOSTNAME}/g" ~/ros_ws/intera.sh && \
    /ros_entrypoint.sh rosdep update
