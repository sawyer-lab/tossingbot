FROM ros:noetic-perception

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

# Install Python 3 pip and development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-dev \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install only essential ROS Python packages
RUN python3 -m pip install \
    rospkg \
    catkin_pkg \
    numpy \
    pyyaml \
    empy

# Install JetBrains Mono fonts
RUN wget -q https://github.com/JetBrains/JetBrainsMono/releases/download/v2.304/JetBrainsMono-2.304.zip -O /tmp/JetBrainsMono.zip \
    && unzip -q /tmp/JetBrainsMono.zip -d /tmp/JetBrainsMono \
    && mkdir -p /usr/share/fonts/truetype/jetbrains-mono \
    && cp /tmp/JetBrainsMono/fonts/ttf/*.ttf /usr/share/fonts/truetype/jetbrains-mono/ \
    && fc-cache -fv \
    && rm -rf /tmp/JetBrainsMono.zip /tmp/JetBrainsMono

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
    ros-${ROS_DISTRO}-qt-build \
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
RUN echo "if [ -f /home/$USER/ros_ws/devel/setup.bash ]; then source /home/$USER/ros_ws/devel/setup.bash; fi" >> /home/$USER/.bashrc

COPY init_ws.sh /home/kid/init_ws.sh
COPY init_robot.sh /home/kid/init_robot.sh
RUN chmod +x /home/kid/init_ws.sh /home/kid/init_robot.sh

USER kid
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws
RUN /ros_entrypoint.sh catkin_make

RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool init .
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_robot.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_simulator.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sns_ik.git -b noetic-devel

RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool merge sawyer_simulator/sawyer_simulator.rosinstall
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool update

# ==============================================================================
# NETWORK CONFIGURATION - HARDCODED
# ==============================================================================
RUN cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/.

# Hardcoded values (no ARG, no override possible)
RUN sed -i 's/ros_version=".*"/ros_version="noetic"/g' ~/ros_ws/intera.sh && \
    sed -i 's/your_ip="192.168.XXX.XXX"/your_ip="192.168.1.100"/g' ~/ros_ws/intera.sh && \
    sed -i 's/my_computer/rog/g' ~/ros_ws/intera.sh && \
    sed -i 's/robot_hostname="robot_hostname.local"/robot_hostname="192.168.1.103"/g' ~/ros_ws/intera.sh

# Verify configurations applied correctly
RUN echo "Verifying intera.sh configuration:" && \
    grep "^your_ip=" ~/ros_ws/intera.sh && \
    grep "^robot_hostname=" ~/ros_ws/intera.sh && \
    echo "Configuration verified successfully"
# ==============================================================================

RUN /ros_entrypoint.sh rosdep update

RUN ln -s /simulation ~/ros_ws/src/.
# RUN ln -s /perception_pkg ~/ros_ws/src/.
RUN ln -s /custom_sawyer_description ~/ros_ws/src/.
RUN ln -s /custom_sawyer_gazebo ~/ros_ws/src/.
RUN ln -s /environments ~/ros_ws/src/.
RUN ln -s /pneumatic_gripper_description ~/ros_ws/src/.
RUN ln -s /depth_perception ~/ros_ws/src/.
RUN ln -s /.vscode ~/ros_ws/src/.
RUN ln -s /plain_perception ~/ros_ws/src/.
RUN ln -s /grasping ~/ros_ws/src/.
RUN ln -s /tossing_system ~/ros_ws/src/.
RUN ln -s /test_scripts ~/.

RUN /ros_entrypoint.sh catkin_make
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/ros_ws/src/environments/models" >> ~/.bashrc
RUN echo "source devel/setup.bash" >> ~/.bashrc