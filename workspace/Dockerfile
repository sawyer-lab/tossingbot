FROM ros:melodic-perception

RUN apt update && apt install -y --no-install-recommends \
    curl \
    git \
    tmux \
    vim-nox \
    && rm -rf /var/lib/apt/lists/*

# https://support.rethinkrobotics.com/support/solutions/articles/80000980134-workstation-setup
# From https://sdk.rethinkrobotics.com/intera/Workstation_Setup
RUN apt update && apt install --no-install-recommends -y \
    build-essential \
    python-argparse \
    python-pip \
    python-rosdep \
    python-rosinstall \
    python-rosinstall-generator \
    python-vcstools \
    python-wstool \
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
    && rm -rf /var/lib/apt/lists/*

RUN apt update && apt install --no-install-recommends -y \
    ros-${ROS_DISTRO}-desktop-full \
    && rm -rf /var/lib/apt/lists/*

# From https://support.rethinkrobotics.com/support/solutions/articles/80000980381-gazebo-tutorial
RUN apt update && apt install --no-install-recommends -y \
    gazebo9 ros-melodic-qt-build ros-melodic-gazebo-ros-control \
    ros-melodic-gazebo-ros-pkgs ros-melodic-ros-control ros-melodic-control-toolbox ros-melodic-realtime-tools \
    ros-melodic-ros-controllers ros-melodic-xacro python-wstool ros-melodic-tf-conversions ros-melodic-kdl-parser  \
    && rm -rf /var/lib/apt/lists/*

## state-publisher was not listed
# ros-${ROS_DISTRO}-joystick-drivers \
# ros-${ROS_DISTRO}-robot-state-publisher \

ARG USER=kid
ARG UID=1000
ARG GID=1000

RUN groupadd -g $GID -o $USER
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $USER

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc
RUN echo "if [ -f /home/$USER/ros_ws/devel/setup.bash ]; then source /home/$USER/ros_ws/devel/setup.bash; fi" >> /home/$USER/.bashrc

#ENTRYPOINT ["/ros_entrypoint.sh"]

USER kid
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws
RUN /ros_entrypoint.sh catkin_make
RUN ln -s /cinves_perception ~/ros_ws/src/.
RUN cd ~/ros_ws/src; /ros_entrypoint.sh catkin_create_pkg test_keny rospy roscpp
RUN /ros_entrypoint.sh catkin_make

