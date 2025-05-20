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
    gazebo9 \
    python-wstool \
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

## state-publisher was not listed
# ros-${ROS_DISTRO}-joystick-drivers \
# ros-${ROS_DISTRO}-robot-state-publisher \

# From https://wiki.ros.org/docker/Tutorials/Hardware%20Acceleration
# for AMD
# but it didn't work in this image
#RUN \
#  apt-get update && \
#  apt-get -y install libgl1-mesa-glx libgl1-mesa-dri && \
#  rm -rf /var/lib/apt/lists/*

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
RUN ln -s /test_keny ~/ros_ws/src/.
RUN ln -s /ros_tutorials_practice ~/ros_ws/src/.
RUN cd ~/ros_ws/src; /ros_entrypoint.sh catkin_create_pkg test_keny2 rospy roscpp
RUN /ros_entrypoint.sh catkin_make
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool init .
RUN cd ~/ros_ws/src; git clone https://github.com/RethinkRobotics/sawyer_robot.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/RethinkRobotics/sawyer_simulator.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/RethinkRobotics-opensource/sns_ik.git -b melodic-devel
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool merge sawyer_simulator/sawyer_simulator.rosinstall
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool update
RUN /ros_entrypoint.sh catkin_make
RUN cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/.
ARG HOST_HOSTNAME=`hostname`
ARG HOST_IP=172.17.0.1
ARG ROBOT_HOSTNAME=021607CP00070.local
RUN sed -i 's/ros_version=".*"/ros_version="melodic"/g' ~/ros_ws/intera.sh
RUN sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh
RUN sed -i "s/my_computer/${HOST_HOSTNAME}/g" ~/ros_ws/intera.sh
RUN sed -i "s/robot_hostname.local/${ROBOT_HOSTNAME}/g" ~/ros_ws/intera.sh
RUN /ros_entrypoint.sh rosdep update

