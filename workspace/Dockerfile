FROM ros:melodic-perception

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

RUN curl https://bootstrap.pypa.io/pip/2.7/get-pip.py --output get-pip.py && \
python2 get-pip.py && \
rm get-pip.py

# Now install legacy libraries
# We use 'python2 -m pip' to be explicit
RUN python2 -m pip install --upgrade "pip<21.0"
RUN python2 -m pip install numpy
RUN python2 -m pip install --no-cache-dir \
    casadi==3.5.5 \
    scipy==1.2.3 \
    matplotlib==2.2.5 
    
# 1. Install Python 3.8 from PPA
RUN apt-get update && apt-get install -y software-properties-common && \
    add-apt-repository ppa:deadsnakes/ppa && \
    apt-get update && \
    apt-get install -y \
    python3.8 \
    python3.8-dev \
    python3.8-distutils \
    curl \
    && rm -rf /var/lib/apt/lists/*

# Install system libraries required by OpenCV (and Open3D)
RUN apt-get update && apt-get install -y \
    libgl1-mesa-glx \
    libglib2.0-0 \
    && rm -rf /var/lib/apt/lists/*

# 2. Install Pip for Python 3.8 (Legacy URL)
RUN curl https://bootstrap.pypa.io/pip/3.8/get-pip.py -o get-pip.py && python3.8 get-pip.py && rm get-pip.py

# 3. Install Modern PyTorch (CUDA 12.1) - CRITICAL UPDATE
RUN python3.8 -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121

# 4. Install Build Dependencies for ROS
RUN python3.8 -m pip install \
    rospkg \
    catkin_pkg \
    numpy \
    pyyaml \
    empy==3.3.4 \
    open3d \
    stable-baselines3 \
    gymnasium \
    tensorboard \
    shimmy \ 
    opencv-python \ 
    casadi \
    scipy \
    matplotlib

# Install ros_numpy directly from GitHub (since it's not on PyPI)
RUN python3.8 -m pip install git+https://github.com/eric-wieser/ros_numpy.git
# -----------------------------------------------------------------------
# 5. Build cv_bridge for Python 3.8
# -----------------------------------------------------------------------
RUN mkdir -p /cv_bridge_ws/src && \
    cd /cv_bridge_ws/src && \
    git clone https://github.com/ros-perception/vision_opencv.git && \
    cd vision_opencv && \
    git checkout melodic && \
    cd /cv_bridge_ws && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
                -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
                -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so"

# -----------------------------------------------------------------------
# 6. Build geometry2 (tf2_ros) for Python 3.8
# -----------------------------------------------------------------------
RUN mkdir -p /geometry2_ws/src && \
    cd /geometry2_ws/src && \
    git clone https://github.com/ros/geometry2.git && \
    cd geometry2 && \
    git checkout melodic-devel && \
    touch tf2_kdl/CATKIN_IGNORE && \
    touch tf2_bullet/CATKIN_IGNORE && \
    touch test_tf2/CATKIN_IGNORE && \
    cd /geometry2_ws && \
    /bin/bash -c "source /opt/ros/melodic/setup.bash && \
    catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3.8 \
                -DPYTHON_INCLUDE_DIR=/usr/include/python3.8 \
                -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.8.so"

                
# Install JetBrains Mono fonts
RUN wget -q https://github.com/JetBrains/JetBrainsMono/releases/download/v2.304/JetBrainsMono-2.304.zip -O /tmp/JetBrainsMono.zip \
    && unzip -q /tmp/JetBrainsMono.zip -d /tmp/JetBrainsMono \
    && mkdir -p /usr/share/fonts/truetype/jetbrains-mono \
    && cp /tmp/JetBrainsMono/fonts/ttf/*.ttf /usr/share/fonts/truetype/jetbrains-mono/ \
    && fc-cache -fv \
    && rm -rf /tmp/JetBrainsMono.zip /tmp/JetBrainsMono

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
    python-numpy \
    python-opencv \
    python-tk \
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



ARG USER=kid
ARG UID=1000
ARG GID=1000
RUN groupadd -g $GID -o $USER
RUN useradd -m -u $UID -g $GID -o -s /bin/bash $USER

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> /home/$USER/.bashrc
RUN echo "if [ -f /home/$USER/ros_ws/devel/setup.bash ]; then source /home/$USER/ros_ws/devel/setup.bash; fi" >> /home/$USER/.bashrc

COPY init_ws.sh /home/kid/init_ws.sh
RUN chmod +x /home/kid/init_ws.sh

USER kid
RUN mkdir -p /home/$USER/ros_ws/src
WORKDIR /home/$USER/ros_ws
RUN /ros_entrypoint.sh catkin_make




RUN /ros_entrypoint.sh catkin_make
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool init .
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_robot.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sawyer_simulator.git -b release-5.2.0
RUN cd ~/ros_ws/src; git clone https://github.com/sawyer-lab/sns_ik.git -b melodic-devel

RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool merge sawyer_simulator/sawyer_simulator.rosinstall
RUN cd ~/ros_ws/src; /ros_entrypoint.sh wstool update
RUN cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/.
ARG HOST_HOSTNAME=hostname
ARG HOST_IP=172.17.0.1
ARG ROBOT_HOSTNAME=021607CP00070.local
RUN sed -i 's/ros_version=".*"/ros_version="melodic"/g' ~/ros_ws/intera.sh
RUN sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh
RUN sed -i "s/my_computer/${HOST_HOSTNAME}/g" ~/ros_ws/intera.sh
RUN sed -i "s/robot_hostname.local/${ROBOT_HOSTNAME}/g" ~/ros_ws/intera.sh
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
RUN ln -s /tossing ~/ros_ws/src/.
RUN ln -s /grasping ~/ros_ws/src/.
RUN ln -s /tossing_system ~/ros_ws/src/.

RUN /ros_entrypoint.sh catkin_make
RUN echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:~/ros_ws/src/environments/models" >> ~/.bashrc
RUN echo "source devel/setup.bash" >> ~/.bashrc