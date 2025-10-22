#!/bin/bash
set -e

cd ~/ros_ws/src

# Merge the .rosinstall if available
if [ -f /sawyer_simulator/sawyer_simulator.rosinstall ]; then
    echo "[INFO] Merging sawyer_simulator .rosinstall..."
    /ros_entrypoint.sh wstool merge /sawyer_simulator/sawyer_simulator.rosinstall
    /ros_entrypoint.sh wstool update
fi

# Copy and patch intera.sh if available
if [ -f ~/ros_ws/src/intera_sdk/intera.sh ]; then
    cp ~/ros_ws/src/intera_sdk/intera.sh ~/ros_ws/
    sed -i 's/ros_version=".*"/ros_version="melodic"/g' ~/ros_ws/intera.sh
    sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh
    sed -i "s/my_computer/${HOST_HOSTNAME}/g" ~/ros_ws/intera.sh
    sed -i "s/robot_hostname.local/${ROBOT_HOSTNAME}/g" ~/ros_ws/intera.sh
fi

/ros_entrypoint.sh rosdep update

exec "$@"
