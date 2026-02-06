#!/bin/bash

xhost +local:root

docker run -d --rm \
    --runtime=nvidia \
    --gpus all \
    --network host \
    --add-host "021607CP00070.local:192.168.1.103" \
    --device=/dev/bus/usb/001/002 \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e TERM \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __VK_LAYER_NV_optimus=NVIDIA_only \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --mount type=bind,source=./ros_src/simulation,target=/simulation \
    --mount type=bind,source=./ros_src/custom_sawyer_description,target=/custom_sawyer_description \
    --mount type=bind,source=./ros_src/custom_sawyer_gazebo,target=/custom_sawyer_gazebo \
    --mount type=bind,source=./ros_src/environments,target=/environments \
    --mount type=bind,source=./ros_src/pneumatic_gripper_description,target=/pneumatic_gripper_description \
    --mount type=bind,source=./ros_src/depth_perception,target=/depth_perception \
    --mount type=bind,source=./ros_src/.vscode,target=/.vscode \
    --mount type=bind,source=./ros_src/plain_perception,target=/plain_perception \
    --mount type=bind,source=./ros_src/grasping,target=/grasping \
    --mount type=bind,source=./ros_src/tossing_system,target=/tossing_system \
    --mount type=bind,source=./test_scripts,target=/test_scripts \
    --name robo2025 \
    -it \
    robo2025-workspace:latest "$@"