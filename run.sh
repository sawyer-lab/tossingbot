#!/bin/bash

xhost +local:root

docker run -d --rm \
    --runtime=nvidia \
    --gpus all \
    --network host \
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
    --name robo2025 \
    -it \
    robo2025-workspace:latest "$@"

# --privileged \
# AMD hardware acceleration did not work
# --device=/dev/dri \ --group-add video \
