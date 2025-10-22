#!/bin/bash

xhost +local:root

docker run -d --rm \
    --runtime=nvidia \
    --gpus all \
    --network host \
    --privileged \
    -e DISPLAY=$DISPLAY \
    -e TERM \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __VK_LAYER_NV_optimus=NVIDIA_only \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --mount type=bind,source=./ros_src/perception_pkg,target=/perception_pkg \
    --mount type=bind,source=./ros_src/simulation,target=/simulation \
    --mount type=bind,source=./ros_src/planning,target=/planning \
    --mount type=bind,source=./ros_src/pneumatic_gripper_description,target=/pneumatic_gripper_description \
    --name robo2025 \
    -it \
    robo2025-workspace:latest "$@"

# --privileged \
# AMD hardware acceleration did not work
# --device=/dev/dri \ --group-add video \
