#!/bin/bash

xhost +local:root

docker run -d --rm \
    --runtime=nvidia \
    --gpus all \
    --network host \
    -e DISPLAY=$DISPLAY \
    -e NVIDIA_DRIVER_CAPABILITIES=all \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e __NV_PRIME_RENDER_OFFLOAD=1 \
    -e __GLX_VENDOR_LIBRARY_NAME=nvidia \
    -e __VK_LAYER_NV_optimus=NVIDIA_only \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    --mount type=bind,source=./ros_src/cinves_perception,target=/cinves_perception \
    --name robo2025 \
    -it \
    --privileged \
    robo2025-workspace:latest "$@"
