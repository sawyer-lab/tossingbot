#!/bin/bash

xhost +local:root

docker run -d --rm \
	--network host \
	-e DISPLAY \
	-e TERM \
	-v /tmp/.X11-unix/:/tmp/.X11-unix/ \
	--mount type=bind,source=./ros_src/cinves_perception,target=/cinves_perception \
	--mount type=bind,source=./ros_src/test_keny,target=/test_keny \
	--mount type=bind,source=./ros_src/ros_tutorials_practice,target=/ros_tutorials_practice \
	--mount type=bind,source=./ros_bags,target=/databags \
	--name robo2025 \
	-it \
	robo2025-workspace:latest "$@"
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
    --mount type=bind,source=./ros_src/turtle_pkg,target=/turtle_pkg \
    --mount type=bind,source=./ros_src/perception_pkg,target=/perception_pkg \
    --mount type=bind,source=./ros_src/bagfiles,target=/bagfiles \
    --name robo2025 \
    -it \
    --privileged \
    robo2025-workspace:latest "$@"

# --privileged \
# AMD hardware acceleration did not work
# --device=/dev/dri \ --group-add video \
