xhost +
docker run -d --rm \
	--network host \
	-e DISPLAY=$DISPLAY \
	-v /tmp/.X11-unix/:/tmp/.X11-unix/ \
	--mount type=bind,source=./ros_src/cinves_perception,target=/cinves_perception \
	--name robo2025 \
	-it \
	--privileged \
	robo2025-workspace:latest "$@"


# --device=/dev/bus/usb/003/002 \
