xhost +
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

# --privileged \
# AMD hardware acceleration did not work
# --device=/dev/dri \ --group-add video \
