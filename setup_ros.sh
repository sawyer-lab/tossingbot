#!/bin/bash
set -e

# Source the ROS base setup
source "/opt/ros/noetic/setup.bash"

# Workspace setup
WORKSPACE_DIR="/home/kid/ros_ws"
cd $WORKSPACE_DIR

# Build if needed (non-blocking, keeps container running)
# We use -DCMAKE_BUILD_TYPE=Release for performance
if [ ! -f "devel/setup.bash" ]; then
    echo "First run: Building ROS workspace..."
    catkin_make -DCMAKE_BUILD_TYPE=Release
else
    # Quick build to pick up new files without a full re-build
    catkin_make
fi

# Source the local workspace
source "devel/setup.bash"

# Finalize environment variables
export GAZEBO_MODEL_PATH=$WORKSPACE_DIR/src/custom/tossingbot_environments/models:$GAZEBO_MODEL_PATH
export DISABLE_ROS1_EOL_WARNINGS=1

# Execute the command passed to the container
exec "$@"
