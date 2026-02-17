#!/bin/bash
# Quick fix for Gazebo API issues in running container
# Run this INSIDE the container: docker exec -it robo2025_dev bash quick_fix.sh

echo "=== Fixing NumPy/SciPy compatibility ==="
pip3 install --upgrade scipy

echo ""
echo "=== Setting GAZEBO_MODEL_PATH ==="
export GAZEBO_MODEL_PATH=/home/kid/ros_ws/src/custom/environments/models
echo "GAZEBO_MODEL_PATH set to: $GAZEBO_MODEL_PATH"

echo ""
echo "=== Verifying Python imports ==="
python3 -c "from environment.gazebo_object_manager import GazeboObjectManager; print('✓ Import successful')"

echo ""
echo "=== Verifying model path ==="
ls -la /home/kid/ros_ws/src/custom/environments/models/ | head -10

echo ""
echo "=== Testing Gazebo model discovery ==="
python3 -c "
import os
models_path = '/home/kid/ros_ws/src/custom/environments/models'
models = [d for d in os.listdir(models_path) if os.path.isdir(os.path.join(models_path, d))]
print(f'Found {len(models)} models: {models[:5]}...')
"

echo ""
echo "==================================================================="
echo "Quick fix complete!"
echo "Now you can run: python3 /robot_api/servers/zmq_server.py"
echo "==================================================================="
