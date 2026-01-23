# Host Control Test Setup

## What was created:
- `host_control/` directory outside of ros_src
- Python virtual environment with `roslibpy` installed
- `test_connection.py` - simple test script

## Steps to test:

### 1. Start the Docker container (if not running):
```bash
cd /home/kid/Code/tossingbot
./run.sh
```

### 2. Check if rosbridge_suite is installed in container:
```bash
docker exec robo2025 bash -c "rospack find rosbridge_server"
```

**If NOT installed**, you need to install it:
```bash
docker exec -it robo2025 bash
# Inside container:
apt-get update
apt-get install ros-melodic-rosbridge-suite
exit
```

### 3. Start rosbridge_websocket in the container:
```bash
docker exec robo2025 bash -c "source /opt/ros/melodic/setup.bash && roslaunch rosbridge_server rosbridge_websocket.launch" &
```

Or in a separate terminal:
```bash
./exec.sh
# Inside container:
roslaunch rosbridge_server rosbridge_websocket.launch
```

### 4. Run the test script from host (Ubuntu 24):
```bash
cd /home/kid/Code/tossingbot/host_control
./venv/bin/python test_connection.py
```

## What the test does:
- Connects to ROS via WebSocket (port 9090)
- Subscribes to `/robot/joint_states` to read robot position
- Publishes to `/robot/limb/right/joint_command` to send commands
- Sends a simple "all zeros" position command
- Monitors feedback for 5 seconds

## Expected output:
```
✓ Connected to ROS bridge!
✓ Subscribed to joint states
Waiting 2 seconds for initial state...
Current positions: ['0.000', '0.300', '0.500', ...]
✓ Sending test joint command (all zeros)...
✓ Command sent!
...
```
