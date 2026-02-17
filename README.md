# Setup

## Host Environment
```bash
conda env create -f environment.yml
conda activate robot_client
pip install -e .
```

## Container

Start (builds automatically):
```bash
docker-compose up -d robot_dev
```

Rebuild:
```bash
docker-compose up -d --build robot_dev
```

Attach:
```bash
docker exec -it robo2025_dev bash
```

Run server inside container:
```bash
python3 /home/kid/ros_ws/src/custom/robot_api/servers/zmq_server.py
```

Stop:
```bash
docker-compose down
```

## Robot Configuration

Set robot IP before starting container:
```bash
HOST_IP=192.168.1.100 ROBOT_IP=192.168.1.103 docker-compose up -d robot_dev
```

Default: HOST_IP=192.168.1.100, ROBOT_IP=192.168.1.103

## Usage

Keyboard control (1q 2w 3e 4r 5t 6y 7u for joints, oc for gripper):
```bash
python src/robot_client/examples/keyboard_control.py
```

Menu-based control:
```bash
python src/robot_client/examples/interactive_control.py
```

Camera capture (press 's' to save, 'q' to quit):
```bash
python src/robot_client/examples/camera_capture.py
```

Full robot demo (lights, head, display, camera):
```bash
python src/robot_client/examples/robot_demo.py
```

Display robot configuration and state:
```bash
python src/robot_client/examples/robot_info.py
```

Programmatic:
```python
from robot_client import RobotClient

robot = RobotClient(protocol='zmq', host='localhost')
robot.move_to_joints([0, 0.3, 0.5, 0, 0, 0, 0])
```

