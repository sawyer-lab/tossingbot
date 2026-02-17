# RGBD Camera Examples

Demonstration scripts for the RGBD camera and contact sensor interfaces.

## Camera Demos

### camera_demo.py
Interactive demo showing point cloud capture and visualization.

**Features:**
- On-demand point cloud capture
- Streaming mode enable/disable
- Matplotlib 3D visualization
- 2D depth/color projection

**Usage:**
```bash
python camera_demo.py
```

**Requirements:**
- matplotlib (optional, for visualization)

**Output:**
- Console statistics about point cloud
- 3D scatter plot (subsampled)
- 2D depth and color projections

---

### camera_stream_viewer.py
Real-time OpenCV-based camera viewer.

**Features:**
- Live depth and color visualization
- FPS counter
- Snapshot saving
- Pause/resume

**Usage:**
```bash
python camera_stream_viewer.py
```

**Controls:**
- `q` - Quit
- `s` - Save snapshot
- `SPACE` - Pause/resume

**Requirements:**
- opencv-python (required)

**Output:**
- Live window showing depth (left) and color (right)
- Saved snapshots: `snapshot_XXXX.png`

---

## Sensor Demos

### landing_detection_demo.py
Demonstrates contact sensor for object landing detection.

**Features:**
- Spawn and drop test object
- Wait for landing with timeout
- Query landing position
- Enable/disable detection

**Usage:**
```bash
python landing_detection_demo.py
```

**Output:**
- Landing position and object name
- Timestamp of contact event

---

### gazebo_demo.py
Full Gazebo object management demonstration.

**Features:**
- Spawn objects with colors
- Randomized batch spawning
- Pose queries and updates
- Cleanup operations

**Usage:**
```bash
python gazebo_demo.py
```

---

## Installation

### Minimal (no visualization)
```bash
pip install zmq
```

### With matplotlib visualization
```bash
pip install zmq matplotlib
```

### With OpenCV streaming
```bash
pip install zmq opencv-python
```

### Full installation
```bash
pip install zmq matplotlib opencv-python numpy
```

---

## Notes

- All demos connect to `localhost:5555` by default
- Gazebo must be running in the container
- ZMQ server must be active
- RGBD cameras must be configured in Gazebo for camera demos
- Contact sensor topic must exist for landing detection
