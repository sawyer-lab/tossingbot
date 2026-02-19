# Complete Guide: Connecting to Sawyer Robot from Docker

This guide explains **what you need** to connect to a Sawyer robot from a Docker container. Discovered through debugging why robot connection wasn't working.

---

## 🔑 Core Concepts

### The Robot Runs ROS Master
- **Critical**: The robot itself runs `roscore` (ROS Master) at port **11311**
- Your container connects **as a client** to the robot's ROS master
- This is different from typical ROS setups where you run your own roscore

### Network Architecture
```
Your PC (192.168.1.100)
    ↓
Container (shares host network)
    ↓
ROS_MASTER_URI → http://192.168.1.103:11311 (robot)
    ↓
Robot (192.168.1.103)
```

---

## 📋 Required Configuration Items

### 1. **Network IPs** ✅
You need to know:
- **Robot IP**: e.g., `192.168.1.103` (check with `ping` or avahi)
- **Robot Hostname**: e.g., `021607CP00070.local` (check robot label)
- **Your PC's IP** on robot network: e.g., `192.168.1.100`
- **Subnet**: Typically `192.168.1.0/24` (Sawyer default)

**How to find:**
```bash
# On host, find robot
avahi-browse -at | grep -i rethink
avahi-resolve -4 -n <ROBOT_HOSTNAME>.local

# Or scan network
nmap -sn 192.168.1.0/24

# Your PC's IP
ip addr show | grep 192.168.1
```

---

### 2. **Docker: Host Network Mode** ✅
**Why**: Container needs direct access to robot network, can't use bridge.

**docker-compose.yml:**
```yaml
services:
  robot:
    network_mode: host
```

**docker run:**
```bash
docker run --network host ...
```

---

### 3. **Docker: Hostname Mapping** ✅
**Why**: Robot SDK (intera.sh) may use `.local` hostname instead of IP.

**docker-compose.yml:**
```yaml
services:
  robot:
    extra_hosts:
      - "021607CP00070.local:192.168.1.103"  # Maps hostname → IP
```

**docker run:**
```bash
docker run --add-host "021607CP00070.local:192.168.1.103" ...
```

**What it does**: Adds entry to container's `/etc/hosts` so hostname resolves to IP.

---

### 4. **ROS Environment Variables** ✅
**Critical exports needed:**

```bash
export ROS_MASTER_URI=http://192.168.1.103:11311  # Robot's roscore
export ROS_IP=192.168.1.100                        # Your PC's IP
export ROS_HOSTNAME=192.168.1.100                  # Same as ROS_IP
```

**Where to set:**

**Option A: docker-compose.yml**
```yaml
environment:
  - ROS_MASTER_URI=http://192.168.1.103:11311
  - ROS_IP=192.168.1.100
```

**Option B: intera.sh** (Rethink SDK script)
- The `intera.sh` script sets these when sourced
- But you need to configure the variables it uses (see next section)

---

### 5. **Configuring intera.sh** ✅
**The Problem**: Original `intera.sh` has placeholders:
```bash
your_ip="192.168.XXX.XXX"              # ❌ Placeholder
robot_hostname="robot_hostname.local"   # ❌ Placeholder
```

**Solution 1: Hardcode at build time** (Dockerfile)
```dockerfile
RUN sed -i 's/your_ip="192.168.XXX.XXX"/your_ip="192.168.1.100"/g' ~/ros_ws/intera.sh
RUN sed -i 's/robot_hostname="robot_hostname.local"/robot_hostname="192.168.1.103"/g' ~/ros_ws/intera.sh
```

**Solution 2: Patch at runtime** (startup script)
```bash
#!/bin/bash
# init_robot.sh
sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh
sed -i "s/robot_hostname=\".*\"/robot_hostname=\"${ROBOT_IP}\"/g" ~/ros_ws/intera.sh
```
Then pass `HOST_IP` and `ROBOT_IP` as environment variables.

**What intera.sh does when sourced:**
```bash
source ~/ros_ws/intera.sh
# Internally exports:
# - ROS_MASTER_URI=http://${robot_hostname}:11311
# - ROS_IP=${your_ip}
```

---

### 6. **ROS Workspace Must Be Built** ✅
**The Issue**: ROS message types (like `intera_core_msgs.msg`) won't be available unless:

1. The ROS workspace containing `intera_sdk` is built with `catkin_make`
2. The workspace is sourced: `source ~/ros_ws/devel/setup.bash`

**If using mounted volumes** (development mode):
```bash
# Inside container, AFTER mounting packages:
cd ~/ros_ws
catkin_make
source devel/setup.bash
```

**If copying packages** (production mode):
```dockerfile
# In Dockerfile, AFTER copying packages:
RUN cd ~/ros_ws && /ros_entrypoint.sh catkin_make
```

---

### 7. **Python Script Requirements** ✅
To use ROS in Python scripts:

```python
#!/usr/bin/env python3
import rospy
from intera_core_msgs.msg import JointCommand  # This requires sourced workspace

# Before running:
# 1. source /opt/ros/noetic/setup.bash
# 2. source ~/ros_ws/devel/setup.bash
# 3. source ~/ros_ws/intera.sh
```

**Wrapper script approach:**
```bash
#!/bin/bash
# start_robot_server.sh
source /opt/ros/noetic/setup.bash
source ~/ros_ws/devel/setup.bash
source ~/ros_ws/intera.sh
exec python3 your_script.py
```

---

## 🔍 How to Verify Connection

### Step 1: Check Network (from host)
```bash
ping 192.168.1.103          # Robot should respond
avahi-resolve -4 -n 021607CP00070.local  # Should show IP
```

### Step 2: Check from Container
```bash
docker exec -it <container> bash
ping 192.168.1.103          # Should work
ping 021607CP00070.local    # Should work (thanks to --add-host)
```

### Step 3: Check ROS Environment
```bash
# Inside container
source ~/ros_ws/intera.sh
echo $ROS_MASTER_URI        # Should show http://192.168.1.103:11311
echo $ROS_IP                # Should show 192.168.1.100
```

### Step 4: Test ROS Connection
```bash
source ~/ros_ws/intera.sh
rostopic list               # Should show robot topics like /robot/joint_states
rostopic echo /robot/joint_states  # Should see live data
```

---

## 🚨 Common Mistakes & Fixes

| Problem | Symptom | Fix |
|---------|---------|-----|
| **Missing --add-host** | `ping: 021607CP00070.local: Name or service not known` | Add `extra_hosts` to docker-compose.yml |
| **Wrong ROS_MASTER_URI** | `rostopic list` shows nothing or localhost topics only | Set to `http://<robot_ip>:11311` not `localhost` |
| **intera.sh not configured** | `your_ip is EMPTY` error | Hardcode IPs or patch at runtime |
| **Workspace not built** | `ModuleNotFoundError: No module named 'intera_core_msgs'` | Run `catkin_make` and source workspace |
| **Workspace not sourced** | Same as above | Add `source devel/setup.bash` to .bashrc or run manually |
| **Bridge network mode** | Can't reach robot even though host can | Use `network_mode: host` |
| **Wrong subnet** | Robot unreachable | Check ethernet interface has 192.168.1.x IP |

---

## 📝 Checklist for Any Robot Connection Setup

- [ ] Robot is powered on and fully booted (2-5 min)
- [ ] Ethernet cable connected to robot network
- [ ] Host PC has IP on robot subnet (192.168.1.x)
- [ ] Container uses `network_mode: host`
- [ ] Container has `--add-host` or `extra_hosts` for robot hostname
- [ ] `ROS_MASTER_URI` points to robot IP:11311, not localhost
- [ ] `ROS_IP` is set to host's IP on robot network
- [ ] `intera.sh` has correct `your_ip` and `robot_hostname`
- [ ] ROS workspace with intera_sdk is built (`catkin_make`)
- [ ] Workspace is sourced before running ROS scripts
- [ ] Can ping robot from container
- [ ] `rostopic list` shows robot topics

---

## 💡 Key Insights

1. **The robot IS the ROS master** - you're not running your own roscore
2. **Port 11311 is hardcoded** - this is ROS's default master port
3. **Two-way communication** - Robot needs to know your IP (ROS_IP), you need robot's IP (ROS_MASTER_URI)
4. **Hostname mapping is critical** - SDK often uses `.local` hostnames
5. **intera.sh doesn't auto-configure** - you MUST set the IPs yourself
6. **Build order matters** - catkin_make must happen AFTER packages are available

---

## 🎯 Minimal Working Example

**docker-compose.yml:**
```yaml
services:
  robot:
    image: ros:noetic
    network_mode: host
    privileged: true
    extra_hosts:
      - "021607CP00070.local:192.168.1.103"
    environment:
      - ROS_MASTER_URI=http://192.168.1.103:11311
      - ROS_IP=192.168.1.100
    command: /bin/bash
```

**Inside container:**
```bash
# Clone and build intera_sdk
cd ~/ros_ws/src
git clone https://github.com/RethinkRobotics/intera_sdk
cd ..
catkin_make

# Configure intera.sh
sed -i 's/your_ip="192.168.XXX.XXX"/your_ip="192.168.1.100"/' ~/ros_ws/intera.sh
sed -i 's/robot_hostname=".*"/robot_hostname="192.168.1.103"/' ~/ros_ws/intera.sh

# Use it
source devel/setup.bash
source intera.sh
rostopic list
```

---

## 📚 References

- **ROS_MASTER_URI**: http://wiki.ros.org/ROS/EnvironmentVariables
- **ROS_IP vs ROS_HOSTNAME**: http://wiki.ros.org/ROS/NetworkSetup
- **Sawyer SDK**: https://github.com/RethinkRobotics/intera_sdk
- **Docker networking**: https://docs.docker.com/network/host/

---

**Created**: Based on debugging actual robot connection issues where missing `--add-host`, wrong `ROS_MASTER_URI`, and unbuild workspace prevented connection.
