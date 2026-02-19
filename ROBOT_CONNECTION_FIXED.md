# Robot Connection Setup - Fixed

## What Was Missing

The latest commit was missing critical robot connection configuration:

1. ❌ **No hostname mapping** - `--add-host` for robot hostname resolution
2. ❌ **Wrong ROS_MASTER_URI** - pointed to localhost instead of robot IP
3. ❌ **Missing convenience scripts** - build.sh, run.sh, exec.sh

## What Was Fixed

### 1. docker-compose.yml
- ✅ Added `extra_hosts` to map `021607CP00070.local` to robot IP
- ✅ Fixed `ROS_MASTER_URI` to point to `http://192.168.1.103:11311`
- ✅ Environment variables properly configured for robot connection

### 2. Convenience Scripts Created
- ✅ `build.sh` - Build the Docker image
- ✅ `run.sh` - Start the container
- ✅ `exec.sh` - Enter the container
- ✅ `check_robot.sh` - Verify robot connection

## Quick Start

### First Time Setup
```bash
# 1. Build the image
./build.sh

# 2. Start the container
./run.sh

# 3. Enter the container
./exec.sh

# 4. Inside container - connect to robot
source ~/ros_ws/intera.sh
rostopic list
```

### Check Connection
```bash
# From host
./check_robot.sh
```

## Configuration

**Robot IPs (can override with environment variables):**
- Robot IP: `192.168.1.103` (set `ROBOT_IP=...`)
- Host IP: `192.168.1.100` (set `HOST_IP=...`)
- Robot Hostname: `021607CP00070.local`

**Override example:**
```bash
ROBOT_IP=192.168.1.101 ./run.sh
```

## How It Works

1. **Build time** (Dockerfile):
   - Hardcodes default IPs into `intera.sh`
   - Installs ROS packages and robot SDK

2. **Runtime** (docker-compose + init_robot.sh):
   - `docker-compose.yml` passes environment variables
   - `init_robot.sh` patches `intera.sh` with runtime IPs
   - Sourced automatically via `.bashrc`

3. **Connection**:
   - `extra_hosts` maps hostname to IP in container's `/etc/hosts`
   - `ROS_MASTER_URI` points to robot's roscore at port 11311
   - `intera.sh` exports ROS environment variables when sourced

## Network Requirements

- Modem must be ON (controls 192.168.1.x network)
- Ethernet cable connected to robot network
- Robot powered on and fully booted
- Host on 192.168.1.100, Robot on 192.168.1.103
