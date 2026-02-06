# Sawyer Robot Connection Notes

> **Quick Start:** See [TESTING_PLAN.md](TESTING_PLAN.md) for complete step-by-step testing procedure

## Robot Information
- **Hostname**: `021607CP00070.local` (configured correctly in Dockerfile)
- **IPv4 Address**: `192.168.1.101`
- **MAC Address**: `18:66:da:00:79:d8`
- **Avahi Service**: `_rethink-robot._tcp`

## Host Network Configuration
- **Ethernet Interface**: `enp0s13f0u4c2`
- **Host IP on Ethernet**: `192.168.1.100/24`
- **WiFi Interface**: `wlan0` (IP: `10.1.1.30/24` - different network)

## Connection Requirements
1. **Modem must be ON** (controls the 192.168.1.x network)
2. **Ethernet cable connected BEFORE robot boots** - the host must be connected when the robot starts up, not after
3. **Robot must be powered on and fully booted**
4. **Disconnect from WiFi** recommended to avoid routing issues

## Verify Connection (from HOST)
```bash
# Check ethernet interface is up with IP
ip addr show enp0s13f0u4c2

# Discover robot via avahi
avahi-browse -at | grep -i rethink

# Resolve robot hostname to IPv4
avahi-resolve -4 -n 021607CP00070.local

# Ping robot
ping -c 3 192.168.1.101
ping -c 3 021607CP00070.local
```

## Docker Container Issues

### Problem 1: mDNS doesn't work inside container (FIXED)
The container didn't have avahi installed, so `.local` hostnames didn't resolve inside it.
- **FIX APPLIED**: Added network tools to Dockerfile (ping, avahi-utils, avahi-daemon, etc.)
- **Rebuild required**: Run `./build.sh` to rebuild the container with these tools

### Problem 2: intera.sh still fails even with IP
When replacing the hostname with IP (`192.168.1.101`) in `~/ros_ws/intera.sh`, it still showed:
```
EXITING - Please edit this file, modifying the 'robot_hostname' variable to reflect your Robot's current hostname.
```

### TODO: Investigate
- Check what validation intera.sh does (might check more than just hostname)
- May need to install avahi-daemon in the container
- May need to configure additional environment variables
- Check if there's a network connectivity check that's failing

## Commands to Edit intera.sh Inside Container
```bash
# Enter container
./exec.sh

# Edit intera.sh (try these options)
sed -i 's/robot_hostname=".*"/robot_hostname="192.168.1.101"/' ~/ros_ws/intera.sh
# OR
vi ~/ros_ws/intera.sh

# Look for these variables to check/modify:
# - robot_hostname
# - your_ip
# - ros_version
```

## Dockerfile Build Args
The Dockerfile accepts these build arguments:
- `HOST_IP` (default: 172.17.0.1) - set at build time to host's IP
- `HOST_HOSTNAME` - set at build time to host's hostname
- `ROBOT_HOSTNAME` (default: 021607CP00070.local)

**FIXED**: `build.sh` now automatically detects ethernet IP (192.168.1.100) and uses it.

To rebuild with automatic settings:
```bash
./build.sh
```

Or manually specify:
```bash
docker build \
    --build-arg HOST_HOSTNAME=$(hostname) \
    --build-arg HOST_IP=192.168.1.100 \
    --build-arg ROBOT_HOSTNAME=021607CP00070.local \
    -t robo2025-workspace workspace
```

## How intera.sh is Configured (from init_ws.sh)
The script `workspace/init_ws.sh` patches intera.sh with these sed commands:
```bash
sed -i 's/ros_version=".*"/ros_version="melodic"/g' ~/ros_ws/intera.sh
sed -i "s/your_ip=\".*\"/your_ip=\"${HOST_IP}\"/g" ~/ros_ws/intera.sh
sed -i "s/my_computer/${HOST_HOSTNAME}/g" ~/ros_ws/intera.sh
sed -i "s/robot_hostname.local/${ROBOT_HOSTNAME}/g" ~/ros_ws/intera.sh
```

**Important**: The sed command looks for `robot_hostname.local` (literal), not the variable.
This might not match the actual content of the original intera.sh file from intera_sdk.

## COMPLETE TESTING PLAN

I've created a comprehensive test suite in `/test_scripts/` to gather all information in one robot power cycle.

### Quick Start (Inside Container)

**Option 1: Run everything automatically**
```bash
cd /test_scripts
./run_all_tests.sh
```

**Option 2: Run tests individually**
```bash
cd /test_scripts

# BEFORE turning on robot
./01_pre_robot_tests.sh

# Optional: Add debug output to intera.sh
./03_add_debug_to_intera.sh

# Turn on robot, then run:
./02_wait_for_robot.sh

# After robot is detected:
./04_post_robot_tests.sh

# To see complete intera.sh content:
./dump_intera_info.sh
```

### Test Scripts Overview

1. **01_pre_robot_tests.sh**
   - Checks network tools availability (ping, avahi, etc.)
   - Verifies container network configuration
   - Shows current intera.sh configuration
   - Tests gateway connectivity
   - Run BEFORE turning on robot

2. **02_wait_for_robot.sh**
   - Monitors multiple detection methods (ping IP, ping hostname, avahi)
   - Shows real-time status of each detection method
   - Auto-detects when robot comes online
   - Run AFTER turning on robot

3. **03_add_debug_to_intera.sh**
   - Adds debug print statements to intera.sh
   - Creates backup of original
   - Shows exactly what values are set and where it fails
   - Optional but recommended

4. **04_post_robot_tests.sh**
   - Comprehensive connectivity tests
   - ROS port scanning
   - Attempts to source intera.sh
   - Lists ROS topics if successful
   - Checks for key robot topics
   - Run AFTER robot is detected

5. **dump_intera_info.sh**
   - Shows complete intera.sh content with line numbers
   - Extracts all variables and conditions
   - Helpful for understanding validation logic

### Expected Timeline

1. Start container, run pre-tests (30 seconds)
2. Turn on robot (you do this)
3. Robot boots (2-5 minutes) - wait_for_robot.sh monitors this
4. Robot detected, run post-tests (1 minute)
5. Review all output

### What Information Will Be Collected

- ✅ Container network configuration
- ✅ Tool availability
- ✅ Current intera.sh configuration values
- ✅ Robot detection timing (how long to boot)
- ✅ Which detection methods work (ping vs avahi)
- ✅ Robot ROS port status (research vs SDK mode)
- ✅ intera.sh execution output with debug info
- ✅ ROS topic availability
- ✅ Complete intera.sh script content
- ✅ Exact failure point if intera.sh fails
