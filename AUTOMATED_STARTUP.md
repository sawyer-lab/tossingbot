# Automated Robot Startup System

## Overview

This system automatically handles all robot connection configuration at runtime, making it robust to hardware changes (USB ports, interface names, IPs) and eliminating manual configuration steps.

## Key Features

### 1. **Dynamic Network Detection**
- Automatically finds the network interface with the robot subnet (192.168.1.x)
- Extracts host IP dynamically
- No hardcoded interface names (handles `enp0s13f0u4c2`, `enp0s13f0u1c2`, etc.)

### 2. **Automatic Firewall Configuration**
- Checks if robot IP is allowed in iptables
- Automatically adds firewall rule: `iptables -I INPUT -s <ROBOT_IP> -j ACCEPT`
- Fixes timeout errors (`OSError: [Errno 110]`)

### 3. **DNS Resolution Fix**
- Uses `--add-host` to map robot hostname to IP
- **Critical**: Allows `rostopic echo` and bidirectional ROS communication
- Without this, robot sends data to its hostname which doesn't resolve

### 4. **Runtime Configuration**
- Passes `HOST_IP`, `ROBOT_IP`, `ROBOT_HOSTNAME` as environment variables
- `init_robot.sh` patches `intera.sh` at container startup
- No need to rebuild image when network changes

### 5. **Passwordless Sudo**
- Container user has full sudo without password prompts
- Needed for avahi-daemon and other system services

## Usage

### Quick Start

```bash
# Start robot and container (auto-configures everything)
./run_robot.sh

# Attach to container (auto-configures intera.sh)
./exec.sh

# Inside container - connect to robot
cd ~/ros_ws && ./intera.sh

# Test
rostopic list
rostopic echo /robot/joint_states
```

### Custom Robot IP/Hostname

```bash
# Specify custom robot IP and hostname
./run_robot.sh 192.168.1.105 sawyer-robot.local
```

## How It Works

### Host Side (`run_robot.sh`)

1. **Detect Network Interface**
   - Scans all interfaces for one with `192.168.1.x` subnet
   - Extracts host IP from that interface

2. **Configure Firewall**
   - Checks if `iptables -C INPUT -s <ROBOT_IP> -j ACCEPT` exists
   - If not, adds the rule with `sudo`

3. **Start Container**
   - Passes environment variables: `HOST_IP`, `ROBOT_IP`, `ROBOT_HOSTNAME`
   - Adds `--add-host` flag for DNS mapping
   - Uses `--network host` for direct network access

4. **Verify Connectivity**
   - Pings robot from container
   - Reports any issues

### Container Side (`init_robot.sh`)

1. **Runtime Configuration**
   - Automatically runs when you attach with `./exec.sh`
   - Reads environment variables (`$HOST_IP`, `$ROBOT_IP`, `$ROBOT_HOSTNAME`)
   - Patches `intera.sh` with actual values using `sed`

2. **Ready to Connect**
   - `intera.sh` now has correct IPs
   - Run `./intera.sh` to connect to robot
   - ROS communication works bidirectionally

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│ HOST (Arch Linux)                                            │
│                                                              │
│  run_robot.sh:                                              │
│  1. Detect interface (enp0s13f0u4c2 or enp0s13f0u1c2, etc) │
│  2. Get HOST_IP from interface (192.168.1.100)             │
│  3. Configure firewall (iptables)                           │
│  4. Start container with env vars and --add-host           │
│                                                              │
│  ┌────────────────────────────────────────────────────────┐│
│  │ CONTAINER (Docker)                                      ││
│  │                                                         ││
│  │  exec.sh → bash:                                       ││
│  │    source init_robot.sh  # Patches intera.sh           ││
│  │                                                         ││
│  │  intera.sh:                                            ││
│  │    your_ip="192.168.1.100"      # From $HOST_IP       ││
│  │    robot_hostname="192.168.1.103"  # From $ROBOT_IP   ││
│  │                                                         ││
│  │  ROS Master: http://192.168.1.103:11311               ││
│  │  Robot hostname resolves via /etc/hosts (--add-host)  ││
│  └────────────────────────────────────────────────────────┘│
│                                                              │
│  Firewall: iptables -I INPUT -s 192.168.1.103 -j ACCEPT   │
└─────────────────────────────────────────────────────────────┘
                             │
                             │ Ethernet (192.168.1.x)
                             ▼
                    ┌─────────────────┐
                    │ Sawyer Robot    │
                    │ 192.168.1.103   │
                    └─────────────────┘
```

## Files Modified/Created

### New Files
- `run_robot.sh` - Main startup script (replaces run.sh)
- `workspace/init_robot.sh` - Container runtime configuration
- `AUTOMATED_STARTUP.md` - This documentation

### Modified Files
- `workspace/Dockerfile` - Added sudo and passwordless configuration
- `exec.sh` - Auto-sources init_robot.sh when attaching

### Deprecated Files
- `run.sh` - Replaced by `run_robot.sh`
- `check_and_start.sh` - Functionality merged into `run_robot.sh`

## Troubleshooting

### Robot Not Detected

```bash
# Check available interfaces
ip -br addr show

# Manually specify robot IP
./run_robot.sh 192.168.1.103
```

### Firewall Issues

```bash
# Check iptables rules
sudo iptables -L INPUT -n | grep 192.168.1.103

# Manually add rule
sudo iptables -I INPUT -s 192.168.1.103 -j ACCEPT
```

### DNS Resolution Issues

Check if `--add-host` was applied:

```bash
# Inside container
cat /etc/hosts | grep 021607CP00070
# Should show: 192.168.1.103 021607CP00070.local
```

### Container Can't Reach Robot

```bash
# Inside container
ping 192.168.1.103
ip route show

# From host
./run_robot.sh  # Will auto-fix
```

## Benefits

### Before (Manual Configuration)
- Hardcoded IPs in Dockerfile
- Manual firewall configuration
- Manual `/etc/hosts` editing
- Rebuild required for IP changes
- Breaks when USB port changes

### After (Automated)
- ✅ Zero manual configuration
- ✅ Detects network automatically
- ✅ Configures firewall automatically
- ✅ DNS resolution automatic
- ✅ Works with any USB port
- ✅ No rebuilds for network changes
- ✅ One command startup: `./run_robot.sh`

## Advanced Usage

### Custom Container Name

Edit `CONTAINER_NAME` in `run_robot.sh`:
```bash
CONTAINER_NAME="my_custom_name"
```

### Multiple Robots

```bash
# Robot 1
./run_robot.sh 192.168.1.103 sawyer1.local

# Robot 2 (need different container name)
# Edit run_robot.sh to change CONTAINER_NAME first
./run_robot.sh 192.168.1.104 sawyer2.local
```

### Save Firewall Rules

```bash
# Make iptables rules persistent
sudo iptables-save > /etc/iptables/iptables.rules
```

## Rebuild Instructions

Only needed when:
- Updating ROS packages
- Changing system dependencies
- Updating Python packages

**Not needed when**:
- Network changes
- USB port changes
- Robot IP changes

```bash
# Rebuild image
./build.sh

# Start with new image
./run_robot.sh
```
