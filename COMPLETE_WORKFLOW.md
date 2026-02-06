# Complete Robot Connection Workflow

## One-Time Setup (Do This Once)

### Step 1: Rebuild Container with Fixes
```bash
# From HOST - ensure ethernet connected
./check_robot.sh

# Rebuild container with automatic IP detection
docker stop robo2025
./build.sh

# This bakes the correct IPs into intera.sh permanently
```

**After this rebuild, you won't need to manually edit intera.sh anymore!**

---

## Daily Workflow (Every Time You Work)

### On HOST

```bash
# 1. Connect ethernet cable
# 2. Turn on robot (wait for it to fully boot)

# 3. Check everything and start container automatically
./check_and_start.sh

# This script:
#   ✓ Checks ethernet connection
#   ✓ Detects robot IP
#   ✓ Tests connectivity
#   ✓ Starts/restarts container if needed
#   ✓ Verifies container can reach robot

# 4. Attach to container
./exec.sh
```

### Inside Container

```bash
# Option A: SIMPLEST - One command to connect
cd /test_scripts
./connect_robot.sh

# This script:
#   ✓ Checks all prerequisites
#   ✓ Runs intera.sh automatically if checks pass
#   ✓ You're ready to use ROS!

# Option B: Manual step-by-step
cd /test_scripts
./check_before_intera.sh   # Pre-flight check
cd ~/ros_ws
./intera.sh                 # Connect to robot

# Verify connection
rostopic list
rostopic echo /robot/joint_states
```

---

## Quick Reference Commands

### Host-Side Scripts

| Command | What It Does |
|---------|-------------|
| `./check_and_start.sh` | Full automated setup (recommended) |
| `./check_robot.sh` | Quick status check (no restart) |
| `./build.sh` | Rebuild container with auto-detected IPs |
| `./run.sh` | Start container manually |
| `./exec.sh` | Attach to running container |

### Container-Side Scripts (in /test_scripts)

| Command | What It Does |
|---------|-------------|
| `./connect_robot.sh` | Check + connect in one command (simplest) |
| `./check_before_intera.sh` | Pre-flight check before intera.sh |
| `./01_pre_robot_tests.sh` | Test before robot boots |
| `./02_wait_for_robot.sh` | Monitor robot boot |
| `./04_post_robot_tests.sh` | Full diagnostic after boot |
| `./dump_intera_info.sh` | Debug intera.sh configuration |

---

## Troubleshooting

### Robot Not Detected

```bash
# From HOST
./check_robot.sh

# Check:
# - Is ethernet cable connected?
# - Is robot powered on?
# - Is robot fully booted? (check screen)

# Scan network
nmap -sn 192.168.1.0/24
avahi-browse -at | grep -i rethink
```

### Container Can't Reach Robot

```bash
# From HOST - restart container
docker stop robo2025
./check_and_start.sh

# Inside container - verify
cd /test_scripts
./check_before_intera.sh
```

### intera.sh Still Fails After Rebuild

```bash
# Inside container - check configuration
cat ~/ros_ws/intera.sh | grep "^your_ip="
cat ~/ros_ws/intera.sh | grep "^robot_hostname="

# Should show:
#   your_ip="192.168.1.100"
#   robot_hostname="021607CP00070.local"

# If not, rebuild was done with wrong network:
# - Exit container
# - Connect ethernet on HOST
# - Rebuild: ./build.sh
```

### Common Errors

| Error | Fix |
|-------|-----|
| `Destination Host Unreachable` | Ethernet cable disconnected or wrong network |
| `Temporary failure in name resolution` | avahi-daemon not running, use IP instead |
| `your_ip is EMPTY` | Rebuild container: `./build.sh` |
| `EXITING - Please edit...` | Robot hostname validation failing, rebuild needed |

---

## Success Indicators

**On HOST:**
- ✓ `./check_and_start.sh` shows all green checks
- ✓ Can ping robot: `ping 192.168.1.103`

**Inside Container:**
- ✓ `./check_before_intera.sh` passes
- ✓ `./intera.sh` runs without "EXITING" errors
- ✓ `rostopic list` shows /robot/* topics
- ✓ `echo $ROS_MASTER_URI` shows robot IP

**You're connected!** You can now:
```bash
rostopic list
rostopic echo /robot/joint_states
# Run your tossing bot code...
```

---

## Network Details

- **Host Ethernet IP**: 192.168.1.100
- **Robot Hostname**: 021607CP00070.local
- **Robot IP**: 192.168.1.103 (may change, hostname stays same)
- **Subnet**: 192.168.1.0/24
- **Gateway**: 192.168.1.1

---

## What If Robot IP Changes?

The robot IP might change (DHCP), but the hostname stays the same. The system handles this automatically:

1. `check_and_start.sh` uses avahi to find current IP
2. `intera.sh` uses hostname (not IP), which mDNS resolves
3. No rebuild needed if only IP changes!

Only rebuild if:
- Your host IP changes (different network)
- Robot hostname changes (unlikely)
- Container network configuration changes
