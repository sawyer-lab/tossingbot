# Rebuild Instructions - Making Robot Connection Permanent

## What Was Fixed

### 1. **Dockerfile** (workspace/Dockerfile:210-212)
**Problem**: sed commands didn't match actual intera.sh content
- Line 210: `your_ip=""` pattern now matches exactly
- Line 212: `robot_hostname="021607CP00070.local"` pattern now matches exactly (was looking for "robot_hostname.local")

**Fix Applied**: Updated sed patterns to match the actual content in intera_sdk's intera.sh

### 2. **build.sh**
**Problem**: `HOST_IP` was picking up first IP (could be WiFi, Docker bridge, etc.)

**Fix Applied**: Now specifically gets IP from ethernet interface `enp0s13f0u4c2` (192.168.1.100), falls back to first IP if ethernet not available

## What You Need to Do

### One-Time Rebuild (Required)

```bash
# 1. Make sure ethernet is connected with robot
./check_and_start.sh  # Verify connectivity first

# 2. Stop any running container
docker stop robo2025

# 3. Rebuild with automatic IP detection
./build.sh

# 4. Start new container
./run.sh && ./exec.sh
```

### Verify It Worked

Inside the container:
```bash
# Option 1: Run full pre-check then connect (recommended)
cd /test_scripts
./connect_robot.sh

# Option 2: Just check configuration
cat ~/ros_ws/intera.sh | grep "^your_ip="
cat ~/ros_ws/intera.sh | grep "^robot_hostname="

# Should show:
#   your_ip="192.168.1.100"
#   robot_hostname="021607CP00070.local"

# Then run pre-check
./check_before_intera.sh

# If checks pass, run intera.sh
cd ~/ros_ws
./intera.sh

# Should NOT show "EXITING - Please edit..." errors
# Should set ROS_MASTER_URI and work
```

## After Rebuild

From now on, whenever you:
1. Connect ethernet
2. Turn on robot
3. Run `./check_and_start.sh` (or `./run.sh`)

The container will automatically have the correct IPs configured and `intera.sh` should work without manual editing!

## If You Need to Change IPs

If your network configuration changes:
```bash
# Rebuild with current ethernet IP
./build.sh

# Or specify manually
docker build \
    --build-arg HOST_IP=<your-new-ip> \
    --build-arg ROBOT_HOSTNAME=021607CP00070.local \
    -t robo2025-workspace workspace
```

## Quick Reference

| Command | Purpose |
|---------|---------|
| `./check_and_start.sh` | Check connectivity & start container |
| `./check_robot.sh` | Quick status check (no restart) |
| `./build.sh` | Rebuild container with auto-detected IPs |
| `./run.sh` | Start container (after rebuild) |
| `./exec.sh` | Attach to running container |

## Notes

- Rebuild is only needed once (or when network config changes)
- After rebuild, `intera.sh` will have correct IPs pre-configured
- Robot's IP might still change (DHCP), but hostname `021607CP00070.local` stays the same
- If robot IP changes, mDNS will resolve the hostname automatically (no rebuild needed)
