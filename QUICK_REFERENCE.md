# Quick Reference - Automated Robot Startup

## One-Time Setup

```bash
# Rebuild container with passwordless sudo
./build.sh
```

## Daily Workflow

```bash
# 1. Start everything (auto-configures firewall, DNS, network)
./run_robot.sh

# 2. Attach to container (auto-configures intera.sh)
./exec.sh

# 3. Connect to robot
cd ~/ros_ws && ./intera.sh

# 4. Test
rostopic list
rostopic echo /robot/joint_states
```

## One-Liner

```bash
./run_robot.sh && ./exec.sh
```

## Custom Robot IP

```bash
./run_robot.sh 192.168.1.105 sawyer-robot.local
```

## What Gets Auto-Configured

✅ Network interface detection
✅ Host IP extraction
✅ Firewall rule (iptables)
✅ DNS mapping (--add-host)
✅ Container environment variables
✅ intera.sh patching

## Troubleshooting

```bash
# Check firewall
sudo iptables -L INPUT -n | grep 192.168.1.103

# Check DNS mapping (inside container)
cat /etc/hosts | grep 021607CP00070

# Check env vars (inside container)
echo $HOST_IP $ROBOT_IP $ROBOT_HOSTNAME

# Manual firewall fix
sudo iptables -I INPUT -s 192.168.1.103 -j ACCEPT
```

## Files

- `run_robot.sh` - Main startup (replaces run.sh)
- `exec.sh` - Attach to container
- `workspace/init_robot.sh` - Runtime config
- `AUTOMATED_STARTUP.md` - Full docs
- `IMPLEMENTATION_SUMMARY.md` - Implementation details
