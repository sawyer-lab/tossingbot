# Robot Connection Test Scripts

These scripts help you test and debug the Sawyer robot connection from inside the Docker container.

## Quick Start

```bash
# Run everything in sequence with prompts
./run_all_tests.sh
```

## Individual Scripts

### check_before_intera.sh
**Run RIGHT BEFORE sourcing intera.sh (recommended)**

Quick pre-flight check that verifies:
- Network connectivity to robot
- Robot is reachable
- intera.sh has correct configuration
- ROS installation is ready
- Catkin workspace is built

Returns:
- Exit 0 if ready to run intera.sh
- Exit 1 if there are blocking errors

Usage:
```bash
cd ~/test_scripts
./check_before_intera.sh && cd ~/ros_ws && ./intera.sh
```

---

### 01_pre_robot_tests.sh
**Run BEFORE turning on robot**

Tests:
- Network tools availability (ping, avahi-resolve, etc.)
- Container network interfaces
- Current intera.sh configuration
- Gateway connectivity
- Avahi daemon status

Output: Shows if container is properly configured for robot connection

---

### 02_wait_for_robot.sh
**Run AFTER turning on robot (while it boots)**

Monitors:
- Ping to robot IP (192.168.1.101)
- Ping to robot hostname (021607CP00070.local)
- Avahi hostname resolution
- Avahi service browsing

Output: Real-time status, auto-detects when robot is fully online

---

### 03_add_debug_to_intera.sh
**Optional - adds debug output to intera.sh**

Actions:
- Backs up original intera.sh to intera.sh.backup
- Adds debug print statements throughout script
- Shows variable values as they're set
- Shows why validation fails (if it does)

To revert:
```bash
cp ~/ros_ws/intera.sh.backup ~/ros_ws/intera.sh
```

---

### 04_post_robot_tests.sh
**Run AFTER robot is online**

Tests:
- Network connectivity to robot (ping, latency)
- mDNS resolution
- ROS port 11311 status (research vs SDK mode)
- intera.sh execution
- ROS environment variables
- ROS topic listing
- Key robot topics presence

Output: Complete diagnostic of robot connection and ROS status

---

### dump_intera_info.sh
**Run anytime - dumps complete intera.sh information**

Shows:
- File permissions
- All variable assignments
- All conditional checks
- All exit statements
- Complete script with line numbers

Output: Everything needed to understand intera.sh logic

---

## Typical Workflow

```bash
# 1. Start container and attach
./run.sh
./exec.sh

# 2. Copy test scripts into container (if not mounted)
# They should be available at /test_scripts if bind-mounted

# 3. Run pre-tests
cd /test_scripts
./01_pre_robot_tests.sh

# 4. Add debugging (optional)
./03_add_debug_to_intera.sh

# 5. Turn on robot (physically)

# 6. Wait for robot
./02_wait_for_robot.sh

# 7. Run comprehensive tests
./04_post_robot_tests.sh

# 8. If intera.sh failed, dump full info
./dump_intera_info.sh
```

## Troubleshooting

**"Command not found" errors**
- Make sure container was rebuilt with network tools: `./build.sh`

**"Cannot reach gateway"**
- Check ethernet cable is connected
- Verify modem/router is on

**"Robot not detected" after 5 minutes**
- Check robot screen - is it fully booted?
- Try unplugging and replugging ethernet
- Check if robot is on same subnet (192.168.1.x)

**"intera.sh failed"**
- Check debug output from 03_add_debug_to_intera.sh
- Run dump_intera_info.sh to see validation logic
- Check if hostname value is exactly "robot_hostname.local" (means sed replacement didn't work)

## Files Created

- `intera.sh.backup` - Original intera.sh (if you ran script 03)
- `/tmp/rostopic_list.txt` - ROS topic list output (from script 04)

## Notes

- All scripts are safe to run multiple times
- Scripts do NOT modify the robot (only read/test)
- Debug mode can be removed by restoring backup
- Container must be run with `--network host` for these to work
