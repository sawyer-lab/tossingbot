# Complete Robot Connection Testing Plan

## Prerequisites
1. ✅ Container rebuilt with network tools (`./build.sh` - already done)
2. ✅ Test scripts created and mounted
3. ✅ Modem/router powered ON
4. Ethernet cable ready to connect

## Step-by-Step Execution Plan

### Phase 1: Container Setup (30 seconds)

```bash
# 1. Start container
./run.sh

# 2. Attach to container
./exec.sh

# You're now inside the container at: kid@rog:~/ros_ws$
```

### Phase 2: Pre-Robot Tests (30 seconds)

```bash
# 3. Run pre-robot diagnostics
cd /test_scripts
./01_pre_robot_tests.sh

# Expected output:
# - ✓ All network tools available
# - ✓ Network interfaces shown
# - ✓ Current intera.sh config displayed
# - Gateway reachable
```

### Phase 3: Add Debugging (10 seconds - Optional but Recommended)

```bash
# 4. Add debug output to intera.sh
./03_add_debug_to_intera.sh

# This creates intera.sh.backup and adds debug prints
# Highly recommended for first run!
```

### Phase 4: Robot Power-On and Detection (2-5 minutes)

**IMPORTANT: Connect ethernet BEFORE turning on robot**

```bash
# 5. Physically:
#    a. Connect ethernet cable (if not already connected)
#    b. Turn ON the Sawyer robot
#
# 6. Run detection monitor
./02_wait_for_robot.sh

# This will show real-time status:
# [045s] ping_ip:✓ ping_hostname:✓ avahi_resolve:✓ avahi_browse:✓
#
# When all show ✓, robot is online!
```

### Phase 5: Comprehensive Testing (1-2 minutes)

```bash
# 7. Run post-robot tests
./04_post_robot_tests.sh

# This will:
# - Test connectivity
# - Attempt to source intera.sh
# - Check ROS environment
# - List ROS topics
# - Verify key robot topics exist
```

### Phase 6: Debug If Needed

**If intera.sh SUCCEEDED:**
```bash
# You should see:
# - ROS_MASTER_URI set
# - rostopic list shows many topics
# - /robot/joint_states exists

# You're done! Robot is connected.
```

**If intera.sh FAILED:**
```bash
# 8. Dump complete intera.sh info
./dump_intera_info.sh > /tmp/intera_debug.txt

# 9. Look at the debug output
cat /tmp/intera_debug.txt

# Key things to check:
# - Is robot_hostname still "robot_hostname.local"? (sed failed)
# - What line is causing the EXITING message?
# - Are all variables set correctly?
```

## What You'll Learn

After running all tests, you'll know:

1. **Network Layer**
   - Can container reach robot? (Yes/No)
   - Which detection method works? (ping IP, hostname, avahi)
   - Latency to robot
   - Is mDNS working in container?

2. **Robot Configuration**
   - Is robot in Research or SDK mode?
   - Which ports are open?
   - Boot time (how long until detectable)

3. **intera.sh Status**
   - Did it execute successfully?
   - If not, exactly where and why it failed
   - What variables are set to what values

4. **ROS Connectivity**
   - Is ROS_MASTER_URI set correctly?
   - Can we list topics?
   - Are robot topics available?

## Quick Reference

| Script | When | Purpose |
|--------|------|---------|
| `01_pre_robot_tests.sh` | Before robot boot | Verify container setup |
| `02_wait_for_robot.sh` | While robot boots | Detect when online |
| `03_add_debug_to_intera.sh` | Optional | Add debug output |
| `04_post_robot_tests.sh` | After robot online | Full connectivity test |
| `dump_intera_info.sh` | If problems | Debug intera.sh |
| `run_all_tests.sh` | Anytime | Run all with prompts |

## Expected Time
- Total: ~5-10 minutes
- Most time is waiting for robot to boot (2-5 min)
- Actual testing: < 2 minutes

## Output Files
- `/tmp/intera_debug.txt` - Full intera.sh dump (if you ran it)
- `/tmp/rostopic_list.txt` - ROS topics (if successful)
- `~/ros_ws/intera.sh.backup` - Original intera.sh (if debugged)

## After Testing

If successful:
```bash
# Source intera.sh
cd ~/ros_ws
source intera.sh

# You can now use ROS commands
rostopic list
rostopic echo /robot/joint_states
```

If failed:
- Review all output
- Check `/tmp/intera_debug.txt`
- Look for the exact error message
- Share output with debugging team

## Notes
- All scripts are safe and non-destructive
- Nothing modifies the robot itself
- Can be run multiple times
- Container needs `--network host` mode (already configured)
