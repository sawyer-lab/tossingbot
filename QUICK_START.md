# Quick Start - Robot Connection Testing

> **For complete workflow**, see [COMPLETE_WORKFLOW.md](COMPLETE_WORKFLOW.md)

## TL;DR - Automated Setup (Recommended)

```bash
# From HOST - checks everything and starts container automatically
./check_and_start.sh

# Then attach and test
./exec.sh
```

## Quick Status Check

```bash
# From HOST - just check status (no restart)
./check_robot.sh
```

## Manual Start

```bash
# 1. Start container
./run.sh && ./exec.sh

# 2. Inside container - run everything
cd /test_scripts && ./run_all_tests.sh
```

## Manual Steps

```bash
# Inside container:
cd /test_scripts

# Before robot boots
./01_pre_robot_tests.sh

# [Turn on robot physically]

# Wait for robot
./02_wait_for_robot.sh

# After robot detected
./04_post_robot_tests.sh
```

## Connect to Robot (Simplest)

```bash
# Inside container - checks then connects automatically
cd /test_scripts
./connect_robot.sh
```

## Before Running intera.sh (Manual)

```bash
# Inside container - check everything first
cd /test_scripts
./check_before_intera.sh

# If all checks pass, run intera.sh
cd ~/ros_ws && ./intera.sh
```

## If intera.sh Fails

```bash
# Add debugging
./03_add_debug_to_intera.sh

# Try again
cd ~/ros_ws && ./intera.sh

# Dump full info
cd /test_scripts && ./dump_intera_info.sh
```

## Key Info
- **Robot IP**: 192.168.1.101
- **Robot Hostname**: 021607CP00070.local
- **Host IP**: 192.168.1.100
- **Ethernet**: enp0s13f0u4c2

## Troubleshooting

| Problem | Solution |
|---------|----------|
| ping not found | Rebuild container: `./build.sh` |
| Robot not detected | Ensure ethernet connected BEFORE robot boot |
| intera.sh fails | Run with debug: `./03_add_debug_to_intera.sh` |
| No ROS topics | Check robot mode (SDK vs Research) |

See [TESTING_PLAN.md](TESTING_PLAN.md) for details.
