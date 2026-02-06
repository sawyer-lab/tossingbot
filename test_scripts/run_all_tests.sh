#!/bin/bash
# Master script to run all robot connection tests
# This guides you through the complete testing process

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

echo "=========================================="
echo "ROBOT CONNECTION TEST SUITE"
echo "=========================================="
echo ""
echo "This will guide you through testing the robot connection."
echo ""

# Step 1: Pre-robot tests
echo "STEP 1: Pre-Robot Boot Tests"
echo "----------------------------"
read -p "Press Enter to run pre-robot tests..."
bash $SCRIPT_DIR/01_pre_robot_tests.sh
echo ""

# Step 2: Add debugging
echo "STEP 2: Add Debugging to intera.sh"
echo "-----------------------------------"
read -p "Add debug output to intera.sh? (y/n) " -n 1 -r
echo ""
if [[ $REPLY =~ ^[Yy]$ ]]; then
    bash $SCRIPT_DIR/03_add_debug_to_intera.sh
fi
echo ""

# Step 3: Wait for robot
echo "STEP 3: Turn On Robot and Wait"
echo "-------------------------------"
echo "Now:"
echo "  1. Make sure ethernet cable is connected"
echo "  2. Turn ON the robot"
echo "  3. Wait for robot to boot (this script will detect it)"
echo ""
read -p "Press Enter when you've turned on the robot..."
echo ""
bash $SCRIPT_DIR/02_wait_for_robot.sh
echo ""

# Step 4: Post-robot tests
echo "STEP 4: Post-Robot Boot Tests"
echo "------------------------------"
read -p "Press Enter to run post-robot tests..."
bash $SCRIPT_DIR/04_post_robot_tests.sh
echo ""

echo "=========================================="
echo "ALL TESTS COMPLETE"
echo "=========================================="
echo ""
echo "Check the output above for any failures."
echo "If intera.sh failed, the debug output should show why."
