#!/bin/bash
# Dumps complete information about intera.sh for debugging
# Run this inside the container to see full script content

INTERA_PATH=~/ros_ws/intera.sh

echo "=========================================="
echo "INTERA.SH COMPLETE INFORMATION DUMP"
echo "=========================================="
echo ""

if [ ! -f "$INTERA_PATH" ]; then
    echo "✗ intera.sh not found at $INTERA_PATH"
    exit 1
fi

echo "1. File location and permissions:"
ls -lh $INTERA_PATH
echo ""

echo "2. Key configuration lines:"
echo "----------------------------"
grep -n "robot_hostname" $INTERA_PATH
grep -n "your_ip" $INTERA_PATH
grep -n "ros_version" $INTERA_PATH
echo ""

echo "3. All variable assignments (first 50):"
echo "----------------------------------------"
grep -n "^[a-zA-Z_][a-zA-Z0-9_]*=" $INTERA_PATH | head -50
echo ""

echo "4. All conditional checks (if/elif):"
echo "-------------------------------------"
grep -n "^[[:space:]]*if\|^[[:space:]]*elif" $INTERA_PATH
echo ""

echo "5. All EXITING/exit statements:"
echo "--------------------------------"
grep -n -i "exit\|EXITING" $INTERA_PATH
echo ""

echo "6. Full script content:"
echo "-----------------------"
echo "(Showing with line numbers)"
echo ""
cat -n $INTERA_PATH
echo ""

echo "=========================================="
echo "END OF DUMP"
echo "=========================================="
