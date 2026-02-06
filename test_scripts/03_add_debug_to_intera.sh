#!/bin/bash
# Add debugging output to intera.sh
# Run this INSIDE the container to add debug prints

INTERA_PATH=~/ros_ws/intera.sh

if [ ! -f "$INTERA_PATH" ]; then
    echo "✗ intera.sh not found at $INTERA_PATH"
    exit 1
fi

echo "Adding debug output to $INTERA_PATH..."

# Backup original
cp $INTERA_PATH ${INTERA_PATH}.backup
echo "✓ Created backup at ${INTERA_PATH}.backup"

# Add debug output after the shebang
# This adds a debug section at the beginning
cat > /tmp/debug_header.sh << 'EOF'

# ============== DEBUG OUTPUT ADDED ==============
echo "=========================================="
echo "INTERA.SH DEBUG MODE"
echo "=========================================="
echo ""
echo "Script variables:"
set -x  # Enable bash debugging
EOF

# Insert debug header after shebang
sed -i '1 a\
# ============== DEBUG OUTPUT ADDED ==============\
echo "=========================================="\
echo "INTERA.SH DEBUG MODE"\
echo "=========================================="\
echo ""\
echo "Reading configuration..."\
' $INTERA_PATH

# Add debug output before the EXITING line
sed -i '/EXITING - Please edit this file/i\
echo ""\
echo "❌ VALIDATION FAILED"\
echo "Current robot_hostname value: ${robot_hostname}"\
echo "Expected pattern: should NOT be robot_hostname.local"\
echo ""\
' $INTERA_PATH

# Add debug output at key validation points
sed -i '/^robot_hostname=/a\
echo "  ✓ robot_hostname = ${robot_hostname}"\
' $INTERA_PATH

sed -i '/^your_ip=/a\
echo "  ✓ your_ip = ${your_ip}"\
' $INTERA_PATH

sed -i '/^ros_version=/a\
echo "  ✓ ros_version = ${ros_version}"\
' $INTERA_PATH

echo "✓ Debug output added to intera.sh"
echo ""
echo "To revert changes:"
echo "  cp ${INTERA_PATH}.backup $INTERA_PATH"
echo ""
echo "Now you can run: ./intera.sh"
