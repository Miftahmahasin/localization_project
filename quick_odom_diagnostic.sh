#!/bin/bash

echo "=========================================="
echo "  Quick Odometry Bridge Diagnostic"
echo "=========================================="

echo ""
echo "Step 1: Check if /robotis/present_joint_states is publishing..."
JOINT_RATE=$(timeout 2s ros2 topic hz /robotis/present_joint_states 2>&1 | grep "average rate" | head -1 | awk '{print $3}')
if [ ! -z "$JOINT_RATE" ]; then
    echo "✓ Joint states publishing at: $JOINT_RATE Hz"
else
    echo "✗ Joint states NOT publishing!"
    echo "  → Start Webots simulation first"
    exit 1
fi

echo ""
echo "Step 2: Check if odom_bridge node is running..."
if ros2 node list | grep -q "/odom_bridge"; then
    echo "✓ odom_bridge node is running"
else
    echo "✗ odom_bridge node NOT running!"
    exit 1
fi

echo ""
echo "Step 3: Check odom_bridge subscriptions..."
echo "Node info:"
ros2 node info /odom_bridge 2>&1 | grep -A5 "Subscriptions:" | head -6

echo ""
echo "Step 4: Check if /odom_combined is publishing..."
timeout 3s ros2 topic hz /odom_combined 2>&1 | head -1

ODOM_TEST=$(timeout 2s ros2 topic echo /odom_combined --once 2>&1)
if echo "$ODOM_TEST" | grep -q "position:"; then
    echo "✓ /odom_combined is publishing data!"
    POS_X=$(echo "$ODOM_TEST" | grep "x:" | head -1 | awk '{print $2}')
    POS_Y=$(echo "$ODOM_TEST" | grep "y:" | head -1 | awk '{print $2}')
    echo "  Current position: X=$POS_X, Y=$POS_Y"
else
    echo "✗ /odom_combined NOT publishing or timeout!"
    echo "  This means odom_bridge is not receiving joint_states"
fi

echo ""
echo "Step 5: Check TF: odom -> base..."
timeout 2s ros2 run tf2_ros tf2_echo odom base 2>&1 | head -5

echo ""
echo "=========================================="
echo "  Diagnostic Complete"
echo "=========================================="
echo ""
echo "If /odom_combined NOT publishing:"
echo "  1. Check odom_bridge logs for errors"
echo "  2. Verify joint_states_topic parameter"
echo "  3. Check if joint_states has correct format"