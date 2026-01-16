#!/bin/bash

echo "=========================================="
echo "   Joint States & Odometry Monitor"
echo "=========================================="

echo ""
echo "Step 1: Check if /robotis/present_joint_states is publishing..."
echo "Expected: Should show rate ~50 Hz"
timeout 3s ros2 topic hz /robotis/present_joint_states

echo ""
echo "Step 2: Check joint_states content..."
ros2 topic echo /robotis/present_joint_states --once | head -20

echo ""
echo "Step 3: Check if odom_bridge receives joint_states..."
echo "Looking for log messages from odom_bridge..."
echo "(Should see 'Odometry initialized' and regular updates)"

echo ""
echo "Step 4: Check /odom_combined output..."
POS1=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep "x:" | head -1 | awk '{print $2}')
echo "Current odometry X: $POS1"

if [ "$POS1" == "0.0" ] || [ -z "$POS1" ]; then
    echo "⚠ WARNING: Odometry is still at (0,0,0)"
    echo ""
    echo "Possible causes:"
    echo "1. odom_bridge_node not receiving joint_states"
    echo "2. Robot not moving in Webots"
    echo "3. Topic remapping incorrect"
else
    echo "✓ Odometry has values (good!)"
fi

echo ""
echo "Step 5: Check TF from robot_state_publisher..."
echo "These should update as robot moves:"
timeout 2s ros2 run tf2_ros tf2_echo base r_ank_roll_link 2>&1 | head -10

echo ""
echo "Step 6: Node list..."
ros2 node list | grep -E "(odom|robot_state)" | while read node; do
    echo "  - $node"
done

echo ""
echo "=========================================="
echo "   Monitor Complete"
echo "=========================================="
echo ""
echo "If joint_states NOT publishing:"
echo "  → Webots simulation not running"
echo ""
echo "If odom_bridge not updating:"
echo "  → Check odom_bridge_node logs"
echo "  → Should see joint_states callbacks"
echo ""
echo "If robot model broken in RViz:"
echo "  → robot_state_publisher not receiving correct joint_states"
echo "  → Check remapping in launch file"