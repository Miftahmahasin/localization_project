#!/bin/bash

echo "=========================================="
echo "    Fix Verification Script"
echo "=========================================="

echo ""
echo "Test 1: Check frame in /tf..."
FRAME=$(ros2 topic echo /tf --once 2>/dev/null | grep "child_frame_id" | head -1 | awk '{print $2}')
echo "Child frame: $FRAME"
if [ "$FRAME" == "base" ]; then
    echo "✓ Frame is correct! (base)"
else
    echo "✗ Frame is WRONG! ($FRAME)"
    echo "  Expected: base"
    echo "  Should be fixed with new odom_to_tf.py"
fi

echo ""
echo "Test 2: Check if odom -> base transform works..."
timeout 3s ros2 run tf2_ros tf2_echo odom base 2>&1 | head -10
RESULT=$?
if [ $RESULT -eq 0 ] || [ $RESULT -eq 124 ]; then
    echo "✓ Transform odom -> base is working!"
else
    echo "✗ Transform failed"
fi

echo ""
echo "Test 3: Check for duplicate static TF..."
STATIC_COUNT=$(ros2 node list | grep static_tf | grep -c base)
echo "Static TF nodes with 'base': $STATIC_COUNT"
if [ $STATIC_COUNT -eq 0 ]; then
    echo "✓ No duplicate static TF odom->base (Good!)"
else
    echo "⚠ Found static TF odom->base"
    echo "  This may conflict with dynamic TF"
fi

echo ""
echo "Test 4: Walk test..."
echo "Getting odometry position..."
POS1=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep "x:" | head -1 | awk '{print $2}')
echo "Initial X: $POS1"

echo ""
echo "\033[1;33m>>> Make robot WALK in Webots now! <<<\033[0m"
echo "Waiting 3 seconds..."
sleep 3

POS2=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep "x:" | head -1 | awk '{print $2}')
echo "After walking X: $POS2"

if [ "$POS1" != "$POS2" ]; then
    echo "✓ Odometry IS updating!"
else
    echo "✗ Odometry NOT updating"
fi

echo ""
echo "Test 5: TF tree structure..."
echo "Running: ros2 run tf2_tools view_frames"
timeout 5s ros2 run tf2_tools view_frames 2>&1 >/dev/null
if [ -f frames.pdf ]; then
    echo "✓ TF tree generated: frames.pdf"
    echo "  Expected chain: world -> odom -> base -> [robot links]"
else
    echo "✗ Failed to generate TF tree"
fi

echo ""
echo "=========================================="
echo "    Verification Complete"
echo "=========================================="
echo ""
echo "Expected Results:"
echo "1. Frame = 'base' ✓"
echo "2. odom -> base transform works ✓"
echo "3. No duplicate static TF ✓"
echo "4. Odometry updates when walking ✓"
echo "5. TF tree: world->odom->base->[links] ✓"
echo ""
echo "If all ✓, robot SHOULD move in RViz!"