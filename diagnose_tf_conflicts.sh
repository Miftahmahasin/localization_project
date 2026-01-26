#!/bin/bash
# Diagnose TF Conflicts and Unstable System

echo "🔍 TF CONFLICT DIAGNOSTIC"
echo "========================================================================"
echo ""

echo "1. Checking TF tree structure..."
echo ""
ros2 run tf2_tools view_frames
echo "   ✅ frames.pdf generated - open it to see TF tree"
echo ""

echo "2. Checking for multiple transform publishers..."
echo ""
echo "   Looking for conflicts on critical frames:"
echo ""

# Check who's publishing what
echo "   a) base_link transforms:"
ros2 topic echo /tf --once 2>&1 | grep -A 5 "child_frame_id: base_link" | head -10

echo ""
echo "   b) cam_link transforms:"
ros2 topic echo /tf --once 2>&1 | grep -A 5 "child_frame_id: cam_link" | head -10

echo ""
echo "   c) odom transforms:"
ros2 topic echo /tf --once 2>&1 | grep -A 5 "child_frame_id: odom" | head -10

echo ""
echo "========================================================================"
echo "3. Checking AMCL base_frame_id parameter..."
echo "========================================================================"
echo ""

BASE_FRAME=$(ros2 param get /amcl base_frame_id 2>/dev/null | grep "String value" | awk '{print $3}')
ODOM_FRAME=$(ros2 param get /amcl odom_frame_id 2>/dev/null | grep "String value" | awk '{print $3}')
GLOBAL_FRAME=$(ros2 param get /amcl global_frame_id 2>/dev/null | grep "String value" | awk '{print $3}')

echo "   AMCL frame configuration:"
echo "     base_frame_id: $BASE_FRAME"
echo "     odom_frame_id: $ODOM_FRAME"
echo "     global_frame_id: $GLOBAL_FRAME"
echo ""

if [ "$BASE_FRAME" != "base_link" ]; then
    echo "   ⚠️  WARNING: base_frame_id is NOT base_link!"
    echo "   This causes issues with TF tree"
fi

echo ""
echo "========================================================================"
echo "4. Checking scan frame..."
echo "========================================================================"
echo ""

SCAN_FRAME=$(timeout 2 ros2 topic echo /field_scan --once 2>&1 | grep "frame_id:" | head -1 | awk '{print $2}')
echo "   Scan frame_id: $SCAN_FRAME"

if [ "$SCAN_FRAME" != "$BASE_FRAME" ]; then
    echo "   ⚠️  WARNING: Scan frame ($SCAN_FRAME) != base_frame ($BASE_FRAME)"
    echo "   AMCL expects scan in base_frame!"
fi

echo ""
echo "========================================================================"
echo "5. Testing transform lookups..."
echo "========================================================================"
echo ""

echo "   a) map → odom:"
timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -5

echo ""
echo "   b) odom → base_link:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -5

echo ""
echo "   c) base_link → cam_link:"
timeout 2 ros2 run tf2_ros tf2_echo base_link cam_link 2>&1 | head -5

echo ""
echo "========================================================================"
echo "6. Checking for transform conflicts..."
echo "========================================================================"
echo ""

# Count how many nodes publishing to /tf
echo "   Publishers to /tf:"
ros2 topic info /tf 2>&1 | grep "Publisher count:" -A 10

echo ""
echo "   ⚠️  If publisher count > 3, there may be conflicts!"
echo "   Expected publishers: static_tf, simple_odom, static_map_odom (or AMCL)"

echo ""
echo "========================================================================"
echo "DIAGNOSIS COMPLETE"
echo "========================================================================"
echo ""

cat << 'EOF'
Common issues causing instability:

1. MULTIPLE PUBLISHERS FOR SAME FRAME
   - Check frames.pdf for loops in TF tree
   - Look for red "MULTIPLE PUBLISHERS" warnings

2. FRAME MISMATCH
   - AMCL base_frame != scan frame
   - AMCL base_frame != robot's actual base frame

3. CONFLICTING STATIC TRANSFORMS
   - Multiple static_transform_publishers
   - Conflicting definitions of same transform

4. MISSING TRANSFORMS
   - Broken chain in TF tree
   - Transforms published but with wrong parent/child

TO FIX:
- Open frames.pdf to visualize TF tree
- Look for conflicts (red text)
- Kill conflicting publishers
- Ensure AMCL base_frame matches robot structure

EOF

echo "========================================================================"