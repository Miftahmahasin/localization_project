#!/bin/bash
# Complete Lifecycle Debug and Fix

echo "🔬 AMCL LIFECYCLE COMPLETE DEBUG"
echo "========================================================================"
echo ""

# 1. Check nodes
echo "1. All running nodes:"
ros2 node list 2>/dev/null

echo ""
echo "2. AMCL nodes specifically:"
AMCL_COUNT=$(ros2 node list 2>/dev/null | grep -c "/amcl")
echo "   Count: $AMCL_COUNT"

if [ $AMCL_COUNT -gt 1 ]; then
    echo "   ❌ PROBLEM: Multiple AMCL nodes! ($AMCL_COUNT found)"
    echo "   Kill all and restart with only one launch file"
    exit 1
elif [ $AMCL_COUNT -eq 0 ]; then
    echo "   ❌ No AMCL node found!"
    exit 1
else
    echo "   ✅ Exactly 1 AMCL node (correct)"
fi

echo ""
echo "========================================================================"
echo "3. AMCL Lifecycle State:"
echo "========================================================================"
STATE=$(ros2 lifecycle get /amcl 2>/dev/null)
echo "   $STATE"

# Parse state
if echo "$STATE" | grep -q "unconfigured"; then
    echo ""
    echo "   ⚠️  AMCL is UNCONFIGURED - needs configuration!"
    echo ""
    read -p "   Configure AMCL now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "   Configuring..."
        ros2 lifecycle set /amcl configure
        sleep 2
        STATE=$(ros2 lifecycle get /amcl)
        echo "   New state: $STATE"
    fi
fi

if echo "$STATE" | grep -q "inactive"; then
    echo ""
    echo "   ⚠️  AMCL is INACTIVE - needs activation!"
    echo ""
    read -p "   Activate AMCL now? (y/n) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "   Activating..."
        ros2 lifecycle set /amcl activate
        sleep 2
        STATE=$(ros2 lifecycle get /amcl)
        echo "   New state: $STATE"
    fi
fi

if echo "$STATE" | grep -q "active"; then
    echo "   ✅ AMCL is ACTIVE"
else
    echo "   ❌ AMCL is NOT active - cannot localize"
fi

echo ""
echo "========================================================================"
echo "4. AMCL Subscriptions (after lifecycle):"
echo "========================================================================"
sleep 1  # Wait for subscriptions to register
ros2 node info /amcl 2>/dev/null | grep "Subscribers" -A 15

# Check if scan subscription exists
HAS_SCAN=$(ros2 node info /amcl 2>/dev/null | grep -E "Subscribers|/scan|/field_scan" | grep -c scan)

if [ $HAS_SCAN -gt 0 ]; then
    echo ""
    echo "   ✅ AMCL is subscribed to scan topic!"
else
    echo ""
    echo "   ❌ AMCL is NOT subscribed to any scan topic!"
    echo ""
    echo "   This means:"
    echo "   - Remapping not working"
    echo "   - OR lifecycle not properly activated"
    echo "   - OR AMCL crashed during activation"
fi

echo ""
echo "========================================================================"
echo "5. Topic Connections:"
echo "========================================================================"
echo "/field_scan info:"
ros2 topic info /field_scan 2>/dev/null

echo ""
echo "/scan info (if exists):"
ros2 topic info /scan 2>/dev/null || echo "  (does not exist - this is expected)"

echo ""
echo "========================================================================"
echo "6. Publishing Initial Pose:"
echo "========================================================================"

ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
  }
}" --once &

sleep 5

echo ""
echo "========================================================================"
echo "7. Final Check:"
echo "========================================================================"

echo ""
echo "Particles:"
timeout 3 ros2 topic hz /particle_cloud 2>&1 | head -3

echo ""
echo "Pose covariance:"
ros2 topic echo /amcl_pose --once 2>/dev/null | grep -A 6 "covariance:" | head -7

echo ""
echo "========================================================================"
echo "DONE"
echo "========================================================================"