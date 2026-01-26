#!/bin/bash
# Activate AMCL and Force Localization

echo "🔄 ACTIVATING AMCL FOR LOCALIZATION"
echo "========================================================================"
echo ""

# Check current state
echo "1. Current AMCL lifecycle state:"
CURRENT_STATE=$(ros2 lifecycle get /amcl 2>/dev/null)
echo "   $CURRENT_STATE"

if ! echo "$CURRENT_STATE" | grep -q "active"; then
    echo ""
    echo "2. AMCL is not active! Activating..."
    
    if echo "$CURRENT_STATE" | grep -q "unconfigured"; then
        echo "   Configuring AMCL..."
        ros2 lifecycle set /amcl configure
        sleep 1
    fi
    
    echo "   Activating AMCL..."
    ros2 lifecycle set /amcl activate
    sleep 2
    
    echo ""
    echo "   New state:"
    ros2 lifecycle get /amcl
else
    echo "   ✅ AMCL is already active"
fi

echo ""
echo "========================================================================"
echo "3. Checking if AMCL received initial pose..."
echo "========================================================================"

# Check if initial pose was set
INITIAL_POSE=$(ros2 param get /amcl set_initial_pose 2>/dev/null)
echo "   set_initial_pose parameter: $INITIAL_POSE"

echo ""
echo "4. Publishing initial pose to trigger localization..."
echo ""

ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
  }
}" --once

echo ""
echo "✅ Initial pose published!"
echo ""

echo "========================================================================"
echo "5. Waiting for AMCL to process (5 seconds)..."
echo "========================================================================"
sleep 5

echo ""
echo "6. Checking AMCL outputs..."
echo ""

# Check pose covariance
echo "   a) Checking /amcl_pose covariance..."
POSE_OUTPUT=$(ros2 topic echo /amcl_pose --once 2>/dev/null)

if echo "$POSE_OUTPUT" | grep -A1 "covariance:" | grep -v "0.0" | grep -q "[1-9]"; then
    echo "      ✅ Covariance has NON-ZERO values! AMCL is localizing!"
    echo ""
    echo "      Covariance (first 6 values):"
    echo "$POSE_OUTPUT" | grep -A6 "covariance:" | head -7
else
    echo "      ❌ Covariance is still all zeros - AMCL not localizing"
fi

echo ""
echo "   b) Checking /particle_cloud..."
timeout 3 ros2 topic hz /particle_cloud 2>&1 | head -3

echo ""
echo "   c) Checking if scan is being received by AMCL..."
echo "      Subscribers to /field_scan:"
ros2 topic info /field_scan | grep "Subscription count"

echo ""
echo "========================================================================"
echo "7. AMCL LOG CHECK"
echo "========================================================================"
echo ""
echo "Checking AMCL subscriptions (should include /field_scan via remapping)..."
ros2 node info /amcl 2>/dev/null | grep "Subscri" -A 10

echo ""
echo "========================================================================"
echo "FINAL DIAGNOSIS"
echo "========================================================================"
echo ""

# Try to get particle cloud one more time
timeout 5 ros2 topic echo /particle_cloud --once > /tmp/particles_final.txt 2>&1

if [ -s /tmp/particles_final.txt ] && ! grep -q "WARNING" /tmp/particles_final.txt; then
    echo "🎉 SUCCESS! AMCL IS PUBLISHING PARTICLES!"
    echo ""
    echo "Particle cloud preview:"
    head -30 /tmp/particles_final.txt
    echo ""
    echo "✅ LOCALIZATION IS WORKING!"
else
    echo "❌ AMCL still not publishing particles"
    echo ""
    echo "Possible remaining issues:"
    echo "  1. Scan topic still not remapped correctly"
    echo "  2. Frame mismatch between scan and base_frame_id"
    echo "  3. Need to check launch file actually has remapping"
    echo ""
    echo "Check if /field_scan is actually being remapped to /scan for AMCL:"
    echo ""
    echo "  ros2 node info /amcl | grep -i scan"
    echo ""
fi

echo "========================================================================"