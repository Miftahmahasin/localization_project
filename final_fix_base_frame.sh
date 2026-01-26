#!/bin/bash
# FINAL FIX: Change AMCL base_frame to base_link

echo "🔧 APPLYING FINAL FIX FOR AMCL LOCALIZATION"
echo "========================================================================"
echo ""

echo "Problem identified:"
echo "  - AMCL base_frame_id = cam_link (WRONG!)"
echo "  - This causes instability because cam_link moves with head"
echo "  - AMCL needs stable base_frame = base_link"
echo ""

echo "Applying fix..."
echo ""

# Change base_frame_id
echo "1. Setting base_frame_id to base_link..."
ros2 param set /amcl base_frame_id base_link

if [ $? -eq 0 ]; then
    echo "   ✅ base_frame_id changed to base_link"
else
    echo "   ❌ Failed to change parameter"
    exit 1
fi

echo ""
echo "2. Verifying configuration..."
BASE_FRAME=$(ros2 param get /amcl base_frame_id 2>&1 | grep "String value" | awk '{print $3}')
echo "   Current base_frame_id: $BASE_FRAME"

echo ""
echo "3. Restarting AMCL lifecycle..."
ros2 lifecycle set /amcl configure
sleep 1
ros2 lifecycle set /amcl activate

echo ""
echo "4. Setting initial pose with correct configuration..."
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.706825, w: 0.707388}
    },
    covariance: [
      0.5, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.5, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.2
    ]
  }
}" --once &

echo "Waiting 5 seconds for initialization..."
sleep 5

echo ""
echo "5. Checking result..."
echo ""

# Check covariance
COVAR=$(ros2 topic echo /amcl_pose --once 2>&1 | grep -A 1 "covariance:" | tail -1 | awk '{print $2}')

if [ "$COVAR" != "0.0" ] && [ ! -z "$COVAR" ]; then
    echo "✅ SUCCESS! AMCL is now localizing!"
    echo "   Covariance: $COVAR (non-zero = working!)"
else
    echo "⚠️  Covariance still zero - checking further..."
fi

echo ""
echo "6. Verifying TF chain..."
timeout 3 ros2 run tf2_ros tf2_echo map base_link 2>&1 | head -8

echo ""
echo "========================================================================"
echo "FIX APPLIED!"
echo "========================================================================"
echo ""

cat << 'EOF'
What was changed:
  base_frame_id: cam_link → base_link

Why this fixes it:
  - base_link is stable (doesn't move with head)
  - cam_link moves when head moves (unstable)
  - AMCL needs stable reference frame
  - Scan still in cam_link but transformed to base_link via TF

Expected behavior now:
  - Pose should update when robot moves
  - Covariance should be non-zero
  - TF frames should be stable
  - Red arrow should track robot

Test it:
  1. Move robot in Webots
  2. Watch pose update in RViz
  3. Check: ros2 topic echo /amcl_pose

If still not working, the issue is scan transformation.
Check: ros2 run tf2_ros tf2_echo base_link cam_link
       Should show stable transform (not changing)

EOF

echo "========================================================================"