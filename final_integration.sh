#!/bin/bash
# FINAL INTEGRATION: Connect Ground Truth Odometry to AMCL

echo "🎯 FINAL INTEGRATION: Ground Truth Odometry + AMCL"
echo "========================================================================"
echo ""

echo "STEP 1: Stop all existing nodes"
echo "----------------------------------------------------------------------"
pkill -9 -f "ros2 launch"
pkill -9 -f "simple_odom"
pkill -9 -f "gt_odom"
pkill -9 -f "python3.*odom"
sleep 3
echo "✅ All stopped"

echo ""
echo "STEP 2: Start main localization system"
echo "----------------------------------------------------------------------"
cd ~/basbot
source install/setup.bash

# Start main system in background
ros2 launch soccer_object_localization amcl_final_fixed.launch.py white_threshold:=170 > /tmp/amcl_system.log 2>&1 &
LAUNCH_PID=$!
echo "✅ AMCL system started (PID: $LAUNCH_PID)"

sleep 15
echo "   Waited 15s for initialization..."

echo ""
echo "STEP 3: Start Ground Truth Odometry Republisher"
echo "----------------------------------------------------------------------"

# Check if gt_odom_to_amcl.py exists
if [ -f ~/basbot/gt_odom_to_amcl.py ]; then
    echo "Using new GT odometry republisher..."
    python3 gt_odom_to_amcl.py > /tmp/gt_odom.log 2>&1 &
    ODOM_PID=$!
    echo "✅ GT Odometry republisher started (PID: $ODOM_PID)"
else
    echo "❌ gt_odom_to_amcl.py not found!"
    echo "   Please copy it to ~/basbot first"
    exit 1
fi

sleep 5

echo ""
echo "STEP 4: Verify System"
echo "----------------------------------------------------------------------"

echo "Checking topics..."
echo ""

# Check /odom topic
if ros2 topic list 2>&1 | grep -q "^/odom$"; then
    echo "  ✅ /odom topic exists"
    
    # Check rate
    ODOM_RATE=$(timeout 3 ros2 topic hz /odom 2>&1 | grep "average rate:" | head -1 | awk '{print $3}')
    if [ ! -z "$ODOM_RATE" ]; then
        echo "     Rate: ${ODOM_RATE} Hz"
    fi
else
    echo "  ❌ /odom topic missing!"
fi

# Check /field_scan topic
if ros2 topic list 2>&1 | grep -q "^/field_scan$"; then
    echo "  ✅ /field_scan topic exists"
else
    echo "  ❌ /field_scan topic missing!"
fi

# Check /map topic
if ros2 topic list 2>&1 | grep -q "^/map$"; then
    echo "  ✅ /map topic exists"
else
    echo "  ❌ /map topic missing!"
fi

echo ""
echo "Checking TF tree..."
echo ""

# Check odom → base_link transform
timeout 3 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8 | grep -E "Translation|Rotation" && \
    echo "  ✅ odom → base_link transform working" || \
    echo "  ⚠️  odom → base_link transform not ready yet"

sleep 2

echo ""
echo "========================================================================"
echo "STEP 5: Set Initial Pose and Test"
echo "========================================================================"
echo ""

echo "Setting initial pose..."
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
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

sleep 5

echo ""
echo "Checking AMCL pose..."
AMCL_POSE=$(timeout 2 ros2 topic echo /amcl_pose --once 2>&1)

if echo "$AMCL_POSE" | grep -q "position:"; then
    echo "✅ AMCL is publishing pose!"
    echo ""
    echo "$AMCL_POSE" | grep -A 3 "position:" | head -4
    echo ""
    echo "$AMCL_POSE" | grep -A 2 "covariance:" | head -3
else
    echo "❌ AMCL not publishing pose yet"
fi

echo ""
echo "========================================================================"
echo "SYSTEM READY!"
echo "========================================================================"
echo ""

cat << 'EOF'
✅ Ground Truth Odometry connected to AMCL!

TESTING:
1. Move robot in Webots
2. Watch pose update:
   ros2 topic echo /amcl_pose | grep -A 3 "position:"

3. Compare with ground truth:
   Terminal 1: ros2 topic echo /odom | grep -A 3 "position:"
   Terminal 2: ros2 topic echo /amcl_pose | grep -A 3 "position:"
   
   Should be VERY close (within 1-2cm)!

VISUALIZATION:
1. Open RViz: rviz2
2. Add displays:
   - Map: /map
   - LaserScan: /field_scan
   - PointCloud: /field_point_cloud
   - PoseWithCovariance: /amcl_pose
   - PoseArray: /particle_cloud
   - Odometry: /odom (optional, for comparison)

3. Fixed Frame: map

LOGS:
  - System: tail -f /tmp/amcl_system.log
  - Odometry: tail -f /tmp/gt_odom.log

EXPECTED BEHAVIOR:
  ✅ Pose tracks robot movement accurately
  ✅ Covariance small (< 0.1 with GT odometry!)
  ✅ Particles clustered tightly
  ✅ No jitter or jumping

FOR REAL ROBOT:
  - Replace /ground_truth/odom with real odometry source
  - Options: 
    1. IMU + step counting (humanoid robots)
    2. Visual odometry from camera
    3. Multi-sensor fusion (IMU + vision + joint states)
    
  - Modify gt_odom_to_amcl.py to subscribe to real odom topic
  - Increase covariance values for real (noisier) odometry

EOF

echo "========================================================================"