#!/bin/bash
# Diagnose Why AMCL Not Tracking Movement

echo "🔍 AMCL MOVEMENT TRACKING DIAGNOSTIC"
echo "========================================================================"
echo ""
echo "This script will help diagnose why AMCL is not tracking robot movement"
echo ""
echo "BEFORE RUNNING: Move the robot in Webots simulator!"
echo ""
read -p "Press Enter when robot has moved to a new position..."
echo ""

echo "1. Checking if camera is updating..."
echo "   Camera topic rate:"
timeout 3 ros2 topic hz /robotis_op3/camera/image_raw 2>&1 | head -2
if [ $? -eq 0 ]; then
    echo "   ✅ Camera is publishing"
else
    echo "   ❌ Camera NOT publishing!"
fi

echo ""
echo "2. Checking if detector is processing..."
echo "   Point cloud rate:"
timeout 3 ros2 topic hz /field_point_cloud 2>&1 | head -2
if [ $? -eq 0 ]; then
    echo "   ✅ Detector is working"
else
    echo "   ❌ Detector NOT working!"
fi

echo ""
echo "3. Checking scan data..."
echo "   Getting scan sample..."
ros2 topic echo /field_scan --once > /tmp/scan_current.txt 2>&1

VALID=$(grep "^- [0-9]" /tmp/scan_current.txt | wc -l)
echo "   Valid ranges: $VALID"

if [ $VALID -gt 10 ]; then
    echo "   ✅ Scan has data"
    echo ""
    echo "   Sample scan values (should reflect NEW robot position):"
    grep "^- [0-9]" /tmp/scan_current.txt | head -5
else
    echo "   ❌ Scan has too few valid ranges!"
fi

echo ""
echo "4. Checking odometry..."
echo "   Current odometry:"
ros2 topic echo /odom --once 2>&1 | grep -A 3 "position:" | head -4

echo ""
echo "   ⚠️  CHECK: Did position values change from (0,0,0)?"
echo "   If still (0,0,0), odometry is STATIC - this is the problem!"

echo ""
echo "5. Checking AMCL pose..."
echo "   Current AMCL pose:"
ros2 topic echo /amcl_pose --once 2>&1 | grep -A 3 "position:" | head -4

echo ""
echo "   ⚠️  CHECK: Did this position change from initial?"

echo ""
echo "6. Checking TF transforms..."
echo "   Transform odom → base_link:"
timeout 2 ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -8

echo ""
echo "   ⚠️  CHECK: Does translation reflect robot's NEW position?"

echo ""
echo "========================================================================"
echo "DIAGNOSIS:"
echo "========================================================================"
echo ""

# Analyze
if [ $VALID -lt 20 ]; then
    echo "❌ PROBLEM 1: Not enough valid scan ranges ($VALID < 20)"
    echo "   Solution:"
    echo "   ros2 param set /detector_fieldline detection.white_threshold 150"
    echo ""
fi

# Check if likely static odom
ros2 topic echo /odom --once 2>&1 | grep "position:" -A 3 | grep -q "x: 0.0" && \
ros2 topic echo /odom --once 2>&1 | grep "position:" -A 3 | grep -q "y: 0.0" && \
    echo "❌ PROBLEM 2: Odometry appears STATIC (always 0,0,0)" && \
    echo "   This is the MOST LIKELY cause!" && \
    echo "   Solutions:" && \
    echo "   A. Use Webots odometry (recommended for simulator)" && \
    echo "      python3 /mnt/user-data/outputs/webots_odom_publisher.py" && \
    echo "" && \
    echo "   B. Make AMCL rely purely on scan matching:" && \
    echo "      ros2 param set /amcl update_min_d 0.01" && \
    echo "      ros2 param set /amcl alpha1 0.0001" && \
    echo "      ros2 param set /amcl alpha2 0.0001" && \
    echo ""

echo "========================================================================"
echo "NEXT STEPS:"
echo "========================================================================"
echo ""

cat << 'EOF'
Based on the diagnosis above, try these fixes in order:

1. If scan quality is low (< 30 valid ranges):
   ros2 param set /detector_fieldline detection.white_threshold 150
   python3 /mnt/user-data/outputs/debug_scan_conversion.py

2. If odometry is static (always 0,0,0):
   Option A: Use Webots odometry
     python3 /mnt/user-data/outputs/webots_odom_publisher.py
   
   Option B: Pure vision tracking
     ros2 param set /amcl update_min_d 0.01
     ros2 param set /amcl update_min_a 0.02
     ros2 param set /amcl alpha1 0.0001
     ros2 param set /amcl alpha2 0.0001
     ros2 param set /amcl alpha3 0.0001
     ros2 param set /amcl alpha4 0.0001

3. Force AMCL to update:
   ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
     header: {frame_id: 'map'},
     pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}},
            covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                         0.0, 0.0, 0.0, 0.0, 0.0, 0.068]
     }
   }" --once

4. Test movement:
   - Move robot in Webots
   - Watch red arrow in RViz
   - Should follow robot now!

EOF

echo "========================================================================"