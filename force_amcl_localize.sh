#!/bin/bash
# Force AMCL to Start Localizing

echo "🔧 FORCING AMCL TO LOCALIZE"
echo "========================================================================"
echo ""

# Step 1: Check AMCL subscription
echo "1. Verifying AMCL scan subscription..."
SCAN_SUB=$(ros2 node info /amcl 2>/dev/null | grep -A 10 "Subscribers" | grep -c "field_scan")

if [ $SCAN_SUB -gt 0 ]; then
    echo "   ✅ AMCL subscribed to /field_scan"
else
    echo "   ❌ AMCL NOT subscribed to /field_scan!"
    echo "   Checking remapping..."
    ros2 node info /amcl 2>/dev/null | grep -A 10 "Subscribers" | grep scan
fi

echo ""
echo "2. Checking current AMCL parameters..."
echo ""

# Check critical parameters
echo "   Update thresholds:"
MIN_D=$(ros2 param get /amcl update_min_d 2>/dev/null | grep "Double" | awk '{print $3}')
MIN_A=$(ros2 param get /amcl update_min_a 2>/dev/null | grep "Double" | awk '{print $3}')
echo "     update_min_d: $MIN_D (lower = more sensitive)"
echo "     update_min_a: $MIN_A (lower = more sensitive)"

echo ""
echo "   Laser parameters:"
MAX_BEAMS=$(ros2 param get /amcl laser_max_beams 2>/dev/null | grep "Integer" | awk '{print $3}')
MIN_RANGE=$(ros2 param get /amcl laser_min_range 2>/dev/null | grep "Double" | awk '{print $3}')
MAX_RANGE=$(ros2 param get /amcl laser_max_range 2>/dev/null | grep "Double" | awk '{print $3}')
echo "     laser_max_beams: $MAX_BEAMS (we have ~34 valid)"
echo "     laser_min_range: $MIN_RANGE"
echo "     laser_max_range: $MAX_RANGE"

echo ""
echo "========================================================================"
echo "3. Adjusting AMCL parameters for low scan count..."
echo "========================================================================"
echo ""

# Reduce laser_max_beams to match our scan
echo "   Setting laser_max_beams to 30 (we only have ~34 valid)..."
ros2 param set /amcl laser_max_beams 30

# Make updates more frequent
echo "   Setting update thresholds lower..."
ros2 param set /amcl update_min_d 0.01  # Update every 1cm
ros2 param set /amcl update_min_a 0.05  # Update every ~3 degrees

# Increase tolerance
echo "   Increasing transform tolerance..."
ros2 param set /amcl transform_tolerance 0.5

echo ""
echo "4. Setting initial pose with proper covariance..."
echo ""

ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: 'map'
  },
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
      0.0, 0.0, 0.0, 0.0, 0.0, 0.1
    ]
  }
}" --times 1 &

sleep 3

echo ""
echo "5. Waiting for AMCL to process (5 seconds)..."
sleep 5

echo ""
echo "6. Checking if covariance changed..."
echo ""

ros2 topic echo /amcl_pose --once 2>/dev/null > /tmp/amcl_check.txt

if grep -A 6 "covariance:" /tmp/amcl_check.txt | grep -q "[1-9]"; then
    echo "   ✅ SUCCESS! Covariance has non-zero values!"
    echo ""
    echo "   Covariance:"
    grep -A 6 "covariance:" /tmp/amcl_check.txt | head -7
    echo ""
    echo "   ✅ AMCL IS NOW LOCALIZING!"
else
    echo "   ❌ Covariance still all zeros"
    echo ""
    echo "   This means AMCL is not localizing despite scan data"
    echo ""
    echo "   Possible issues:"
    echo "   1. Not enough valid scan ranges (need 30+, have ~34)"
    echo "   2. Scan quality too poor"
    echo "   3. Map not matching scan"
    echo "   4. AMCL parameters too strict"
    echo ""
    echo "   Try:"
    echo "   a) Improve scan quality (adjust white_threshold)"
    echo "   b) Check AMCL logs: ros2 topic echo /rosout | grep amcl"
    echo "   c) Verify map matches environment"
fi

echo ""
echo "========================================================================"
echo "7. Checking particle cloud..."
echo "========================================================================"
echo ""

timeout 5 ros2 topic echo /particle_cloud --once > /tmp/particles.txt 2>&1

if grep -q "poses:" /tmp/particles.txt; then
    PARTICLE_COUNT=$(grep -c "position:" /tmp/particles.txt)
    echo "   ✅ Particles publishing! (~$PARTICLE_COUNT particles)"
else
    echo "   ❌ No particles yet"
    echo ""
    echo "   Particles only appear when AMCL is actively localizing"
    echo "   Since covariance is zero, AMCL hasn't started localizing"
fi

echo ""
echo "========================================================================"
echo "SUMMARY"
echo "========================================================================"
echo ""

cat << 'EOF'
Current status:
- Scan: ✅ 34 valid ranges (but low count)
- AMCL parameters: Adjusted
- Initial pose: Set
- Covariance: Check output above

If still not localizing:

1. Increase valid scan ranges:
   # Adjust detector threshold
   ros2 param set /detector_fieldline detection.white_threshold 150

2. Check AMCL is receiving scans:
   ros2 topic echo /rosout | grep -i "amcl\|scan"

3. Restart with better parameters:
   ./kill_all_ros2_nodes.sh
   # Edit launch file to increase laser_max_beams, reduce update thresholds
   ros2 launch ...

4. Check map matches environment:
   # In RViz, overlay map on camera view
   # Field lines should align with map

EOF

echo "========================================================================"