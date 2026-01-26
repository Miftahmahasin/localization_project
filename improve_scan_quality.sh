#!/bin/bash
# COMPLETE SCAN QUALITY IMPROVEMENT
# Addresses the 22.2% → 60%+ coverage goal

echo "🎯 SCAN QUALITY IMPROVEMENT SYSTEM"
echo "========================================================================"
echo ""
echo "Current status:"
echo "  - 387 points detected ✓"
echo "  - Only 80/360 valid ranges (22.2%) ✗"
echo "  - Only center circle detected ✗"
echo "  - Missing: sidelines, penalty boxes, goal boxes ✗"
echo ""
echo "Target:"
echo "  - 200+ valid ranges (55%+)"
echo "  - Multiple line types detected"
echo "  - Better angular coverage"
echo ""
echo "========================================================================"
echo ""

echo "STEP 1: Optimize Detector Parameters"
echo "----------------------------------------------------------------------"
echo ""

echo "Setting detector parameters for more lines..."

# Detect shorter line segments
ros2 param set /detector_fieldline detection.min_line_length 15
echo "  ✓ min_line_length: 20 → 15 (detect shorter segments)"

# Bridge larger gaps
ros2 param set /detector_fieldline detection.max_line_gap 15
echo "  ✓ max_line_gap: 10 → 15 (connect broken lines)"

# Denser point sampling
ros2 param set /detector_fieldline point_cloud.spacing 10
echo "  ✓ spacing: 20 → 10 (2x more points)"

# Lower min points threshold
ros2 param set /detector_fieldline point_cloud.min_points 3
echo "  ✓ min_points: 5 → 3 (keep more lines)"

echo ""
echo "Waiting 3 seconds for detector to adapt..."
sleep 3

echo ""
echo "========================================================================"
echo "STEP 2: Replace Scan Converter"
echo "----------------------------------------------------------------------"
echo ""

echo "Stopping old converter..."
pkill -f simple_pc2scan
sleep 2

echo "Starting improved converter with:"
echo "  - 0.5° resolution (was 1°)"
echo "  - Gap interpolation"
echo "  - Point averaging per bin"
echo ""

cd ~/basbot
python3 /mnt/user-data/outputs/improved_pc2scan2.py > /tmp/improved_scan.log 2>&1 &
CONVERTER_PID=$!

echo "  ✓ Improved converter started (PID: $CONVERTER_PID)"
echo "  ✓ Log: /tmp/improved_scan.log"

sleep 3

echo ""
echo "========================================================================"
echo "STEP 3: Verify Improvements"
echo "----------------------------------------------------------------------"
echo ""

echo "Checking scan quality..."
sleep 2

timeout 3 python3 debug_scan_conversion.py 2>&1 | tail -20

echo ""
echo "========================================================================"
echo "STEP 4: Monitor Real-time"
echo "----------------------------------------------------------------------"
echo ""

echo "Checking rates..."
echo ""

echo "Point cloud:"
timeout 3 ros2 topic hz /field_point_cloud 2>&1 | head -3

echo ""
echo "Laser scan:"
timeout 3 ros2 topic hz /field_scan 2>&1 | head -3

echo ""
echo "========================================================================"
echo "RESULTS SUMMARY"
echo "========================================================================"
echo ""

# Get scan stats
SCAN_INFO=$(timeout 2 ros2 topic echo /field_scan --once 2>&1)
VALID_RANGES=$(echo "$SCAN_INFO" | grep -v "inf" | grep -c "^- ")

echo "Scan coverage: $VALID_RANGES valid ranges"
echo ""

if [ "$VALID_RANGES" -gt 150 ]; then
    echo "🎉 EXCELLENT! Coverage improved significantly!"
elif [ "$VALID_RANGES" -gt 100 ]; then
    echo "✓ GOOD! Coverage improved!"
else
    echo "⚠ Needs more work. Try:"
    echo "  - Lower threshold: ros2 param set /detector_fieldline detection.white_threshold 160"
    echo "  - Smaller spacing: ros2 param set /detector_fieldline point_cloud.spacing 5"
fi

echo ""
echo "========================================================================"
echo "NEXT STEPS"
echo "========================================================================"
echo ""

cat << 'EOF'
1. Check debug image:
   - Open RViz
   - Add Image display
   - Topic: /camera/line_image
   - Should see MORE lines detected

2. Monitor scan in RViz:
   - LaserScan display should have MORE red dots
   - Should cover wider angular range
   - Should include sidelines and boxes

3. Test AMCL:
   ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
     header: {frame_id: 'map'},
     pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}
   }" --once
   
   Then move robot - pose should update now!

4. Fine-tune if needed:
   - If still low coverage:
     ros2 param set /detector_fieldline point_cloud.spacing 5
   
   - If too noisy:
     ros2 param set /detector_fieldline detection.min_line_length 20

Keep improved_pc2scan running instead of simple_pc2scan!

EOF

echo "========================================================================"