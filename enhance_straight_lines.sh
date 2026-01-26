#!/bin/bash
# Enhanced Detector Parameters for Straight Line Detection

echo "🔧 OPTIMIZING FOR STRAIGHT LINES"
echo "========================================================================"
echo ""

echo "Current issue:"
echo "  - Center circle: GOOD (748-903 points)"
echo "  - Straight lines: POOR (broken/missing)"
echo "  - Coverage: Still 20% (need 30%+)"
echo ""

echo "Solution: Aggressive straight line detection"
echo ""

echo "Setting enhanced parameters..."
echo ""

# CRITICAL: Detect very short line segments
ros2 param set /detector_fieldline detection.min_line_length 10
echo "  ✓ min_line_length: 15 → 10 (detect shorter segments)"

# CRITICAL: Bridge large gaps
ros2 param set /detector_fieldline detection.max_line_gap 20
echo "  ✓ max_line_gap: 15 → 20 (connect distant segments)"

# Even denser sampling
ros2 param set /detector_fieldline point_cloud.spacing 8
echo "  ✓ spacing: 10 → 8 (even denser)"

# Accept single-point lines
ros2 param set /detector_fieldline point_cloud.min_points 1
echo "  ✓ min_points: 3 → 1 (accept all detected lines)"

# Lower threshold slightly (detect dimmer lines)
ros2 param set /detector_fieldline detection.white_threshold 165
echo "  ✓ white_threshold: 170 → 165 (slightly more sensitive)"

echo ""
echo "Waiting 5 seconds for detector to adapt..."
sleep 5

echo ""
echo "========================================================================"
echo "TESTING NEW PARAMETERS"
echo "========================================================================"
echo ""

echo "Checking scan quality..."
python3 /mnt/user-data/outputs/debug_scan_conversion.py 2>&1 | tail -25

echo ""
echo "========================================================================"
echo "EXPECTED IMPROVEMENTS:"
echo "========================================================================"
echo ""

cat << 'EOF'
BEFORE:
  - Points: 650-900
  - Valid ranges: 73-81 (20%)
  - Lines: Mostly center circle

AFTER (Target):
  - Points: 1000-1500
  - Valid ranges: 120-150 (33-40%)
  - Lines: Center circle + sidelines + boxes + goal lines

If still poor:
  - Check /camera/line_image in rqt_image_view
  - Should see MORE straight line segments
  - If not, detector has different issue

Alternative: Multi-threshold detection
  - Detect at threshold 165, 180, 200
  - Combine all detections
  - Gets lines at different brightness levels

EOF

echo "========================================================================"