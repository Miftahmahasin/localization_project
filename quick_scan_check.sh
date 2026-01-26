#!/bin/bash
# Quick Scan Parameter Check

echo "🔍 QUICK SCAN & CONVERTER CHECK"
echo "========================================================================"
echo ""

echo "1. Checking if converter is running..."
if ros2 node list 2>/dev/null | grep -q simple_pc2scan; then
    echo "   ✅ Converter node running"
else
    echo "   ❌ Converter node NOT running!"
    echo "   This is why scan is all .inf!"
    exit 1
fi

echo ""
echo "2. Checking converter parameters..."
echo ""
echo "Range limits:"
ros2 param get /simple_pc2scan range_min 2>/dev/null || echo "   Could not get range_min"
ros2 param get /simple_pc2scan range_max 2>/dev/null || echo "   Could not get range_max"

echo ""
echo "Angle limits (radians):"
ros2 param get /simple_pc2scan angle_min 2>/dev/null || echo "   Could not get angle_min"
ros2 param get /simple_pc2scan angle_max 2>/dev/null || echo "   Could not get angle_max"

echo ""
echo "3. Checking scan data..."
timeout 3 ros2 topic echo /field_scan --once > /tmp/scan_check.txt 2>&1

if grep -q "ranges:" /tmp/scan_check.txt; then
    TOTAL=$(grep -c "^- " /tmp/scan_check.txt)
    VALID=$(grep -c "^- [0-9]" /tmp/scan_check.txt)
    INF=$(grep -c "^- .inf" /tmp/scan_check.txt)
    
    echo "   Total ranges: $TOTAL"
    echo "   Valid: $VALID"
    echo "   Inf: $INF"
    
    if [ $VALID -gt 0 ]; then
        echo ""
        echo "   ✅ Scan has valid data!"
        echo ""
        echo "   Sample values:"
        grep "^- [0-9]" /tmp/scan_check.txt | head -5
    else
        echo ""
        echo "   ❌ All ranges are .inf!"
    fi
else
    echo "   ❌ Could not get scan data"
fi

echo ""
echo "4. Checking point cloud..."
timeout 3 ros2 topic echo /field_point_cloud --once 2>&1 | grep "width:" | head -1

echo ""
echo "5. Checking converter status from logs..."
timeout 3 ros2 topic echo /rosout --once 2>&1 | grep -i "simple_pc2scan\|convert" | tail -5

echo ""
echo "========================================================================"
echo "RECOMMENDATIONS:"
echo "========================================================================"
echo ""

if [ $VALID -eq 0 ]; then
    cat << 'HELP'
Scan is all .inf despite having point cloud. Possible fixes:

1. Check converter is actually running:
   ros2 node list | grep simple_pc2scan

2. Try adjusting range limits:
   ros2 param set /simple_pc2scan range_min 0.05
   ros2 param set /simple_pc2scan range_max 10.0

3. Check converter logs for errors:
   ros2 topic echo /rosout | grep simple_pc2scan

4. Restart converter by killing launch and restarting

5. Run detailed diagnostic:
   python3 /mnt/user-data/outputs/debug_scan_conversion.py
HELP
fi

echo ""
echo "========================================================================"