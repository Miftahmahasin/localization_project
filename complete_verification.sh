#!/bin/bash
# Complete System Verification After Launch

echo "🔬 COMPLETE SYSTEM VERIFICATION"
echo "========================================================================"
echo ""
echo "This script will verify all components of the localization system"
echo "Make sure the launch file is running before executing this!"
echo ""
read -p "Press Enter to continue..."
echo ""

PASS_COUNT=0
FAIL_COUNT=0

# Function to check and report
check_component() {
    local name=$1
    local check_command=$2
    local expected=$3
    
    echo -n "Checking $name... "
    result=$(eval $check_command 2>&1)
    
    if echo "$result" | grep -q "$expected"; then
        echo "✅ PASS"
        ((PASS_COUNT++))
        return 0
    else
        echo "❌ FAIL"
        echo "   Output: $result"
        ((FAIL_COUNT++))
        return 1
    fi
}

echo "1. NODE CHECKS"
echo "========================================================================"

# Check nodes exist
check_component "Detector node" \
    "ros2 node list | grep detector_fieldline" \
    "detector_fieldline"

check_component "Converter node" \
    "ros2 node list | grep simple_pc2scan" \
    "simple_pc2scan"

check_component "Map server" \
    "ros2 node list | grep map_server" \
    "map_server"

check_component "AMCL node" \
    "ros2 node list | grep -w amcl" \
    "amcl"

# Check for duplicates
echo -n "Checking for duplicate AMCL... "
AMCL_COUNT=$(ros2 node list 2>/dev/null | grep -c "^/amcl$")
if [ $AMCL_COUNT -eq 1 ]; then
    echo "✅ PASS (exactly 1)"
    ((PASS_COUNT++))
else
    echo "❌ FAIL (found $AMCL_COUNT)"
    ((FAIL_COUNT++))
fi

echo ""
echo "2. TOPIC CHECKS"
echo "========================================================================"

check_component "Camera image" \
    "timeout 2 ros2 topic hz /robotis_op3/camera/image_raw 2>&1" \
    "average rate"

check_component "Point cloud" \
    "timeout 2 ros2 topic hz /field_point_cloud 2>&1" \
    "average rate"

check_component "Laser scan" \
    "timeout 2 ros2 topic hz /field_scan 2>&1" \
    "average rate"

check_component "Map" \
    "timeout 3 ros2 topic echo /map --once 2>&1 | head -5" \
    "header:"

echo ""
echo "3. AMCL CHECKS"
echo "========================================================================"

check_component "AMCL lifecycle" \
    "ros2 lifecycle get /amcl" \
    "active"

check_component "AMCL scan subscription" \
    "ros2 node info /amcl 2>&1 | grep -A 10 Subscribers" \
    "/field_scan"

check_component "AMCL map subscription" \
    "ros2 node info /amcl 2>&1 | grep -A 10 Subscribers" \
    "/map"

check_component "AMCL pose publishing" \
    "timeout 2 ros2 topic hz /amcl_pose 2>&1" \
    "average rate"

# Check covariance is non-zero
echo -n "Checking AMCL covariance... "
COVAR=$(timeout 3 ros2 topic echo /amcl_pose --once 2>&1 | grep -A 1 "covariance:" | tail -1 | grep -v "  - 0.0")
if [ -n "$COVAR" ]; then
    echo "✅ PASS (non-zero = localizing)"
    ((PASS_COUNT++))
else
    echo "❌ FAIL (all zeros = not localizing)"
    ((FAIL_COUNT++))
fi

# Check particles
echo -n "Checking particle cloud... "
PARTICLES=$(timeout 3 ros2 topic hz /particle_cloud 2>&1)
if echo "$PARTICLES" | grep -q "average rate"; then
    echo "✅ PASS (publishing)"
    ((PASS_COUNT++))
else
    echo "⚠️  WARNING (not publishing - may be normal if low update rate)"
    echo "   Try: ros2 topic echo /particle_cloud --once"
fi

echo ""
echo "4. TRANSFORM CHECKS"
echo "========================================================================"

check_component "TF map→odom" \
    "timeout 2 ros2 run tf2_ros tf2_echo map odom 2>&1" \
    "Translation:"

check_component "TF odom→cam_link" \
    "timeout 2 ros2 run tf2_ros tf2_echo odom cam_link 2>&1" \
    "Translation:"

echo ""
echo "5. DATA QUALITY CHECKS"
echo "========================================================================"

# Check scan has valid ranges
echo -n "Checking scan data quality... "
SCAN_DATA=$(timeout 3 ros2 topic echo /field_scan --once 2>&1 | grep ranges -A 20)
VALID_RANGES=$(echo "$SCAN_DATA" | grep -v "inf" | grep -E "^- [0-9]" | wc -l)
if [ $VALID_RANGES -gt 10 ]; then
    echo "✅ PASS ($VALID_RANGES valid ranges)"
    ((PASS_COUNT++))
else
    echo "❌ FAIL (only $VALID_RANGES valid ranges)"
    ((FAIL_COUNT++))
fi

# Check point cloud size
echo -n "Checking point cloud size... "
PC_SIZE=$(timeout 3 ros2 topic echo /field_point_cloud --once 2>&1 | grep "width:" | awk '{print $2}')
if [ -n "$PC_SIZE" ] && [ $PC_SIZE -gt 100 ]; then
    echo "✅ PASS ($PC_SIZE points)"
    ((PASS_COUNT++))
else
    echo "❌ FAIL ($PC_SIZE points - too few)"
    ((FAIL_COUNT++))
fi

echo ""
echo "========================================================================"
echo "VERIFICATION SUMMARY"
echo "========================================================================"
echo ""
echo "Passed: $PASS_COUNT"
echo "Failed: $FAIL_COUNT"
echo ""

TOTAL=$((PASS_COUNT + FAIL_COUNT))
PERCENTAGE=$((PASS_COUNT * 100 / TOTAL))

if [ $PERCENTAGE -ge 90 ]; then
    echo "✅ EXCELLENT! System is working properly ($PERCENTAGE% passed)"
    echo ""
    echo "Next steps:"
    echo "  - Open RViz2 to visualize"
    echo "  - Add displays for /map, /field_scan, /particle_cloud, /amcl_pose"
    echo "  - Test robot movement and watch localization update"
elif [ $PERCENTAGE -ge 70 ]; then
    echo "⚠️  GOOD but with issues ($PERCENTAGE% passed)"
    echo ""
    echo "Check the failed components above"
elif [ $PERCENTAGE -ge 50 ]; then
    echo "⚠️  PARTIAL success ($PERCENTAGE% passed)"
    echo ""
    echo "Major issues detected. Check failed components"
else
    echo "❌ SYSTEM NOT WORKING ($PERCENTAGE% passed)"
    echo ""
    echo "Multiple critical failures. Recommend restart:"
    echo "  1. Kill all: ./kill_all_ros2_nodes.sh"
    echo "  2. Verify clean: ./verify_clean_state.sh"
    echo "  3. Launch again"
fi

echo ""
echo "========================================================================"
echo ""
echo "Detailed logs available at: ~/.ros/log/"
echo ""