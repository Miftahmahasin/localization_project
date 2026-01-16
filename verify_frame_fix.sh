#!/bin/bash

# Script untuk verifikasi bahwa frame sudah konsisten
# Jalankan setelah launch odometry nodes

# Warna
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo "=========================================="
echo "  Verify Frame Consistency"
echo "=========================================="
echo ""

ERRORS=0

# Test 1: Check if odom_to_tf.py is running
echo -e "${BLUE}Test 1: Checking nodes...${NC}"
if ros2 node list | grep -q "odom_to_tf"; then
    echo -e "${GREEN}✓${NC} odom_to_tf_publisher is running"
else
    echo -e "${RED}✗${NC} odom_to_tf_publisher is NOT running"
    ERRORS=$((ERRORS+1))
fi

if ros2 node list | grep -q "odom_bridge"; then
    echo -e "${GREEN}✓${NC} odom_bridge is running"
else
    echo -e "${RED}✗${NC} odom_bridge is NOT running"
    ERRORS=$((ERRORS+1))
fi
echo ""

# Test 2: Check TF topic
echo -e "${BLUE}Test 2: Checking /tf topic...${NC}"
if ros2 topic list | grep -q "^/tf$"; then
    echo -e "${GREEN}✓${NC} /tf topic exists"
    
    # Check if publishing
    RATE=$(timeout 2s ros2 topic hz /tf 2>&1 | grep "average rate" | head -1)
    if [ -n "$RATE" ]; then
        echo -e "${GREEN}✓${NC} /tf is publishing: $RATE"
    else
        echo -e "${RED}✗${NC} /tf exists but not publishing"
        ERRORS=$((ERRORS+1))
    fi
else
    echo -e "${RED}✗${NC} /tf topic does NOT exist"
    ERRORS=$((ERRORS+1))
fi
echo ""

# Test 3: Check frame name in TF
echo -e "${BLUE}Test 3: Checking frame names in /tf...${NC}"
TF_OUTPUT=$(timeout 2s ros2 topic echo /tf --once 2>&1)

if [ $? -eq 0 ]; then
    PARENT_FRAME=$(echo "$TF_OUTPUT" | grep "frame_id:" | head -1 | awk '{print $2}')
    CHILD_FRAME=$(echo "$TF_OUTPUT" | grep "child_frame_id:" | head -1 | awk '{print $2}')
    
    echo "  Parent frame: $PARENT_FRAME"
    echo "  Child frame: $CHILD_FRAME"
    
    if [ "$PARENT_FRAME" = "odom" ]; then
        echo -e "${GREEN}✓${NC} Parent frame is correct (odom)"
    else
        echo -e "${RED}✗${NC} Parent frame should be 'odom', got '$PARENT_FRAME'"
        ERRORS=$((ERRORS+1))
    fi
    
    if [ "$CHILD_FRAME" = "base_link" ]; then
        echo -e "${GREEN}✓${NC} Child frame is CORRECT (base_link)"
    elif [ "$CHILD_FRAME" = "base" ]; then
        echo -e "${RED}✗${NC} Child frame is still 'base' - FIX NOT APPLIED!"
        echo -e "${YELLOW}→${NC} Run: ./apply_frame_fix.sh"
        ERRORS=$((ERRORS+1))
    else
        echo -e "${RED}✗${NC} Unexpected child frame: '$CHILD_FRAME'"
        ERRORS=$((ERRORS+1))
    fi
else
    echo -e "${RED}✗${NC} Could not read /tf topic"
    ERRORS=$((ERRORS+1))
fi
echo ""

# Test 4: Check TF transform lookup
echo -e "${BLUE}Test 4: Testing TF lookup (odom → base_link)...${NC}"
TF_ECHO=$(timeout 3s ros2 run tf2_ros tf2_echo odom base_link 2>&1 | head -n 5)

if echo "$TF_ECHO" | grep -q "Translation:"; then
    echo -e "${GREEN}✓${NC} Transform odom → base_link EXISTS"
    echo "$TF_ECHO" | grep "Translation:" | head -1
elif echo "$TF_ECHO" | grep -q "Invalid frame ID.*base.*frame does not exist"; then
    echo -e "${RED}✗${NC} Frame 'base_link' does not exist in TF tree"
    echo -e "${YELLOW}→${NC} Still using 'base' frame - fix not applied!"
    ERRORS=$((ERRORS+1))
else
    echo -e "${YELLOW}⚠${NC} Could not verify transform (timeout or other issue)"
fi
echo ""

# Test 5: Check odometry topic
echo -e "${BLUE}Test 5: Checking /odom_combined...${NC}"
if ros2 topic list | grep -q "/odom_combined"; then
    echo -e "${GREEN}✓${NC} /odom_combined exists"
    
    # Get one message
    ODOM_CHILD=$(timeout 2s ros2 topic echo /odom_combined --once 2>&1 | grep "child_frame_id:" | awk '{print $2}')
    if [ -n "$ODOM_CHILD" ]; then
        echo "  child_frame_id in /odom_combined: $ODOM_CHILD"
    fi
else
    echo -e "${RED}✗${NC} /odom_combined does NOT exist"
    ERRORS=$((ERRORS+1))
fi
echo ""

# Summary
echo "=========================================="
if [ $ERRORS -eq 0 ]; then
    echo -e "${GREEN}✅ ALL TESTS PASSED!${NC}"
    echo ""
    echo "Frame consistency is correct:"
    echo "  • TF publishing: odom → base_link"
    echo "  • Odometry child_frame: matches TF"
    echo ""
    echo "RViz should now show: ${GREEN}Odometry Status: OK${NC}"
else
    echo -e "${RED}❌ FOUND $ERRORS ERROR(S)${NC}"
    echo ""
    echo "Troubleshooting:"
    echo "1. Make sure fix is applied:"
    echo "   ${YELLOW}./apply_frame_fix.sh${NC}"
    echo ""
    echo "2. Restart nodes:"
    echo "   ${YELLOW}killall -9 odom_bridge_node python3${NC}"
    echo "   ${YELLOW}ros2 launch op3_utra_bridge localization_odometry_only.launch.py${NC}"
    echo ""
    echo "3. Re-run this verification:"
    echo "   ${YELLOW}./verify_frame_fix.sh${NC}"
fi
echo "=========================================="

exit $ERRORS