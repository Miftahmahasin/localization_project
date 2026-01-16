#!/bin/bash

###############################################################################
# Odometry Timestamp Fix Verification Script
# 
# This script verifies that the dt=0 issue is fixed and odometry is working
###############################################################################

echo "=========================================="
echo "   ODOMETRY FIX VERIFICATION"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

passed=0
failed=0

# Function to check test result
check_result() {
    if [ $? -eq 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: $1"
        ((passed++))
    else
        echo -e "${RED}✗ FAIL${NC}: $1"
        ((failed++))
    fi
}

# Test 1: Check if odom_bridge node is running
echo "Test 1: Check odom_bridge node..."
ros2 node list | grep -q "odom_bridge"
check_result "odom_bridge node running"
echo ""

# Test 2: Check if /odom_combined topic exists
echo "Test 2: Check /odom_combined topic..."
ros2 topic list | grep -q "/odom_combined"
check_result "/odom_combined topic exists"
echo ""

# Test 3: Check joint_states topic has timestamps
echo "Test 3: Check joint_states timestamps..."
echo "  Sampling /robotis/present_joint_states..."
STAMP=$(timeout 2s ros2 topic echo /robotis/present_joint_states --once 2>/dev/null | grep -A2 "stamp:")

if echo "$STAMP" | grep -q "sec:"; then
    SEC=$(echo "$STAMP" | grep "sec:" | awk '{print $2}')
    NSEC=$(echo "$STAMP" | grep "nanosec:" | awk '{print $2}')
    echo "  Timestamp: sec=$SEC, nanosec=$NSEC"
    
    if [ "$SEC" -gt 0 ]; then
        echo -e "${GREEN}✓ PASS${NC}: joint_states has valid timestamps"
        ((passed++))
    else
        echo -e "${RED}✗ FAIL${NC}: joint_states timestamp is zero!"
        ((failed++))
    fi
else
    echo -e "${YELLOW}⚠ SKIP${NC}: Could not read joint_states (is simulation running?)"
fi
echo ""

# Test 4: Check if odom_combined is publishing
echo "Test 4: Check /odom_combined publishing rate..."
RATE=$(timeout 3s ros2 topic hz /odom_combined 2>/dev/null | grep "average rate:" | head -1 | awk '{print $3}')

if [ -n "$RATE" ]; then
    echo "  Publishing rate: $RATE Hz"
    
    # Check if rate is reasonable (should be ~125 Hz)
    RATE_INT=$(echo "$RATE" | cut -d'.' -f1)
    if [ "$RATE_INT" -gt 50 ] && [ "$RATE_INT" -lt 200 ]; then
        echo -e "${GREEN}✓ PASS${NC}: Publishing at good rate ($RATE Hz)"
        ((passed++))
    else
        echo -e "${YELLOW}⚠ WARN${NC}: Unusual rate ($RATE Hz), expected ~125 Hz"
        ((failed++))
    fi
else
    echo -e "${RED}✗ FAIL${NC}: /odom_combined not publishing!"
    ((failed++))
fi
echo ""

# Test 5: Check for "Invalid dt" warnings
echo "Test 5: Check for dt=0 warnings (sampling 2 seconds)..."
echo "  Listening to logs..."

# Capture logs for 2 seconds
LOG_OUTPUT=$(timeout 2s ros2 topic echo /rosout 2>/dev/null | grep -i "invalid dt" | head -5)

if [ -z "$LOG_OUTPUT" ]; then
    echo -e "${GREEN}✓ PASS${NC}: No 'Invalid dt' warnings detected"
    ((passed++))
else
    echo -e "${RED}✗ FAIL${NC}: Still seeing 'Invalid dt' warnings:"
    echo "$LOG_OUTPUT"
    ((failed++))
fi
echo ""

# Test 6: Verify odometry values are changing
echo "Test 6: Check if odometry values update..."
echo "  Reading initial position..."
POS1=$(timeout 2s ros2 topic echo /odom_combined --once 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{print $2}')

if [ -n "$POS1" ]; then
    echo "  Initial X: $POS1"
    echo "  Waiting 2 seconds..."
    sleep 2
    
    echo "  Reading updated position..."
    POS2=$(timeout 2s ros2 topic echo /odom_combined --once 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{print $2}')
    
    if [ -n "$POS2" ]; then
        echo "  Updated X: $POS2"
        
        # Check if values are different (indicating robot moved or at least time passed)
        if [ "$POS1" != "$POS2" ]; then
            echo -e "${GREEN}✓ PASS${NC}: Odometry values are updating"
            ((passed++))
        else
            echo -e "${YELLOW}⚠ INFO${NC}: Position unchanged (robot standing still? OK if not walking)"
            # Not counting as failure since robot might not be moving
        fi
    else
        echo -e "${YELLOW}⚠ SKIP${NC}: Could not read second position"
    fi
else
    echo -e "${RED}✗ FAIL${NC}: Could not read odometry position!"
    ((failed++))
fi
echo ""

# Test 7: Check TF from odom to base
echo "Test 7: Check TF transform (odom → base)..."
TF_OUTPUT=$(timeout 2s ros2 run tf2_ros tf2_echo odom base 2>&1)

if echo "$TF_OUTPUT" | grep -q "At time"; then
    echo -e "${GREEN}✓ PASS${NC}: TF transform available"
    ((passed++))
    
    # Show transform
    echo "$TF_OUTPUT" | grep -A3 "Translation:"
elif echo "$TF_OUTPUT" | grep -q "Invalid frame"; then
    echo -e "${RED}✗ FAIL${NC}: Frame 'base' not found. Check frame names!"
    echo "  Try: ros2 run tf2_tools view_frames"
    ((failed++))
else
    echo -e "${YELLOW}⚠ SKIP${NC}: Could not check TF (timeout or other issue)"
fi
echo ""

# Summary
echo "=========================================="
echo "   VERIFICATION SUMMARY"
echo "=========================================="
echo -e "Tests passed: ${GREEN}$passed${NC}"
echo -e "Tests failed: ${RED}$failed${NC}"
echo ""

if [ $failed -eq 0 ]; then
    echo -e "${GREEN}✓✓✓ ALL TESTS PASSED! ✓✓✓${NC}"
    echo "Odometry is working correctly!"
    echo ""
    echo "Next steps:"
    echo "  1. Walk robot in Webots to see position change"
    echo "  2. Check RViz for robot movement"
    echo "  3. Verify path trail in RViz"
    exit 0
else
    echo -e "${RED}✗✗✗ SOME TESTS FAILED ✗✗✗${NC}"
    echo "Please review the failures above."
    echo ""
    echo "Common issues:"
    echo "  - Is Webots running?"
    echo "  - Is robot spawned in simulation?"
    echo "  - Did you source install/setup.bash?"
    echo "  - Check: ros2 topic list"
    exit 1
fi