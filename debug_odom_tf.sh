#!/bin/bash
# Debug script for odometry and TF issues

echo "=========================================="
echo "  Odometry & TF Debug Script"
echo "=========================================="
echo ""

# Colors
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Test 1: Check if /odom_combined is publishing
echo "Test 1: Checking /odom_combined topic..."
if ros2 topic list | grep -q "/odom_combined"; then
    echo -e "${GREEN}✓${NC} /odom_combined exists"
    
    # Get initial position
    echo "Getting initial position..."
    INITIAL=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{print $2}')
    echo "Initial X position: $INITIAL"
    
    echo ""
    echo "${YELLOW}>>> Please make robot WALK in Webots now! <<<${NC}"
    echo "Waiting 5 seconds..."
    sleep 5
    
    # Get position after walking
    echo "Getting position after walking..."
    AFTER=$(ros2 topic echo /odom_combined --once 2>/dev/null | grep -A3 "position:" | grep "x:" | awk '{print $2}')
    echo "After X position: $AFTER"
    
    if [ "$INITIAL" != "$AFTER" ]; then
        echo -e "${GREEN}✓${NC} Odometry IS updating! (Good)"
    else
        echo -e "${RED}✗${NC} Odometry NOT updating! (Problem: odom_bridge not working)"
    fi
else
    echo -e "${RED}✗${NC} /odom_combined does not exist!"
fi

echo ""
echo "=========================================="

# Test 2: Check if TF is being published
echo "Test 2: Checking TF..."
if ros2 topic list | grep -q "/tf"; then
    echo -e "${GREEN}✓${NC} /tf topic exists"
    
    # Check rate
    RATE=$(timeout 2s ros2 topic hz /tf 2>/dev/null | grep "average rate:" | awk '{print $3}')
    if [ ! -z "$RATE" ]; then
        echo -e "${GREEN}✓${NC} /tf publishing at $RATE Hz"
    else
        echo -e "${RED}✗${NC} /tf not publishing"
    fi
else
    echo -e "${RED}✗${NC} /tf topic does not exist!"
fi

echo ""
echo "=========================================="

# Test 3: Check specific transform
echo "Test 3: Checking odom -> base transform..."
TF_OUTPUT=$(timeout 2s ros2 run tf2_ros tf2_echo odom base 2>&1)
if echo "$TF_OUTPUT" | grep -q "Translation:"; then
    echo -e "${GREEN}✓${NC} Transform odom -> base exists"
    echo "$TF_OUTPUT" | grep "Translation:"
else
    echo -e "${RED}✗${NC} Transform odom -> base NOT found"
    echo "Error: $TF_OUTPUT"
fi

echo ""
echo "=========================================="

# Test 4: Check nodes
echo "Test 4: Checking required nodes..."

NODES=("odom_bridge" "odom_to_tf_publisher")
for node in "${NODES[@]}"; do
    if ros2 node list | grep -q "/$node"; then
        echo -e "${GREEN}✓${NC} Node /$node is running"
    else
        echo -e "${RED}✗${NC} Node /$node is NOT running"
    fi
done

echo ""
echo "=========================================="

# Test 5: Generate TF tree
echo "Test 5: Generating TF tree..."
echo "Running: ros2 run tf2_tools view_frames"
ros2 run tf2_tools view_frames 2>/dev/null &
VIEWFRAMES_PID=$!

echo "Waiting 5 seconds for TF data..."
sleep 5

if [ -f frames.pdf ]; then
    echo -e "${GREEN}✓${NC} TF tree generated: frames.pdf"
    echo "Opening PDF..."
    evince frames.pdf 2>/dev/null &
else
    echo -e "${RED}✗${NC} Failed to generate TF tree"
fi

kill $VIEWFRAMES_PID 2>/dev/null

echo ""
echo "=========================================="
echo "  Debug Complete"
echo "=========================================="
echo ""
echo "Summary:"
echo "1. If odometry NOT updating -> Problem in odom_bridge_node"
echo "2. If /tf not publishing -> Problem in odom_to_tf.py"
echo "3. If transform not found -> Frame name mismatch"
echo ""