#!/bin/bash
# Complete ROS2 Node Killer and Cleanup Script

echo "🛑 COMPLETE ROS2 NODE CLEANUP"
echo "========================================================================"
echo ""

# Function to list all running nodes
list_nodes() {
    echo "Current running nodes:"
    NODES=$(ros2 node list 2>/dev/null)
    if [ -z "$NODES" ]; then
        echo "  (none - all clean!)"
        return 0
    else
        echo "$NODES" | while read node; do
            echo "  - $node"
        done
        return 1
    fi
}

# Step 1: Try graceful shutdown first
echo "1. Attempting graceful shutdown..."
echo "   Press Ctrl+C in all terminal windows running launch files"
echo "   (Give yourself 5 seconds to do this)"
echo ""
for i in {5..1}; do
    echo -ne "   Waiting... $i\r"
    sleep 1
done
echo "   Waiting... Done"
echo ""

# Step 2: Check what's still running
echo "2. Checking remaining nodes..."
list_nodes
STILL_RUNNING=$?

if [ $STILL_RUNNING -eq 0 ]; then
    echo ""
    echo "✅ All nodes stopped gracefully!"
    exit 0
fi

echo ""
echo "3. Some nodes still running. Attempting force kill..."
echo ""

# Step 3: Kill launch files
echo "   a) Killing all ros2 launch processes..."
pkill -9 -f "ros2 launch" 2>/dev/null
sleep 1

# Step 4: Kill specific nodes
echo "   b) Killing specific ROS2 nodes..."
# Kill AMCL
pkill -9 -f "amcl" 2>/dev/null
# Kill map_server
pkill -9 -f "map_server" 2>/dev/null
# Kill lifecycle managers
pkill -9 -f "lifecycle_manager" 2>/dev/null
# Kill detector
pkill -9 -f "detector_fieldline" 2>/dev/null
# Kill converter
pkill -9 -f "simple_pc2scan" 2>/dev/null
# Kill static publishers
pkill -9 -f "static_transform_publisher" 2>/dev/null
pkill -9 -f "odom_publisher" 2>/dev/null

sleep 2

# Step 5: Nuclear option - kill all Python ROS2 processes
echo "   c) Killing all Python ROS2 processes..."
pkill -9 -f "python.*ros2" 2>/dev/null
sleep 1

# Step 6: Final check
echo ""
echo "4. Final verification..."
list_nodes
FINAL_CHECK=$?

if [ $FINAL_CHECK -eq 0 ]; then
    echo ""
    echo "✅ ALL NODES SUCCESSFULLY KILLED!"
    echo ""
else
    echo ""
    echo "⚠️  Some nodes still running. Manual intervention needed."
    echo ""
    echo "Try these commands:"
    echo "  killall -9 python3"
    echo "  killall -9 amcl"
    echo "  killall -9 map_server"
    echo ""
    echo "Then verify:"
    echo "  ros2 node list"
    echo ""
fi

echo "========================================================================"
echo "5. Checking for zombie processes..."
ps aux | grep -E "ros2|amcl|map_server|detector" | grep -v grep | head -10

echo ""
echo "========================================================================"
echo "CLEANUP COMPLETE"
echo "========================================================================"
echo ""
echo "To verify everything is clean:"
echo "  ros2 node list"
echo ""
echo "Expected output: (empty or just /transform_listener_*)"
echo ""