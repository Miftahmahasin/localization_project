#!/bin/bash
# Manually Activate Map Server

echo "🗺️  MANUAL MAP SERVER ACTIVATION"
echo "========================================================================"
echo ""

# Check current state
echo "Current map_server state:"
CURRENT_STATE=$(ros2 lifecycle get /map_server 2>/dev/null)
echo "$CURRENT_STATE"

if echo "$CURRENT_STATE" | grep -q "active"; then
    echo ""
    echo "✅ Map server is already ACTIVE"
    echo ""
    echo "Checking if map is being published..."
    timeout 3 ros2 topic echo /map --once > /dev/null 2>&1
    if [ $? -eq 0 ]; then
        echo "✅ Map IS being published!"
    else
        echo "❌ Map NOT being published despite active state"
        echo "   Try restarting the launch file"
    fi
    exit 0
fi

if echo "$CURRENT_STATE" | grep -q "unconfigured"; then
    echo ""
    echo "📝 Map server is UNCONFIGURED, configuring..."
    ros2 lifecycle set /map_server configure
    sleep 1
fi

if echo "$CURRENT_STATE" | grep -q "inactive"; then
    echo ""
    echo "⚡ Map server is INACTIVE, activating..."
    ros2 lifecycle set /map_server activate
    sleep 1
fi

# Check new state
echo ""
echo "New map_server state:"
ros2 lifecycle get /map_server

echo ""
echo "Waiting for map to be published..."
sleep 2

timeout 5 ros2 topic echo /map --once > /tmp/map_check.txt 2>&1

if [ -s /tmp/map_check.txt ] && ! grep -q "WARNING" /tmp/map_check.txt; then
    echo "✅ SUCCESS! Map is now being published!"
    echo ""
    echo "Map info:"
    head -20 /tmp/map_check.txt
    
    echo ""
    echo "========================================================================"
    echo "✅ Map server activated successfully!"
    echo ""
    echo "Now check AMCL:"
    echo "  python3 /mnt/user-data/outputs/amcl_debugger.py"
    echo ""
    echo "If map is received, particles should appear!"
    echo "========================================================================"
else
    echo "❌ Map still not being published"
    echo ""
    echo "Error:"
    cat /tmp/map_check.txt
    echo ""
    echo "Try:"
    echo "  1. Check map file exists: diagnose_map.sh"
    echo "  2. Restart launch file"
    echo "  3. Check logs for errors"
fi