#!/bin/bash
# Force Map Server to Publish
# Use this if lifecycle manager is not working properly

echo "🗺️  FORCE MAP SERVER ACTIVATION"
echo "========================================================================"
echo ""

# Find map file
MAP_FILE="$HOME/basbot/install/soccer_object_localization/share/soccer_object_localization/maps/soccer_field.yaml"

if [ ! -f "$MAP_FILE" ]; then
    echo "❌ Map file not found: $MAP_FILE"
    exit 1
fi

echo "✅ Map file found: $MAP_FILE"
echo ""

# Kill existing map_server if any
echo "Killing existing map_server..."
pkill -f "map_server" 2>/dev/null
sleep 1

# Start map_server manually
echo "Starting map_server manually..."
ros2 run nav2_map_server map_server \
    --ros-args \
    -p yaml_filename:="$MAP_FILE" \
    -p topic_name:=map \
    -p frame_id:=map &

MAP_PID=$!
echo "Map server started with PID: $MAP_PID"

# Wait for node to start
sleep 2

# Configure
echo ""
echo "Configuring map_server..."
ros2 lifecycle set /map_server configure
sleep 1

# Activate
echo "Activating map_server..."
ros2 lifecycle set /map_server activate
sleep 2

# Check
echo ""
echo "Checking if map is published..."
timeout 5 ros2 topic echo /map --once > /tmp/map_verify.txt 2>&1

if [ -s /tmp/map_verify.txt ] && ! grep -q "WARNING" /tmp/map_verify.txt; then
    echo "✅ SUCCESS! Map is being published!"
    echo ""
    head -20 /tmp/map_verify.txt
    echo ""
    echo "========================================================================"
    echo "✅ Map server is running and publishing!"
    echo ""
    echo "Keep this terminal open (map_server is running here)"
    echo "In another terminal, check AMCL:"
    echo "  python3 /mnt/user-data/outputs/amcl_debugger.py"
    echo ""
    echo "Press Ctrl+C to stop map_server"
    echo "========================================================================"
    
    # Keep alive
    wait $MAP_PID
else
    echo "❌ Map still not being published"
    echo ""
    cat /tmp/map_verify.txt
    
    # Kill map_server
    kill $MAP_PID 2>/dev/null
fi