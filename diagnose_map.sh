#!/bin/bash
# Map Server Diagnostics

echo "🗺️  MAP SERVER DIAGNOSTICS"
echo "========================================================================"
echo ""

# Check if map_server node exists
echo "1. Checking if map_server node is running..."
if ros2 node list | grep -q "map_server"; then
    echo "   ✅ map_server node is running"
else
    echo "   ❌ map_server node NOT FOUND!"
    exit 1
fi

echo ""
echo "2. Checking map_server state..."
ros2 lifecycle get /map_server

echo ""
echo "3. Checking /map topic..."
echo "   Publishers:"
ros2 topic info /map | grep "Publisher count"
echo "   Subscribers:"
ros2 topic info /map | grep "Subscription count"
echo "   QoS:"
ros2 topic info /map --verbose | grep -A 5 "QoS profile"

echo ""
echo "4. Trying to receive map with different QoS..."
echo "   Attempting to receive 1 message (timeout 5s)..."
timeout 5 ros2 topic echo /map --once > /tmp/map_test.txt 2>&1

if [ -s /tmp/map_test.txt ]; then
    echo "   ✅ Map received!"
    head -30 /tmp/map_test.txt
else
    echo "   ❌ No map received within 5 seconds"
    cat /tmp/map_test.txt
fi

echo ""
echo "5. Checking map file..."
MAP_FILE=$(ros2 param get /map_server yaml_filename 2>/dev/null | grep "String value" | cut -d: -f2 | xargs)
if [ -z "$MAP_FILE" ]; then
    echo "   ❌ Cannot get map file path from parameters"
else
    echo "   Map file: $MAP_FILE"
    if [ -f "$MAP_FILE" ]; then
        echo "   ✅ Map YAML file exists"
        echo "   Content:"
        cat "$MAP_FILE"
        
        # Check PGM file
        echo ""
        IMAGE_FILE=$(grep "^image:" "$MAP_FILE" | cut -d: -f2 | xargs)
        if [ -z "$IMAGE_FILE" ]; then
            echo "   ❌ No image field in YAML"
        else
            # Handle relative path
            MAP_DIR=$(dirname "$MAP_FILE")
            FULL_IMAGE_PATH="$MAP_DIR/$IMAGE_FILE"
            
            if [ -f "$FULL_IMAGE_PATH" ]; then
                echo "   ✅ Map image exists: $FULL_IMAGE_PATH"
                file "$FULL_IMAGE_PATH"
                ls -lh "$FULL_IMAGE_PATH"
            else
                echo "   ❌ Map image NOT FOUND: $FULL_IMAGE_PATH"
            fi
        fi
    else
        echo "   ❌ Map YAML file NOT FOUND: $MAP_FILE"
    fi
fi

echo ""
echo "6. Checking lifecycle manager..."
echo "   Lifecycle manager for map:"
ros2 node list | grep lifecycle_manager

echo ""
echo "   Bond status:"
ros2 topic echo /bond --once 2>&1 | head -20

echo ""
echo "========================================================================"
echo "✅ Diagnostics complete!"
echo ""
echo "Common fixes:"
echo "  - If lifecycle state is 'inactive', manually activate:"
echo "    ros2 lifecycle set /map_server activate"
echo "  - If map file missing, check path in launch file"
echo "  - If QoS mismatch, map_server needs 'transient_local' durability"
echo ""