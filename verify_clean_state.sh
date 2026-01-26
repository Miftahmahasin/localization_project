#!/bin/bash
# Verify ROS2 System is Clean Before Launch

echo "✅ PRE-LAUNCH VERIFICATION"
echo "========================================================================"
echo ""

READY=true

# Check 1: No nodes running
echo "1. Checking for running nodes..."
NODES=$(ros2 node list 2>/dev/null | grep -v "transform_listener")
if [ -z "$NODES" ]; then
    echo "   ✅ No nodes running (clean state)"
else
    echo "   ❌ Found running nodes:"
    echo "$NODES" | while read node; do
        echo "      - $node"
    done
    READY=false
fi

echo ""
echo "2. Checking for stale processes..."
STALE=$(ps aux | grep -E "amcl|map_server|detector_fieldline|simple_pc2scan" | grep -v grep | wc -l)
if [ $STALE -eq 0 ]; then
    echo "   ✅ No stale ROS2 processes"
else
    echo "   ❌ Found $STALE stale processes"
    ps aux | grep -E "amcl|map_server|detector_fieldline|simple_pc2scan" | grep -v grep | head -5
    READY=false
fi

echo ""
echo "3. Checking ROS2 environment..."
if [ -z "$ROS_DISTRO" ]; then
    echo "   ❌ ROS_DISTRO not set!"
    echo "      Run: source ~/basbot/install/setup.bash"
    READY=false
else
    echo "   ✅ ROS_DISTRO: $ROS_DISTRO"
fi

echo ""
echo "4. Checking required files..."
LAUNCH_FILE="$HOME/basbot/install/soccer_object_localization/share/soccer_object_localization/launch/amcl_final_fixed.launch.py"
if [ -f "$LAUNCH_FILE" ]; then
    echo "   ✅ Launch file exists"
else
    echo "   ❌ Launch file not found: $LAUNCH_FILE"
    READY=false
fi

MAP_FILE="$HOME/basbot/install/soccer_object_localization/share/soccer_object_localization/maps/soccer_field.yaml"
if [ -f "$MAP_FILE" ]; then
    echo "   ✅ Map file exists"
else
    echo "   ❌ Map file not found: $MAP_FILE"
    READY=false
fi

echo ""
echo "5. Checking camera topic..."
timeout 2 ros2 topic hz /robotis_op3/camera/image_raw 2>&1 | grep -q "average rate"
if [ $? -eq 0 ]; then
    echo "   ✅ Camera publishing"
else
    echo "   ⚠️  Camera not publishing (may need to start simulator/robot first)"
fi

echo ""
echo "========================================================================"
if [ "$READY" = true ]; then
    echo "✅ SYSTEM READY FOR LAUNCH!"
    echo ""
    echo "You can now run:"
    echo "  ros2 launch soccer_object_localization amcl_final_fixed.launch.py white_threshold:=170"
    echo ""
else
    echo "❌ SYSTEM NOT READY"
    echo ""
    echo "Please fix the issues above before launching"
    echo ""
fi
echo "========================================================================"