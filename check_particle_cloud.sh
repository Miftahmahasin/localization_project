#!/bin/bash
# MANUAL CHECK - Particle Cloud Diagnostic

echo "🔍 PARTICLE CLOUD DIAGNOSTIC"
echo "========================================================================"
echo ""

echo "1. Checking AMCL node..."
if ros2 node list 2>&1 | grep -q "^/amcl$"; then
    echo "✅ AMCL node running"
    
    # Check state
    AMCL_STATE=$(ros2 lifecycle get /amcl 2>&1 | grep "state id" | awk '{print $4}')
    if [ "$AMCL_STATE" == "3" ]; then
        echo "✅ AMCL state: ACTIVE"
    else
        echo "⚠️  AMCL state: $AMCL_STATE (not active!)"
        echo "   Fix: ros2 lifecycle set /amcl activate"
    fi
else
    echo "❌ AMCL node NOT running!"
    echo "   Start AMCL first!"
    exit 1
fi

echo ""
echo "2. Checking /particle_cloud topic..."
if ros2 topic list 2>&1 | grep -q "^/particle_cloud$"; then
    echo "✅ Topic exists"
    
    echo ""
    echo "3. Checking publishers..."
    PUB_COUNT=$(ros2 topic info /particle_cloud 2>&1 | grep "Publisher count:" | awk '{print $3}')
    if [ "$PUB_COUNT" == "1" ]; then
        echo "✅ 1 publisher (AMCL)"
    else
        echo "⚠️  Publisher count: $PUB_COUNT"
    fi
    
    echo ""
    echo "4. Checking message type..."
    MSG_TYPE=$(ros2 topic info /particle_cloud 2>&1 | grep "Type:" | awk '{print $2}')
    echo "   Type: $MSG_TYPE"
    if [ "$MSG_TYPE" == "geometry_msgs/msg/PoseArray" ]; then
        echo "✅ Correct type"
    else
        echo "⚠️  Wrong type!"
    fi
    
    echo ""
    echo "5. Testing if data is being published..."
    echo "   Waiting for message (5 second timeout)..."
    
    if timeout 5 ros2 topic echo /particle_cloud --once > /tmp/particle_test.txt 2>&1; then
        POSE_COUNT=$(grep -c "pose:" /tmp/particle_test.txt)
        echo "✅ Data publishing! ($POSE_COUNT particles)"
        
        if [ "$POSE_COUNT" -lt 100 ]; then
            echo "⚠️  Very few particles ($POSE_COUNT)"
            echo "   Increase: ros2 param set /amcl min_particles 500"
        fi
    else
        echo "❌ No data received!"
        echo ""
        echo "Possible causes:"
        echo "  1. AMCL not initialized (need to set initial pose)"
        echo "  2. AMCL in wrong state"
        echo "  3. No localization running"
        echo ""
        echo "Try setting initial pose:"
        echo "  ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \\"
        echo "    '{header: {frame_id: \"map\"}, pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}' --once"
    fi
    
else
    echo "❌ Topic does NOT exist!"
    echo ""
    echo "Possible causes:"
    echo "  1. AMCL not running"
    echo "  2. AMCL not configured correctly"
    echo "  3. Launch file issue"
    echo ""
    echo "Check AMCL configuration in launch file:"
    echo "  - Make sure AMCL node is started"
    echo "  - Check lifecycle manager includes 'amcl'"
fi

echo ""
echo "========================================================================"
echo "PARAMETER CHECK"
echo "========================================================================"
echo ""

if ros2 node list 2>&1 | grep -q "^/amcl$"; then
    echo "Key AMCL parameters:"
    echo ""
    
    ros2 param get /amcl min_particles 2>&1 | head -1
    ros2 param get /amcl max_particles 2>&1 | head -1
    ros2 param get /amcl resample_interval 2>&1 | head -1
    ros2 param get /amcl update_min_d 2>&1 | head -1
    ros2 param get /amcl update_min_a 2>&1 | head -1
    
    echo ""
    echo "Frame IDs:"
    ros2 param get /amcl global_frame_id 2>&1 | head -1
    ros2 param get /amcl odom_frame_id 2>&1 | head -1
    ros2 param get /amcl base_frame_id 2>&1 | head -1
fi

echo ""
echo "========================================================================"
echo "QUICK FIX"
echo "========================================================================"
echo ""

cat << 'EOF'
If /particle_cloud still not working, try this:

1. Set initial pose:
   ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped \
     '{header: {frame_id: "map"}, 
       pose: {pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}' --once

2. Wait 3 seconds, then check again:
   ros2 topic echo /particle_cloud --once

3. If still no data, restart AMCL:
   ros2 lifecycle set /amcl deactivate
   ros2 lifecycle set /amcl cleanup
   ros2 lifecycle set /amcl configure
   ros2 lifecycle set /amcl activate
   
   Then set initial pose again (step 1)

4. In RViz:
   - Remove PoseArray display
   - Add it again with topic: /particle_cloud
   - Make sure "Status: Ok" (not "Error")

EOF

echo "========================================================================"