#!/bin/bash
# ENABLE PARTICLE CLOUD PUBLISHING

echo "🔧 ENABLING PARTICLE CLOUD PUBLISHING"
echo "========================================================================"
echo ""

echo "Step 1: Check current AMCL parameters..."
echo ""

# Check if save_pose_rate exists
SAVE_RATE=$(ros2 param get /amcl save_pose_rate 2>&1)
if echo "$SAVE_RATE" | grep -q "not set\|Unknown"; then
    echo "⚠️  save_pose_rate not set (this is OK)"
else
    echo "Current save_pose_rate: $SAVE_RATE"
fi

echo ""
echo "Step 2: Setting parameters to enable particle publishing..."
echo ""

# Enable particle cloud publishing (usually enabled by default, but make sure)
# Note: Nav2 AMCL always publishes particle cloud, no parameter needed
# But we can adjust the rate

# Set minimum update thresholds (particles update when robot moves)
ros2 param set /amcl update_min_d 0.02
echo "✅ update_min_d: 0.02 (update every 2cm)"

ros2 param set /amcl update_min_a 0.05
echo "✅ update_min_a: 0.05 (update every 3°)"

# Set resample interval (how often to resample particles)
ros2 param set /amcl resample_interval 1
echo "✅ resample_interval: 1 (resample every update)"

# Make sure we have enough particles to see
ros2 param set /amcl min_particles 500
ros2 param set /amcl max_particles 2000
echo "✅ particles: 500-2000"

echo ""
echo "Step 3: Restart AMCL to apply settings..."
echo ""

ros2 lifecycle set /amcl deactivate
sleep 1
ros2 lifecycle set /amcl cleanup  
sleep 1
ros2 lifecycle set /amcl configure
sleep 1
ros2 lifecycle set /amcl activate
sleep 2

echo "✅ AMCL restarted"

echo ""
echo "Step 4: Set initial pose to trigger particle generation..."
echo ""

ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}},
    covariance: [0.25, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.25, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                 0.0, 0.0, 0.0, 0.0, 0.0, 0.1]
  }
}" --once &

sleep 5

echo ""
echo "========================================================================"
echo "VERIFICATION"
echo "========================================================================"
echo ""

echo "Checking if /particle_cloud topic exists..."
if ros2 topic list 2>&1 | grep -q "^/particle_cloud$"; then
    echo "✅ /particle_cloud topic exists!"
    
    echo ""
    echo "Checking topic info..."
    ros2 topic info /particle_cloud
    
    echo ""
    echo "Checking publish rate..."
    timeout 5 ros2 topic hz /particle_cloud 2>&1 | head -3
    
    echo ""
    echo "Sample data (first message):"
    timeout 3 ros2 topic echo /particle_cloud --once 2>&1 | head -20
else
    echo "❌ /particle_cloud topic NOT found!"
    echo ""
    echo "Troubleshooting:"
    echo "1. Check AMCL is running: ros2 node list | grep amcl"
    echo "2. Check AMCL state: ros2 lifecycle get /amcl"
    echo "3. Check AMCL logs for errors"
fi

echo ""
echo "========================================================================"
echo "RVIZ CONFIGURATION"
echo "========================================================================"
echo ""

cat << 'EOF'
In RViz, add PoseArray display:

1. Click "Add" button
2. By topic tab → /particle_cloud
3. Select "PoseArray"
4. Click OK

Settings:
  - Topic: /particle_cloud
  - Color: Yellow (255, 255, 0) - stands out!
  - Alpha: 0.8
  - Arrow Length: 0.3
  - Shape: Arrow (Flat)

What you should see:
  - Yellow arrows spread around robot position
  - More arrows = more particles
  - Arrows should cluster near actual position
  - If spread out = high uncertainty
  - If jumping around = ambiguity problem

EOF

echo "========================================================================"