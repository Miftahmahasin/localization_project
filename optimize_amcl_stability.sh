#!/bin/bash
# Optimize AMCL for Low Scan Quality / Noisy Environment

echo "🔧 OPTIMIZING AMCL FOR STABILITY"
echo "========================================================================"
echo ""

echo "Current AMCL status:"
ros2 lifecycle get /amcl

echo ""
echo "Setting AMCL parameters for better stability..."
echo ""

# Increase particles for better coverage
echo "1. Increasing particle count..."
ros2 param set /amcl min_particles 2000
ros2 param set /amcl max_particles 5000

# More tolerant to noise
echo "2. Adjusting noise tolerance..."
ros2 param set /amcl laser_likelihood_max_dist 1.0
ros2 param set /amcl laser_sigma_hit 0.3

# Slower updates but more stable
echo "3. Adjusting update thresholds for stability..."
ros2 param set /amcl update_min_d 0.05  # 5cm
ros2 param set /amcl update_min_a 0.1   # ~6 degrees

# Trust scan less, use more particles
echo "4. Adjusting resample parameters..."
ros2 param set /amcl resample_interval 2

# Longer transform tolerance
echo "5. Increasing transform tolerance..."
ros2 param set /amcl transform_tolerance 1.0

echo ""
echo "✅ Parameters updated!"
echo ""

echo "6. Resetting AMCL with new parameters..."
echo ""

# Set initial pose with larger uncertainty
ros2 topic pub /initialpose geometry_msgs/PoseWithCovarianceStamped "{
  header: {frame_id: 'map'},
  pose: {
    pose: {
      position: {x: 0.0, y: 0.0, z: 0.0},
      orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
    },
    covariance: [
      1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
      0.0, 0.0, 0.0, 0.0, 0.0, 0.2
    ]
  }
}" --once &

echo "Waiting 5 seconds for AMCL to initialize..."
sleep 5

echo ""
echo "7. Checking result..."
echo ""

ros2 topic echo /amcl_pose --once 2>&1 | grep -A 6 "covariance:" | head -7

echo ""
echo "========================================================================"
echo "DONE"
echo "========================================================================"
echo ""

cat << 'EOF'
If still unstable:

1. Improve scan quality:
   ros2 param set /detector_fieldline detection.white_threshold 140
   
   Lower threshold = more detected lines = better scan

2. Reduce scan noise:
   The scan dots should align with field lines in RViz
   If they don't, detector is detecting wrong things

3. Check robot is actually on the field in simulator
   AMCL can only localize if scan matches map!

4. Try global localization:
   ros2 param set /amcl set_initial_pose false
   
   This makes AMCL search entire map

EOF

echo "========================================================================"