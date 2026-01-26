#!/bin/bash
# Comprehensive AMCL Status Check

echo "🔬 COMPREHENSIVE AMCL STATUS CHECK"
echo "========================================================================"
echo ""

# 1. Check if AMCL node exists
echo "1. AMCL Node Status:"
if ros2 node list | grep -q "amcl"; then
    echo "   ✅ AMCL node is running"
else
    echo "   ❌ AMCL node NOT FOUND!"
    exit 1
fi

echo ""
echo "2. AMCL Lifecycle State:"
ros2 lifecycle get /amcl 2>/dev/null || echo "   Not a lifecycle node"

echo ""
echo "3. Check AMCL Publications:"
echo ""
echo "   a) Checking /amcl_pose..."
timeout 3 ros2 topic echo /amcl_pose --once > /tmp/amcl_pose_check.txt 2>&1
if [ -s /tmp/amcl_pose_check.txt ] && ! grep -q "WARNING" /tmp/amcl_pose_check.txt; then
    echo "      ✅ AMCL IS PUBLISHING POSE!"
    echo "      Pose preview:"
    head -15 /tmp/amcl_pose_check.txt | grep -A 5 "position:"
else
    echo "      ❌ AMCL NOT publishing pose"
fi

echo ""
echo "   b) Checking /particle_cloud..."
timeout 3 ros2 topic echo /particle_cloud --once > /tmp/particles_check.txt 2>&1
if [ -s /tmp/particles_check.txt ] && ! grep -q "WARNING" /tmp/particles_check.txt; then
    echo "      ✅ AMCL IS PUBLISHING PARTICLES!"
    echo "      Particle count:"
    grep -m1 "poses:" /tmp/particles_check.txt -A 1
else
    echo "      ❌ AMCL NOT publishing particles"
fi

echo ""
echo "   c) Checking /particlecloud (alternate spelling)..."
timeout 3 ros2 topic echo /particlecloud --once > /tmp/particlecloud_check.txt 2>&1
if [ -s /tmp/particlecloud_check.txt ] && ! grep -q "WARNING" /tmp/particlecloud_check.txt; then
    echo "      ✅ FOUND at /particlecloud!"
else
    echo "      ❌ Not at /particlecloud"
fi

echo ""
echo "   d) Checking /tf (AMCL should publish map→odom)..."
timeout 2 ros2 topic echo /tf --once > /tmp/tf_check.txt 2>&1
if grep -q "map" /tmp/tf_check.txt && grep -q "odom" /tmp/tf_check.txt; then
    echo "      ✅ TF contains map→odom transform"
else
    echo "      ⚠️  No map→odom in /tf"
fi

echo ""
echo "4. Check AMCL is receiving scan:"
timeout 2 ros2 topic hz /field_scan 2>&1 | head -3

echo ""
echo "5. Check what topics AMCL is actually publishing:"
ros2 node info /amcl 2>/dev/null | grep "Publishers:" -A 10

echo ""
echo "6. Check AMCL parameters:"
echo "   scan_topic:"
ros2 param get /amcl scan_topic 2>/dev/null

echo ""
echo "   set_initial_pose:"
ros2 param get /amcl set_initial_pose 2>/dev/null

echo ""
echo "   base_frame_id:"
ros2 param get /amcl base_frame_id 2>/dev/null

echo ""
echo "7. List ALL available topics (to find particle topic):"
ros2 topic list | grep -i "particle\|amcl"

echo ""
echo "========================================================================"
echo "DIAGNOSIS:"
echo "========================================================================"

# Diagnosis
HAS_POSE=false
HAS_PARTICLES=false

[ -s /tmp/amcl_pose_check.txt ] && ! grep -q "WARNING" /tmp/amcl_pose_check.txt && HAS_POSE=true
[ -s /tmp/particles_check.txt ] && ! grep -q "WARNING" /tmp/particles_check.txt && HAS_PARTICLES=true

if $HAS_POSE && $HAS_PARTICLES; then
    echo "✅ AMCL IS WORKING PERFECTLY!"
    echo "   - Publishing pose"
    echo "   - Publishing particles"
    echo "   - Localization is active"
    echo ""
    echo "Check in RViz:"
    echo "   - Add PoseWithCovarianceStamped → /amcl_pose"
    echo "   - Add PoseArray or ParticleCloud → /particle_cloud"
    echo ""
elif $HAS_POSE && ! $HAS_PARTICLES; then
    echo "⚠️  AMCL is publishing pose but not particles"
    echo "   This might be normal - particles published less frequently"
    echo "   Wait a few seconds and check /particle_cloud again"
    echo ""
elif ! $HAS_POSE && ! $HAS_PARTICLES; then
    echo "❌ AMCL IS NOT LOCALIZING"
    echo ""
    echo "Possible causes:"
    echo "   1. Map not received (but diagnostics say map is OK...)"
    echo "   2. Scan frame mismatch (base_frame_id should match scan frame)"
    echo "   3. No initial pose set"
    echo "   4. Transform issues"
    echo ""
    echo "Try setting initial pose:"
    echo "   python3 /mnt/user-data/outputs/set_initial_pose.py"
    echo ""
fi

echo "========================================================================"