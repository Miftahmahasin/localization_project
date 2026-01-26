#!/bin/bash
echo "=== COMPLETE DIAGNOSTIC DATA COLLECTION ==="
echo ""

echo "1. Topic list:"
ros2 topic list

echo ""
echo "2. Node list:"
ros2 node list

echo ""
echo "3. AMCL ALL parameters:"
ros2 param list /amcl
ros2 param dump /amcl

echo ""
echo "4. Scan sample (first message):"
timeout 2 ros2 topic echo /field_scan --once | head -100

echo ""
echo "5. Point cloud sample:"
timeout 2 ros2 topic echo /field_point_cloud --once | head -50

echo ""
echo "6. AMCL pose (3 samples):"
for i in {1..3}; do
  echo "Sample $i:"
  ros2 topic echo /amcl_pose --once | head -30
  sleep 1
done

echo ""
echo "7. TF /tf topic sample:"
timeout 2 ros2 topic echo /tf | head -100

echo ""
echo "8. All running nodes info:"
for node in $(ros2 node list); do
  echo "=== $node ==="
  ros2 node info $node 2>&1 | head -50
done