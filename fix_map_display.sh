#!/bin/bash
# Fix Map Display in RViz

echo "🗺️  MAP DISPLAY FIX FOR RVIZ"
echo "========================================================================"
echo ""

echo "1. Verifying map is publishing..."
echo ""

timeout 5 ros2 topic echo /map --once > /tmp/map_test.txt 2>&1

if grep -q "header:" /tmp/map_test.txt; then
    echo "   ✅ Map IS publishing"
    
    # Extract info
    WIDTH=$(grep "width:" /tmp/map_test.txt | awk '{print $2}')
    HEIGHT=$(grep "height:" /tmp/map_test.txt | awk '{print $2}')
    RESOLUTION=$(grep "resolution:" /tmp/map_test.txt | awk '{print $2}')
    FRAME=$(grep "frame_id:" /tmp/map_test.txt | head -1 | awk '{print $2}')
    
    echo "   Map info:"
    echo "     Frame: $FRAME"
    echo "     Size: ${WIDTH}x${HEIGHT} cells"
    echo "     Resolution: ${RESOLUTION}m/cell"
else
    echo "   ❌ Map NOT publishing!"
    exit 1
fi

echo ""
echo "2. Checking map QoS..."
echo ""

# Map uses TRANSIENT_LOCAL durability
ros2 topic info /map --verbose 2>&1 | grep -A 5 "QoS profile:" | head -10

echo ""
echo "========================================================================"
echo "3. RVIZ CONFIGURATION FIX"
echo "========================================================================"
echo ""

cat << 'EOF'
The "No map received" error in RViz is usually due to:

A. WRONG FIXED FRAME
   ✅ FIX: Set Fixed Frame to "map" at TOP of RViz window

B. MAP DISPLAY NOT CONFIGURED CORRECTLY
   ✅ FIX: 
   1. Delete existing "Map" display
   2. Click "Add" button
   3. Select "Map" from list
   4. Configure:
      - Topic: /map
      - Color Scheme: map
      - Alpha: 0.7 or 0.8
      - Update Topic: (leave empty)

C. QOS MISMATCH (rare)
   ✅ FIX: RViz should auto-detect, but if not:
   - Reliability QoS: Reliable
   - Durability QoS: Transient Local

D. MAP FRAME NOT IN TF TREE
   ✅ FIX: Check TF includes 'map' frame
   
   Run this command:
   ros2 run tf2_ros tf2_echo map odom

   Should show transform. If error, static_transform_publisher not running.

STEP-BY-STEP FIX IN RVIZ:

1. Check Fixed Frame (top of window):
   [Fixed Frame: map ▼]
   
   If it says "odom" or "cam_link", change to "map"

2. Delete old Map display:
   - Click "Map" in left panel
   - Click "Remove" button at bottom

3. Add new Map:
   - Click "Add" button
   - By display type → Map
   - Click "OK"

4. Configure Map display:
   Click "Map" in left panel, then set:
   
   Topic:
     [/map                          ▼]
   
   Color Scheme:
     [map                           ▼]
   
   Alpha:
     [0.7                           ]
   
   Draw Behind:
     [✓] (check this box)

5. If map still not showing, check Status at bottom:
   
   Status: Ok                  ← Should show this
   
   NOT:
   Status: Error               ← Means problem
   Status: No map received     ← Means QoS/topic issue

6. Force refresh:
   - Toggle Map display off/on (checkbox in left panel)
   - Or restart RViz

EOF

echo ""
echo "========================================================================"
echo "4. MANUAL VERIFICATION"
echo "========================================================================"
echo ""

cat << 'EOF'
To verify map is truly available to RViz:

# Test 1: Check topic with default QoS
ros2 topic echo /map --once | head -30

# Test 2: Check with RViz-compatible QoS (Python)
python3 << 'PYEOF'
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from nav_msgs.msg import OccupancyGrid

rclpy.init()
node = Node('map_test')

# RViz-compatible QoS
qos = QoSProfile(
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
    history=HistoryPolicy.KEEP_LAST
)

received = False

def callback(msg):
    global received
    if not received:
        received = True
        print(f"✅ Map received: {msg.info.width}x{msg.info.height}")
        print(f"   Frame: {msg.header.frame_id}")
        print(f"   Resolution: {msg.info.resolution}m/cell")
        rclpy.shutdown()

sub = node.create_subscription(OccupancyGrid, '/map', callback, qos)

print("Waiting for map (5 seconds)...")
import time
time.sleep(5)

if not received:
    print("❌ Map not received with RViz QoS!")
    
rclpy.shutdown()
PYEOF

EOF

echo ""
echo "========================================================================"
echo "5. TF TREE CHECK"
echo "========================================================================"
echo ""

# Check if map frame exists in TF
echo "Checking if 'map' frame exists in TF tree..."
timeout 3 ros2 run tf2_ros tf2_echo map odom 2>&1 | head -5

echo ""
echo "If you see 'Invalid frame ID', map is not in TF tree!"
echo "This means static_transform_publisher is not running."
echo ""

echo "========================================================================"
echo "DONE"
echo "========================================================================"
echo ""
echo "After following the RViz configuration steps above,"
echo "you should see a white field outline on black background."
echo ""
echo "If still not working, save RViz config and restart RViz:"
echo "  File → Save Config As → backup.rviz"
echo "  Close RViz"
echo "  Start fresh: rviz2"
echo ""