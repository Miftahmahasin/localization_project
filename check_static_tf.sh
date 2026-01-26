#!/bin/bash
# Check Static TF Transforms

echo "🔍 CHECKING STATIC TF TRANSFORMS"
echo "========================================================================"
echo ""

echo "1. Current base_link → head_link transform:"
echo "----------------------------------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_echo base_link head_link 2>&1 | head -15

echo ""
echo "2. Current head_link → cam_link transform:"
echo "----------------------------------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_echo head_link cam_link 2>&1 | head -15

echo ""
echo "3. Full transform base_link → cam_link:"
echo "----------------------------------------------------------------------"
timeout 3 ros2 run tf2_ros tf2_echo base_link cam_link 2>&1 | head -15

echo ""
echo "========================================================================"
echo "ANALYSIS"
echo "========================================================================"
echo ""

cat << 'EOF'
The scan/pointcloud floating above field means the Z translation is wrong!

Expected robot geometry (typical humanoid):
- base_link: Robot base (on ground or at hip)
- head_link: ~0.40-0.50m above base_link
- cam_link: ~0.05-0.10m above head_link, tilted down ~20°

Common issue: cam_link Z is TOO HIGH

If current transform shows Z > 0.6m, that's the problem!

To fix, we need to either:
A. Fix op3_static_transforms.py
B. Replace with correct static_transform_publisher in launch file

EOF

echo "========================================================================"
echo ""

echo "4. Checking if op3_static_transforms.py exists:"
echo "----------------------------------------------------------------------"

SCRIPT_PATH=$(find ~/basbot -name "op3_static_transforms.py" 2>/dev/null | head -1)

if [ ! -z "$SCRIPT_PATH" ]; then
    echo "Found: $SCRIPT_PATH"
    echo ""
    echo "Contents:"
    cat "$SCRIPT_PATH"
else
    echo "❌ op3_static_transforms.py NOT FOUND!"
    echo ""
    echo "This means static_tf_publisher in launch file will fail!"
    echo "We need to replace it with correct static_transform_publisher nodes."
fi

echo ""
echo "========================================================================"
echo "RECOMMENDED FIX"
echo "========================================================================"
echo ""

cat << 'EOF'
Option 1: If op3_static_transforms.py exists but has wrong values
────────────────────────────────────────────────────────────────────
Edit the script to correct Z values.

Option 2: Replace with inline static_transform_publisher in launch file
────────────────────────────────────────────────────────────────────
More reliable! Add these to launch file:

    # base_link → head_link (adjust Z to match robot)
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_head_tf',
        arguments=['0', '0', '0.395', '0', '0', '0', 'base_link', 'head_link']
    ),
    
    # head_link → cam_link (tilt down ~20 degrees)
    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='head_to_cam_tf',
        arguments=['0.08', '0', '0.08', '0', '-0.174', '0', '0.985', 'head_link', 'cam_link']
    ),

Option 3: No head_link needed - direct base_link → cam_link
────────────────────────────────────────────────────────────────────
Simpler! Just one transform:

    Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_cam_tf',
        arguments=['0.08', '0', '0.475', '0', '-0.174', '0', '0.985', 'base_link', 'cam_link']
    ),

This puts camera 47.5cm above base, tilted down 20°.

EOF

echo "========================================================================"