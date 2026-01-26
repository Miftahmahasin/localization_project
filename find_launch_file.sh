#!/bin/bash
# Find and Verify Active Launch File

echo "🔍 FINDING ACTIVE LAUNCH FILE"
echo "========================================================================"
echo ""

# Method 1: Check install directory (what ROS2 actually uses)
echo "1. Checking ROS2 install directory..."
INSTALL_LAUNCH=$(find ~/basbot/install -name "amcl*.launch.py" 2>/dev/null)

if [ -n "$INSTALL_LAUNCH" ]; then
    echo "   Found launch files in install:"
    echo "$INSTALL_LAUNCH" | while read file; do
        echo "     - $file"
    done
    
    echo ""
    echo "2. Checking which one has REMAPPING..."
    echo "$INSTALL_LAUNCH" | while read file; do
        if grep -q "remappings" "$file"; then
            if grep -q "field_scan" "$file"; then
                echo "   ✅ $file HAS remapping to field_scan"
            else
                echo "   ⚠️  $file has remappings but NOT to field_scan"
            fi
        else
            echo "   ❌ $file NO remappings found"
        fi
    done
else
    echo "   ❌ No launch files found in install directory!"
fi

echo ""
echo "========================================================================"
echo "3. Checking source directory..."
SRC_LAUNCH=$(find ~/basbot/src -name "amcl*.launch.py" 2>/dev/null)

if [ -n "$SRC_LAUNCH" ]; then
    echo "   Found in source:"
    echo "$SRC_LAUNCH" | while read file; do
        echo "     - $file"
    done
else
    echo "   No launch files in src"
fi

echo ""
echo "========================================================================"
echo "4. Checking which launch file YOU USED..."
echo ""
echo "What did you run? Probably:"
echo "  ros2 launch soccer_object_localization amcl_final_fixed.launch.py"
echo ""

# Check if that file exists
EXPECTED_INSTALL="$HOME/basbot/install/soccer_object_localization/share/soccer_object_localization/launch/amcl_final_fixed.launch.py"

if [ -f "$EXPECTED_INSTALL" ]; then
    echo "✅ amcl_final_fixed.launch.py EXISTS in install directory"
    echo ""
    echo "Checking its content..."
    if grep -q "remappings" "$EXPECTED_INSTALL"; then
        if grep -q "field_scan" "$EXPECTED_INSTALL"; then
            echo "   ✅ HAS remapping to /field_scan!"
            echo ""
            echo "   Remapping section:"
            grep -A 3 "remappings" "$EXPECTED_INSTALL" | head -5
        else
            echo "   ❌ Has remappings but NOT to field_scan!"
        fi
    else
        echo "   ❌ NO remappings found!"
        echo "   This is the problem - launch file doesn't have scan remapping"
    fi
else
    echo "❌ amcl_final_fixed.launch.py NOT FOUND in install!"
    echo ""
    echo "You need to:"
    echo "  1. Copy it from outputs:"
    echo "     cp /mnt/user-data/outputs/amcl_final_fixed.launch.py \\"
    echo "        PATH_TO/soccer_object_localization/launch/"
    echo ""
    echo "  2. Find the correct path first:"
    find ~/basbot -type d -name "launch" | grep soccer_object_localization
fi

echo ""
echo "========================================================================"
echo "5. SOLUTION"
echo "========================================================================"
echo ""

echo "To fix, you need to:"
echo ""
echo "A. Find your package source directory:"
PKG_SRC=$(find ~/basbot/src -type d -name "soccer_object_localization" | head -1)

if [ -n "$PKG_SRC" ]; then
    echo "   Found: $PKG_SRC"
    echo ""
    echo "B. Copy the fixed launch file:"
    echo "   cp /mnt/user-data/outputs/amcl_final_fixed.launch.py \\"
    echo "      $PKG_SRC/launch/"
    echo ""
    echo "C. Make it executable:"
    echo "   chmod +x $PKG_SRC/launch/amcl_final_fixed.launch.py"
    echo ""
    echo "D. Rebuild:"
    echo "   cd ~/basbot"
    echo "   colcon build --packages-select soccer_object_localization"
    echo "   source install/setup.bash"
    echo ""
    echo "E. Launch:"
    echo "   ros2 launch soccer_object_localization amcl_final_fixed.launch.py"
else
    echo "   ❌ Could not find soccer_object_localization in ~/basbot/src"
    echo "   Please locate it manually"
fi

echo ""
echo "========================================================================"