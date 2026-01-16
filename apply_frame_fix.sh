#!/bin/bash

# Script untuk fix frame name inconsistency
# Solusi untuk: Odometry Status: Error di RViz

set -e

# Warna
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
PKG_DIR="$HOME/basbot/src/motion_webots/src/localization_ws/op3_utra_bridge"

echo "=========================================="
echo "  Fix Odometry Frame Name"
echo "=========================================="
echo ""

# 1. Backup file original
echo -e "${BLUE}📋 Backing up original file...${NC}"
BACKUP_FILE="${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py.backup_$(date +%Y%m%d_%H%M%S)"
cp "${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py" "$BACKUP_FILE"
echo -e "${GREEN}✓${NC} Backup saved to: $BACKUP_FILE"
echo ""

# 2. Apply fix menggunakan sed
echo -e "${BLUE}🔧 Applying fix...${NC}"
sed -i "s/self.declare_parameter('child_frame', 'base')/self.declare_parameter('child_frame', 'base_link')/" \
    "${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py"

# Verify fix
if grep -q "child_frame', 'base_link'" "${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py"; then
    echo -e "${GREEN}✓${NC} Fix applied successfully!"
    echo "  Changed: 'base' → 'base_link'"
else
    echo -e "${RED}✗${NC} Failed to apply fix!"
    echo "  Restoring backup..."
    cp "$BACKUP_FILE" "${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py"
    exit 1
fi
echo ""

# 3. Show the change
echo -e "${BLUE}📝 Changed line:${NC}"
grep -n "child_frame" "${PKG_DIR}/src/op3_utra_bridge/odom_to_tf.py" | grep "base_link"
echo ""

# 4. Rebuild package
echo -e "${BLUE}🔨 Rebuilding package...${NC}"
cd "$HOME/basbot"
source /opt/ros/humble/setup.bash
colcon build --packages-select op3_utra_bridge --cmake-clean-cache

if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}✓${NC} Build successful!"
else
    echo ""
    echo -e "${RED}✗${NC} Build failed!"
    exit 1
fi

echo ""
echo "=========================================="
echo "  Fix Applied Successfully!"
echo "=========================================="
echo ""
echo -e "${GREEN}✅ Next steps:${NC}"
echo ""
echo "1. Source the workspace:"
echo "   ${YELLOW}source ~/basbot/install/setup.bash${NC}"
echo ""
echo "2. Kill any running odometry nodes:"
echo "   ${YELLOW}killall -9 odom_bridge_node${NC}"
echo "   ${YELLOW}killall -9 python3${NC}  # (if odom_to_tf.py is running)"
echo ""
echo "3. Launch again:"
echo "   ${YELLOW}ros2 launch op3_utra_bridge localization_odometry_only.launch.py${NC}"
echo ""
echo "4. Verify frame in new terminal:"
echo "   ${YELLOW}ros2 topic echo /tf --once | grep child_frame_id${NC}"
echo "   ${GREEN}Should show: base_link${NC}"
echo ""
echo "5. Open RViz:"
echo "   - Fixed Frame: ${GREEN}odom${NC}"
echo "   - Add Odometry display"
echo "   - Topic: ${GREEN}/odom_combined${NC}"
echo ""
echo "Expected: ${GREEN}Odometry Status: OK${NC} (not Error)"
echo "=========================================="
