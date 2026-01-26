#!/bin/bash
# Find Particle Topic Name

echo "🔍 FINDING PARTICLE CLOUD TOPIC"
echo "========================================================================"
echo ""

echo "All topics containing 'particle':"
ros2 topic list | grep -i particle

echo ""
echo "All topics from AMCL node:"
ros2 node info /amcl 2>/dev/null | grep "Publishers:" -A 15 | grep -E "particle|cloud" || echo "  (none with 'particle' or 'cloud')"

echo ""
echo "All AMCL-related topics:"
ros2 topic list | grep -i amcl

echo ""
echo "========================================================================"
echo "Now trying to echo each potential topic..."
echo "========================================================================"
echo ""

# Try all variations
for topic in "/particle_cloud" "/particlecloud" "/particles" "/amcl/particles" "/amcl/particle_cloud"; do
    echo "Trying: $topic"
    timeout 2 ros2 topic echo $topic --once > /tmp/topic_test.txt 2>&1
    
    if [ -s /tmp/topic_test.txt ] && ! grep -q "does not appear" /tmp/topic_test.txt; then
        echo "   ✅ FOUND! Topic exists and has data"
        echo "   Preview:"
        head -20 /tmp/topic_test.txt
        echo ""
    else
        echo "   ❌ Not found or no data"
    fi
    echo ""
done

echo "========================================================================"
echo "Complete topic list for reference:"
ros2 topic list
echo "========================================================================"