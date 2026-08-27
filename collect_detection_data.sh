#!/bin/bash
# DETECTION PERFORMANCE DATA COLLECTOR
# Collects metrics to compare simple vs enhanced detection

echo "📊 DETECTION PERFORMANCE BENCHMARK"
echo "========================================================================"
echo ""

# Configuration
DURATION=30  # seconds to collect data
OUTPUT_DIR="$HOME/detection_benchmark_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$OUTPUT_DIR"

echo "Output directory: $OUTPUT_DIR"
echo "Collection duration: ${DURATION}s"
echo ""

# Check if system is running
echo "Checking system status..."
if ! ros2 topic list | grep -q "/field_point_cloud"; then
    echo "❌ ERROR: Localization system not running!"
    echo "   Please start: ros2 launch soccer_object_localization amcl_final_fixed4.launch.py"
    exit 1
fi
echo "✅ System detected"
echo ""

# ===== COLLECT POINT CLOUD DATA =====
echo "1️⃣  Collecting point cloud data..."
echo "   Recording for ${DURATION}s..."

ros2 topic echo /field_point_cloud --once > "$OUTPUT_DIR/pointcloud_sample.txt" &
PC_PID=$!

# Collect multiple samples
for i in {1..30}; do
    COUNT=$(timeout 2 ros2 topic echo /field_point_cloud --once 2>/dev/null | grep -c "x:")
    if [ ! -z "$COUNT" ]; then
        echo "$COUNT" >> "$OUTPUT_DIR/pointcloud_counts.txt"
    fi
    sleep 1
done

wait $PC_PID 2>/dev/null
echo "   ✅ Point cloud data collected"

# ===== COLLECT LASER SCAN DATA =====
echo ""
echo "2️⃣  Collecting laser scan data..."

ros2 topic echo /field_scan --once > "$OUTPUT_DIR/laserscan_sample.txt" &
SCAN_PID=$!

# Collect scan statistics
for i in {1..30}; do
    # Count valid ranges (not inf)
    VALID=$(timeout 2 ros2 topic echo /field_scan --once 2>/dev/null | \
            grep "ranges:" | tr ',' '\n' | grep -v "inf" | grep -v "nan" | wc -l)
    
    # Count total ranges
    TOTAL=$(timeout 2 ros2 topic echo /field_scan --once 2>/dev/null | \
            grep "ranges:" | tr ',' '\n' | wc -l)
    
    if [ ! -z "$VALID" ] && [ ! -z "$TOTAL" ] && [ "$TOTAL" -gt 0 ]; then
        COVERAGE=$(awk "BEGIN {printf \"%.2f\", ($VALID/$TOTAL)*100}")
        echo "$VALID $TOTAL $COVERAGE" >> "$OUTPUT_DIR/laserscan_stats.txt"
    fi
    sleep 1
done

wait $SCAN_PID 2>/dev/null
echo "   ✅ Laser scan data collected"

# ===== COLLECT AMCL DATA =====
echo ""
echo "3️⃣  Collecting AMCL localization data..."

# Sample AMCL pose
ros2 topic echo /amcl_pose --once > "$OUTPUT_DIR/amcl_pose_sample.txt" 2>/dev/null &
AMCL_PID=$!

# Collect covariance over time
for i in {1..30}; do
    COV=$(timeout 2 ros2 topic echo /amcl_pose 2>/dev/null | \
          grep -A 36 "covariance:" | grep "^ *[0-9]" | \
          head -1 | awk '{print $1}')
    
    if [ ! -z "$COV" ]; then
        echo "$COV" >> "$OUTPUT_DIR/amcl_covariance.txt"
    fi
    sleep 1
done

wait $AMCL_PID 2>/dev/null
echo "   ✅ AMCL data collected"

# ===== COLLECT PARTICLE CLOUD DATA =====
echo ""
echo "4️⃣  Collecting particle cloud data..."

if ros2 topic list | grep -q "/particle_cloud_viz"; then
    ros2 topic echo /particle_cloud_viz --once > "$OUTPUT_DIR/particles_sample.txt" 2>/dev/null &
    PART_PID=$!
    
    for i in {1..10}; do
        PARTICLE_COUNT=$(timeout 2 ros2 topic echo /particle_cloud_viz --once 2>/dev/null | \
                        grep -c "position:")
        if [ ! -z "$PARTICLE_COUNT" ]; then
            echo "$PARTICLE_COUNT" >> "$OUTPUT_DIR/particle_counts.txt"
        fi
        sleep 2
    done
    
    wait $PART_PID 2>/dev/null
    echo "   ✅ Particle data collected"
else
    echo "   ⚠️  Particle cloud topic not available"
fi

# ===== COLLECT SYSTEM PERFORMANCE =====
echo ""
echo "5️⃣  Collecting system performance..."

# Topic rates
echo "Point cloud rate:" > "$OUTPUT_DIR/topic_rates.txt"
timeout 10 ros2 topic hz /field_point_cloud 2>/dev/null | grep "average rate" >> "$OUTPUT_DIR/topic_rates.txt"

echo "Laser scan rate:" >> "$OUTPUT_DIR/topic_rates.txt"
timeout 10 ros2 topic hz /field_scan 2>/dev/null | grep "average rate" >> "$OUTPUT_DIR/topic_rates.txt"

echo "AMCL pose rate:" >> "$OUTPUT_DIR/topic_rates.txt"
timeout 10 ros2 topic hz /amcl_pose 2>/dev/null | grep "average rate" >> "$OUTPUT_DIR/topic_rates.txt"

# Node list
ros2 node list > "$OUTPUT_DIR/active_nodes.txt"

# Parameter dump
if ros2 node list | grep -q "detector_fieldline"; then
    ros2 param dump /detector_fieldline > "$OUTPUT_DIR/detector_params.yaml" 2>/dev/null
fi

if ros2 node list | grep -q "amcl"; then
    ros2 param dump /amcl > "$OUTPUT_DIR/amcl_params.yaml" 2>/dev/null
fi

echo "   ✅ System performance collected"

# ===== ANALYZE DATA =====
echo ""
echo "6️⃣  Analyzing collected data..."
echo ""

# Create analysis report
REPORT="$OUTPUT_DIR/ANALYSIS_REPORT.txt"

cat > "$REPORT" << 'EOF'
╔══════════════════════════════════════════════════════════════════════╗
║           DETECTION PERFORMANCE ANALYSIS REPORT                      ║
╚══════════════════════════════════════════════════════════════════════╝

EOF

echo "Timestamp: $(date)" >> "$REPORT"
echo "" >> "$REPORT"

# Point Cloud Analysis
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "POINT CLOUD STATISTICS" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"

if [ -f "$OUTPUT_DIR/pointcloud_counts.txt" ]; then
    PC_MIN=$(sort -n "$OUTPUT_DIR/pointcloud_counts.txt" | head -1)
    PC_MAX=$(sort -n "$OUTPUT_DIR/pointcloud_counts.txt" | tail -1)
    PC_AVG=$(awk '{sum+=$1; count++} END {printf "%.0f", sum/count}' "$OUTPUT_DIR/pointcloud_counts.txt")
    PC_SAMPLES=$(wc -l < "$OUTPUT_DIR/pointcloud_counts.txt")
    
    echo "Samples collected: $PC_SAMPLES" >> "$REPORT"
    echo "Point count (avg): $PC_AVG points" >> "$REPORT"
    echo "Point count (min): $PC_MIN points" >> "$REPORT"
    echo "Point count (max): $PC_MAX points" >> "$REPORT"
    echo "" >> "$REPORT"
    
    # Classification
    if [ "$PC_AVG" -lt 600 ]; then
        echo "Assessment: ⚠️  LOW - Need improvement" >> "$REPORT"
    elif [ "$PC_AVG" -lt 1000 ]; then
        echo "Assessment: ✓ FAIR - Basic detection working" >> "$REPORT"
    elif [ "$PC_AVG" -lt 1500 ]; then
        echo "Assessment: ✓✓ GOOD - Enhanced detection effective" >> "$REPORT"
    else
        echo "Assessment: ✓✓✓ EXCELLENT - Very dense detection" >> "$REPORT"
    fi
else
    echo "ERROR: No point cloud data collected" >> "$REPORT"
fi

echo "" >> "$REPORT"

# Laser Scan Analysis
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "LASER SCAN STATISTICS" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"

if [ -f "$OUTPUT_DIR/laserscan_stats.txt" ]; then
    SCAN_AVG_VALID=$(awk '{sum+=$1; count++} END {printf "%.0f", sum/count}' "$OUTPUT_DIR/laserscan_stats.txt")
    SCAN_AVG_TOTAL=$(awk '{sum+=$2; count++} END {printf "%.0f", sum/count}' "$OUTPUT_DIR/laserscan_stats.txt")
    SCAN_AVG_COV=$(awk '{sum+=$3; count++} END {printf "%.1f", sum/count}' "$OUTPUT_DIR/laserscan_stats.txt")
    SCAN_MIN_VALID=$(awk '{print $1}' "$OUTPUT_DIR/laserscan_stats.txt" | sort -n | head -1)
    SCAN_MAX_VALID=$(awk '{print $1}' "$OUTPUT_DIR/laserscan_stats.txt" | sort -n | tail -1)
    
    echo "Total ranges: $SCAN_AVG_TOTAL" >> "$REPORT"
    echo "Valid ranges (avg): $SCAN_AVG_VALID" >> "$REPORT"
    echo "Valid ranges (min): $SCAN_MIN_VALID" >> "$REPORT"
    echo "Valid ranges (max): $SCAN_MAX_VALID" >> "$REPORT"
    echo "Coverage: $SCAN_AVG_COV%" >> "$REPORT"
    echo "" >> "$REPORT"
    
    # Classification
    if (( $(echo "$SCAN_AVG_COV < 20" | bc -l) )); then
        echo "Assessment: ⚠️  LOW (<20%) - Simple threshold level" >> "$REPORT"
    elif (( $(echo "$SCAN_AVG_COV < 30" | bc -l) )); then
        echo "Assessment: ✓ FAIR (20-30%) - Improved detection" >> "$REPORT"
    elif (( $(echo "$SCAN_AVG_COV < 40" | bc -l) )); then
        echo "Assessment: ✓✓ GOOD (30-40%) - Enhanced detection target" >> "$REPORT"
    else
        echo "Assessment: ✓✓✓ EXCELLENT (>40%) - Exceptional coverage" >> "$REPORT"
    fi
else
    echo "ERROR: No laser scan data collected" >> "$REPORT"
fi

echo "" >> "$REPORT"

# AMCL Analysis
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "AMCL LOCALIZATION STATISTICS" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"

if [ -f "$OUTPUT_DIR/amcl_covariance.txt" ]; then
    COV_AVG=$(awk '{sum+=$1; count++} END {printf "%.4f", sum/count}' "$OUTPUT_DIR/amcl_covariance.txt")
    COV_MIN=$(sort -n "$OUTPUT_DIR/amcl_covariance.txt" | head -1)
    COV_MAX=$(sort -n "$OUTPUT_DIR/amcl_covariance.txt" | tail -1)
    
    echo "Covariance (avg): $COV_AVG" >> "$REPORT"
    echo "Covariance (min): $COV_MIN (best)" >> "$REPORT"
    echo "Covariance (max): $COV_MAX (worst)" >> "$REPORT"
    echo "" >> "$REPORT"
    
    # Classification (lower is better!)
    if (( $(echo "$COV_AVG < 0.15" | bc -l) )); then
        echo "Assessment: ✓✓✓ EXCELLENT (<0.15) - Very confident" >> "$REPORT"
    elif (( $(echo "$COV_AVG < 0.25" | bc -l) )); then
        echo "Assessment: ✓✓ GOOD (0.15-0.25) - Confident" >> "$REPORT"
    elif (( $(echo "$COV_AVG < 0.50" | bc -l) )); then
        echo "Assessment: ✓ FAIR (0.25-0.50) - Acceptable" >> "$REPORT"
    else
        echo "Assessment: ⚠️  HIGH (>0.50) - Uncertain" >> "$REPORT"
    fi
else
    echo "ERROR: No AMCL data collected" >> "$REPORT"
fi

echo "" >> "$REPORT"

# Particle Analysis
if [ -f "$OUTPUT_DIR/particle_counts.txt" ]; then
    echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
    echo "PARTICLE CLOUD STATISTICS" >> "$REPORT"
    echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
    
    PART_AVG=$(awk '{sum+=$1; count++} END {printf "%.0f", sum/count}' "$OUTPUT_DIR/particle_counts.txt")
    echo "Particle count (avg): $PART_AVG" >> "$REPORT"
    echo "" >> "$REPORT"
fi

# Topic Rates
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "TOPIC RATES" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"

if [ -f "$OUTPUT_DIR/topic_rates.txt" ]; then
    cat "$OUTPUT_DIR/topic_rates.txt" >> "$REPORT"
else
    echo "No rate data collected" >> "$REPORT"
fi

echo "" >> "$REPORT"

# Overall Assessment
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "OVERALL SYSTEM ASSESSMENT" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "" >> "$REPORT"

# Calculate overall score
SCORE=0

# Point cloud contribution (0-30 points)
if [ ! -z "$PC_AVG" ]; then
    if [ "$PC_AVG" -ge 1500 ]; then SCORE=$((SCORE + 30))
    elif [ "$PC_AVG" -ge 1000 ]; then SCORE=$((SCORE + 25))
    elif [ "$PC_AVG" -ge 600 ]; then SCORE=$((SCORE + 15))
    else SCORE=$((SCORE + 5))
    fi
fi

# Coverage contribution (0-40 points)
if [ ! -z "$SCAN_AVG_COV" ]; then
    if (( $(echo "$SCAN_AVG_COV >= 40" | bc -l) )); then SCORE=$((SCORE + 40))
    elif (( $(echo "$SCAN_AVG_COV >= 30" | bc -l) )); then SCORE=$((SCORE + 35))
    elif (( $(echo "$SCAN_AVG_COV >= 20" | bc -l) )); then SCORE=$((SCORE + 25))
    else SCORE=$((SCORE + 10))
    fi
fi

# AMCL contribution (0-30 points)
if [ ! -z "$COV_AVG" ]; then
    if (( $(echo "$COV_AVG < 0.15" | bc -l) )); then SCORE=$((SCORE + 30))
    elif (( $(echo "$COV_AVG < 0.25" | bc -l) )); then SCORE=$((SCORE + 25))
    elif (( $(echo "$COV_AVG < 0.50" | bc -l) )); then SCORE=$((SCORE + 15))
    else SCORE=$((SCORE + 5))
    fi
fi

echo "Overall Score: $SCORE / 100" >> "$REPORT"
echo "" >> "$REPORT"

if [ "$SCORE" -ge 85 ]; then
    echo "System Grade: A (EXCELLENT) ✓✓✓" >> "$REPORT"
    echo "Status: Production ready!" >> "$REPORT"
elif [ "$SCORE" -ge 70 ]; then
    echo "System Grade: B (GOOD) ✓✓" >> "$REPORT"
    echo "Status: Minor tuning recommended" >> "$REPORT"
elif [ "$SCORE" -ge 50 ]; then
    echo "System Grade: C (FAIR) ✓" >> "$REPORT"
    echo "Status: Needs improvement" >> "$REPORT"
else
    echo "System Grade: D (POOR) ⚠️" >> "$REPORT"
    echo "Status: Major issues detected" >> "$REPORT"
fi

echo "" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "COMPARISON WITH TARGETS" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "" >> "$REPORT"
echo "TARGET VALUES (Enhanced Detection):" >> "$REPORT"
echo "  Point cloud: 1200-1800 points" >> "$REPORT"
echo "  Coverage: 35-40%" >> "$REPORT"
echo "  Covariance: <0.18" >> "$REPORT"
echo "" >> "$REPORT"
echo "YOUR ACTUAL VALUES:" >> "$REPORT"
echo "  Point cloud: $PC_AVG points" >> "$REPORT"
echo "  Coverage: $SCAN_AVG_COV%" >> "$REPORT"
echo "  Covariance: $COV_AVG" >> "$REPORT"
echo "" >> "$REPORT"

echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"
echo "END OF REPORT" >> "$REPORT"
echo "═══════════════════════════════════════════════════════════════" >> "$REPORT"

# Display report
cat "$REPORT"

# ===== GENERATE CSV FOR PLOTTING =====
echo ""
echo "7️⃣  Generating CSV files for analysis..."

# Point cloud CSV
if [ -f "$OUTPUT_DIR/pointcloud_counts.txt" ]; then
    echo "sample,point_count" > "$OUTPUT_DIR/pointcloud_data.csv"
    awk '{print NR","$1}' "$OUTPUT_DIR/pointcloud_counts.txt" >> "$OUTPUT_DIR/pointcloud_data.csv"
fi

# Laser scan CSV
if [ -f "$OUTPUT_DIR/laserscan_stats.txt" ]; then
    echo "sample,valid_ranges,total_ranges,coverage_percent" > "$OUTPUT_DIR/laserscan_data.csv"
    awk '{print NR","$1","$2","$3}' "$OUTPUT_DIR/laserscan_stats.txt" >> "$OUTPUT_DIR/laserscan_data.csv"
fi

# AMCL CSV
if [ -f "$OUTPUT_DIR/amcl_covariance.txt" ]; then
    echo "sample,covariance" > "$OUTPUT_DIR/amcl_data.csv"
    awk '{print NR","$1}' "$OUTPUT_DIR/amcl_covariance.txt" >> "$OUTPUT_DIR/amcl_data.csv"
fi

echo "   ✅ CSV files generated"

# ===== SAVE METADATA =====
cat > "$OUTPUT_DIR/metadata.txt" << EOF
Collection Date: $(date)
Duration: ${DURATION}s
ROS2 Distro: $ROS_DISTRO
System: $(uname -a)
EOF

# ===== SUMMARY =====
echo ""
echo "========================================================================"
echo "✅ DATA COLLECTION COMPLETE!"
echo "========================================================================"
echo ""
echo "Output directory: $OUTPUT_DIR"
echo ""
echo "Generated files:"
ls -lh "$OUTPUT_DIR/" | tail -n +2 | awk '{print "  " $9 " (" $5 ")"}'
echo ""
echo "Key files:"
echo "  📊 ANALYSIS_REPORT.txt - Main analysis report"
echo "  📈 pointcloud_data.csv - Point cloud over time"
echo "  📈 laserscan_data.csv - Scan coverage over time"
echo "  📈 amcl_data.csv - Localization confidence over time"
echo ""
echo "To view report:"
echo "  cat $OUTPUT_DIR/ANALYSIS_REPORT.txt"
echo ""
echo "To plot data (with gnuplot):"
echo "  gnuplot -e \"set terminal png; set output 'plot.png'; \" \\"
echo "          -e \"plot '$OUTPUT_DIR/pointcloud_data.csv' using 1:2 with lines\""
echo ""
echo "========================================================================"