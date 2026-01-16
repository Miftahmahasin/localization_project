#!/bin/bash

echo "=========================================="
echo "   Checking Joint States Topics"
echo "=========================================="

echo ""
echo "Available joint_states topics:"
ros2 topic list | grep joint

echo ""
echo "Test 1: /robotis_op3/joint_states"
timeout 2s ros2 topic hz /robotis_op3/joint_states 2>&1 | head -3

echo ""
echo "Test 2: /robotis/present_joint_states"
timeout 2s ros2 topic hz /robotis/present_joint_states 2>&1 | head -3

echo ""
echo "Test 3: Check message on active topic"
echo "Checking /robotis/present_joint_states..."
ros2 topic echo /robotis/present_joint_states --once | head -30

echo ""
echo "=========================================="
echo "Which topic is publishing?"
echo "=========================================="