#!/usr/bin/env python3
# filepath: /home/farhan/motion_webots/src/ROBOTIS-OP3-Tools/op3_action_editor/scripts/webots_executor.py

import subprocess
import rclpy
import time
import os
import shutil
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory

class ActionEditorExecutor(Node):
    def __init__(self):
        super().__init__('op3_action_editor_executor')
        self.get_logger().info('Starting Action Editor with Webots simulation')

def check_webots_topics():
    """Check if Webots is publishing the expected topics"""
    print("\n🔍 Checking Webots topics...")
    
    try:
        topics_output = subprocess.getoutput('ros2 topic list')
        print("📋 Available ROS topics:")
        
        # Look for actual Webots topics
        webots_topics = [
            '/robotis_op3/joint_states',
            '/robotis_op3/r_sho_pitch_position/command',
            '/robotis_op3/camera/image_raw',
            '/robotis_op3/imu'
        ]
        
        for topic in webots_topics:
            if topic in topics_output:
                print(f"✅ Webots topic found: {topic}")
                # Get topic type
                try:
                    topic_type = subprocess.getoutput(f'ros2 topic type {topic}')
                    print(f"   Type: {topic_type}")
                except:
                    pass
            else:
                print(f"❌ Missing Webots topic: {topic}")
        
        # Count joint command topics
        joint_commands = [t for t in topics_output.split('\n') if '_position/command' in t]
        print(f"📊 Found {len(joint_commands)} joint command topics")
        
        # Test joint state data
        print("\n🔬 Testing joint state data...")
        joint_state_test = subprocess.getoutput('timeout 2 ros2 topic echo /robotis_op3/joint_states --once')
        if 'name:' in joint_state_test:
            print("✅ Joint states are publishing data")
        else:
            print("❌ No joint state data received")
                
    except Exception as e:
        print(f"Error checking topics: {e}")    

def main(args=None):
    rclpy.init(args=args)
    node = ActionEditorExecutor()
    
    # Check terminal size
    columns, rows = shutil.get_terminal_size()
    print(f"Current terminal size: {columns}x{rows}")
    
    if columns != 80 or rows != 24:
        print("\nWARNING: Action Editor requires terminal size of 80x24")
        print("Your terminal size is currently", columns, "x", rows)
        print("\nResizing terminal. Please run:")
        print("stty cols 80 rows 24")
        print("Then run this script again.")
        
        try:
            print("\nAttempting to automatically resize terminal...")
            os.system("stty cols 80 rows 24")
            print("Terminal resized. If the interface still looks wrong, resize manually.")
        except:
            pass

    # Start Webots first and check topics
    print("\n🚀 Starting Webots simulation...")
    webots_proc = None
    try:
        webots_proc = subprocess.Popen(
            ['ros2', 'launch', 'op3_webots_ros2', 'robot_launch.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        print("⏳ Waiting for Webots to initialize...")
        time.sleep(10)  # Wait longer for Webots to fully start
        
        if webots_proc.poll() is not None:
            stdout, stderr = webots_proc.communicate()
            print(f"❌ Webots failed to start:")
            print(f"STDOUT: {stdout.decode()}")
            print(f"STDERR: {stderr.decode()}")
            return 1
        
        print("✅ Webots started")
        
        # Check topics after Webots starts
        check_webots_topics()
        
    except Exception as e:
        print(f"ERROR: Failed to start Webots: {e}")
        return 1

    # Define action editor parameters
    package = "op3_action_editor"
    executable = "op3_action_editor"
    webots_simulation = True
    robot_name_default = 'robotis_op3'

    offset_file_path_default = get_package_share_directory('op3_manager') + '/config/offset.yaml'
    robot_file_path_default = get_package_share_directory('op3_manager') + '/config/OP3.robot'
    init_file_path_default = get_package_share_directory('op3_manager') + '/config/dxl_init_OP3.yaml'
    action_file_path_default = get_package_share_directory('op3_action_module') + '/data/motion_4095.bin'
    device_name_default = '/dev/null'

    params = [
        '--ros-args',
        '-p', f'gazebo:={webots_simulation}',
        '-p', f'gazebo_robot_name:={robot_name_default}',
        '-p', f'offset_file_path:={offset_file_path_default}',
        '-p', f'robot_file_path:={robot_file_path_default}',
        '-p', f'init_file_path:={init_file_path_default}',
        '-p', f'action_file_path:={action_file_path_default}',
        '-p', f'device_name:={device_name_default}'
    ]

    # Start OpenCR simulator
    print("\n🔧 Starting OpenCR simulator...")
    open_cr_proc = None
    try:
        open_cr_proc = subprocess.Popen(
            ['ros2', 'launch', 'open_cr_module', 'open_cr.launch.py', 
             'use_dummy_data:=true'],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )
        time.sleep(2)
        print("✅ OpenCR simulator started")
    except Exception as e:
        print(f"❌ Failed to start OpenCR simulator: {e}")

    # Start Enhanced Bridge with better debugging
    print("\n🌉 Starting Enhanced Webots Bridge...")
    bridge_proc = None
    try:
        bridge_proc = subprocess.Popen(
            ['ros2', 'run', 'op3_action_editor', 'bridge_webots.py'],
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE
        )
        time.sleep(3)
        print("✅ Webots Bridge started")
        
        # Check if bridge is working
        bridge_check = subprocess.getoutput('ros2 node list | grep bridge')
        if 'bridge' in bridge_check:
            print("✅ Bridge node verified in ROS graph")
        else:
            print("⚠️  Bridge node not found in ROS graph")
            
    except Exception as e:
        print(f"❌ Failed to start Webots Bridge: {e}")

    # Start audio player
    try:
        print("\n🔊 Starting audio player...")
        proc_player = subprocess.Popen(['ros2', 'run', 'ros_mpg321_player', 'ros_mpg321_player'],
                      stdout=subprocess.DEVNULL,
                      stderr=subprocess.DEVNULL
        )
        print("✅ Audio player started")
    except Exception as e:
        print(f"⚠️  Audio player failed: {e}")

    # Start action editor
    try:
        print("\n🎯 Starting Action Editor...")
        print("="*60)
        print("WEBOTS ACTION EDITOR READY")
        print("="*60)
        print("• Action Editor should show joint states from Webots")
        print("• Play commands should move the robot in Webots")
        print("• Monitor the bridge debug messages")
        print("="*60)
        
        proc_editor = subprocess.Popen(
            ['ros2', 'run', package, executable] + params,
            stdin=None,
            stdout=None,
            stderr=None
        )
        print("✅ Action editor started")
        
        # Give additional instructions
        print("\n📋 Debug Instructions:")
        print("1. Check if joint states show in action editor (not ----)")
        print("2. Try 'Go' command on a step to test communication")
        print("3. Try 'Play' command to test full action playback")
        print("4. Monitor bridge output in another terminal:")
        print("   ros2 topic echo /webots_bridge/debug")
        
    except Exception as e:
        print(f"❌ Error starting action editor: {e}")
        return 1

    print("\n🟢 All processes running. Keep this terminal open.")
    print("Press Ctrl+C to exit when finished.")

    try:
        while True:
            # Check if critical processes are still running
            if webots_proc and webots_proc.poll() is not None:
                print("❌ Webots has terminated")
                break
            if proc_editor and proc_editor.poll() is not None:
                print(f"❌ Action editor terminated (code: {proc_editor.poll()})")
                break
            if bridge_proc and bridge_proc.poll() is not None:
                print("❌ Bridge has terminated")
                break
            
            time.sleep(2)
    except KeyboardInterrupt:
        print("\n🛑 Shutting down by user request")
    finally:
        print("🧹 Cleaning up all processes...")
        for proc, name in [(proc_player, "Audio Player"), 
                           (proc_editor, "Action Editor"), 
                           (open_cr_proc, "OpenCR Simulator"),
                           (bridge_proc, "Webots Bridge"),
                           (webots_proc, "Webots")]:
            if proc and proc.poll() is None:
                print(f"  Terminating {name}...")
                proc.terminate()
                try:
                    proc.wait(timeout=3)
                except subprocess.TimeoutExpired:
                    proc.kill()

    rclpy.shutdown()
    print("✅ All processes terminated successfully.")
    return 0

if __name__ == '__main__':
    main()