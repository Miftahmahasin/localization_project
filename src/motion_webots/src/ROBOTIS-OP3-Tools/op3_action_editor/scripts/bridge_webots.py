#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float64
from sensor_msgs.msg import JointState
import json

class WebotsBridge(Node):
    def __init__(self):
        super().__init__('webots_bridge')
        
        # Joint state from Webots
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/robotis_op3/joint_states',  # Actual Webots topic
            self.joint_state_callback,
            10
        )
        
        # Individual joint command publishers for Webots
        self.joint_publishers = {}
        joint_names = [
            'r_sho_pitch', 'l_sho_pitch', 'r_sho_roll', 'l_sho_roll',
            'r_el', 'l_el', 'r_hip_yaw', 'l_hip_yaw',
            'r_hip_roll', 'l_hip_roll', 'r_hip_pitch', 'l_hip_pitch',
            'r_knee', 'l_knee', 'r_ank_pitch', 'l_ank_pitch',
            'r_ank_roll', 'l_ank_roll', 'head_pan', 'head_tilt'
        ]
        
        for joint_name in joint_names:
            topic_name = f'/robotis_op3/{joint_name}_position/command'
            self.joint_publishers[joint_name] = self.create_publisher(
                Float64, topic_name, 10
            )
        
        # Action Editor communication
        self.action_command_sub = self.create_subscription(
            String,
            '/webots/action_command',
            self.action_command_callback,
            10
        )
        
        self.joint_positions_sub = self.create_subscription(
            String,  # Changed from Float64MultiArray to String for easier JSON parsing
            '/webots/joint_positions',
            self.joint_positions_callback,
            10
        )
        
        # Feedback to Action Editor
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',  # Topic that Action Editor expects
            10
        )
        
        # Debug publisher
        self.debug_pub = self.create_publisher(
            String,
            '/webots_bridge/debug',
            10
        )
        
        # Current joint states
        self.current_joint_states = JointState()
        self.received_webots_data = False
        
        # Timer for status updates
        self.timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('🌉 Webots Bridge initialized with correct topic names')
        self.get_logger().info(f'📡 Subscribed to: /robotis_op3/joint_states')
        self.get_logger().info(f'📤 Publishing to: /joint_states')
        self.get_logger().info(f'🎯 Ready for individual joint commands')

    def joint_state_callback(self, msg):
        """Receive joint states from Webots and forward to Action Editor"""
        self.current_joint_states = msg
        self.received_webots_data = True
        
        # Forward to Action Editor with expected topic name
        self.joint_state_pub.publish(msg)
        
        # Debug info
        joint_info = []
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                joint_info.append(f"{name}: {msg.position[i]:.3f}")
        
        self.get_logger().info(f'📊 Joint states: {len(msg.name)} joints updated')
        
    def action_command_callback(self, msg):
        """Handle action commands from Action Editor"""
        self.get_logger().info(f'🎬 Action command: {msg.data}')
        
        try:
            # Parse action command (expecting JSON format)
            command_data = json.loads(msg.data)
            
            if 'action' in command_data:
                action_type = command_data['action']
                self.get_logger().info(f'🎯 Executing action: {action_type}')
                
                # Handle different action types
                if action_type == 'play':
                    self.handle_play_action(command_data)
                elif action_type == 'stop':
                    self.handle_stop_action()
                    
        except json.JSONDecodeError:
            # Simple string command
            self.get_logger().info(f'🎵 Simple action: {msg.data}')
            
    def joint_positions_callback(self, msg):
        """Handle joint position commands from Action Editor"""
        try:
            # Parse joint positions (expecting JSON format)
            positions_data = json.loads(msg.data)
            
            self.get_logger().info(f'🎯 Setting joint positions: {len(positions_data)} joints')
            
            # Send commands to individual Webots joint topics
            for joint_name, position in positions_data.items():
                if joint_name in self.joint_publishers:
                    command_msg = Float64()
                    command_msg.data = float(position)
                    self.joint_publishers[joint_name].publish(command_msg)
                    
            self.get_logger().info(f'✅ Joint commands sent to Webots')
            
        except (json.JSONDecodeError, ValueError) as e:
            self.get_logger().warn(f'❌ Failed to parse joint positions: {e}')
            
    def handle_play_action(self, command_data):
        """Handle play action command"""
        self.get_logger().info('▶️ Playing action in Webots')
        
        # Example: Extract joint positions from action data
        if 'positions' in command_data:
            positions = command_data['positions']
            for joint_name, position in positions.items():
                if joint_name in self.joint_publishers:
                    command_msg = Float64()
                    command_msg.data = float(position)
                    self.joint_publishers[joint_name].publish(command_msg)
                    
    def handle_stop_action(self):
        """Handle stop action command"""
        self.get_logger().info('⏹️ Stopping action in Webots')
        # Maintain current positions
        
    def publish_status(self):
        """Publish bridge status"""
        status = f"Bridge OK. Webots data: {self.received_webots_data}. " \
                f"Joints: {len(self.current_joint_states.name)}"
        
        debug_msg = String()
        debug_msg.data = status
        self.debug_pub.publish(debug_msg)
        
        if self.received_webots_data:
            self.get_logger().info(f'✅ {status}')
        else:
            self.get_logger().warn(f'⚠️ No Webots data received yet')

def main(args=None):
    rclpy.init(args=args)
    
    print("🌉 Starting Enhanced Webots Bridge with correct topic mapping...")
    bridge = WebotsBridge()
    
    try:
        print("✅ Bridge running - monitoring Webots topics")
        print("📡 Subscribing to: /robotis_op3/joint_states")
        print("📤 Publishing to: /joint_states")
        print("🎯 Ready for joint commands")
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        print("🛑 Bridge stopped")
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()