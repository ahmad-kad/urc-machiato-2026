#!/usr/bin/env python3
"""
🚀 Autonomous Typing Subsystem Node

URC 2026 Autonomous Typing Subsystem implementation.
Integrates keyboard detection, arm control, and sequence execution.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from autonomy_interfaces.action import PerformTyping
import numpy as np
import time

from autonomy_autonomous_typing.keyboard_localization import KeyboardLocalizationNode
from autonomy_autonomous_typing.arm_controller import ArmController
from autonomy_autonomous_typing.typing_executor import TypingExecutor


class AutonomousTypingNode(Node):
    """
    Main autonomous typing node orchestrating all subsystems.
    
    Coordinates:
    - Keyboard localization (ArUco markers via vision)
    - Arm control (motion planning and execution)
    - Typing execution (sequence coordination)
    """
    
    def __init__(self):
        super().__init__('autonomous_typing_node')
        
        # Initialize subsystems
        self.arm_controller = ArmController(self)
        self.typing_executor = TypingExecutor(self, self.arm_controller)
        
        # Publishers
        self.status_publisher = self.create_publisher(
            String, 'autonomous_typing/status', 10)
        self.typing_result_publisher = self.create_publisher(
            String, 'typing_result', 10)
        
        # Subscribers
        self.keyboard_pose_sub = self.create_subscription(
            PoseStamped,
            'keyboard_pose',
            self._keyboard_pose_callback,
            10
        )
        
        # Action server for typing tasks
        self.typing_action_server = ActionServer(
            self, PerformTyping, 'perform_typing', self.execute_typing_callback)
        
        # Status tracking
        self.keyboard_pose_received = False
        self.last_keyboard_pose: PoseStamped = None
        
        # Status timer
        self.status_timer = self.create_timer(1.0, self._status_callback)
        
        self.get_logger().info('Autonomous typing node initialized')
    
    def _keyboard_pose_callback(self, msg: PoseStamped) -> None:
        """Handle keyboard pose updates from localization node."""
        try:
            self.last_keyboard_pose = msg
            self.keyboard_pose_received = True
            
            # Extract position and orientation
            pos = msg.pose.position
            ori = msg.pose.orientation
            
            position = np.array([pos.x, pos.y, pos.z])
            orientation = np.array([ori.x, ori.y, ori.z, ori.w])
            
            # Update typing executor with keyboard pose
            self.typing_executor.set_keyboard_pose(position, orientation)
            
            self.get_logger().debug(
                f'Keyboard pose updated: pos={position}, ori={orientation}')
            
        except Exception as e:
            self.get_logger().error(f'Error processing keyboard pose: {e}')
    
    def execute_typing_callback(self, goal_handle):
        """
        Execute typing action for competition mission.
        
        Goal: PerformTyping with sequence string
        Feedback: characters_typed, current_character
        Result: success, message, characters_typed
        """
        try:
            typing_sequence = goal_handle.request.sequence
            self.get_logger().info(f'Received typing goal: "{typing_sequence}"')
            
            # Validate preconditions
            if not self.keyboard_pose_received:
                self.get_logger().error('Keyboard pose not yet available')
                result = PerformTyping.Result()
                result.success = False
                result.message = 'Keyboard not detected'
                result.characters_typed = 0
                goal_handle.abort()
                goal_handle.set_aborted(result)
                return result
            
            if not self.arm_controller.is_ready():
                self.get_logger().error('Arm controller not ready')
                result = PerformTyping.Result()
                result.success = False
                result.message = 'Arm controller not ready'
                result.characters_typed = 0
                goal_handle.abort()
                goal_handle.set_aborted(result)
                return result
            
            # Prepare feedback message
            feedback_msg = PerformTyping.Feedback()
            
            # Define feedback callback for typing executor
            def feedback_callback(char_index: int, char: str, success: bool):
                feedback_msg.characters_typed = char_index + (1 if success else 0)
                feedback_msg.current_character = char
                goal_handle.publish_feedback(feedback_msg)
            
            # Execute typing sequence
            exec_result = self.typing_executor.execute_sequence(
                typing_sequence,
                feedback_callback=feedback_callback
            )
            
            # Prepare result message
            result = PerformTyping.Result()
            result.success = exec_result['success']
            result.characters_typed = exec_result['characters_typed']
            result.message = (
                f'Successfully typed {exec_result["characters_typed"]}'
                f'/{exec_result["total_characters"]} characters'
            )
            
            # Handle result
            if result.success:
                self.get_logger().info(result.message)
                goal_handle.succeed()
                goal_handle.set_succeeded(result)
            else:
                self.get_logger().warn(result.message)
                goal_handle.abort()
                goal_handle.set_aborted(result)
            
            return result
            
        except Exception as e:
            self.get_logger().error(f'Typing execution error: {e}')
            result = PerformTyping.Result()
            result.success = False
            result.message = f'Execution error: {str(e)}'
            result.characters_typed = 0
            goal_handle.abort()
            goal_handle.set_aborted(result)
            return result
    
    def _status_callback(self) -> None:
        """Publish system status periodically."""
        try:
            status_msg = String()
            
            keyboard_status = (
                'detected' if self.keyboard_pose_received else 'not detected'
            )
            arm_status = (
                'ready' if self.arm_controller.is_ready() else 'not ready'
            )
            exec_status = self.typing_executor.get_status()['status']
            
            status_msg.data = (
                f'Typing System | Keyboard: {keyboard_status} | '
                f'Arm: {arm_status} | Status: {exec_status}'
            )
            
            self.status_publisher.publish(status_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error publishing status: {e}')


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)
    
    # Create and spin the node
    node = AutonomousTypingNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
