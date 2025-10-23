#!/usr/bin/env python3
"""
State Management Node - Central coordinator for autonomy system.

This node manages:
- Mission state and progress tracking
- Mode switching (autonomous/teleoperation)
- Health monitoring and fault recovery
- Inter-subsystem coordination

Author: URC 2026 Autonomy Team
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseStamped

from enum import Enum
from typing import Dict, List, Optional
from dataclasses import dataclass


class MissionState(Enum):
    """Overall mission states"""
    PRE_MISSION = "pre_mission"
    AUTONOMOUS_NAVIGATION = "autonomous_navigation"
    EQUIPMENT_SERVICING = "equipment_servicing"
    TELEOPERATION = "teleoperation"
    EMERGENCY = "emergency"
    COMPLETED = "completed"
    FAILED = "failed"


class SystemMode(Enum):
    """System operational modes"""
    IDLE = "idle"
    AUTONOMOUS = "autonomous"
    TELEOPERATION = "teleoperation"
    MANUAL_OVERRIDE = "manual_override"


@dataclass
class SubsystemStatus:
    """Individual subsystem status"""
    name: str
    state: str
    health: float  # 0.0 to 1.0
    last_update: float
    active: bool = True


class StateManagementNode(Node):
    """
    Central state management node coordinating all autonomy subsystems.

    Manages:
    - Mission lifecycle and state transitions
    - Mode switching with safety validation
    - Health monitoring and fault response
    - Inter-subsystem communication
    """

    def __init__(self):
        super().__init__('state_management_node')

        # Initialize parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('mission_duration_limit', 1800.0),  # seconds
                ('health_check_interval', 1.0),       # seconds
                ('emergency_stop_timeout', 5.0),     # seconds
                ('auto_recovery_enabled', True),
            ]
        )

        # Get parameters
        self.mission_duration_limit = self.get_parameter('mission_duration_limit').value
        self.health_check_interval = self.get_parameter('health_check_interval').value
        self.emergency_stop_timeout = self.get_parameter('emergency_stop_timeout').value
        self.auto_recovery_enabled = self.get_parameter('auto_recovery_enabled').value

        # State variables
        self.mission_state = MissionState.PRE_MISSION
        self.system_mode = SystemMode.IDLE
        self.mission_start_time: Optional[float] = None
        self.emergency_active = False

        # Mission coordination
        self.waypoints: List[Dict] = []
        self.current_waypoint_index = 0
        self.mission_objectives: List[str] = []

        # Subsystem tracking
        self.subsystems: Dict[str, SubsystemStatus] = {}

        # Performance tracking
        self.performance_metrics = {
            'mission_start_time': None,
            'total_distance_traveled': 0.0,
            'waypoints_completed': 0,
            'navigation_attempts': 0,
            'navigation_successes': 0,
            'emergency_stops': 0,
            'system_resets': 0,
            'average_response_time': 0.0,
            'cpu_usage': 0.0,
            'memory_usage': 0.0
        }

        # Publishers
        self.mission_status_publisher = self.create_publisher(String, 'mission_status', 10)
        self.system_mode_publisher = self.create_publisher(String, 'system_mode', 10)
        self.emergency_stop_publisher = self.create_publisher(Bool, 'emergency_stop', 10)
        self.waypoint_goal_publisher = self.create_publisher(PoseStamped, 'waypoint_goal', 10)

        # Health monitoring publishers
        self.health_status_publisher = self.create_publisher(DiagnosticArray, 'autonomy_health', 10)
        self.performance_metrics_publisher = self.create_publisher(String, 'performance_metrics', 10)

        # Subscribers
        self.emergency_stop_subscription = self.create_subscription(
            Bool, 'emergency_stop_request', self.emergency_stop_callback, 10)

        # Services
        self.start_mission_service = self.create_service(
            Trigger, 'start_mission', self.start_mission_callback)
        self.stop_mission_service = self.create_service(
            Trigger, 'stop_mission', self.stop_mission_callback)
        self.switch_mode_service = self.create_service(
            Trigger, 'switch_mode', self.switch_mode_callback)
        self.reset_autonomy_service = self.create_service(
            Trigger, 'reset_autonomy', self.reset_autonomy_callback)

        # Mission configuration service
        # Note: Using a simple service for now - can be replaced with proper message types later
        self.configure_mission_service = self.create_service(
            Trigger, 'configure_mission', self.configure_mission_callback)

        # Testing service for LED coordination
        self.simulate_target_reached_service = self.create_service(
            Trigger, 'simulate_target_reached', self.simulate_target_reached_callback)

        # Timers
        self.health_check_timer = self.create_timer(
            self.health_check_interval, self.health_check_callback)
        self.status_publish_timer = self.create_timer(1.0, self.status_publish_callback)

        # Initialize subsystems
        self.initialize_subsystems()

        self.get_logger().info('State management node initialized')

    def initialize_subsystems(self):
        """Initialize subsystem status tracking"""
        subsystem_names = [
            'navigation', 'slam', 'computer_vision',
            'autonomous_typing', 'led_status'
        ]

        for name in subsystem_names:
            self.subsystems[name] = SubsystemStatus(
                name=name,
                state='unknown',
                health=1.0,
                last_update=self.get_clock().now().seconds_nanoseconds()[0]
            )

    def configure_mission_callback(self, request, response):
        """Configure mission parameters (placeholder for now)"""
        # TODO: Implement proper mission configuration with waypoints and objectives
        # For now, set up a basic mission with sample waypoints

        # Sample waypoints for testing (can be loaded from parameters or services later)
        self.waypoints = [
            {"latitude": 37.7749, "longitude": -122.4194, "name": "waypoint_1"},
            {"latitude": 37.7849, "longitude": -122.4094, "name": "waypoint_2"},
            {"latitude": 37.7949, "longitude": -122.3994, "name": "waypoint_3"},
        ]
        self.mission_objectives = ["navigate", "typing"]
        self.current_waypoint_index = 0

        response.success = True
        response.message = f"Mission configured with {len(self.waypoints)} waypoints"
        self.get_logger().info(f'Mission configured: {len(self.waypoints)} waypoints')
        return response

    def start_mission_callback(self, request, response):
        """Start autonomous mission"""
        if self.mission_state == MissionState.PRE_MISSION:
            if not self.waypoints:
                response.success = False
                response.message = "No waypoints configured. Call configure_mission first."
                return response

            self.mission_state = MissionState.AUTONOMOUS_NAVIGATION
            self.system_mode = SystemMode.AUTONOMOUS
            self.mission_start_time = self.get_clock().now().seconds_nanoseconds()[0]
            self.performance_metrics['mission_start_time'] = self.mission_start_time
            self.current_waypoint_index = 0

            # Send first waypoint
            self.send_next_waypoint()

            response.success = True
            response.message = "Mission started successfully"
            self.get_logger().info('Autonomous mission started')
        else:
            response.success = False
            response.message = f"Cannot start mission from state: {self.mission_state.value}"

        return response

    def stop_mission_callback(self, request, response):
        """Stop current mission"""
        self.mission_state = MissionState.PRE_MISSION
        self.system_mode = SystemMode.IDLE
        self.emergency_active = False

        response.success = True
        response.message = "Mission stopped"
        self.get_logger().info('Mission stopped')

        return response

    def switch_mode_callback(self, request, response):
        """Switch between autonomous and teleoperation modes"""
        # Validate current state for mode switching
        if self.mission_state == MissionState.EMERGENCY:
            response.success = False
            response.message = "Cannot switch modes during emergency state"
            return response

        # Determine target mode (toggle between autonomous and teleoperation)
        if self.system_mode == SystemMode.AUTONOMOUS:
            target_mode = SystemMode.TELEOPERATION
            target_mission_state = MissionState.TELEOPERATION
        elif self.system_mode == SystemMode.TELEOPERATION:
            target_mode = SystemMode.AUTONOMOUS
            target_mission_state = MissionState.AUTONOMOUS_NAVIGATION
        elif self.system_mode == SystemMode.IDLE:
            target_mode = SystemMode.AUTONOMOUS
            target_mission_state = MissionState.AUTONOMOUS_NAVIGATION
        else:
            target_mode = SystemMode.TELEOPERATION
            target_mission_state = MissionState.TELEOPERATION

        # Validate transition safety
        if not self.validate_mode_transition(target_mode):
            response.success = False
            response.message = f"Unsafe to transition to {target_mode.value} mode"
            return response

        # Execute mode transition
        old_mode = self.system_mode
        self.system_mode = target_mode
        self.mission_state = target_mission_state

        # Notify subsystems of mode change
        self.notify_mode_change(target_mode)

        response.success = True
        response.message = f"Successfully switched from {old_mode.value} to {target_mode.value}"
        self.get_logger().info(f'Mode switched: {old_mode.value} -> {target_mode.value}')
        return response

    def simulate_target_reached_callback(self, request, response):
        """Simulate reaching a target for LED testing"""
        if self.mission_state == MissionState.AUTONOMOUS_NAVIGATION:
            # Simulate successful target arrival
            self.mission_state = MissionState.COMPLETED

            response.success = True
            response.message = "Target reached - mission completed (LED should flash green)"
            self.get_logger().info('Simulated target reached - LED should flash green')
        else:
            response.success = False
            response.message = f"Cannot simulate target reached from state: {self.mission_state.value}"

        return response

    def reset_autonomy_callback(self, request, response):
        """Reset autonomy system to initial state"""
        self.get_logger().info('Resetting autonomy system...')

        # Reset mission state
        self.mission_state = MissionState.PRE_MISSION
        self.system_mode = SystemMode.IDLE
        self.mission_start_time = None
        self.emergency_active = False

        # Clear mission data
        self.waypoints.clear()
        self.current_waypoint_index = 0
        self.mission_objectives.clear()

        # Reset subsystem health
        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        for status in self.subsystems.values():
            status.state = 'unknown'
            status.health = 1.0
            status.last_update = current_time

        # Reset performance metrics
        self.performance_metrics['system_resets'] += 1
        self.performance_metrics['mission_start_time'] = None
        self.performance_metrics['total_distance_traveled'] = 0.0
        self.performance_metrics['waypoints_completed'] = 0
        self.performance_metrics['navigation_attempts'] = 0
        self.performance_metrics['navigation_successes'] = 0
        self.performance_metrics['emergency_stops'] = 0

        # Publish reset status
        self.publish_health_status()

        response.success = True
        response.message = "Autonomy system reset to initial state"
        self.get_logger().info('Autonomy system reset complete')

        return response

    def emergency_stop_callback(self, msg: Bool):
        """Handle emergency stop requests"""
        if msg.data:
            self.emergency_active = True
            self.mission_state = MissionState.EMERGENCY
            self.system_mode = SystemMode.IDLE

            # Record emergency stop for performance tracking
            self.record_emergency_stop()

            # Publish emergency stop to all subsystems
            emergency_msg = Bool()
            emergency_msg.data = True
            self.emergency_stop_publisher.publish(emergency_msg)

            self.get_logger().warn('Emergency stop activated!')

    def health_check_callback(self):
        """Periodic health check of all subsystems"""
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Check each subsystem's health
        for subsystem_name, status in self.subsystems.items():
            # Check if subsystem has been updating recently (within 5 seconds)
            time_since_update = current_time - status.last_update
            if time_since_update > 5.0:
                status.health = max(0.0, status.health - 0.1)  # Degrade health
                status.state = "unresponsive"
                self.get_logger().warn(f'Subsystem {subsystem_name} is unresponsive')
            else:
                # Basic health recovery if responsive
                status.health = min(1.0, status.health + 0.05)

        # Publish comprehensive health status
        self.publish_health_status()

        # Monitor system performance
        self.update_performance_metrics()

    def status_publish_callback(self):
        """Publish current system status"""
        # Mission status - use LED controller compatible format
        mission_msg = String()
        mission_msg.data = self.mission_state.value  # "pre_mission", "autonomous_navigation", "completed", etc.
        self.mission_status_publisher.publish(mission_msg)

        # System mode - use LED controller compatible format
        mode_msg = String()
        mode_msg.data = self.system_mode.value  # "idle", "autonomous", "teleoperation", etc.
        self.system_mode_publisher.publish(mode_msg)

    def update_subsystem_status(self, subsystem_name: str, state: str, health: float):
        """Update individual subsystem status"""
        if subsystem_name in self.subsystems:
            self.subsystems[subsystem_name].state = state
            self.subsystems[subsystem_name].health = health
            self.subsystems[subsystem_name].last_update = self.get_clock().now().seconds_nanoseconds()[0]

    def get_system_health(self) -> float:
        """Get overall system health score"""
        if not self.subsystems:
            return 1.0

        total_health = sum(subsystem.health for subsystem in self.subsystems.values())
        return total_health / len(self.subsystems)

    def check_mission_timeout(self) -> bool:
        """Check if mission has exceeded time limit"""
        if not self.mission_start_time:
            return False

        elapsed = self.get_clock().now().seconds_nanoseconds()[0] - self.mission_start_time
        return elapsed > self.mission_duration_limit

    def validate_mode_transition(self, target_mode: SystemMode) -> bool:
        """Validate if mode transition is safe"""
        # Check subsystem health before transitioning to autonomous
        if target_mode == SystemMode.AUTONOMOUS:
            critical_subsystems = ['navigation', 'slam']
            for subsystem_name in critical_subsystems:
                if subsystem_name in self.subsystems:
                    subsystem = self.subsystems[subsystem_name]
                    if subsystem.health < 0.7:  # Require 70% health for autonomous
                        self.get_logger().warn(f'Subsystem {subsystem_name} health too low for autonomous mode')
                        return False

        # Check for emergency conditions
        if self.emergency_active:
            self.get_logger().warn('Emergency active - mode transitions blocked')
            return False

        return True

    def notify_mode_change(self, new_mode: SystemMode):
        """Notify all subsystems of mode change"""
        # TODO: Implement subsystem notifications
        # This could involve publishing to specific topics or calling services
        # For now, just log the mode change
        self.get_logger().info(f'Notified subsystems of mode change to: {new_mode.value}')

    def validate_state_transition(self, new_state: MissionState) -> bool:
        """Validate if state transition is allowed"""
        # Basic validation - can be extended based on requirements
        current_state = self.mission_state

        # Define allowed transitions
        allowed_transitions = {
            MissionState.PRE_MISSION: [MissionState.AUTONOMOUS_NAVIGATION, MissionState.TELEOPERATION],
            MissionState.AUTONOMOUS_NAVIGATION: [MissionState.TELEOPERATION, MissionState.EMERGENCY, MissionState.COMPLETED, MissionState.FAILED],
            MissionState.TELEOPERATION: [MissionState.AUTONOMOUS_NAVIGATION, MissionState.EMERGENCY, MissionState.PRE_MISSION],
            MissionState.EQUIPMENT_SERVICING: [MissionState.TELEOPERATION, MissionState.EMERGENCY, MissionState.COMPLETED, MissionState.FAILED],
            MissionState.EMERGENCY: [MissionState.PRE_MISSION],  # Only allow reset from emergency
            MissionState.COMPLETED: [MissionState.PRE_MISSION],
            MissionState.FAILED: [MissionState.PRE_MISSION],
        }

        if new_state not in allowed_transitions.get(current_state, []):
            self.get_logger().warn(f'Invalid state transition: {current_state.value} -> {new_state.value}')
            return False

        return True

    def send_next_waypoint(self):
        """Send the next waypoint to navigation system"""
        if self.current_waypoint_index < len(self.waypoints):
            waypoint = self.waypoints[self.current_waypoint_index]

            # Create PoseStamped message for waypoint
            pose_msg = PoseStamped()
            pose_msg.header.stamp = self.get_clock().now().to_msg()
            pose_msg.header.frame_id = "map"

            # Convert lat/lon to approximate local coordinates (placeholder)
            # TODO: Implement proper coordinate transformation
            pose_msg.pose.position.x = (waypoint["longitude"] + 122.4) * 100000  # Rough approximation
            pose_msg.pose.position.y = (waypoint["latitude"] - 37.77) * 100000
            pose_msg.pose.position.z = 0.0
            pose_msg.pose.orientation.w = 1.0  # No rotation

            self.waypoint_goal_publisher.publish(pose_msg)
            self.get_logger().info(f'Sent waypoint {self.current_waypoint_index + 1}/{len(self.waypoints)}: {waypoint["name"]}')

    def advance_to_next_waypoint(self):
        """Advance to the next waypoint in the mission"""
        self.current_waypoint_index += 1

        if self.current_waypoint_index >= len(self.waypoints):
            # Mission complete
            self.mission_state = MissionState.EQUIPMENT_SERVICING  # Or COMPLETED depending on objectives
            self.get_logger().info('All waypoints completed - transitioning to equipment servicing')
        else:
            # Send next waypoint
            self.send_next_waypoint()

    def waypoint_reached_callback(self, waypoint_name: str):
        """Called when a waypoint is successfully reached"""
        self.get_logger().info(f'Waypoint reached: {waypoint_name}')

        # Record successful navigation for performance tracking
        self.record_navigation_attempt(success=True)
        self.record_waypoint_completion()

        self.advance_to_next_waypoint()

    def publish_health_status(self):
        """Publish comprehensive health status for all subsystems"""
        diagnostic_array = DiagnosticArray()
        diagnostic_array.header.stamp = self.get_clock().now().to_msg()
        diagnostic_array.header.frame_id = "autonomy_system"

        for subsystem_name, status in self.subsystems.items():
            diagnostic_status = DiagnosticStatus()

            # Set status level based on health
            if status.health >= 0.8:
                diagnostic_status.level = DiagnosticStatus.OK
            elif status.health >= 0.5:
                diagnostic_status.level = DiagnosticStatus.WARN
            else:
                diagnostic_status.level = DiagnosticStatus.ERROR

            diagnostic_status.name = f"autonomy/{subsystem_name}"
            diagnostic_status.message = status.state
            diagnostic_status.hardware_id = f"subsystem_{subsystem_name}"

            # Add key-value pairs with health metrics
            diagnostic_status.values.append(KeyValue(key="health_score", value=f"{status.health:.2f}"))
            diagnostic_status.values.append(KeyValue(key="last_update", value=f"{status.last_update}"))
            diagnostic_status.values.append(KeyValue(key="active", value=str(status.active)))

            diagnostic_array.status.append(diagnostic_status)

        self.health_status_publisher.publish(diagnostic_array)

    def update_performance_metrics(self):
        """Update and publish performance metrics"""
        # Calculate system uptime
        uptime = 0.0
        if self.mission_start_time:
            uptime = self.get_clock().now().seconds_nanoseconds()[0] - self.mission_start_time

        # Calculate navigation success rate
        nav_success_rate = 0.0
        if self.performance_metrics['navigation_attempts'] > 0:
            nav_success_rate = self.performance_metrics['navigation_successes'] / self.performance_metrics['navigation_attempts']

        # Create performance summary
        performance_summary = f"""Performance Metrics:
        Uptime: {uptime:.1f}s
        Waypoints Completed: {self.performance_metrics['waypoints_completed']}
        Navigation Success Rate: {nav_success_rate:.1%}
        Emergency Stops: {self.performance_metrics['emergency_stops']}
        System Resets: {self.performance_metrics['system_resets']}
        Overall Health: {self.get_system_health():.1%}
        """

        # Publish performance metrics
        msg = String()
        msg.data = performance_summary
        self.performance_metrics_publisher.publish(msg)

    def record_navigation_attempt(self, success: bool):
        """Record navigation attempt for performance tracking"""
        self.performance_metrics['navigation_attempts'] += 1
        if success:
            self.performance_metrics['navigation_successes'] += 1

    def record_emergency_stop(self):
        """Record emergency stop for performance tracking"""
        self.performance_metrics['emergency_stops'] += 1

    def record_waypoint_completion(self):
        """Record waypoint completion for performance tracking"""
        self.performance_metrics['waypoints_completed'] += 1

    def shutdown(self):
        """Shutdown state management system"""
        # Publish final health status
        self.publish_health_status()

        # Log final performance metrics
        self.get_logger().info(f'Final performance: {self.performance_metrics}')

        # TODO: Clean shutdown
        # - Stop all subsystems
        # - Save mission state
        # - Close communications
        self.get_logger().info('State management node shutting down')


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    state_node = StateManagementNode()

    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_node.shutdown()
        state_node.destroy_node()
        rclpy.shutdown()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    state_node = StateManagementNode()

    try:
        rclpy.spin(state_node)
    except KeyboardInterrupt:
        pass
    finally:
        state_node.shutdown()
        state_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
