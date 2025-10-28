#!/usr/bin/env python3
"""
LED Status Controller for URC 2026 Competition

Handles LED signaling for:
- 🔴 Red: Autonomous operation
- 🔵 Blue: Teleoperation (manual driving)
- 🟢 Flashing Green: Successful target arrival
- 🟡 Yellow: Calibration and boot states
- ⚪ White: Idle/ready state

Competition Requirements:
- LED status must be judge-visible from 50m distance
- Colors must clearly indicate rover operational mode
- Success indication requires flashing green pattern
- Integration with hierarchical state management system
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
import time
import threading
from enum import Enum

# LED color definitions
class LEDColor(Enum):
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    YELLOW = "yellow"
    WHITE = "white"
    OFF = "off"

# LED pattern definitions
class LEDPattern(Enum):
    SOLID = "solid"
    BLINK = "blink"
    FAST_BLINK = "fast_blink"
    FADE = "fade"
    PULSE = "pulse"

class LEDController(Node):
    """
    ROS2 node for controlling competition LED status signaling.

    Subscribes to:
    - /state_machine/led_info: LED information from state machine
    - /state_machine/system_state: Current system state
    - /mission_status: Current mission progress state (legacy)

    Controls LED hardware to provide visual status indication.
    """

    def __init__(self):
        super().__init__('led_controller')

        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # ROS2 subscriptions
        self.led_info_sub = self.create_subscription(
            String,
            '/state_machine/led_info',
            self.led_info_callback,
            qos_profile
        )

        self.system_state_sub = self.create_subscription(
            String,
            '/state_machine/system_state',
            self.system_state_callback,
            qos_profile
        )

        # Legacy mission status subscription for backward compatibility
        self.mission_status_sub = self.create_subscription(
            String,
            '/mission_status',
            self.mission_status_callback,
            qos_profile
        )

        # Current state
        self.current_led_info = "UNKNOWN_STATE"
        self.current_system_state = "UNKNOWN"
        self.current_mission_status = "pre_mission"
        self.success_flashing = False
        self.flash_thread = None
        self.current_pattern = LEDPattern.SOLID
        self.current_color = LEDColor.OFF

        # Initialize LED hardware (placeholder for actual hardware)
        self.initialize_led_hardware()

        self.get_logger().info('LED Controller initialized with state management integration')

    def initialize_led_hardware(self):
        """
        Initialize LED hardware interface.

        TODO: Replace with actual hardware control (GPIO, PWM, etc.)
        """
        try:
            # Placeholder for hardware initialization
            # This will be replaced with actual GPIO/PWM setup
            self.led_hardware = LEDHardwareInterface()
            self.get_logger().info('LED hardware initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize LED hardware: {e}')
            # Continue with software-only mode for testing

    def led_info_callback(self, msg: String):
        """
        Handle LED information from state machine.

        This is the primary method for LED control based on state management.
        """
        led_info = msg.data
        self.get_logger().debug(f'LED info received: {led_info}')

        if led_info != self.current_led_info:
            self.current_led_info = led_info
            self.process_led_info(led_info)

    def system_state_callback(self, msg: String):
        """
        Handle system state changes from state machine.

        This provides additional context for LED control decisions.
        """
        system_state = msg.data
        self.get_logger().debug(f'System state changed to: {system_state}')
        self.current_system_state = system_state

    def mission_status_callback(self, msg: String):
        """
        Handle mission status changes (legacy support).

        Competition Requirements:
        - 🟢 Flashing Green on successful target arrival
        """
        new_status = msg.data
        self.get_logger().debug(f'Mission status changed to: {new_status}')

        self.current_mission_status = new_status

        # Check for success condition (only if not already flashing)
        if not self.success_flashing:
            if new_status == "completed" or "arrived" in new_status:
                self.start_success_flashing()
            elif new_status in ["failed", "emergency", "idle"]:
                self.stop_success_flashing()

    def process_led_info(self, led_info: str):
        """
        Process LED information from state machine and set appropriate LED state.

        Args:
            led_info: LED information string from state machine
        """
        # Stop any ongoing flashing when processing new LED info
        self.stop_success_flashing()

        # Parse LED information and set appropriate color/pattern
        if led_info == "AUTONOMOUS_RED":
            self.set_led_state(LEDColor.RED, LEDPattern.SOLID)
            
        elif led_info == "TELEOPERATION_BLUE":
            self.set_led_state(LEDColor.BLUE, LEDPattern.SOLID)
            
        elif led_info == "SAFETY_RED_BLINK":
            self.set_led_state(LEDColor.RED, LEDPattern.FAST_BLINK)
            
        elif led_info == "BOOT_INITIALIZING":
            self.set_led_state(LEDColor.YELLOW, LEDPattern.BLINK)
            
        elif led_info == "CALIBRATION_YELLOW":
            self.set_led_state(LEDColor.YELLOW, LEDPattern.SOLID)
            
        elif led_info == "IDLE_GREEN":
            self.set_led_state(LEDColor.GREEN, LEDPattern.SOLID)
            
        elif led_info == "SHUTDOWN_RED_FADE":
            self.set_led_state(LEDColor.RED, LEDPattern.FADE)
            
        elif led_info == "WAYPOINT_SUCCESS":
            self.start_success_flashing()
            
        elif led_info == "TRANSITION_IN_PROGRESS":
            self.set_led_state(LEDColor.WHITE, LEDPattern.PULSE)
            
        elif led_info.startswith("ERROR_"):
            error_type = led_info.replace("ERROR_", "")
            self.set_led_state(LEDColor.RED, LEDPattern.FAST_BLINK)
            self.get_logger().warn(f'Error LED pattern set for: {error_type}')
            
        elif led_info.startswith("SUCCESS_"):
            success_type = led_info.replace("SUCCESS_", "")
            self.start_success_flashing()
            self.get_logger().info(f'Success LED pattern set for: {success_type}')
            
        else:
            self.get_logger().warning(f'Unknown LED info: {led_info}')
            self.set_led_state(LEDColor.OFF, LEDPattern.SOLID)

    def set_led_state(self, color: LEDColor, pattern: LEDPattern):
        """
        Set LED to specified color and pattern.

        Args:
            color: LEDColor enum value
            pattern: LEDPattern enum value
        """
        try:
            self.current_color = color
            self.current_pattern = pattern
            
            if hasattr(self, 'led_hardware'):
                self.led_hardware.set_color_and_pattern(color, pattern)
            else:
                # Software simulation
                self.get_logger().info(f'LED set to: {color.value} {pattern.value} (simulation)')

        except Exception as e:
            self.get_logger().error(f'Failed to set LED state: {e}')

    def set_led_color(self, color: LEDColor):
        """
        Set LED to specified color (legacy method for backward compatibility).

        Args:
            color: LEDColor enum value
        """
        self.set_led_state(color, LEDPattern.SOLID)

    def start_success_flashing(self):
        """
        Start flashing green LED for success indication.

        Competition Requirements:
        - Flashing green pattern for target success
        - Must be clearly visible to judges
        """
        if not self.success_flashing:
            self.success_flashing = True
            self.flash_thread = threading.Thread(target=self._flash_green_pattern)
            self.flash_thread.daemon = True
            self.flash_thread.start()
            self.get_logger().info('Started success flashing pattern')

    def stop_success_flashing(self):
        """Stop flashing pattern."""
        if self.success_flashing:
            self.success_flashing = False
            if self.flash_thread and self.flash_thread.is_alive():
                self.flash_thread.join(timeout=1.0)
            self.get_logger().info('Stopped success flashing pattern')

    def _flash_green_pattern(self):
        """
        Flash green LED in success pattern.

        Pattern: 1 Hz flashing (on 0.5s, off 0.5s)
        """
        while self.success_flashing:
            try:
                self.set_led_color(LEDColor.GREEN)
                time.sleep(0.5)
                self.set_led_color(LEDColor.OFF)
                time.sleep(0.5)
            except Exception as e:
                self.get_logger().error(f'Error in flashing pattern: {e}')
                break

    def destroy_node(self):
        """Clean up resources."""
        self.stop_success_flashing()
        super().destroy_node()


class LEDHardwareInterface:
    """
    Hardware interface for LED control.

    TODO: Implement actual hardware control based on available hardware.
    Options:
    - GPIO direct control (Raspberry Pi)
    - PWM control for brightness
    - Dedicated LED controller (Arduino, etc.)
    - Addressable LED strips
    """

    def __init__(self):
        # TODO: Initialize hardware interface
        # Examples:
        # - RPi.GPIO setup
        # - pigpio for PWM
        # - Serial communication to microcontroller
        pass

    def set_color_and_pattern(self, color: LEDColor, pattern: LEDPattern):
        """
        Set LED to specified color and pattern.

        Args:
            color: LEDColor enum value
            pattern: LEDPattern enum value
        """
        # TODO: Implement actual hardware control
        # Example GPIO implementation:
        """
        if color == LEDColor.RED:
            GPIO.output(RED_PIN, GPIO.HIGH)
            GPIO.output(BLUE_PIN, GPIO.LOW)
            GPIO.output(GREEN_PIN, GPIO.LOW)
        elif color == LEDColor.BLUE:
            GPIO.output(RED_PIN, GPIO.LOW)
            GPIO.output(BLUE_PIN, GPIO.HIGH)
            GPIO.output(GREEN_PIN, GPIO.LOW)
        elif color == LEDColor.GREEN:
            GPIO.output(RED_PIN, GPIO.LOW)
            GPIO.output(BLUE_PIN, GPIO.LOW)
            GPIO.output(GREEN_PIN, GPIO.HIGH)
        # etc.
        
        # Handle patterns
        if pattern == LEDPattern.BLINK:
            # Implement blinking logic
            pass
        elif pattern == LEDPattern.FAST_BLINK:
            # Implement fast blinking logic
            pass
        elif pattern == LEDPattern.FADE:
            # Implement fading logic with PWM
            pass
        elif pattern == LEDPattern.PULSE:
            # Implement pulsing logic
            pass
        """
        pass

    def set_color(self, color: LEDColor):
        """
        Set LED to specified color (legacy method).

        Args:
            color: LEDColor enum value
        """
        self.set_color_and_pattern(color, LEDPattern.SOLID)


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        led_controller = LEDController()
        rclpy.spin(led_controller)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
