#!/usr/bin/env python3
"""
LED Status Controller for URC 2026 Competition

Handles LED signaling for:
- 🔴 Red: Autonomous operation
- 🔵 Blue: Teleoperation (manual driving)
- 🟢 Flashing Green: Successful target arrival

Competition Requirements:
- LED status must be judge-visible from 50m distance
- Colors must clearly indicate rover operational mode
- Success indication requires flashing green pattern
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time
import threading

# LED color definitions
class LEDColor:
    RED = "red"
    BLUE = "blue"
    GREEN = "green"
    OFF = "off"

class LEDController(Node):
    """
    ROS2 node for controlling competition LED status signaling.

    Subscribes to:
    - /system_mode: Current system operating mode
    - /mission_status: Current mission progress state

    Controls LED hardware to provide visual status indication.
    """

    def __init__(self):
        super().__init__('led_controller')

        # ROS2 subscriptions
        self.system_mode_sub = self.create_subscription(
            String,
            'system_mode',
            self.system_mode_callback,
            10
        )

        self.mission_status_sub = self.create_subscription(
            String,
            'mission_status',
            self.mission_status_callback,
            10
        )

        # Current state
        self.current_mode = "idle"
        self.current_mission_status = "pre_mission"
        self.success_flashing = False
        self.flash_thread = None

        # Initialize LED hardware (placeholder for actual hardware)
        self.initialize_led_hardware()

        self.get_logger().info('LED Controller initialized')

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

    def system_mode_callback(self, msg):
        """
        Handle system mode changes.

        Competition Requirements:
        - 🔴 Red during autonomous operation
        - 🔵 Blue during teleoperation
        """
        new_mode = msg.data
        self.get_logger().info(f'System mode changed to: {new_mode}')

        if new_mode != self.current_mode:
            self.current_mode = new_mode

            # Stop any ongoing flashing
            self.stop_success_flashing()

            # Set LED based on mode
            if new_mode == "autonomous":
                self.set_led_color(LEDColor.RED)
            elif new_mode == "teleoperation" or new_mode == "manual_override":
                self.set_led_color(LEDColor.BLUE)
            elif new_mode == "idle":
                self.set_led_color(LEDColor.OFF)
            else:
                self.get_logger().warning(f'Unknown system mode: {new_mode}')

    def mission_status_callback(self, msg):
        """
        Handle mission status changes.

        Competition Requirements:
        - 🟢 Flashing Green on successful target arrival
        """
        new_status = msg.data
        self.get_logger().info(f'Mission status changed to: {new_status}')

        self.current_mission_status = new_status

        # Check for success condition
        if new_status == "completed" or "arrived" in new_status:
            self.start_success_flashing()
        elif new_status in ["failed", "emergency", "idle"]:
            self.stop_success_flashing()
            if self.current_mode == "idle":
                self.set_led_color(LEDColor.OFF)

    def set_led_color(self, color):
        """
        Set LED to specified color.

        Args:
            color: LEDColor enum value
        """
        try:
            if hasattr(self, 'led_hardware'):
                self.led_hardware.set_color(color)
            else:
                # Software simulation
                self.get_logger().info(f'LED set to: {color} (simulation)')

        except Exception as e:
            self.get_logger().error(f'Failed to set LED color: {e}')

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

    def set_color(self, color):
        """
        Set LED to specified color.

        Args:
            color: LEDColor enum value
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
        # etc.
        """
        pass


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
