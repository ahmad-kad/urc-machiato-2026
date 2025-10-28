"""
Integration tests for state machine.

Tests state machine integration with ROS2, subsystem coordination,
and frontend communication.
"""

import pytest
import time
import rclpy
from rclpy.node import Node
from autonomy_interfaces.srv import ChangeState, GetSystemState
from autonomy_interfaces.msg import SystemState as SystemStateMsg


@pytest.fixture
def ros_context():
    """Initialize and cleanup ROS context."""
    rclpy.init()
    yield
    rclpy.shutdown()


class TestStateMachineIntegration:
    """Integration tests for state machine director node."""

    def test_state_machine_node_creation(self, ros_context):
        """Test that state machine node can be created."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        node = StateMachineDirector()
        assert node is not None
        assert node.get_name() == "state_machine_director"
        node.destroy_node()

    def test_service_availability(self, ros_context):
        """Test that state machine services are available."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        node = StateMachineDirector()

        # Spin briefly to allow services to be advertised
        rclpy.spin_once(node, timeout_sec=0.5)

        # Create test client
        client = node.create_client(ChangeState, "/state_machine/change_state")
        assert client.wait_for_service(timeout_sec=2.0)

        node.destroy_node()

    def test_state_publishing(self, ros_context):
        """Test that state machine publishes state updates."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector

        director = StateMachineDirector()

        # Create subscriber node
        class StateListener(Node):
            def __init__(self):
                super().__init__("test_listener")
                self.received_states = []
                self.subscription = self.create_subscription(
                    SystemStateMsg,
                    "/state_machine/current_state",
                    self.state_callback,
                    10,
                )

            def state_callback(self, msg):
                self.received_states.append(msg)

        listener = StateListener()

        # Spin both nodes for a bit
        for _ in range(10):
            rclpy.spin_once(director, timeout_sec=0.1)
            rclpy.spin_once(listener, timeout_sec=0.1)
            time.sleep(0.1)

        # Should have received some state updates
        assert len(listener.received_states) > 0
        assert listener.received_states[0].current_state == "BOOT"

        director.destroy_node()
        listener.destroy_node()


class TestFrontendIntegration:
    """Integration tests for frontend interface."""

    def test_frontend_interface_creation(self, ros_context):
        """Test that frontend interface can be created."""
        from autonomy_state_machine.frontend_interface import FrontendInterface

        node = Node("test_frontend")
        interface = FrontendInterface(node)
        assert interface is not None

        node.destroy_node()

    def test_state_callback_registration(self, ros_context):
        """Test that state callbacks can be registered."""
        from autonomy_state_machine.frontend_interface import FrontendInterface

        node = Node("test_frontend")
        interface = FrontendInterface(node)

        callback_called = [False]

        def test_callback(msg):
            callback_called[0] = True

        interface.register_state_callback(test_callback)

        # Simulate state update
        from autonomy_interfaces.msg import SystemState as SystemStateMsg

        msg = SystemStateMsg()
        msg.current_state = "IDLE"
        interface._state_callback(msg)

        assert callback_called[0] is True

        node.destroy_node()


class TestSubsystemCoordination:
    """Integration tests for subsystem coordination."""

    def test_subsystem_coordinator_creation(self, ros_context):
        """Test that subsystem coordinator can be created."""
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)
        assert coordinator is not None

        node.destroy_node()

    def test_subsystem_initialization(self, ros_context):
        """Test subsystem initialization."""
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)

        coordinator.initialize_subsystems()
        active = coordinator.get_active_subsystems()
        assert len(active) > 0

        node.destroy_node()

    def test_subsystem_activation(self, ros_context):
        """Test subsystem activation for different states."""
        from autonomy_state_machine.subsystem_coordinator import SubsystemCoordinator
        from autonomy_state_machine.states import AutonomousSubstate

        node = Node("test_node")
        coordinator = SubsystemCoordinator(node)

        # Test autonomous mode activation
        coordinator.enable_autonomous(AutonomousSubstate.AUTONOMOUS_NAVIGATION)
        active = coordinator.get_active_subsystems()
        assert "navigation" in active
        assert "computer_vision" in active

        node.destroy_node()


class TestSafetyIntegration:
    """Integration tests for safety handling."""

    def test_safety_status_publishing(self, ros_context):
        """Test that safety status is published."""
        from autonomy_state_machine.state_machine_director import StateMachineDirector
        from autonomy_interfaces.msg import SafetyStatus as SafetyStatusMsg

        director = StateMachineDirector()

        # Create subscriber for safety status
        class SafetyListener(Node):
            def __init__(self):
                super().__init__("test_safety_listener")
                self.received_status = []
                self.subscription = self.create_subscription(
                    SafetyStatusMsg,
                    "/state_machine/safety_status",
                    self.safety_callback,
                    10,
                )

            def safety_callback(self, msg):
                self.received_status.append(msg)

        listener = SafetyListener()

        # Spin both nodes
        for _ in range(10):
            rclpy.spin_once(director, timeout_sec=0.1)
            rclpy.spin_once(listener, timeout_sec=0.1)
            time.sleep(0.1)

        # Should have received safety status
        assert len(listener.received_status) > 0

        director.destroy_node()
        listener.destroy_node()


if __name__ == "__main__":
    pytest.main([__file__, "-v"])

