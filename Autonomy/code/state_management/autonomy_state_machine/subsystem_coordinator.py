"""
Subsystem coordinator for state machine.

Coordinates subsystem activation, monitors subsystem status,
and ensures subsystems are properly configured for each state.
"""

from typing import List, Set, Optional, Dict
import structlog
from rclpy.node import Node
from autonomy_interfaces.srv import GetSubsystemStatus

from .states import AutonomousSubstate

logger = structlog.get_logger(__name__)


class SubsystemCoordinator:
    """
    Coordinates subsystem operations for state machine.

    Manages subsystem lifecycle, monitors status, and ensures
    proper subsystem configuration for each state.
    """

    def __init__(self, node: Node):
        """
        Initialize subsystem coordinator.

        Args:
            node: Parent ROS2 node
        """
        self.node = node
        self._active_subsystems: Set[str] = set()
        self._failed_subsystems: Set[str] = set()
        self._subsystem_status: Dict[str, Dict] = {}

        # Known subsystems
        self._known_subsystems = [
            "camera",
            "navigation",
            "computer_vision",
            "slam",
            "manipulation",
            "science_instruments",
            "autonomous_typing",
        ]

        logger.info("Subsystem coordinator initialized")

    def initialize_subsystems(self) -> None:
        """Initialize all subsystems during boot."""
        logger.info("Initializing subsystems")

        # In a real implementation, this would:
        # 1. Check which subsystems are available
        # 2. Initialize communication with each
        # 3. Verify subsystems are ready

        # For now, assume basic subsystems are available
        self._active_subsystems = {"camera", "navigation"}
        logger.info("Basic subsystems initialized", subsystems=list(self._active_subsystems))

    def start_calibration(self) -> None:
        """Start calibration process for sensors."""
        logger.info("Starting calibration")

        # Activate calibration-related subsystems
        self._active_subsystems.add("camera")
        self._active_subsystems.add("navigation")

        # In real implementation:
        # - Call calibration services on camera nodes
        # - Start SLAM calibration
        # - Verify calibration results

        logger.info("Calibration started")

    def enable_teleoperation(self) -> None:
        """Enable subsystems for teleoperation mode."""
        logger.info("Enabling teleoperation mode")

        # Ensure navigation is active
        self._active_subsystems.add("navigation")

        # Optionally add camera for teleoperation visualization
        self._active_subsystems.add("camera")

        logger.info(
            "Teleoperation enabled", active_subsystems=list(self._active_subsystems)
        )

    def enable_autonomous(self, substate: Optional[AutonomousSubstate] = None) -> None:
        """
        Enable subsystems for autonomous mode.

        Args:
            substate: Specific autonomous mission substate
        """
        logger.info("Enabling autonomous mode", substate=str(substate) if substate else None)

        # Core autonomous subsystems
        self._active_subsystems.update(["navigation", "computer_vision", "slam"])

        # Mission-specific subsystems
        if substate == AutonomousSubstate.SCIENCE:
            self._active_subsystems.add("science_instruments")
            logger.info("Science mission subsystems enabled")

        elif substate == AutonomousSubstate.DELIVERY:
            self._active_subsystems.add("manipulation")
            logger.info("Delivery mission subsystems enabled")

        elif substate == AutonomousSubstate.EQUIPMENT_SERVICING:
            self._active_subsystems.update(["manipulation", "autonomous_typing"])
            logger.info("Equipment servicing subsystems enabled")

        elif substate == AutonomousSubstate.AUTONOMOUS_NAVIGATION:
            # Autonomous navigation uses core subsystems only
            logger.info("Autonomous navigation subsystems enabled")

        logger.info(
            "Autonomous mode enabled", active_subsystems=list(self._active_subsystems)
        )

    def engage_safety(self) -> None:
        """Engage safety mode - stop all motion."""
        logger.warning("Engaging safety mode")

        # In real implementation:
        # - Send stop commands to navigation
        # - Halt all manipulator movement
        # - Disable autonomous operations
        # - Keep camera and communication active

        # Keep minimal subsystems active for safety monitoring
        safe_subsystems = {"camera"}
        self._active_subsystems = safe_subsystems

        logger.warning("Safety mode engaged", active_subsystems=list(self._active_subsystems))

    def shutdown_subsystems(self) -> None:
        """Shutdown all subsystems gracefully."""
        logger.info("Shutting down subsystems")

        # In real implementation:
        # - Send shutdown signals to each subsystem
        # - Wait for acknowledgments
        # - Close communication channels

        self._active_subsystems.clear()
        logger.info("All subsystems shutdown")

    def check_subsystem_status(self, subsystem: str) -> Dict:
        """
        Check status of a specific subsystem.

        Args:
            subsystem: Name of subsystem to check

        Returns:
            Dictionary with subsystem status information
        """
        # In real implementation, this would:
        # - Call GetSubsystemStatus service
        # - Return actual status from subsystem

        # For now, return mock status
        is_active = subsystem in self._active_subsystems
        is_failed = subsystem in self._failed_subsystems

        status = {
            "name": subsystem,
            "active": is_active,
            "failed": is_failed,
            "healthy": is_active and not is_failed,
        }

        logger.debug("Subsystem status checked", subsystem=subsystem, status=status)
        return status

    def get_active_subsystems(self) -> List[str]:
        """Get list of currently active subsystems."""
        return list(self._active_subsystems)

    def get_failed_subsystems(self) -> List[str]:
        """Get list of failed subsystems."""
        return list(self._failed_subsystems)

    def mark_subsystem_failed(self, subsystem: str, reason: str = "") -> None:
        """
        Mark a subsystem as failed.

        Args:
            subsystem: Subsystem name
            reason: Reason for failure
        """
        self._failed_subsystems.add(subsystem)
        self._active_subsystems.discard(subsystem)

        logger.error(
            "Subsystem marked as failed",
            subsystem=subsystem,
            reason=reason,
        )

    def recover_subsystem(self, subsystem: str) -> bool:
        """
        Attempt to recover a failed subsystem.

        Args:
            subsystem: Subsystem to recover

        Returns:
            True if recovery successful
        """
        logger.info("Attempting subsystem recovery", subsystem=subsystem)

        # In real implementation:
        # - Restart subsystem
        # - Verify functionality
        # - Re-initialize if needed

        if subsystem in self._failed_subsystems:
            self._failed_subsystems.remove(subsystem)
            self._active_subsystems.add(subsystem)
            logger.info("Subsystem recovered", subsystem=subsystem)
            return True

        logger.warning("Subsystem recovery failed", subsystem=subsystem)
        return False

    def verify_subsystems_ready(self, required_subsystems: List[str]) -> bool:
        """
        Verify that required subsystems are ready.

        Args:
            required_subsystems: List of required subsystem names

        Returns:
            True if all required subsystems are active and healthy
        """
        for subsystem in required_subsystems:
            if subsystem not in self._active_subsystems:
                logger.warning(
                    "Required subsystem not active",
                    subsystem=subsystem,
                )
                return False

            if subsystem in self._failed_subsystems:
                logger.warning(
                    "Required subsystem failed",
                    subsystem=subsystem,
                )
                return False

        logger.debug("All required subsystems ready", subsystems=required_subsystems)
        return True

    def get_subsystem_summary(self) -> Dict:
        """
        Get comprehensive subsystem status summary.

        Returns:
            Dictionary with subsystem summary information
        """
        return {
            "total_known": len(self._known_subsystems),
            "active": len(self._active_subsystems),
            "failed": len(self._failed_subsystems),
            "active_list": list(self._active_subsystems),
            "failed_list": list(self._failed_subsystems),
            "inactive_list": list(
                set(self._known_subsystems) - self._active_subsystems - self._failed_subsystems
            ),
        }

