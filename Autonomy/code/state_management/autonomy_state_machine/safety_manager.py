"""
Safety manager for context-aware safety handling.

Manages safety triggers, determines appropriate responses based on context,
and coordinates recovery procedures.
"""

from enum import Enum, auto
from dataclasses import dataclass
from typing import List, Optional, Dict

from .states import SystemState, AutonomousSubstate


class SafetyTriggerType(Enum):
    """Types of safety triggers."""

    EMERGENCY_STOP = "EMERGENCY_STOP"  # Physical E-stop pressed
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"  # Lost communication link
    BATTERY_CRITICAL = "BATTERY_CRITICAL"  # Battery critically low
    THERMAL_WARNING = "THERMAL_WARNING"  # System overheating
    SENSOR_FAILURE = "SENSOR_FAILURE"  # Critical sensor failure
    OBSTACLE_CRITICAL = "OBSTACLE_CRITICAL"  # Imminent collision
    SYSTEM_FAULT = "SYSTEM_FAULT"  # General system fault
    MANUAL_INTERVENTION = "MANUAL_INTERVENTION"  # Manual safety trigger

    def __str__(self) -> str:
        return self.value


class SafetySeverity(Enum):
    """Severity levels for safety triggers."""

    INFO = "INFO"  # Informational, no action needed
    WARNING = "WARNING"  # Warning, monitor but continue
    CRITICAL = "CRITICAL"  # Critical, degrade operations
    EMERGENCY = "EMERGENCY"  # Emergency, immediate halt

    def __str__(self) -> str:
        return self.value


@dataclass
class RecoveryBehavior:
    """
    Defines recovery behavior for a safety trigger.

    Attributes:
        requires_manual_intervention: Whether manual intervention is required
        can_auto_recover: Whether automatic recovery is possible
        recovery_steps: List of steps needed for recovery
        safe_mode_config: Configuration for safe mode if applicable
        estimated_recovery_time: Estimated time to recover (seconds)
    """

    requires_manual_intervention: bool
    can_auto_recover: bool
    recovery_steps: List[str]
    safe_mode_config: Optional[Dict[str, any]] = None
    estimated_recovery_time: float = 0.0


class SafetyManager:
    """
    Manages safety triggers and recovery procedures.

    Provides context-aware safety handling that adjusts behavior based on
    current state and mission phase.
    """

    def __init__(self, logger=None):
        """Initialize the safety manager."""
        self.logger = logger  # Will be set by state machine director
        self._active_triggers: Dict[SafetyTriggerType, SafetySeverity] = {}
        self._trigger_history: List[Dict] = []
        self._battery_level: float = 100.0
        self._temperature: float = 25.0
        self._communication_ok: bool = True
        self._sensors_ok: bool = True

    def trigger_safety(
        self,
        trigger_type: SafetyTriggerType,
        severity: SafetySeverity,
        description: str,
        source: str = "unknown",
    ) -> None:
        """
        Trigger a safety event.

        Args:
            trigger_type: Type of safety trigger
            severity: Severity level
            description: Human-readable description
            source: Source subsystem that triggered safety
        """
        self._active_triggers[trigger_type] = severity

        trigger_info = {
            "type": trigger_type,
            "severity": severity,
            "description": description,
            "source": source,
        }
        self._trigger_history.append(trigger_info)

        if self.logger:
            self.logger.warning(
                f"Safety triggered: {trigger_type.value} ({severity.value}) - {description} "
                f"(source: {source})"
            )

    def clear_trigger(self, trigger_type: SafetyTriggerType) -> None:
        """Clear a specific safety trigger."""
        if trigger_type in self._active_triggers:
            del self._active_triggers[trigger_type]
            if self.logger:
                self.logger.info(f"Safety trigger cleared: {trigger_type.value}")

    def clear_all_triggers(self) -> None:
        """Clear all active safety triggers."""
        self._active_triggers.clear()
        if self.logger:
            self.logger.info("All safety triggers cleared")

    def get_active_triggers(self) -> List[SafetyTriggerType]:
        """Get list of currently active safety triggers."""
        return list(self._active_triggers.keys())

    def get_highest_severity(self) -> Optional[SafetySeverity]:
        """Get the highest severity of active triggers."""
        if not self._active_triggers:
            return None

        severities = list(self._active_triggers.values())
        # Order: EMERGENCY > CRITICAL > WARNING > INFO
        severity_order = [
            SafetySeverity.EMERGENCY,
            SafetySeverity.CRITICAL,
            SafetySeverity.WARNING,
            SafetySeverity.INFO,
        ]

        for severity in severity_order:
            if severity in severities:
                return severity

        return None

    def determine_recovery(
        self,
        trigger_type: SafetyTriggerType,
        current_state: SystemState,
        mission_phase: Optional[AutonomousSubstate] = None,
    ) -> RecoveryBehavior:
        """
        Determine appropriate recovery behavior based on context.

        Args:
            trigger_type: Type of safety trigger
            current_state: Current system state
            mission_phase: Current mission phase if in autonomous state

        Returns:
            RecoveryBehavior object describing recovery procedure
        """
        if self.logger:
            self.logger.info(
                f"Determining recovery behavior for {trigger_type.value} in {current_state.value} state"
            )

        # Emergency stop always requires manual intervention
        if trigger_type == SafetyTriggerType.EMERGENCY_STOP:
            return RecoveryBehavior(
                requires_manual_intervention=True,
                can_auto_recover=False,
                recovery_steps=[
                    "Reset emergency stop button",
                    "Verify rover systems",
                    "Manually clear safety state",
                ],
                estimated_recovery_time=60.0,
            )

        # Communication loss - different behavior for teleoperation vs autonomous
        if trigger_type == SafetyTriggerType.COMMUNICATION_LOSS:
            if current_state == SystemState.TELEOPERATION:
                # In teleoperation, stop immediately
                return RecoveryBehavior(
                    requires_manual_intervention=True,
                    can_auto_recover=False,
                    recovery_steps=[
                        "Restore communication link",
                        "Verify rover position",
                        "Resume teleoperation",
                    ],
                    estimated_recovery_time=120.0,
                )
            else:
                # In autonomous, can attempt auto-recovery
                return RecoveryBehavior(
                    requires_manual_intervention=False,
                    can_auto_recover=True,
                    recovery_steps=[
                        "Switch to backup communication",
                        "Return to last known good position",
                        "Wait for communication restoration",
                    ],
                    safe_mode_config={"max_velocity": 0.5, "use_backup_comms": True},
                    estimated_recovery_time=180.0,
                )

        # Battery critical - graceful shutdown
        if trigger_type == SafetyTriggerType.BATTERY_CRITICAL:
            return RecoveryBehavior(
                requires_manual_intervention=True,
                can_auto_recover=False,
                recovery_steps=[
                    "Navigate to safe position if possible",
                    "Shutdown non-critical systems",
                    "Save state and prepare for shutdown",
                    "Replace/recharge battery",
                ],
                safe_mode_config={
                    "disable_non_essential": True,
                    "emergency_navigation": True,
                },
                estimated_recovery_time=300.0,
            )

        # Thermal warning - throttle operations
        if trigger_type == SafetyTriggerType.THERMAL_WARNING:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Reduce computational load",
                    "Throttle motor operations",
                    "Monitor temperature",
                    "Resume when temperature normal",
                ],
                safe_mode_config={
                    "max_velocity": 0.3,
                    "reduce_vision_fps": True,
                    "disable_intensive_tasks": True,
                },
                estimated_recovery_time=300.0,
            )

        # Sensor failure - degrade to safe operating mode
        if trigger_type == SafetyTriggerType.SENSOR_FAILURE:
            # More critical during autonomous navigation
            if current_state == SystemState.AUTONOMOUS:
                return RecoveryBehavior(
                    requires_manual_intervention=True,
                    can_auto_recover=False,
                    recovery_steps=[
                        "Stop autonomous operations",
                        "Switch to teleoperation",
                        "Diagnose sensor issue",
                        "Replace or recalibrate sensor",
                    ],
                    estimated_recovery_time=600.0,
                )
            else:
                return RecoveryBehavior(
                    requires_manual_intervention=False,
                    can_auto_recover=True,
                    recovery_steps=[
                        "Switch to backup sensor",
                        "Continue with degraded capability",
                        "Log sensor failure for maintenance",
                    ],
                    safe_mode_config={"use_backup_sensors": True},
                    estimated_recovery_time=30.0,
                )

        # Obstacle critical - immediate stop
        if trigger_type == SafetyTriggerType.OBSTACLE_CRITICAL:
            return RecoveryBehavior(
                requires_manual_intervention=False,
                can_auto_recover=True,
                recovery_steps=[
                    "Emergency stop",
                    "Assess obstacle",
                    "Plan alternative path",
                    "Resume navigation",
                ],
                safe_mode_config={"max_velocity": 0.1, "increase_obstacle_margin": True},
                estimated_recovery_time=60.0,
            )

        # Default system fault behavior
        return RecoveryBehavior(
            requires_manual_intervention=True,
            can_auto_recover=False,
            recovery_steps=[
                "Stop all operations",
                "Diagnose fault",
                "Apply fix",
                "Verify systems",
                "Manually clear safety",
            ],
            estimated_recovery_time=300.0,
        )

    def update_system_health(
        self,
        battery_level: Optional[float] = None,
        temperature: Optional[float] = None,
        communication_ok: Optional[bool] = None,
        sensors_ok: Optional[bool] = None,
    ) -> None:
        """
        Update system health parameters.

        Args:
            battery_level: Battery percentage (0-100)
            temperature: System temperature in Celsius
            communication_ok: Communication link status
            sensors_ok: Sensor system status
        """
        if battery_level is not None:
            self._battery_level = battery_level
            # Auto-trigger battery critical if below threshold
            if battery_level < 10.0:
                self.trigger_safety(
                    SafetyTriggerType.BATTERY_CRITICAL,
                    SafetySeverity.EMERGENCY,
                    f"Battery critically low: {battery_level}%",
                    "power_management",
                )

        if temperature is not None:
            self._temperature = temperature
            # Auto-trigger thermal warning if too hot
            if temperature > 70.0:
                self.trigger_safety(
                    SafetyTriggerType.THERMAL_WARNING,
                    SafetySeverity.CRITICAL,
                    f"System temperature high: {temperature}°C",
                    "thermal_management",
                )

        if communication_ok is not None:
            prev_status = self._communication_ok
            self._communication_ok = communication_ok
            # Trigger if communication lost
            if prev_status and not communication_ok:
                self.trigger_safety(
                    SafetyTriggerType.COMMUNICATION_LOSS,
                    SafetySeverity.EMERGENCY,
                    "Communication link lost",
                    "communication_system",
                )
            # Clear trigger if communication restored
            elif not prev_status and communication_ok:
                self.clear_trigger(SafetyTriggerType.COMMUNICATION_LOSS)

        if sensors_ok is not None:
            prev_status = self._sensors_ok
            self._sensors_ok = sensors_ok
            # Trigger if sensors fail
            if prev_status and not sensors_ok:
                self.trigger_safety(
                    SafetyTriggerType.SENSOR_FAILURE,
                    SafetySeverity.CRITICAL,
                    "Critical sensor failure detected",
                    "sensor_system",
                )
            # Clear trigger if sensors recover
            elif not prev_status and sensors_ok:
                self.clear_trigger(SafetyTriggerType.SENSOR_FAILURE)

    def get_safety_status(self) -> Dict:
        """
        Get comprehensive safety status.

        Returns:
            Dictionary with safety status information
        """
        return {
            "is_safe": len(self._active_triggers) == 0,
            "active_triggers": [str(t) for t in self._active_triggers.keys()],
            "highest_severity": (
                str(self.get_highest_severity())
                if self.get_highest_severity()
                else "NONE"
            ),
            "battery_level": self._battery_level,
            "temperature": self._temperature,
            "communication_ok": self._communication_ok,
            "sensors_ok": self._sensors_ok,
        }

