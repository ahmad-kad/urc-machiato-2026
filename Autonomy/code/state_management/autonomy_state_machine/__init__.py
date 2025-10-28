"""
Autonomy State Machine Package.

This package provides a hierarchical, event-driven state machine for the URC 2026
rover that coordinates all subsystems and provides state management for frontend
integration.
"""

__version__ = "1.0.0"
__author__ = "URC Machiato Team"

from .states import (
    SystemState,
    AutonomousSubstate,
    EquipmentServicingSubstate,
    StateMetadata,
)
from .state_machine_director import StateMachineDirector
from .safety_manager import SafetyManager, SafetyTriggerType, SafetySeverity
from .transition_validator import TransitionValidator

__all__ = [
    "SystemState",
    "AutonomousSubstate",
    "EquipmentServicingSubstate",
    "StateMetadata",
    "StateMachineDirector",
    "SafetyManager",
    "SafetyTriggerType",
    "SafetySeverity",
    "TransitionValidator",
]

