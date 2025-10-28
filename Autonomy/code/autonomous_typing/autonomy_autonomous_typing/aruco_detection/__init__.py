"""
ArUco Detection Module for Autonomous Typing.

Provides ArUco tag detection and alignment calculation for autonomous typing missions.
Works with any ArUco tag IDs detected in the scene.
"""

from .typing_alignment_calculator import TypingAlignmentCalculator
from .typing_aruco_detector import TypingArUcoDetector

__all__ = [
    'TypingAlignmentCalculator',
    'TypingArUcoDetector',
]
