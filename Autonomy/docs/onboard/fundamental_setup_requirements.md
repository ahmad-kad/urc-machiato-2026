# Fundamental Setup Requirements by Subsystem

Based on analysis of TODO.md files, here are the critical setup requirements for each track:

## 1. State Management (Foundation Layer)
**Critical Path Items (Days 1-8):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- Mission state definitions and transitions
- Health monitoring system
- Emergency stop handling
- Inter-subsystem communication protocols

**Dependencies:** None (central hub)

## 2. Computer Vision (Camera-Heavy)
**Critical Path Items (Days 1-10):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- **Camera Calibration** (Days 1-3) - Affects SLAM/Navigation
- Basic image processing pipeline
- ArUco marker detection (7x7 dictionary)
- Object detection framework setup

**Hardware Requirements:**
- Oak-D RGB-D camera (primary)
- Raspberry Pi AI camera with 360° gimbal
- Intel RealSense D435i (backup)

**Calibration Impact:** Required before SLAM or navigation can function

## 3. SLAM (Localization & Mapping)
**Critical Path Items (Days 1-10):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- Sensor integration (Lidar + Cameras)
- Basic SLAM pipeline (ORB-SLAM3/SLAM Toolbox)
- Multi-modal sensor fusion
- Pose estimation accuracy validation

**Hardware Requirements:**
- RPLidar A2/A3 (360° coverage)
- RGB-D cameras (calibrated)
- IMU for motion compensation

**Calibration Dependencies:** Camera intrinsics/extrinsics, IMU calibration

## 4. Navigation (Motion Control)
**Critical Path Items (Days 1-8):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- Sensor integration (GNSS + IMU)
- Basic movement control (velocity commands)
- GNSS waypoint navigation
- AR tag precision approaches

**Hardware Requirements:**
- RTK GNSS receiver ( centimeter accuracy)
- IMU for heading estimation
- Motor controllers with encoders
- Cameras for visual navigation

**Calibration Dependencies:** Camera calibration, GNSS accuracy validation

## 5. Autonomous Typing (Precision Manipulation)
**Critical Path Items (Days 1-10):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- Robotic arm integration
- Computer vision integration (target detection)
- Precision pose estimation
- Keyboard interaction algorithms

**Hardware Requirements:**
- 6-DOF robotic arm ($1500-2500 budget)
- Force/torque sensors
- Precision cameras for target detection

**Calibration Dependencies:** Camera calibration, hand-eye calibration (arm to camera)

## 6. LED Status (Simple Signaling)
**Critical Path Items (Days 1-4):**
- ROS2 Package Setup (✅ Infrastructure Complete)
- GPIO interface setup
- Status code definitions
- Judge visibility requirements

**Hardware Requirements:**
- RGB LED arrays (high visibility)
- GPIO controller (Raspberry Pi)

**Calibration Dependencies:** None (simple on/off signaling)

---

## Cross-Subsystem Calibration Requirements

### Camera Calibration Priority (Week 1):
1. **Oak-D Camera** → Vision, SLAM, Navigation
2. **Pi Camera + Gimbal** → Vision, Navigation
3. **RealSense D435i** → SLAM backup
4. **Hand-Eye Calibration** → Typing system

### Setup Sequence:
```
Week 1: Infrastructure + Camera Calibration
├── Day 1-2: ROS2 packages + basic camera setup
├── Day 3-5: Camera intrinsic calibration (all cameras)
├── Day 6-7: Camera extrinsic calibration + gimbal calibration
└── Day 8-10: Hand-eye calibration + system integration testing

Week 2: Core Functionality
├── Vision: ArUco detection + basic object recognition
├── SLAM: Sensor fusion + pose estimation
├── Navigation: GNSS waypoints + basic path following
├── Typing: Target detection + basic arm control
└── Integration: Cross-subsystem communication testing
```

### Critical Path Blockers:
- **Camera calibration incomplete** → Vision, SLAM, Navigation blocked
- **Robotic arm not integrated** → Typing system blocked
- **GNSS not configured** → Navigation blocked

### Testing Milestones:
- **Day 7:** Individual subsystems start without errors
- **Day 14:** Basic cross-subsystem communication working
- **Day 21:** Integrated mission simulation functional
- **Day 28:** Competition-ready performance validation
