# Autonomy Team Tasks - University Rover Challenge 2026

## Overview
As part of the autonomy team, your primary responsibility is implementing autonomous capabilities for the rover to complete the Autonomous Navigation Mission and autonomous portions of the Equipment Servicing Mission. The rover must be able to operate without human intervention for specific tasks while maintaining the ability to switch between autonomous and teleoperation modes.

## Autonomy Subsystems Breakdown

### 1. Navigation & Path Planning
**Primary Responsibility**: Implement autonomous navigation to reach specified targets while avoiding obstacles and managing terrain traversal.

#### Key Tasks:
- **GNSS-Only Navigation (2 targets)**:
  - Navigate to highly accurate GNSS coordinates without visual markers
  - Must stop within 3 meters of the target location
  - Computer vision cannot assist with these targets
  - Implement differential GNSS for higher accuracy (strongly encouraged)

- **AR Tag Navigation (2 targets)**:
  - Navigate to posts using GNSS coordinates as initial guidance (5-10m and 10-20m accuracy)
  - Must stop within 2 meters of the post
  - Integrate with computer vision for final approach and verification

- **Global Path Planning**:
  - Plan routes across varied terrain covering up to 2 km total distance
  - Handle desert environment challenges (sand, rocks, slopes)
  - Optimize for 30-minute time limit across 7 targets

- **Local Path Planning & Obstacle Avoidance**:
  - Navigate boulder fields around ground objects
  - Real-time obstacle detection and avoidance
  - Terrain adaptability (soft sand, gravel, rocky areas)

#### Technical Requirements:
- Autonomous decision-making for target acquisition
- Path planning algorithms (A*, RRT, or similar)
- Terrain classification and traversal optimization
- Integration with drive system controls

---

### 2. SLAM (Simultaneous Localization and Mapping)
**Primary Responsibility**: Create and maintain accurate maps of the environment while tracking rover position relative to those maps.

#### Key Tasks:
- **Environmental Mapping**:
  - Build 3D maps of terrain features and obstacles
  - Handle dynamic outdoor environments (dust, wind, lighting changes)
  - Map construction for path planning and obstacle avoidance

- **Localization**:
  - Maintain accurate rover pose estimation
  - Fuse multiple sensor inputs (GNSS, IMU, odometry, vision)
  - Handle GPS-denied scenarios and signal degradation

- **Loop Closure & Map Updates**:
  - Detect when rover revisits previously mapped areas
  - Update maps with new information
  - Maintain consistency across large areas (up to 2km)

#### Technical Requirements:
- SLAM algorithm implementation (Lidar-based or visual SLAM)
- Sensor fusion for robust localization
- Real-time performance in outdoor conditions

---

### 3. Computer Vision & Object Classification
**Primary Responsibility**: Detect, classify, and track visual targets including AR tags and ground objects.

#### Key Tasks:
- **AR Tag Detection (Navigation Posts)**:
  - Detect and decode ArUco tags (4x4_50 library)
  - Handle 3-sided markers (20x20cm faces, 0.5-1.5m height)
  - White border detection (1 cell, 2.5cm cells)
  - Robust detection in outdoor lighting conditions

- **Object Detection & Classification (Ground Objects)**:
  - Detect 3 specific objects autonomously:
    1. Orange rubber mallet
    2. Rock pick hammer
    3. Standard 1L plastic water bottle (21.5cm tall, 9cm diameter)
  - Real-time object recognition and tracking
  - Handle varying colors, orientations, and partial occlusions
  - **Mandatory**: Highlight/designate detected objects on C2 station display
  - Only highlight 1 object at a time on display

- **Visual Servoing**:
  - Use vision for precise final approach to targets
  - Integration with navigation system for accurate positioning

#### Technical Requirements:
- Convolutional neural networks for object detection
- ArUco marker detection and pose estimation
- Real-time image processing pipeline
- Robust performance in desert conditions (dust, wind, temperature)

---

### 4. Autonomous Typing System
**Primary Responsibility**: Implement robotic keyboard interaction for the Equipment Servicing Mission.

#### Key Tasks:
- **Keyboard Detection & Alignment**:
  - Locate vertically mounted keyboard using ArUco fiducial markers (2x2cm)
  - Calculate precise keyboard pose and orientation
  - Handle alignment variations and mounting positions

- **Character Recognition & Input**:
  - Recognize keyboard layout and key positions
  - Execute precise typing of 3-6 letter launch codes
  - Implement backspace/delete functionality for corrections
  - Handle multiple attempts with autonomous error recovery

- **Precision Manipulation**:
  - Control robotic arm/finger mechanisms for key pressing
  - Maintain accuracy despite vibrations and movement
  - Ensure consistent contact pressure and timing

#### Technical Requirements:
- Computer vision for keyboard detection
- Robotic manipulation control algorithms
- Error detection and correction systems
- Integration with mission state management

---

### 5. State Management & Mode Control
**Primary Responsibility**: Manage autonomous vs teleoperation modes and overall mission state.

#### Key Tasks:
- **Mode Switching**:
  - Seamless transition between autonomous and teleoperation modes
  - Maintain system state during transitions
  - Handle emergency mode switches

- **Mission State Tracking**:
  - Track progress through target sequence
  - Manage autonomous decision-making for target completion
  - Implement abort and return-to-previous functionality

- **Autonomous Decision Making**:
  - Determine when targets are successfully reached
  - Trigger appropriate LED signals and C2 station displays
  - Handle mission completion and timeout conditions

#### Technical Requirements:
- Finite state machine implementation
- Real-time state synchronization
- Error handling and recovery protocols

---

### 6. LED Control & Status Signaling
**Primary Responsibility**: Implement visual status indicators for judges and operators.

#### Key Tasks:
- **LED System Design**:
  - High-power LED array visible in bright daylight
  - Three distinct signaling states:
    - Red: Autonomous operation
    - Blue: Teleoperation (manual driving)
    - Flashing Green: Successful arrival at target
  - Reliable operation in desert environment

- **Status Integration**:
  - Synchronize LED states with autonomy system modes
  - Ensure visibility from required distances and angles

#### Technical Requirements:
- Hardware control interface
- Power management for LED system
- Integration with main autonomy controller

## Technical Requirements

### Communication & Control
- Real-time wireless communication with no time delay
- Ability to switch between autonomous and teleoperation modes
- Operators cannot directly view rover (blocked visibility)
- Line-of-sight communication not guaranteed for all missions

### Hardware Integration
- LED indicator system (high power LED/array for daylight visibility)
- GNSS system (WGS 84 datum, differential GNSS encouraged)
- Computer vision system for object detection and AR tag recognition
- Keyboard interaction capabilities for Equipment Servicing Mission

### Software Architecture
- Autonomous decision-making for target acquisition
- Path planning and obstacle avoidance
- Object recognition algorithms
- State management for autonomous vs teleop modes
- Emergency stop integration (red push button)

## Development Timeline
- **Preliminary Design Review (Dec 3, 2025)**: Include autonomy system design and development plans
- **System Acceptance Review (Feb 27, 2026)**: Demonstrate autonomous capabilities
- **Science Plan Submission (May 15, 2026)**: May include autonomy testing details
- **Field Competition (May 27-30, 2026)**: Execute autonomous missions

## Testing & Validation
- Test autonomous navigation in varied terrain conditions
- Validate object detection under different lighting/wind conditions
- Ensure reliable switching between autonomous and teleop modes
- Test abort and return-to-previous-location functionality
- Validate LED indicators and C2 station signaling

## Success Criteria
- Successful autonomous navigation to all 7 targets within time limits
- Accurate object detection and designation
- Reliable autonomous typing functionality
- Seamless mode switching without system failures
- Compliance with all LED signaling requirements

## Key Challenges
- Operating in desert environment (dust, wind, temperature variations)
- Limited communication range and potential interference
- Precise GNSS accuracy requirements
- Real-time computer vision in outdoor conditions
- Integration with overall rover control systems

## Resources Needed
- GNSS receivers (differential capable)
- Cameras for computer vision
- Computing hardware for real-time processing
- LED lighting system
- Keyboard interaction hardware (for Equipment Servicing)
- Development and testing time in relevant environments
