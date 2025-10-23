# Interface Contract: Autonomy Pi ↔ Other Teams

## Overview

This document defines the **formal contract** for communication between the Autonomy Pi and other system components. All teams must adhere to these interfaces for successful integration.

---

## 📋 Contract Terms

### **Version**: 1.0
### **Effective Date**: Competition Development Period
### **Parties**: Autonomy Team, Control Team, Hardware Team
### **Governing Technology**: ROS2 Humble

### **Obligations**
1. **Interface Stability**: Interfaces remain backward compatible
2. **Documentation Updates**: Interface changes documented immediately
3. **Testing Requirements**: All interfaces tested before integration
4. **Error Handling**: Clear error reporting and recovery protocols

---

## 🔗 ROS2 Topic Interfaces

### **1. Mission Control (State Management → Subsystems)**

#### **`/autonomy/mission_status`**
- **Type**: `std_msgs/String`
- **Purpose**: Current mission state
- **Publisher**: State Management Node
- **Subscriber**: LED Controller, Control Pi
- **Rate**: 1 Hz
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

**Possible Values:**
- `"pre_mission"` - Ready for mission start
- `"autonomous_navigation"` - Executing waypoint navigation
- `"equipment_servicing"` - Performing equipment interaction
- `"teleoperation"` - Under manual control
- `"emergency"` - Emergency stop active
- `"completed"` - Mission completed successfully
- `"failed"` - Mission failed

#### **`/autonomy/system_mode`**
- **Type**: `std_msgs/String`
- **Purpose**: Current system operating mode
- **Publisher**: State Management Node
- **Subscriber**: All components, LED Controller
- **Rate**: On change
- **Reliability**: Reliable + Transient Local
- **Namespace**: `/autonomy`

**Possible Values:**
- `"idle"` - System idle, ready for commands
- `"autonomous"` - Full autonomy mode
- `"teleoperation"` - Manual control mode
- `"manual_override"` - Direct hardware control
- `"emergency"` - Emergency state active

#### **`/autonomy/waypoint_goal`**
- **Type**: `geometry_msgs/PoseStamped`
- **Purpose**: Navigate to specific waypoint
- **Publisher**: State Management Node
- **Subscriber**: Navigation Node
- **Rate**: On-demand
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

#### **`/autonomy/emergency_stop`**
- **Type**: `std_msgs/Bool`
- **Purpose**: Emergency stop command (broadcast to all components)
- **Publisher**: State Management Node
- **Subscriber**: All components
- **Rate**: On-demand
- **Reliability**: Reliable + Transient Local (latches last value)
- **Namespace**: `/autonomy`

### **2. Navigation Subsystem**

#### **`/autonomy/navigation/status`**
- **Type**: `std_msgs/String`
- **Purpose**: Navigation subsystem status
- **Publisher**: Navigation Node
- **Subscriber**: State Management, Control Pi
- **Rate**: 1 Hz
- **Reliability**: Reliable
- **Namespace**: `/autonomy/navigation`

**Possible Values:**
- `"Navigation: idle"`
- `"Navigation: planning"`
- `"Navigation: navigating | Goal: waypoint_1"`
- `"Navigation: arrived"`

#### **`/autonomy/navigation/current_waypoint`**
- **Type**: `geometry_msgs/PoseStamped`
- **Purpose**: Current navigation waypoint
- **Publisher**: Navigation Node
- **Subscriber**: State Management, Control Pi
- **Rate**: On change
- **Reliability**: Reliable
- **Namespace**: `/autonomy/navigation`

#### **`/autonomy/navigation/waypoint_reached`**
- **Type**: `std_msgs/String`
- **Purpose**: Notification when waypoint is reached
- **Publisher**: Navigation Node
- **Subscriber**: State Management
- **Rate**: On event
- **Reliability**: Reliable
- **Namespace**: `/autonomy/navigation`

### **3. Hardware Control (Autonomy Pi ↔ Microcontrollers)**

#### **`/autonomy/cmd_vel`**
- **Type**: `geometry_msgs/Twist`
- **Purpose**: Velocity commands to wheel motors
- **Publisher**: Navigation Node
- **Subscriber**: Wheel Microcontroller
- **Rate**: 10-50 Hz (control loop dependent)
- **Reliability**: Best Effort (real-time control)
- **Namespace**: `/autonomy`

**Message Format:**
```json
{
  "linear": {"x": 0.8, "y": 0.0, "z": 0.0},    // Forward velocity (m/s)
  "angular": {"x": 0.0, "y": 0.0, "z": 0.2}    // Rotation velocity (rad/s)
}
```

#### **`/autonomy/wheel/odom`**
- **Type**: `nav_msgs/Odometry`
- **Purpose**: Wheel encoder-based odometry
- **Publisher**: Wheel Microcontroller
- **Subscriber**: SLAM Node
- **Rate**: 10-50 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

### **4. SLAM Subsystem**

#### **`/autonomy/slam/pose`**
- **Type**: `geometry_msgs/PoseWithCovarianceStamped`
- **Purpose**: SLAM pose estimate
- **Publisher**: SLAM Node
- **Subscriber**: Navigation Node, State Management
- **Rate**: 10 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy/slam`

#### **`/autonomy/slam/odom`**
- **Type**: `nav_msgs/Odometry`
- **Purpose**: SLAM odometry
- **Publisher**: SLAM Node
- **Subscriber**: Navigation Node
- **Rate**: 10 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy/slam`

#### **`/autonomy/slam/status`**
- **Type**: `std_msgs/String`
- **Purpose**: SLAM subsystem status
- **Publisher**: SLAM Node
- **Subscriber**: State Management
- **Rate**: 1 Hz
- **Reliability**: Reliable
- **Namespace**: `/autonomy/slam`

### **5. Computer Vision Subsystem**

#### **`/autonomy/vision/detections`**
- **Type**: `autonomy_interfaces/VisionDetection`
- **Purpose**: Object detections from computer vision
- **Publisher**: Computer Vision Node
- **Subscriber**: Navigation Node, State Management
- **Rate**: Variable (per detection)
- **Reliability**: Best Effort
- **Namespace**: `/autonomy/vision`

#### **`/autonomy/vision/aruco_markers`**
- **Type**: `vision_msgs/Detection3DArray`
- **Purpose**: ArUco marker detections
- **Publisher**: Computer Vision Node
- **Subscriber**: Navigation Node
- **Rate**: Variable
- **Reliability**: Best Effort
- **Namespace**: `/autonomy/vision`

#### **`/autonomy/vision/status`**
- **Type**: `std_msgs/String`
- **Purpose**: Vision subsystem status
- **Publisher**: Computer Vision Node
- **Subscriber**: State Management
- **Rate**: 1 Hz
- **Reliability**: Reliable
- **Namespace**: `/autonomy/vision`

### **6. Autonomous Typing Subsystem**

#### **`/autonomy/typing/status`**
- **Type**: `std_msgs/String`
- **Purpose**: Autonomous typing subsystem status
- **Publisher**: Autonomous Typing Node
- **Subscriber**: State Management
- **Rate**: 1 Hz
- **Reliability**: Reliable
- **Namespace**: `/autonomy/typing`

#### **`/autonomy/typing/target_pose`**
- **Type**: `geometry_msgs/PoseStamped`
- **Purpose**: Target pose for typing operation
- **Publisher**: Autonomous Typing Node
- **Subscriber**: Navigation Node
- **Rate**: On-demand
- **Reliability**: Reliable
- **Namespace**: `/autonomy/typing`

### **7. LED Status Subsystem**

#### **`/autonomy/led/status`**
- **Type**: `std_msgs/String`
- **Purpose**: LED status indicator state
- **Publisher**: LED Controller
- **Subscriber**: State Management
- **Rate**: On change
- **Reliability**: Reliable
- **Namespace**: `/autonomy/led`

#### **`/autonomy/led/command`**
- **Type**: `autonomy_interfaces/LedCommand`
- **Purpose**: Manual LED control commands
- **Publisher**: State Management
- **Subscriber**: LED Controller
- **Rate**: On-demand
- **Reliability**: Reliable
- **Namespace**: `/autonomy/led`

### **8. Sensor Data Interfaces**

#### **`/autonomy/gnss/fix`**
- **Type**: `sensor_msgs/NavSatFix`
- **Purpose**: GNSS/GPS position and satellite fix information
- **Publisher**: SLAM Node (republishes sensor data)
- **Subscriber**: Navigation Node, SLAM Node
- **Rate**: Variable (1-10 Hz typical)
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

#### **`/autonomy/imu/data`**
- **Type**: `sensor_msgs/Imu`
- **Purpose**: IMU inertial measurement data (acceleration, angular velocity, orientation)
- **Publisher**: SLAM Node (republishes sensor data)
- **Subscriber**: Navigation Node, SLAM Node
- **Rate**: 50-200 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

#### **`/autonomy/wheel/odom`**
- **Type**: `nav_msgs/Odometry`
- **Purpose**: Wheel encoder-based odometry
- **Publisher**: SLAM Node (republishes sensor data)
- **Subscriber**: SLAM Node, Navigation Node
- **Rate**: 10-50 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

#### **`/autonomy/camera/image_raw`**
- **Type**: `sensor_msgs/Image`
- **Purpose**: Raw camera RGB images
- **Publisher**: Computer Vision Node (republishes sensor data)
- **Subscriber**: Computer Vision Node
- **Rate**: 10-30 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

#### **`/autonomy/camera/depth/image_raw`**
- **Type**: `sensor_msgs/Image`
- **Purpose**: Raw camera depth images
- **Publisher**: Computer Vision Node (republishes sensor data)
- **Subscriber**: Computer Vision Node, Navigation Node
- **Rate**: 10-30 Hz
- **Reliability**: Best Effort
- **Namespace**: `/autonomy`

#### **`/autonomy/camera/camera_info`**
- **Type**: `sensor_msgs/CameraInfo`
- **Purpose**: Camera calibration and configuration information
- **Publisher**: Computer Vision Node (republishes sensor data)
- **Subscriber**: Computer Vision Node
- **Rate**: On change
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

#### **`/autonomy/mast_camera/command`**
- **Type**: `autonomy_interfaces/CameraCommand`
- **Purpose**: Pan-tilt-zoom commands for mast-mounted camera
- **Publisher**: Computer Vision Node, Navigation Node
- **Subscriber**: Camera Microcontroller
- **Rate**: On-demand
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

### **9. Health Monitoring & Performance**

#### **`/autonomy/autonomy_health`**
- **Type**: `diagnostic_msgs/DiagnosticArray`
- **Purpose**: Comprehensive health status of all autonomy subsystems
- **Publisher**: State Management Node
- **Subscriber**: Control Pi, Monitoring Systems
- **Rate**: 1 Hz (health check interval)
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

**Health Status Levels:**
- `OK` (0): Health >= 80%
- `WARN` (1): Health 50-79%
- `ERROR` (2): Health < 50%

**Subsystem Health Metrics:**
- Health score (0.0-1.0)
- Last update timestamp
- Active status
- Current state

#### **`/autonomy/performance_metrics`**
- **Type**: `std_msgs/String`
- **Purpose**: Real-time performance statistics and metrics
- **Publisher**: State Management Node
- **Subscriber**: Control Pi, Monitoring Systems
- **Rate**: 1 Hz (health check interval)
- **Reliability**: Reliable
- **Namespace**: `/autonomy`

**Performance Metrics Include:**
- System uptime
- Waypoints completed
- Navigation success rate
- Emergency stops count
- System resets count
- Overall system health

---

## 🔧 ROS2 Service Interfaces

### **State Management Services**

#### **`/autonomy/start_mission`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Start autonomous mission execution
- **Namespace**: `/autonomy`

**Request:**
```json
{}  // Empty trigger
```

**Response:**
```json
{
  "success": true,
  "message": "Mission started successfully"
}
```

#### **`/autonomy/stop_mission`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Stop current autonomous mission
- **Namespace**: `/autonomy`

#### **`/autonomy/configure_mission`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Configure mission parameters (currently simplified)
- **Namespace**: `/autonomy`

**Note:** Currently uses Trigger service. Will be upgraded to full ConfigureMission service.

#### **`/autonomy/switch_mode`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Switch between autonomous and teleoperation modes
- **Namespace**: `/autonomy`

#### **`/autonomy/reset_autonomy`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Reset autonomy system to initial state
- **Namespace**: `/autonomy`

**Request:**
```json
{}  // Empty trigger
```

**Response:**
```json
{
  "success": true,
  "message": "Autonomy system reset to initial state"
}
```

**Reset Operations:**
- Mission state → PRE_MISSION
- System mode → IDLE
- Clear waypoints and objectives
- Reset subsystem health scores
- Reset performance metrics
- Clear emergency state

### **Navigation Services**

#### **`/autonomy/navigation/navigate_to_waypoint`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Navigate to next waypoint in mission
- **Namespace**: `/autonomy/navigation`

#### **`/autonomy/navigation/stop`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Stop current navigation
- **Namespace**: `/autonomy/navigation`

#### **`/autonomy/navigation/get_current_waypoint`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Get information about current waypoint
- **Namespace**: `/autonomy/navigation`

### **Testing Services**

#### **`/autonomy/simulate_target_reached`**
- **Type**: `std_srvs/Trigger`
- **Purpose**: Simulate successful target arrival (for LED testing)
- **Namespace**: `/autonomy`

### **Subsystem Status Services**

#### **`/autonomy/get_subsystem_status`**
- **Type**: `autonomy_interfaces/srv/GetSubsystemStatus`
- **Purpose**: Query health status of autonomy components
- **Namespace**: `/autonomy`

**Request:**
```json
{
  "subsystem_name": "navigation"  // or "slam", "computer_vision", "autonomous_typing", "led_status", "all"
}
```

**Response:**
```json
{
  "success": true,
  "error_message": "",
  "subsystem_names": ["navigation", "slam"],
  "subsystem_states": ["navigating", "active"],
  "subsystem_health": [0.95, 0.88],
  "status_messages": ["Approaching waypoint", "SLAM active"]
}
```

---

## ⚡ ROS2 Action Interfaces

### **Navigation Actions**

#### **`/autonomy/navigate_to_pose`**
- **Type**: `nav2_msgs/action/NavigateToPose`
- **Purpose**: Navigate to specific pose with progress feedback
- **Namespace**: `/autonomy`

**Goal:**
```json
{
  "pose": {
    "header": {"frame_id": "map"},
    "pose": {
      "position": {"x": 15.0, "y": 12.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  }
}
```

**Feedback:**
```json
{
  "current_pose": {
    "header": {"frame_id": "map"},
    "pose": {
      "position": {"x": 12.5, "y": 10.0, "z": 0.0},
      "orientation": {"x": 0.0, "y": 0.0, "z": 0.0, "w": 1.0}
    }
  },
  "navigation_time": 8.5,
  "estimated_time_remaining": 4.2,
  "number_of_recoveries": 0,
  "distance_remaining": 3.8
}
```

**Result:**
```json
{
  "result": {
    "code": 4,  // SUCCEEDED
    "message": "Goal succeeded"
  }
}
```

**Note:** Currently uses standard Nav2 NavigateToPose action. Custom autonomy_interfaces/NavigateToPose action is defined but not yet implemented.

#### **`/autonomy/perform_typing`**
- **Type**: `autonomy_interfaces/action/PerformTyping`
- **Purpose**: Execute autonomous typing sequence
- **Namespace**: `/autonomy`
- **Status**: Defined but not yet implemented

---

## 📊 Performance Requirements

### **Timing Guarantees**
- **Control Loops**: <100ms latency for `/cmd_vel`
- **Status Updates**: <1 second for status topics
- **Service Calls**: <5 second response time
- **Action Goals**: <10 second initial response

### **Reliability Requirements**
- **Uptime**: >99.5% for all interfaces
- **Message Loss**: <0.1% for critical topics
- **Connection Recovery**: <5 seconds after network interruption

### **Data Quality**
- **GPS Accuracy**: <1m CEP with RTK corrections
- **Pose Estimates**: <10cm RMS error
- **Timing Sync**: <10ms between all components

---

## 🚨 Error Handling

### **Error Codes and Messages**
All interfaces must provide clear error information:

```json
{
  "success": false,
  "message": "GPS fix lost - switching to dead reckoning",
  "error_code": "GPS_LOST"
}
```

### **Recovery Protocols**
1. **Network Failure**: Continue with last valid commands for 30 seconds
2. **Component Failure**: Degrade gracefully, notify other components
3. **Invalid Data**: Log error, use default values, continue operation
4. **Timeout**: Cancel operation, return to safe state

### **Emergency Procedures**
- **Emergency Stop**: All components respond within 100ms
- **System Reset**: Coordinated restart procedure
- **Safe State**: Known safe configuration for all actuators

---

## 🧪 Testing Requirements

### **Interface Testing**
- [ ] Topic discovery: `ros2 topic list`
- [ ] Message publishing: `ros2 topic pub /topic_name MsgType "{}"`
- [ ] Message monitoring: `ros2 topic echo /topic_name`
- [ ] Service calls: `ros2 service call /service_name SrvType "{}"`
- [ ] Action execution: `ros2 action send_goal /action_name ActionType "{}"`

### **Integration Testing**
- [ ] End-to-end mission execution
- [ ] Emergency stop verification
- [ ] Failure mode testing
- [ ] Performance benchmarking

### **Compatibility Testing**
- [ ] ROS2 version compatibility
- [ ] Network configuration testing
- [ ] Multi-component startup verification

---

## 📞 Communication Protocols

### **Interface Changes**
1. **Proposal**: Team submits change request with justification
2. **Review**: All affected teams review impact
3. **Implementation**: 48-hour notice before deployment
4. **Documentation**: Update interface contract immediately
5. **Testing**: Validate with all affected components

### **Issue Reporting**
- **Critical Issues**: Immediate notification to all teams
- **Blockers**: Daily standup discussion
- **Enhancements**: GitHub issues with prioritization

### **Version Control**
- **Interface Versions**: Semantic versioning (MAJOR.MINOR.PATCH)
- **Breaking Changes**: Major version increment
- **Backwards Compatibility**: Maintained within major versions
- **Deprecation**: 2-week notice before removal

---

## 📋 Implementation Status

### **Autonomy Team Deliverables**
- [x] State management node with mission lifecycle coordination
- [x] Navigation node with waypoint following and obstacle avoidance
- [x] SLAM node with sensor fusion (GPS, IMU, wheel odometry)
- [x] Computer vision node (framework established)
- [x] Autonomous typing node (framework established)
- [x] LED status controller with competition-compliant signaling
- [x] Core ROS2 topics implemented with `/autonomy` namespace
- [x] Service interfaces for mission control and navigation
- [x] Emergency stop handling across all subsystems
- [x] Custom message types fully implemented (CameraCommand, ConfigureMission added)
- [x] Action servers for complex operations (NavigateToPose, PerformTyping implemented)
- [x] Sensor data interfaces (GNSS, IMU, camera topics) implemented
- [x] Standardized topic naming across all subsystems
- [x] Comprehensive health monitoring and error reporting implemented
- [x] Performance validation and metrics tracking implemented
- [x] ResetAutonomy service for system reset implemented

### **Control Team Deliverables**
- [ ] User interface for mission configuration (pending)
- [ ] System status display and monitoring (pending)
- [ ] Emergency controls and overrides (pending)
- [ ] Message routing and coordination (framework ready)
- [ ] Integration testing with autonomy interfaces (ongoing)

### **Hardware Team Deliverables**
- [ ] Microcontroller firmware implementing ROS2 interfaces (pending)
- [ ] Hardware control loops meeting timing requirements (pending)
- [ ] Sensor data publishing to ROS2 topics (pending)
- [ ] Safety interlocks and emergency stop handling (pending)
- [ ] Reliability testing under various conditions (pending)

---

## 🔄 Interface Contract Updates Required

### **Immediate Updates Needed**
1. **Add missing message type definitions:**
   - [x] `autonomy_interfaces/CameraCommand` for mast camera control
   - [x] Complete `autonomy_interfaces/PerformTyping` action definition (already existed)
   - [x] `autonomy_interfaces/ConfigureMission` service definition

2. **Standardize topic naming:**
   - [x] All topics now use consistent `/autonomy` namespace
   - [x] Launch file updated with standardized remappings

3. **Implement missing services:**
   - [x] Full `ConfigureMission` service with waypoint configuration
   - [x] `ResetAutonomy` service for system reset
   - [ ] Additional calibration services integration

4. **Complete action servers:**
   - [x] Implement custom `NavigateToPose` action (replaced Nav2 standard)
   - [x] Implement `PerformTyping` action server (already existed)

5. **Add sensor interfaces:**
   - [x] GNSS data topics (`/autonomy/gnss/fix`, `/autonomy/imu/data`)
   - [x] Camera data topics (`/autonomy/camera/image_raw`, etc.)
   - [x] Wheel odometry and camera info topics added

### **Future Enhancements**
- [x] **Health monitoring:** Comprehensive subsystem health checking implemented
- [x] **Performance validation:** Latency and throughput monitoring implemented
- **Error recovery:** Define automatic recovery protocols (pending)
- **Security:** Add authentication and authorization for critical commands (pending)
- **Additional calibration services integration** (pending)

---

## 🎯 Success Criteria

### **Functional Success**
- **Zero integration failures** in combined system testing
- **All interfaces operational** under normal and failure conditions
- **Performance requirements met** across all scenarios
- **Emergency procedures validated** with all teams

### **Operational Success**
- **Clear communication** between all teams
- **Rapid issue resolution** when problems arise
- **Regular integration testing** throughout development
- **Comprehensive documentation** of all interfaces

### **Competition Readiness**
- **System integration complete** 2 weeks before competition
- **All failure modes tested** and recovery procedures documented
- **Performance validated** under competition-like conditions
- **Team coordination procedures** established and practiced

---

## 📞 Contact Information

### **Interface Coordination**
- **Primary Contact**: Project Integration Lead
- **Backup Contact**: Team leads from each group
- **Documentation**: This interface contract document
- **Updates**: GitHub repository commits

### **Emergency Contacts**
- **Critical Issues**: Immediate notification to all team leads
- **Competition Day**: Designated communication channels
- **Remote Support**: VPN access and remote debugging procedures

---

*This interface contract ensures reliable communication between all robot components. All teams must comply with these specifications for successful system integration.*
