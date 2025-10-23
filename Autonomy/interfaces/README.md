# ROS2 Interface Design Guide - Clean API Contracts

## 🎯 Interface Design Philosophy

ROS2 interfaces are APIs - define contracts, not implementations. Good interfaces are:

- **Self-documenting**: Clear naming, obvious purpose
- **Minimal**: Only essential data, no bloat
- **Stable**: Rarely change once defined
- **Composable**: Work well with other interfaces
- **Testable**: Easy to mock and validate

---

## 📋 Interface Design Principles

### 1. **Topic Interfaces (Publish/Subscribe)**

#### ✅ Good: Clear Purpose, Minimal Data
```python
# autonomy_interfaces/msg/NavigationState.msg
# Clear what this message represents
std_msgs/Header header
string state           # "idle", "planning", "navigating", "arrived"
float32 progress       # 0.0 to 1.0
string status_message  # Human-readable status
```

#### ❌ Bad: Overloaded, Unclear Purpose
```python
# Bad: Too much data, unclear responsibility
std_msgs/Header header
string state
float32 progress
string status_message
geometry_msgs/Pose current_pose      # Belongs in separate topic
sensor_msgs/NavSatFix gps_data       # Belongs in separate topic
float32[] sensor_readings           # Too generic
```

### 2. **Service Interfaces (Request/Response)**

#### ✅ Good: Specific Operation with Clear Contract
```python
# autonomy_interfaces/srv/NavigateToWaypoint.srv
# Request: What to do
int32 waypoint_id
geometry_msgs/PoseStamped target_pose
float32 approach_tolerance  # meters

---
# Response: Result of operation
bool success
string message
float32 estimated_time     # seconds to completion
```

#### ❌ Bad: Too Generic, Unclear Contract
```python
# Bad: Generic "do something" service
# Request
string command
string[] parameters
---
# Response
bool success
string result
```

### 3. **Action Interfaces (Long-Running Operations)**

#### ✅ Good: Goal, Feedback, Result Contract
```python
# autonomy_interfaces/action/AutonomousTyping.action
# Goal: What to accomplish
string target_text
geometry_msgs/PoseStamped keyboard_location
float32 timeout

---
# Feedback: Progress updates
float32 progress        # 0.0 to 1.0
string current_char     # Currently typing
geometry_msgs/Pose current_hand_pose

---
# Result: Final outcome
bool success
string completed_text
int32 characters_typed
float32 total_time
```

---

## 🔧 Interface Design Patterns for Your Project

### **State Management Hub Pattern**
```python
# Topics for state distribution
/mission_status         # MissionState: "pre_mission", "autonomous", "emergency"
/system_mode           # SystemMode: "idle", "autonomous", "teleop"
/emergency_stop        # std_msgs/Bool: Emergency stop broadcast

# Services for state control
/start_mission         # std_srvs/Trigger: Begin autonomous operation
/stop_mission          # std_srvs/Trigger: End current mission
/switch_mode           # autonomy_interfaces/srv/SwitchMode
```

### **Sensor Data Flow Pattern**
```python
# Raw sensor data (standard ROS messages)
/camera/image_raw          # sensor_msgs/Image
/camera/depth/image_raw    # sensor_msgs/Image
/gnss/fix                  # sensor_msgs/NavSatFix
/imu/data                  # sensor_msgs/Imu

# Processed sensor data (your interfaces)
/vision/detections         # vision_interfaces/DetectionArray
/slam/pose                # geometry_msgs/PoseStamped
/navigation/status        # autonomy_interfaces/NavigationStatus
```

### **Command Flow Pattern**
```python
# High-level commands
/navigation/goal          # geometry_msgs/PoseStamped: Where to go
/typing/target           # autonomy_interfaces/TypingGoal: What to type

# Low-level commands
/cmd_vel                 # geometry_msgs/Twist: Motor velocities
/led/command            # autonomy_interfaces/LedCommand: Status lights
```

---

## 📝 Interface Definition Best Practices

### **1. Message Naming Conventions**
```python
# Use PascalCase for message names
NavigationStatus.msg     # ✓ Good
navigation_status.msg    # ❌ Bad

# Use descriptive field names
float32 waypoint_tolerance_meters  # ✓ Good
float32 wpt_tol                   # ❌ Bad
```

### **2. Field Ordering**
```python
# Put most important fields first
std_msgs/Header header     # Always first
string state              # Primary state
float32 progress          # Key metrics
string status_message     # Details last
```

### **3. Units and Ranges**
```python
# Always document units and valid ranges
float32 progress    # 0.0 to 1.0, where 1.0 = 100% complete
float32 confidence  # 0.0 to 1.0, probability estimate
float32 distance_m  # meters, positive values only
```

### **4. Error Handling**
```python
# Include error information in responses
bool success
string error_message  # Empty if success
int32 error_code      # 0 = success, negative = error codes
```

---

## 🛠️ Creating Interfaces for Your Project

### **Step 1: Define Interface Package**
```bash
cd Autonomy/ros2_ws/src
ros2 pkg create autonomy_interfaces --build-type ament_python
```

### **Step 2: Create Message Definitions**
```python
# autonomy_interfaces/msg/NavigationStatus.msg
std_msgs/Header header
string state
float32 progress
string status_message

# autonomy_interfaces/msg/Detection.msg
std_msgs/Header header
string class_name
float32 confidence
geometry_msgs/Pose pose
geometry_msgs/Vector3 size
```

### **Step 3: Create Service Definitions**
```python
# autonomy_interfaces/srv/SwitchMode.srv
string mode  # "autonomous", "teleoperation", "manual"
---
bool success
string message
```

### **Step 4: Create Action Definitions**
```python
# autonomy_interfaces/action/NavigateToPose.action
geometry_msgs/PoseStamped pose
float32 tolerance
---
float32 distance_to_goal
---
bool success
string message
geometry_msgs/PoseStamped final_pose
```

### **Step 5: Update package.xml**
```xml
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>sensor_msgs</depend>
<!-- Your custom interfaces -->
<member_of_group>rosidl_interface_packages</member_of_group>
```

---

## 🧪 Interface Testing Patterns

### **Topic Testing**
```python
def test_navigation_status_topic():
    # Publish navigation status
    publisher = node.create_publisher(NavigationStatus, 'navigation/status', 10)

    msg = NavigationStatus()
    msg.state = "navigating"
    msg.progress = 0.75
    msg.status_message = "Approaching waypoint"

    publisher.publish(msg)

    # Verify subscriber receives correct data
    assert received_msg.state == "navigating"
    assert received_msg.progress == 0.75
```

### **Service Testing**
```python
def test_switch_mode_service():
    # Call service
    request = SwitchMode.Request()
    request.mode = "autonomous"

    response = switch_mode_client.call(request)

    # Verify response
    assert response.success == True
    assert "autonomous" in response.message
```

---

## 🚨 Common Interface Design Mistakes

### **1. Too Much Data in One Message**
```python
# ❌ Bad: Everything in one message
message RobotState {
  # Navigation
  geometry_msgs/Pose pose
  string nav_state
  # Vision
  Detection[] detections
  # SLAM
  nav_msgs/OccupancyGrid map
  # Too much!
}
```

### **2. Unclear Semantics**
```python
# ❌ Bad: What does "ready" mean?
bool ready  # Ready for what?

# ✅ Good: Clear semantics
bool navigation_ready
bool vision_ready
bool slam_ready
```

### **3. No Error Information**
```python
# ❌ Bad: Silent failures
bool success

# ✅ Good: Informative errors
bool success
string error_message
int32 error_code
```

### **4. Breaking Changes**
```python
# ❌ Bad: Adding required fields breaks existing code
message DetectionV2 {  # Breaking change!
  string class_name      # Was optional, now required
  float32 confidence     # New required field
}

# ✅ Good: Use new message type
message DetectionV2 {
  DetectionV1 base       # Optional base message
  float32 confidence     # New optional field
}
```

---

## 🎯 Your Project's Interface Architecture

Based on your system analysis, here's the recommended interface structure:

```
📁 autonomy_interfaces/
├── 📁 msg/
│   ├── NavigationStatus.msg
│   ├── VisionDetection.msg
│   ├── SlamPose.msg
│   ├── TypingStatus.msg
│   └── LedCommand.msg
├── 📁 srv/
│   ├── StartMission.srv
│   ├── StopMission.srv
│   └── SwitchMode.srv
└── 📁 action/
    ├── NavigateToPose.action
    └── PerformTyping.action
```

This creates clean API contracts that each subsystem can implement independently while maintaining clear communication patterns.

**Remember**: Good interfaces evolve slowly and break rarely. Invest time in getting them right early! 🎯
