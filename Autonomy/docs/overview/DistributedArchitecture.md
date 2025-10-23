# Distributed Architecture: Autonomy Pi + External Control Pi

## Overview

The autonomy system is designed as one component in a distributed multi-Pi architecture:

```
┌─────────────────┐    ┌──────────────────┐    ┌─────────────────┐
│   Control Pi    │    │  Autonomy Pi     │    │  External Pi    │
│ (User Interface)│◄──►│ (Autonomy Logic) │◄──►│ (System Integration)
│                 │    │                  │    │                 │
│ • User Controls │    │ • Path Planning  │    │ • Message Routing│
│ • System Status │    │ • SLAM           │    │ • State Mgmt    │
│ • Mission Config│    │ • Computer Vision│    │ • Coordination  │
└─────────────────┘    └──────────────────┘    └─────────────────┘
       ▲                       ▲                       ▲
       │                       │                       │
       └───────── ROS2 Network ────────────────────────┘
```

---

## 🎯 Autonomy Pi Responsibilities (Your Focus)

### **Pure Autonomy Execution**
The autonomy Pi focuses **exclusively** on autonomy algorithms:

#### **Inputs (from External Systems)**
- **Mission Commands**: Start/stop autonomy, switch modes
- **Waypoint Goals**: Target locations for navigation
- **Environmental Data**: Sensor feeds, maps
- **System State**: Emergency stop, mode changes

#### **Outputs (to External Systems)**
- **Navigation Status**: Current position, progress, state
- **System Health**: Subsystem status, diagnostics
- **Autonomy Results**: Mission completion, detection results

#### **Internal Processing**
- **SLAM**: Localization and mapping
- **Path Planning**: Optimal route calculation
- **Computer Vision**: Object detection, ArUco tracking
- **Motion Control**: Velocity commands to actuators

### **Clean Interface Contract**
```yaml
# Autonomy Pi exposes these ROS2 interfaces:
topics:
  input:
    - mission_start          # std_srvs/Trigger
    - waypoint_goal          # geometry_msgs/PoseStamped
    - emergency_stop         # std_msgs/Bool
    - system_mode           # std_msgs/String

  output:
    - navigation_status      # autonomy_interfaces/NavigationStatus
    - autonomy_health       # diagnostic_msgs/DiagnosticArray
    - detection_results     # autonomy_interfaces/VisionDetection
    - slam_pose            # geometry_msgs/PoseWithCovarianceStamped

services:
  - configure_mission       # autonomy_interfaces/srv/ConfigureMission
  - get_autonomy_status    # autonomy_interfaces/srv/GetSubsystemStatus

actions:
  - navigate_to_pose       # autonomy_interfaces/action/NavigateToPose
  - perform_autonomous_task # autonomy_interfaces/action/PerformTyping
```

---

## 🔄 External Pi Responsibilities (Other Team)

### **System Integration & Coordination**
The external Pi handles higher-level system coordination:

#### **Multi-System Coordination**
- **State Management**: Overall system state (not just autonomy)
- **Mode Coordination**: Autonomous ↔ Teleop ↔ Manual transitions
- **Resource Allocation**: Power management, compute distribution
- **Failure Recovery**: System-level fault handling

#### **User Interface Bridge**
- **Control Input**: Joystick, keyboard, touchscreen
- **Status Display**: System status, autonomy progress
- **Mission Configuration**: Waypoint setup, parameter tuning
- **Emergency Controls**: Override, stop, reset functions

#### **Message Routing**
- **Protocol Translation**: Convert between different interfaces
- **Network Management**: Handle communication reliability
- **Data Fusion**: Combine inputs from multiple sources
- **Command Arbitration**: Resolve conflicting commands

---

## 🏗️ Interface Design for Distributed System

### **Autonomy Pi: Black Box Component**

The autonomy system becomes a **well-defined component** with clear boundaries:

#### **External Dependencies**
```xml
<!-- Autonomy Pi package.xml - minimal external dependencies -->
<depend>rclpy</depend>
<depend>autonomy_interfaces</depend>  <!-- Only its own interfaces -->
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<!-- No dependencies on external state management or controls -->
```

#### **Clean API Surface**
```python
class AutonomySystem:
    """
    Autonomy black box component.

    Inputs: Commands and sensor data
    Outputs: Status and results
    Internal: Pure autonomy algorithms
    """

    def __init__(self):
        # Only autonomy-specific initialization
        self.slam = SLAMProcessor()
        self.navigation = NavigationSystem()
        self.vision = ComputerVisionSystem()

    def execute_mission(self, mission_config):
        """Execute autonomy mission - pure function"""
        # Internal autonomy logic only
        pass

    def report_status(self):
        """Report autonomy status"""
        # Only autonomy health/metrics
        pass
```

### **Communication Patterns**

#### **Command Flow (External → Autonomy)**
```
Control Pi → External Pi → Autonomy Pi
    ↓           ↓           ↓
User Input → Coordination → Autonomy Execution
```

#### **Status Flow (Autonomy → External)**
```
Autonomy Pi → External Pi → Control Pi
    ↓           ↓           ↓
Results → Processing → User Display
```

---

## 🔧 Implementation Benefits

### **For Your Autonomy Team**

#### **✅ Focused Development**
- **No system integration concerns**: Focus purely on autonomy algorithms
- **Clean interfaces**: Well-defined inputs/outputs
- **Independent testing**: Test autonomy logic in isolation
- **Scalable architecture**: Easy to integrate with different external systems

#### **✅ Easier Maintenance**
- **Modular updates**: Update autonomy without affecting system integration
- **Clear ownership**: Autonomy team owns autonomy code, external team owns integration
- **Independent deployment**: Deploy autonomy updates without system downtime

### **For External Integration Team**

#### **✅ Flexible Integration**
- **Standard ROS2 interfaces**: Easy to connect to existing systems
- **Configurable parameters**: Tune autonomy behavior without code changes
- **Health monitoring**: Clear visibility into autonomy system status
- **Graceful degradation**: Handle autonomy system failures

---

## 📋 Integration Checklist

### **Autonomy Pi Deliverables**
- [ ] ROS2 package with clean interfaces
- [ ] Comprehensive parameter configuration
- [ ] Health monitoring and diagnostics
- [ ] Emergency stop handling
- [ ] Comprehensive documentation
- [ ] Unit and integration tests

### **Interface Contract**
- [ ] Defined topic/service/action interfaces
- [ ] Parameter specifications
- [ ] Error handling protocols
- [ ] Version compatibility guarantees

### **Integration Testing**
- [ ] Interface compatibility testing
- [ ] Message format validation
- [ ] Parameter range testing
- [ ] Failure mode testing

---

## 🎯 Key Design Principles

### **1. Interface Stability**
- **Version contracts**: Interfaces remain stable across updates
- **Backward compatibility**: New versions don't break existing integrations
- **Clear deprecation**: Planned changes communicated in advance

### **2. Error Boundaries**
- **Graceful degradation**: Autonomy system handles external failures
- **Clear error reporting**: External systems know autonomy status
- **Recovery protocols**: Defined procedures for system recovery

### **3. Performance Contracts**
- **Timing guarantees**: Response times for critical operations
- **Resource limits**: CPU/memory usage boundaries
- **Throughput requirements**: Message processing rates

---

## 🚀 Migration Strategy

### **Phase 1: Interface Definition**
1. Define clean ROS2 interfaces between systems
2. Document parameter specifications
3. Establish error handling protocols

### **Phase 2: Independent Development**
1. Autonomy team develops autonomy algorithms
2. External team develops integration/control systems
3. Parallel testing of individual components

### **Phase 3: Integration Testing**
1. Connect systems through defined interfaces
2. End-to-end testing of distributed system
3. Performance and reliability validation

### **Phase 4: Deployment**
1. Deploy autonomy Pi as independent component
2. Deploy external Pi with integration logic
3. Establish monitoring and maintenance procedures

---

## 📞 Communication Protocols

### **Status Reporting**
```python
# Autonomy Pi regularly reports status
status_msg = NavigationStatus()
status_msg.state = "navigating"
status_msg.mission_progress = 0.75
status_msg.distance_to_goal = 5.2
publisher.publish(status_msg)
```

### **Command Reception**
```python
# Autonomy Pi receives commands
def mission_callback(self, request, response):
    # Execute mission command
    success = self.execute_mission(request.mission_config)
    response.success = success
    return response
```

### **Health Monitoring**
```python
# Regular health reports
health_msg = DiagnosticArray()
health_msg.status.append(create_diagnostic("slam", "ok", "Tracking stable"))
health_msg.status.append(create_diagnostic("vision", "warning", "Low light"))
publisher.publish(health_msg)
```

---

## 🎉 Benefits of This Architecture

### **For Competition Success**
- **Parallel development**: Teams work simultaneously
- **Independent expertise**: Each team focuses on their domain
- **Scalable system**: Easy to add more Pis or redistribute functions
- **Robust integration**: Well-defined interfaces prevent integration issues

### **For Future Development**
- **Modular upgrades**: Update autonomy without affecting controls
- **Technology swap**: Replace Pi with Jetson/NVIDIA without changing interfaces
- **Multi-platform**: Same autonomy code works on different robot platforms

**This distributed architecture maximizes your team's effectiveness!** 🚀
