# 🎯 Autonomy Goals & Success Metrics

## Overview
This guide defines what success looks like for the URC 2026 autonomy system. It provides clear, measurable goals for each subsystem and the overall system, ensuring everyone knows what "good enough" means.

## 🏆 Mission Success Criteria

### **Primary Competition Goals**
The URC 2026 autonomy system must successfully complete **two main missions**:

#### **Mission 1.f: Autonomous Navigation (30 minutes)**
Navigate to **7 targets** across a **2km terrain** within **30 minutes**:
- **2 GNSS-only locations** (basic GPS waypoints)
- **2 posts with 3-sided visual markers** (20×20cm faces, 2.5cm cells, ArUco 4x4_50)
- **3 ground objects** (mallet hammer + rock pick hammer + water bottle detection)

**Success = All 7 targets reached within time limit**

#### **Mission 1.e: Equipment Servicing (Variable time)**
Interact with keyboard to input **3-6 letter codes**:
- **Detect keyboard** (ArUco markers + template matching)
- **Navigate arm** to keyboard position
- **Type characters** with precision manipulation
- **Verify input** and handle errors

**Success = All codes entered correctly**

## 📊 Success Metrics by Subsystem

### **🎯 Navigation Subsystem**

#### **Functional Goals**
- [ ] **Waypoint Navigation**: Reach GPS waypoints within 3m accuracy
- [ ] **AR Tag Precision**: Reach AprilTag targets within 0.5m accuracy
- [ ] **Obstacle Avoidance**: Detect and avoid obstacles >0.3m high
- [ ] **Terrain Adaptation**: Handle 30° slope variations
- [ ] **Speed Control**: Maintain optimal speed (0.5-1.5 m/s)

#### **Performance Metrics**
- **Accuracy**: <1m RMS position error in open terrain
- **Reliability**: >95% successful waypoint reaching
- **Safety**: Zero collisions in testing
- **Efficiency**: Complete 2km course in <25 minutes

#### **Technical Specifications**
- **Update Rate**: 10Hz control loop
- **Planning Time**: <100ms for path recalculation
- **Memory Usage**: <50MB RAM
- **CPU Usage**: <30% on Raspberry Pi 4

---

### **🗺️ SLAM Subsystem**

#### **Functional Goals**
- [ ] **Pose Estimation**: Track robot position within 0.5m accuracy
- [ ] **Map Building**: Create 2D occupancy grid of environment
- [ ] **Loop Closure**: Detect when robot returns to known areas
- [ ] **Sensor Fusion**: Combine LIDAR, IMU, and wheel odometry
- [ ] **Real-time Updates**: Update map at 5Hz minimum

#### **Performance Metrics**
- **Accuracy**: <0.3m drift over 30-minute mission
- **Completeness**: >90% of navigable terrain mapped
- **Consistency**: No map contradictions or jumps
- **Speed**: Process 8000 LIDAR points per second

#### **Technical Specifications**
- **Map Resolution**: 5cm grid cells
- **Update Rate**: 5Hz mapping, 20Hz pose estimation
- **Memory Usage**: <100MB for map storage
- **CPU Usage**: <40% on Raspberry Pi 4

---

### **👁️ Computer Vision Subsystem**

#### **Functional Goals**
- [ ] **Object Detection**: Find mallet hammer and water bottle
- [ ] **ArUco Tracking**: Detect and localize AprilTags
- [ ] **Depth Sensing**: Measure distances to objects
- [ ] **Image Processing**: Handle varying lighting conditions
- [ ] **Real-time Processing**: Process 1080p images at 10Hz

#### **Performance Metrics**
- **Detection Rate**: >90% true positive rate for targets
- **False Positive Rate**: <5% false detections
- **Localization Accuracy**: <0.1m distance, <5° angle error
- **Processing Speed**: <100ms per frame

#### **Technical Specifications**
- **Resolution**: 1920×1080 RGB, 1280×720 depth
- **Field of View**: 69° horizontal, 42° vertical
- **Range**: 0.2-10m depth sensing
- **Lighting**: Operate in 100-10000 lux conditions

---

### **⌨️ Autonomous Typing Subsystem**

#### **Functional Goals**
- [ ] **Keyboard Detection**: Locate keyboard using markers + template
- [ ] **Pose Estimation**: Calculate arm position relative to keyboard
- [ ] **Trajectory Planning**: Plan safe arm movements
- [ ] **Character Input**: Press correct keys with precision
- [ ] **Error Recovery**: Handle missed keys and repositioning

#### **Performance Metrics**
- **Detection Accuracy**: Locate keyboard within 1cm, 1°
- [ ] **Typing Accuracy**: >95% correct key presses
- [ ] **Speed**: Complete 6-character code in <2 minutes
- [ ] **Safety**: No collisions with keyboard or surroundings

#### **Technical Specifications**
- **Precision**: ±1mm positioning accuracy
- [ ] **Force Control**: 1-10N adjustable force
- [ ] **DOF**: 6-axis arm movement
- [ ] **Feedback**: Force/torque sensing at end effector

---

### **🎛️ State Management Subsystem**

#### **Functional Goals**
- [ ] **Mode Switching**: Seamless teleop ↔ autonomous transitions
- [ ] **Health Monitoring**: Track all subsystem statuses
- [ ] **Mission Planning**: Execute mission sequences
- [ ] **Error Handling**: Respond to subsystem failures
- [ ] **Data Logging**: Record all mission events

#### **Performance Metrics**
- [ ] **Switching Time**: <1 second mode transitions
- [ ] **Uptime**: >99% system availability
- [ ] **Response Time**: <100ms to critical events
- [ ] **Logging**: 100% mission events captured

#### **Technical Specifications**
- [ ] **State Tracking**: 10+ concurrent subsystem states
- [ ] **Event Rate**: Handle 100+ events per second
- [ ] **Storage**: 1GB mission data capacity
- [ ] **Monitoring**: Real-time dashboard updates

---

### **🔴 LED Status Subsystem**

#### **Functional Goals**
- [ ] **Status Indication**: Show current system state
- [ ] **Mode Signaling**: Autonomous (red) vs Teleop (blue)
- [ ] **Success Indication**: Flashing green for mission success
- [ ] **Error Signaling**: Different patterns for error types
- [ ] **Visibility**: Judge-visible from 50m distance

#### **Performance Metrics**
- [ ] **Visibility**: Clear from 50m in daylight
- [ ] **Response Time**: <100ms state change response
- [ ] **Reliability**: 100% uptime, no LED failures
- [ ] **Power Efficiency**: <1W average power consumption

#### **Technical Specifications**
- [ ] **Brightness**: 1000+ candela output
- [ ] **Colors**: Red, Blue, Green, White
- [ ] **Patterns**: Steady, flashing, pulsing
- [ ] **Control**: PWM brightness control

---

## ⚠️ Competition Compliance Validation

### **Mission 1.e: Equipment Servicing Success Criteria**
- **Autonomous Typing**: Successfully input 3-6 letter launch codes
- **Error Handling**: Use backspace/delete for corrections when needed
- **ArUco Detection**: Locate 2x2cm markers at keyboard corners, 1x1cm at USB slot
- **USB Reading**: Extract GNSS coordinates from memory card
- **Operator Limits**: No intervention except for mode exit/abort/restart

### **Mission 1.f: Autonomous Navigation Success Criteria**
- **LED Compliance**: 🔴 Red during autonomous, 🔵 Blue during teleoperation, 🟢 Flashing Green on success
- **Waypoint Accuracy**: 3m for GNSS-only, 2m for posts, 10m for objects
- **Target Completion**: Visit all 7 targets (flexible order)
- **Object Detection**: Identify mallet, rock pick hammer, water bottle
- **C2 Display**: Highlight detected objects (one at a time)
- **Abort Recovery**: Return to previous location with teleoperation penalty

### **General Competition Requirements**
- **No Antenna Camera**: All cameras must comply with mounting restrictions
- **Mission Time**: Complete within specified time limits
- **Safety**: All autonomous operations must be safe and recoverable

---

## 🎖️ System-Level Success Criteria

### **Integration Goals**
- [ ] **Communication**: All subsystems communicate via ROS 2
- [ ] **Synchronization**: <10ms timing synchronization between nodes
- [ ] **Data Flow**: End-to-end data pipeline from sensors to actuators
- [ ] **Fault Tolerance**: Continue operation with 1 subsystem failure

### **Performance Goals**
- [ ] **Total Power**: <50W system power consumption
- [ ] **Weight**: <5kg additional autonomy hardware
- [ ] **Reliability**: >95% mission success rate in testing
- [ ] **Setup Time**: <10 minutes deployment time

### **Competition Readiness**
- [ ] **Environmental Testing**: Operates in 0-45°C, dusty conditions
- [ ] **Electromagnetic**: Immune to competition radio interference
- [ ] **Safety**: Fails safely without harming robot or environment
- [ ] **Documentation**: Complete setup and troubleshooting guides

---

## 📈 Development Milestones

### **Month 1: Foundation (Weeks 1-4)**
- [ ] Basic ROS 2 communication between 2 subsystems
- [ ] Individual sensor data collection
- [ ] Simple navigation (straight line following)
- [ ] Basic computer vision object detection

### **Month 2: Integration (Weeks 5-8)**
- [ ] Multi-subsystem communication (4+ subsystems)
- [ ] Sensor fusion for localization
- [ ] Path planning with obstacle avoidance
- [ ] Autonomous mode switching

### **Month 3: Optimization (Weeks 9-12)**
- [ ] Performance optimization (<80% CPU usage)
- [ ] Fault tolerance and error recovery
- [ ] Mission scenario testing
- [ ] Hardware integration completion

### **Month 4: Competition Prep (Weeks 13-16)**
- [ ] Full mission testing in desert conditions
- [ ] Competition day procedures
- [ ] Backup systems validation
- [ ] Documentation finalization

---

## 🏁 Definition of Done (DoD)

### **Subsystem Ready**
- [ ] All functional goals implemented
- [ ] Performance metrics met in testing
- [ ] Unit tests written and passing
- [ ] Integration tests with other subsystems
- [ ] Documentation complete
- [ ] Code reviewed and approved

### **System Ready**
- [ ] All subsystems integrated and communicating
- [ ] End-to-end mission testing successful
- [ ] Performance requirements met
- [ ] Safety and reliability validated
- [ ] Competition procedures documented

### **Team Ready**
- [ ] All team members can operate the system
- [ ] Backup procedures tested
- [ ] Troubleshooting guides available
- [ ] Competition timeline rehearsed

---

## 📊 Measurement & Tracking

### **Daily Standups**
- What did you accomplish yesterday?
- What will you work on today?
- Any blockers or help needed?
- Status of your subsystem goals

### **Weekly Reviews**
- Progress against subsystem goals
- Integration testing results
- Performance metrics review
- Risk assessment and mitigation

### **Milestone Reviews**
- Demonstration of completed features
- Performance validation
- Code quality assessment
- Integration testing results

### **Tools for Tracking**
```bash
# Check individual progress
./scripts/check_code.sh                    # Code quality status
cat code/[subsystem]/[subsystem]_TODO.md       # Task completion
./scripts/navigate.sh api                  # Communication interfaces

# Check system integration
ros2 node list                     # Active subsystems
ros2 topic list                    # Available topics
ros2 topic hz /odom               # Performance metrics
ros2 topic echo /state/system_state # System status
```

---

## 🎯 Success Mindset

### **Focus on Outcomes, Not Features**
- **Competition Success**: Can we complete the missions?
- **Reliability**: Does it work consistently?
- **Performance**: Does it meet timing requirements?
- **Safety**: Does it fail gracefully?

### **Iterative Improvement**
- **Start Simple**: Get basic functionality working first
- **Add Complexity**: Enhance features incrementally
- **Test Frequently**: Validate at each step
- **Learn Fast**: Use failures as learning opportunities

### **Team Collaboration**
- **Shared Goals**: Everyone understands success criteria
- **Regular Communication**: Daily standups, weekly reviews
- **Help Each Other**: Knowledge sharing across subsystems
- **Celebrate Wins**: Recognize progress and achievements

---

## 🚀 Motivation & Mindset

### **Why We're Here**
- **Innovation**: Push boundaries of robot autonomy
- **Competition**: Win URC 2026 through technical excellence
- **Learning**: Develop skills in cutting-edge robotics
- **Impact**: Advance autonomous systems technology

### **Our Advantage**
- **Clear Goals**: Everyone knows what success looks like
- **Structured Approach**: Proven development methodology
- **Comprehensive Tools**: Full development environment
- **Team Expertise**: Combined skills across multiple domains

### **Remember**
**"Success is not final, failure is not fatal: It is the courage to continue that counts."** - Winston Churchill

**Stay focused on the mission, work together, and let's win URC 2026!** 🏆🤖
