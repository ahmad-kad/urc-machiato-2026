# Simulation & Digital Twin Subsystem

This subsystem provides comprehensive simulation capabilities and digital twin functionality for the URC 2026 rover autonomy system. It enables testing, validation, and development of autonomy algorithms in both simulated and real-time synchronized environments.

## 📍 **Location & Architecture**

This code lives in the **development workspace** (`Autonomy/code/simulation/`) following the project's two-workspace architecture:

- **Edit here**: `Autonomy/code/simulation/` (your IDE workspace)
- **Build here**: `Autonomy/ros2_ws/` (ROS2 finds it via symlink)

## 🎯 **Components**

### **Core Simulation**
- **Sensor Simulator**: Mock GPS, IMU, and camera data for testing
- **Robot Models**: Enhanced URDF with ROS2 control interfaces
- **Mars Environment**: Custom worlds with competition course elements
- **Launch Files**: Easy startup configurations for different scenarios

### **Digital Twin System** ⚠️ **PLACEHOLDER VALUES**
- **Twin Manager**: Real-time synchronization between physical and virtual rover
- **Prediction Engine**: PLACEHOLDER motion and behavior prediction
- **Health Monitoring**: PLACEHOLDER system health assessment
- **Configuration**: PLACEHOLDER parameter management

## 🚀 **Quick Start**

### **Basic Simulation Testing**

```bash
# 1. Build in ROS2 workspace
cd Autonomy/ros2_ws
colcon build --packages-select autonomy_simulation

# 2. Launch basic simulation
ros2 launch autonomy_simulation basic_simulation.launch.py

# 3. Check sensor topics
ros2 topic list
ros2 topic echo /rover/gps/fix --once
```

### **Digital Twin Testing** ⚠️ **PLACEHOLDER**

```bash
# Launch with digital twin (PLACEHOLDER values)
ros2 launch autonomy_simulation full_simulation.launch.py

# Monitor twin status (PLACEHOLDER data)
ros2 topic echo /digital_twin/sync_status
```

## 📁 **Directory Structure**

```
simulation/
├── autonomy_simulation/          # Python package
│   ├── __init__.py              # Package initialization
│   └── sensor_simulator.py      # Sensor simulation node
├── digitaltwins/                 # Digital twin components ⚠️ PLACEHOLDER
│   ├── config/                  # Configuration files
│   ├── models/                  # Data models and utilities
│   ├── scripts/                 # Digital twin executables
│   └── launch/                  # Launch configurations
├── launch/                      # ROS2 launch files
├── urdf/                        # Robot description files
├── worlds/                      # Gazebo world files
├── rviz/                        # Visualization configurations
├── config/                      # Package configuration
├── scripts/                     # Additional scripts
├── models/                      # Simulation models
├── resource/                    # ROS2 package resources
├── test/                        # Unit tests
├── package.xml                  # ROS2 package metadata
├── CMakeLists.txt              # Build configuration
├── setup.py                     # Python package setup
├── setup.cfg                    # Build tool configuration
├── simulation_TODO.md           # Development roadmap
└── README.md                    # This file
```

## 🔧 **Development Workflow**

Following the project's two-workspace architecture:

### **1. Edit Code Here** (Development Workspace)
```bash
cd Autonomy/code/simulation
# Edit files in your IDE
code autonomy_simulation/sensor_simulator.py
```

### **2. Build & Test There** (ROS2 Workspace)
```bash
cd Autonomy/ros2_ws
colcon build --packages-select autonomy_simulation
source install/setup.bash
ros2 run autonomy_simulation sensor_simulator
```

## ⚠️ **PLACEHOLDER VALUES WARNING**

**The digital twin implementation currently uses PLACEHOLDER values and simplified logic.** Key areas requiring real implementation:

### **Critical PLACEHOLDER Items**
- **Physical Parameters**: Mass (50kg), inertia, dimensions
- **Sensor Models**: GPS accuracy (3m), IMU noise, camera specs
- **Actuator Limits**: Motor torque (50Nm), wheel speeds
- **Environmental**: Mars gravity (3.711 m/s²), soil properties
- **Algorithms**: Synchronization, prediction, health monitoring

### **Development Status**
- ✅ **Basic Structure**: ROS2 package with proper architecture
- ✅ **Sensor Simulation**: Functional mock sensors for testing
- ✅ **Launch Integration**: Works with autonomy system
- 🔄 **Digital Twin**: PLACEHOLDER implementation needs calibration
- 🔄 **Physics Models**: Basic kinematics, needs full dynamics
- 🔄 **Validation**: Requires real hardware testing

## 🎯 **Usage Scenarios**

### **Algorithm Development**
- Test navigation algorithms with realistic sensor data
- Validate computer vision pipelines with simulated cameras
- Develop SLAM algorithms with controlled environments

### **Integration Testing**
- Test subsystem interactions in simulation
- Validate ROS2 communication patterns
- Debug multi-node autonomy systems

### **Digital Twin Applications** (Future)
- Real-time synchronization with physical rover
- Predictive maintenance and failure detection
- Performance optimization suggestions
- Virtual testing of autonomy algorithms

## 🔗 **Integration Points**

### **Autonomy Subsystems**
- **Navigation**: GPS/IMU data for localization
- **SLAM**: Odometry and sensor data for mapping
- **Computer Vision**: Camera feeds for object detection
- **State Management**: System health and status monitoring

### **External Systems**
- **Competition Infrastructure**: GNSS targets, AR tags
- **Development Tools**: RViz visualization, testing frameworks
- **Hardware**: Real rover for digital twin synchronization

## 📊 **Topics Published**

```
/rover/gps/fix              # GPS position data
/rover/imu/data             # IMU acceleration/orientation
/rover/camera/image_raw     # Camera feed
/rover/camera/camera_info   # Camera intrinsics
/odom                       # Robot odometry
/digital_twin/state         # Digital twin state (PLACEHOLDER)
/digital_twin/prediction    # Motion predictions (PLACEHOLDER)
/digital_twin/sync_status   # Synchronization status (PLACEHOLDER)
/digital_twin/health        # System health (PLACEHOLDER)
```

## 🚨 **Important Notes**

1. **Two-Workspace System**: Edit in `code/`, build in `ros2_ws/`
2. **PLACEHOLDER Values**: Digital twin needs real calibration
3. **Testing First**: Always test in simulation before hardware
4. **Documentation**: Keep TODO.md updated with progress

## 📚 **Resources**

- **TODO.md**: Detailed development roadmap and tasks
- **digitaltwins/README.md**: PLACEHOLDER documentation and warnings
- **DevelopmentWorkflow.md**: Project development practices
- **SystemArchitecture.md**: Overall system design

---

## 🎯 **Next Steps**

1. **Review TODO.md** for immediate development tasks
2. **Calibrate PLACEHOLDER values** with real hardware measurements
3. **Implement physics models** for accurate simulation
4. **Validate against real rover** data and performance
5. **Integrate with autonomy subsystems** for end-to-end testing

**Remember**: This subsystem provides the foundation for robust autonomy development. Start with sensor simulation, then progress to full digital twin capabilities as hardware calibration data becomes available.
