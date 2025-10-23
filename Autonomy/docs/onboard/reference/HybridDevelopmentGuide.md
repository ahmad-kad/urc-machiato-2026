# Hybrid Development Guide - Docker + Native Integration

## Overview
This guide explains how to combine Docker containerized development with native system access for subsystems that require direct hardware interaction.

## 🏗️ **Architecture Overview**

```
┌─────────────────────────────────────┐
│          Docker Services            │
├─────────────────────────────────────┤
│ • Computer Vision (95% suitable)    │
│ • SLAM Processing (85% suitable)    │
│ • State Management (100% suitable) │
│ • Navigation Algorithms (70% suit.) │
└─────────────────────────────────────┘
                   │
            ROS 2 DDS Bridge
                   │
┌─────────────────────────────────────┐
│         Native Components           │
├─────────────────────────────────────┤
│ • Motor Control (Navigation)        │
│ • LED GPIO Control                  │
│ • Robotic Arm Control (Typing)      │
│ • Real-time Hardware Interfaces     │
└─────────────────────────────────────┘
```

## 🔧 **Setting Up Hybrid Development**

### 1. Docker Services (Algorithm Development)
```bash
# Start Docker services for algorithm development
cd Autonomy/docker
./scripts/docker-dev.sh start

# Enter development container
./scripts/docker-dev.sh enter

# Develop algorithms in Docker
cd /workspace/code/navigation
python3 src/navigation_node.py  # Simulation testing
```

### 2. Native Components (Hardware Interface)
```bash
# On host system, set up ROS 2 workspace
source /opt/ros/humble/setup.bash  # If installed natively
export ROS_DOMAIN_ID=42

# Run hardware interface nodes natively
ros2 run navigation motor_controller_node
ros2 run led_status led_controller_node
ros2 run autonomous_typing arm_controller_node
```

### 3. ROS 2 DDS Communication
```bash
# Docker services and native nodes communicate via ROS 2 DDS
# Same ROS_DOMAIN_ID ensures proper discovery
# Network mode: host enables seamless communication

# Test communication
# In Docker container:
ros2 topic list  # Should see native node topics

# On native system:
ros2 node list   # Should see Docker service nodes
```

## 📋 **Subsystem-Specific Hybrid Setup**

### Navigation Subsystem (70% Docker, 30% Native)

**Docker Components:**
```bash
# In Docker container - Algorithm development
cd /workspace/code/navigation
ros2 run navigation navigation_node      # Path planning, terrain analysis
ros2 run navigation gnss_processor       # GPS data processing
ros2 run navigation path_planner         # A* and trajectory optimization
```

**Native Components:**
```bash
# On host - Hardware control
ros2 run navigation motor_controller     # Wheel motor PWM control
ros2 run navigation odometry_publisher   # Encoder data publishing
ros2 run navigation emergency_stop       # Hardware E-stop handling
```

**Integration:**
```python
# Navigation node in Docker subscribes to hardware topics from native system
odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
motor_cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)  # Commands native controller
```

### LED Status Subsystem (60% Docker, 40% Native)

**Docker Components:**
```bash
# In Docker container - Logic and coordination
ros2 run led_status led_logic_node       # State management and timing
ros2 run led_status status_coordinator   # Integration with state management
```

**Native Components:**
```bash
# On host - Hardware control
ros2 run led_status gpio_controller      # Direct GPIO LED control
ros2 run led_status pwm_generator        # Hardware PWM for flashing
```

### Autonomous Typing Subsystem (40% Docker, 60% Native)

**Docker Components:**
```bash
# In Docker container - Planning and simulation
ros2 run autonomous_typing sequence_planner   # Key sequence generation
ros2 run autonomous_typing trajectory_planner # Arm motion planning
ros2 run autonomous_typing error_detector     # Failure analysis
```

**Native Components:**
```bash
# On host - Hardware control
ros2 run autonomous_typing arm_controller     # Direct servo control
ros2 run autonomous_typing force_sensor       # Tactile feedback processing
ros2 run autonomous_typing keyboard_detector  # ArUco marker detection
```

## 🚀 **Quick Start Hybrid Development**

### Step 1: Start Docker Services
```bash
cd Autonomy/docker
./scripts/docker-dev.sh start
```

### Step 2: Launch Native ROS 2 Environment
```bash
# Terminal 1: Native ROS 2 core (if not using Docker ROS core)
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
ros2 daemon start

# Terminal 2: Launch native hardware interfaces
ros2 launch navigation hardware.launch.xml    # Motor controllers, sensors
ros2 launch led_status hardware.launch.xml     # GPIO interfaces
ros2 launch autonomous_typing hardware.launch.xml  # Arm interfaces
```

### Step 3: Launch Docker Algorithm Services
```bash
# In Docker container
./scripts/docker-dev.sh enter
cd /workspace

# Launch algorithm services
ros2 launch navigation algorithms.launch.xml      # Path planning, terrain analysis
ros2 launch computer_vision processing.launch.xml # Object detection, ArUco
ros2 launch slam processing.launch.xml            # Pose estimation, mapping
ros2 launch state_management coordination.launch.xml  # Mission control
```

### Step 4: Test Integration
```bash
# Check ROS 2 communication
ros2 topic list  # Should see topics from both Docker and native
ros2 node list   # Should see nodes from both environments

# Test specific integrations
ros2 topic echo /cmd_vel          # Navigation commands
ros2 topic echo /led_status       # LED control signals
ros2 topic echo /arm_command      # Typing commands
```

## 🔄 **ROS 2 DDS Bridge Configuration**

### Domain ID Configuration
```bash
# Both Docker and native must use same ROS_DOMAIN_ID
export ROS_DOMAIN_ID=42

# Docker environment variables set this automatically
# Native systems must set this manually
```

### Network Configuration
```yaml
# Docker docker-compose.yml network settings
networks:
  autonomy_network:
    driver: bridge
    ipam:
      config:
        - subnet: 172.20.0.0/16

# Native system: Use host networking for ROS 2 discovery
# Docker services use network_mode: host for direct communication
```

### Topic/Service Naming Conventions
```python
# Consistent naming across Docker/native boundary
NAVIGATION_CMD_VEL = '/navigation/cmd_vel'
LED_STATUS_COMMAND = '/led_status/command'
ARM_JOINT_COMMANDS = '/arm/joint_commands'
SENSOR_ODOMETRY = '/sensors/odometry'
VISION_DETECTIONS = '/vision/object_detections'
```

## 🧪 **Testing Hybrid Setup**

### Unit Testing (Individual Components)
```bash
# Test Docker components in isolation
cd /workspace/code/navigation
python3 -m pytest test/ -v

# Test native components in isolation
ros2 test navigation motor_controller.test
```

### Integration Testing (Docker + Native)
```bash
# Test communication between environments
ros2 topic pub /test_topic std_msgs/String "data: 'integration_test'"

# Monitor topic bridging
ros2 topic echo /cmd_vel          # Should receive from navigation algorithms
ros2 topic echo /odom             # Should receive from native sensors
```

### System Testing (Full Autonomy)
```bash
# Launch complete system
# 1. Start Docker services (algorithms)
# 2. Start native hardware interfaces
# 3. Launch integration launch files
# 4. Run mission scenarios

ros2 launch autonomy_system full_system.launch.xml
```

## 🐛 **Troubleshooting Hybrid Setup**

### Communication Issues
```bash
# Check ROS 2 discovery
ros2 node list  # Should show nodes from both environments

# Check network connectivity
ros2 multicast receive  # Test DDS multicast
ros2 multicast send     # Send test multicast

# Verify domain ID
echo $ROS_DOMAIN_ID  # Should be 42 for both
```

### Performance Issues
```bash
# Monitor system resources
htop  # Check CPU/memory usage
nvidia-smi  # Check GPU usage (if applicable)

# Check ROS 2 performance
ros2 topic hz /cmd_vel  # Check message frequency
ros2 topic delay /cmd_vel  # Check message latency
```

### Hardware Access Issues
```bash
# Check device permissions
ls -la /dev/ttyUSB*  # Check serial device access
ls -la /dev/i2c-*    # Check I2C device access

# Test GPIO access (if applicable)
gpio readall  # Check GPIO pin access
```

## 📊 **Performance Monitoring**

### Docker Performance
```bash
# Monitor container resources
docker stats autonomy_development

# Check ROS 2 performance in container
ros2 topic hz /camera/image_raw
```

### Native Performance
```bash
# Monitor system performance
top -p $(pgrep ros2)
rostopic hz /cmd_vel
```

### Integration Performance
```bash
# Monitor cross-environment communication
ros2 topic delay /cmd_vel
ros2 topic hz /odom
```

## 🔧 **Best Practices**

### Development Workflow
1. **Develop algorithms in Docker** (consistent environment, easy testing)
2. **Implement hardware interfaces natively** (direct access, real-time performance)
3. **Test integration incrementally** (start simple, add complexity)
4. **Monitor performance continuously** (identify bottlenecks early)

### Code Organization
- **Docker services**: Algorithm implementations, data processing
- **Native components**: Hardware drivers, real-time controllers
- **Shared interfaces**: ROS 2 messages, services, actions
- **Configuration**: YAML files for both environments

### Version Control
- **Single repository**: Both Docker and native code
- **Tagged releases**: Synchronized versions of Docker images and native code
- **Branch strategy**: Feature branches for both environments

## 🎯 **Migration Strategy**

### Phase 1: Algorithm Development (100% Docker)
- Computer vision model training
- SLAM algorithm optimization
- Path planning development
- State machine logic

### Phase 2: Hardware Integration (Hybrid)
- Add native hardware interfaces
- Establish ROS 2 communication bridges
- Test integrated subsystems
- Performance optimization

### Phase 3: Full System Integration (Hybrid)
- Complete autonomy system testing
- Mission scenario validation
- Performance benchmarking
- Competition preparation

This hybrid approach maximizes development productivity while ensuring real-time hardware performance for robotics applications.
