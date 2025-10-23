# Development Pipeline - University Rover Challenge 2026

## Overview
This document outlines the complete development pipeline for the autonomy system, from initial setup through deployment and testing. It covers tools, workflows, version control, testing frameworks, and integration procedures.

## 🛠️ Development Environment Setup

### Core Development Tools

#### 1. Version Control & Collaboration
```bash
# Initialize autonomy repository structure
cd ~/robotics2025
git init
git submodule add https://github.com/IntelRealSense/librealsense.git third_party/librealsense
git submodule add https://github.com/opencv/opencv.git third_party/opencv

# Branching strategy
git checkout -b develop
git checkout -b feature/navigation-basic
git checkout -b feature/slam-integration
```

#### 2. ROS 2 Humble Installation (Primary Framework)
```bash
# Ubuntu 22.04 ROS 2 Installation
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-desktop
source /opt/ros/humble/setup.bash

# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/ros2/examples.git
```

#### 3. Python Environment (Conda/Miniforge)
```bash
# Install Miniforge
wget https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh
bash Miniforge3-Linux-x86_64.sh

# Create autonomy environment
conda create -n autonomy python=3.10
conda activate autonomy

# Install core packages
pip install numpy scipy matplotlib
pip install opencv-contrib-python
pip install torch torchvision torchaudio
pip install ros-humble-cv-bridge ros-humble-image-transport
```

#### 4. IDE & Development Tools
```bash
# VS Code with ROS extensions
sudo snap install code --classic
code --install-extension ms-vscode.cpptools
code --install-extension ms-python.python
code --install-extension ms-iot.vscode-ros

# Additional development tools
sudo apt install cmake build-essential
pip install black flake8 mypy  # Code formatting/linting
```

## 🔄 Development Workflow

### Git Workflow
```
Main Branches:
├── main (production-ready code)
├── develop (integration branch)
└── feature/* (individual features)

Workflow:
1. Create feature branch from develop
2. Implement and test feature
3. Create pull request to develop
4. Code review and testing
5. Merge to develop
6. Release to main when stable
```

### Code Standards
```python
# Python formatting (black)
black --line-length 88 autonomy/

# Linting (flake8)
flake8 --max-line-length 88 autonomy/

# Type checking (mypy)
mypy --ignore-missing-imports autonomy/
```

## 🧪 Testing Pipeline

### 1. Unit Testing Framework
```bash
# Install testing tools
pip install pytest pytest-cov pytest-mock
pip install ros-testing

# ROS testing setup
sudo apt install ros-humble-launch-testing ros-humble-launch-testing-ament-cmake
```

### 2. Unit Test Structure
```
tests/
├── unit/
│   ├── test_navigation.py
│   ├── test_slam.py
│   ├── test_computer_vision.py
│   └── test_state_management.py
├── integration/
│   ├── test_nav_vision_integration.py
│   └── test_slam_nav_integration.py
└── system/
    ├── test_full_autonomy.py
    └── test_mission_scenarios.py
```

### 3. Gazebo Simulation Setup
```bash
# Install Gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Create simulation worlds
mkdir -p ~/ros2_ws/src/autonomy_simulation
cd ~/ros2_ws/src/autonomy_simulation

# Mars desert environment world
# Include terrain models, lighting conditions, dust effects
```

### 4. CI/CD Pipeline (GitHub Actions)
```yaml
# .github/workflows/ci.yml
name: CI
on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
    - uses: actions/checkout@v2
    - name: Setup ROS
      uses: ros-tooling/setup-ros@v0.2
      with:
        required-ros-distributions: humble
    - name: Build and test
      run: |
        source /opt/ros/humble/setup.bash
        colcon build
        colcon test
```

## 🔗 Integration Pipeline

### ROS 2 Node Architecture
```
autonomy_system/
├── navigation/           # Navigation stack
├── slam/                # SLAM stack
├── computer_vision/     # Vision stack
├── state_management/    # Coordination layer
├── hardware_interface/  # Hardware abstraction
└── common/             # Shared utilities
```

### Communication Patterns

#### 1. Publisher-Subscriber (Data Streaming)
```python
# Camera data publishing
class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        # Intel RealSense or Oak-D integration

# Navigation subscribing to camera
class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation')
        self.subscription = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
```

#### 2. Service-Client (Request-Response)
```python
# State management service
class StateManager(Node):
    def __init__(self):
        self.srv = self.create_service(
            SetMode, 'state_manager/set_mode', self.set_mode_callback)

# Navigation requesting mode change
class NavigationNode(Node):
    def __init__(self):
        self.cli = self.create_client(SetMode, 'state_manager/set_mode')

    def request_mode_change(self, mode):
        req = SetMode.Request()
        req.mode = mode
        self.cli.call_async(req)
```

#### 3. Action Server-Client (Long-Running Tasks)
```python
# Autonomous navigation action
class NavigationActionServer(Node):
    def __init__(self):
        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            self.execute_callback)
```

### Data Flow Architecture
```
Sensor Data → Preprocessing → Feature Extraction → Algorithm → Control Commands
    ↓            ↓              ↓                ↓            ↓
Hardware   →  ROS Topics   →  ROS Topics    →  ROS Services → Actuator Commands
Interfaces     (QoS)          (Filtered)        (Actions)       (Safety)
```

## 🏗️ Hardware Integration Pipeline

### 1. Hardware Abstraction Layer
```python
# Hardware interface base class
class HardwareInterface(ABC):
    @abstractmethod
    def initialize(self):
        pass

    @abstractmethod
    def read_sensors(self):
        pass

    @abstractmethod
    def write_actuators(self, commands):
        pass

# Specific implementations
class GpsInterface(HardwareInterface):
    def __init__(self, port='/dev/ttyUSB0'):
        self.serial_port = serial.Serial(port, 9600)

class CameraInterface(HardwareInterface):
    def __init__(self, device_id=0):
        self.cap = cv2.VideoCapture(device_id)
```

### 2. Device Configuration Management
```yaml
# hardware_config.yaml
hardware:
  gps:
    port: "/dev/ttyUSB0"
    baudrate: 9600
    protocol: "NMEA"

  camera_oak_d:
    resolution: [1920, 1080]
    fps: 30
    depth_mode: "HIGH_ACCURACY"

  imu:
    i2c_address: "0x68"
    sample_rate: 100
    calibration_file: "imu_calibration.json"
```

## 📊 Monitoring & Debugging Pipeline

### 1. Logging Configuration
```python
# ROS 2 logging setup
import rclpy.logging
logger = rclpy.logging.get_logger('autonomy')

# Structured logging
logger.info('Navigation started',
           extra={'target': target_pose,
                  'confidence': confidence_score,
                  'timestamp': time.time()})
```

### 2. Real-time Monitoring Tools
```bash
# ROS 2 monitoring
ros2 topic list
ros2 topic echo /camera/image_raw
ros2 node list
ros2 service list

# System monitoring
htop  # Process monitoring
nvidia-smi  # GPU monitoring (if applicable)
```

### 3. Performance Profiling
```python
# Code profiling
import cProfile
cProfile.run('main_function()', 'profile_output.prof')

# Memory profiling
from memory_profiler import profile

@profile
def vision_processing():
    # Vision pipeline code
    pass
```

## 🚀 Deployment Pipeline

### 1. Build System (Colcon)
```bash
# Build autonomy packages
cd ~/ros2_ws
colcon build --symlink-install --packages-select autonomy_*

# Test build
colcon test --packages-select autonomy_*

# Install build
colcon build --install-base /opt/ros/humble
```

### 2. Configuration Management
```bash
# Environment setup script
#!/bin/bash
export AUTONOMY_ROOT=/home/user/robotics2025
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
source ~/miniforge/envs/autonomy/bin/activate
```

### 3. Launch Files
```xml
<!-- autonomy.launch.xml -->
<launch>
    <node pkg="autonomy_state_management" exec="state_manager_node"/>
    <node pkg="autonomy_navigation" exec="navigation_node"/>
    <node pkg="autonomy_slam" exec="slam_node"/>
    <include file="camera.launch.xml"/>
</launch>
```

## 🔧 Maintenance & Update Pipeline

### 1. Dependency Management
```bash
# Update all dependencies
pip list --outdated
pip install --upgrade [package_name]

# ROS dependencies
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### 2. Firmware Updates
```bash
# Camera firmware updates
# Intel RealSense
rs-fw-update -l  # List available firmware
rs-fw-update -f firmware.bin  # Update firmware

# Oak-D updates through DepthAI
python -m depthai_sdk.firmware
```

### 3. Calibration Pipeline
```bash
# Automated calibration check
python scripts/check_calibration.py --all

# Recalibration trigger
python scripts/calibrate_camera.py --camera oak_d --output calibration/
```

## 📈 Performance Optimization Pipeline

### 1. Profiling & Analysis
```python
# Real-time performance monitoring
import time
start_time = time.perf_counter()
# Process frame
end_time = time.perf_counter()
processing_time = end_time - start_time
logger.info(f'Processing time: {processing_time:.3f}s')
```

### 2. Optimization Strategies
- **CPU Optimization**: Multi-threading, SIMD instructions
- **GPU Acceleration**: CUDA for computer vision, tensor operations
- **Memory Management**: Efficient data structures, garbage collection
- **Network Optimization**: QoS settings, compression, prioritization

### 3. Benchmarking Suite
```python
# Performance benchmarks
def benchmark_vision_pipeline():
    # Load test images
    # Measure processing time
    # Calculate FPS, latency, accuracy
    # Generate performance report
    pass
```

## 🎯 Getting Started Checklist

### Week 1: Environment Setup
- [ ] Install Ubuntu 22.04
- [ ] Set up ROS 2 Humble
- [ ] Create conda environment
- [ ] Install development tools
- [ ] Clone repository and set up workspace

### Week 2: Basic Integration
- [ ] Create ROS 2 packages for each subsystem
- [ ] Set up basic communication between nodes
- [ ] Implement hardware interfaces
- [ ] Create launch files

### Week 3: Core Functionality
- [ ] Implement basic versions of each subsystem
- [ ] Set up simulation environment
- [ ] Create unit tests
- [ ] Establish integration testing

### Week 4: Optimization & Testing
- [ ] Performance profiling and optimization
- [ ] Comprehensive testing suite
- [ ] Documentation updates
- [ ] Deployment preparation

This development pipeline provides a structured approach to building a robust autonomy system. Start with the environment setup and gradually build up complexity while maintaining good testing and integration practices.
