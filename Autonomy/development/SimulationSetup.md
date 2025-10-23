# Simulation Environment Setup - University Rover Challenge 2026

## Overview
This guide covers setting up a comprehensive simulation environment using Gazebo for testing and development of the autonomy system. The simulation environment includes Mars-like terrain, realistic physics, sensor simulation, and competition scenario testing.

## Prerequisites

### System Requirements
- **OS**: Ubuntu 22.04 LTS (recommended)
- **RAM**: 16GB minimum, 32GB recommended
- **GPU**: NVIDIA GPU with CUDA support (for GPU acceleration)
- **Storage**: 50GB free space for simulation worlds and data

### Software Prerequisites
```bash
# Update system
sudo apt update && sudo apt upgrade

# Install NVIDIA drivers (if applicable)
ubuntu-drivers autoinstall

# Verify GPU setup
nvidia-smi
```

## 1. ROS 2 Humble + Gazebo Installation

### Install ROS 2 Humble
```bash
# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### Install Gazebo
```bash
# Install Gazebo Fortress (compatible with ROS 2 Humble)
sudo apt install wget
sudo wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | sudo tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
sudo apt update
sudo apt install gazebo
sudo apt install ros-humble-gazebo-ros-pkgs

# Install development tools
sudo apt install ros-humble-gazebo-ros2-control ros-humble-xacro
```

### Create ROS 2 Workspace
```bash
# Create workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Clone essential packages
git clone https://github.com/ros-planning/navigation2.git -b humble
git clone https://github.com/ros-perception/vision_opencv.git -b humble
git clone https://github.com/ros-perception/image_common.git -b humble
git clone https://github.com/ros-simulation/gazebo_ros2_control.git -b humble

# Build workspace
cd ~/ros2_ws
colcon build --symlink-install
echo "source ~/ros2_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

## 2. Mars Desert Simulation Environment

### Create Mars World Package
```bash
# Create package for Mars simulation
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake mars_simulation --dependencies gazebo_ros_pkgs

# Package structure
cd mars_simulation
mkdir -p worlds models launch config
```

### Mars Terrain Generation
```xml
<!-- worlds/mars_desert.world -->
<?xml version="1.0"?>
<sdf version="1.7">
  <world name="mars_desert">
    <!-- Mars-like atmosphere -->
    <atmosphere type="mars"/>

    <!-- Lighting conditions -->
    <light name="sun" type="directional">
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.8 0.8 0.8 1</diffuse>
      <specular>0.2 0.2 0.2 1</specular>
      <direction>-0.5 0.5 -1</direction>
    </light>

    <!-- Mars terrain -->
    <model name="mars_ground">
      <static>true</static>
      <link name="link">
        <collision name="collision">
          <geometry>
            <heightmap>
              <uri>model://mars_ground/materials/textures/mars_heightmap.png</uri>
              <size>1000 1000 50</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
        </collision>
        <visual name="visual">
          <geometry>
            <heightmap>
              <uri>model://mars_ground/materials/textures/mars_heightmap.png</uri>
              <size>1000 1000 50</size>
              <pos>0 0 0</pos>
            </heightmap>
          </geometry>
          <material>
            <script>
              <uri>model://mars_ground/materials/scripts/mars.material</uri>
              <name>Mars/Mars</name>
            </script>
          </material>
        </visual>
      </link>
    </model>

    <!-- Dust simulation -->
    <plugin name="dust" filename="libDustEffect.so">
      <dust_density>0.01</dust_density>
      <wind_velocity>2.0</wind_velocity>
    </plugin>
  </world>
</sdf>
```

### Competition Course Models
```xml
<!-- models/competition_course/model.sdf -->
<?xml version="1.0"?>
<sdf version="1.7">
  <model name="competition_course">
    <!-- GNSS-only target markers (invisible in simulation) -->
    <model name="gnss_target_1">
      <pose>100 50 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <sphere>
              <radius>0.1</radius>
            </sphere>
          </geometry>
          <material>
            <diffuse>1 0 0 0.3</diffuse>
          </material>
        </visual>
      </link>
    </model>

    <!-- AR tag posts -->
    <model name="ar_post_1">
      <pose>200 -100 0 0 0 0</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <box>
              <size>0.2 0.2 1.5</size>
            </box>
          </geometry>
        </visual>
        <!-- AR tag texture would be applied here -->
      </link>
    </model>

    <!-- Ground objects -->
    <model name="orange_mallet">
      <pose>50 150 0 0 0 0.3</pose>
      <static>true</static>
      <link name="link">
        <visual name="visual">
          <geometry>
            <cylinder>
              <radius>0.03</radius>
              <length>0.8</length>
            </cylinder>
          </geometry>
          <material>
            <diffuse>1 0.5 0 1</diffuse> <!-- Orange -->
          </material>
        </visual>
      </link>
    </model>

    <!-- Add similar models for hammer and water bottle -->
  </model>
</sdf>
```

## 3. Rover Model Setup

### Create Rover URDF/XACRO
```xml
<!-- urdf/rover.urdf.xacro -->
<?xml version="1.0"?>
<robot xmlns:xacro="http://wiki.ros.org/xacro" name="rover">

  <!-- Properties -->
  <xacro:property name="wheel_radius" value="0.15"/>
  <xacro:property name="wheel_width" value="0.10"/>
  <xacro:property name="track" value="0.50"/>
  <xacro:property name="wheelbase" value="0.60"/>

  <!-- Base Link -->
  <link name="base_link">
    <visual>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.8 0.6 0.2"/>
      </geometry>
      <material name="gray"/>
    </visual>
    <collision>
      <origin xyz="0 0 0.1" rpy="0 0 0"/>
      <geometry>
        <box size="0.8 0.6 0.2"/>
      </geometry>
    </collision>
    <inertial>
      <mass value="50.0"/>
      <inertia ixx="1.0" ixy="0.0" ixz="0.0" iyy="1.0" iyz="0.0" izz="1.0"/>
    </inertial>
  </link>

  <!-- Wheels -->
  <xacro:macro name="wheel" params="prefix x y z yaw">
    <link name="${prefix}_wheel">
      <visual>
        <origin xyz="0 0 0" rpy="1.5708 0 ${yaw}"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
        <material name="black"/>
      </visual>
      <collision>
        <origin xyz="0 0 0" rpy="1.5708 0 ${yaw}"/>
        <geometry>
          <cylinder radius="${wheel_radius}" length="${wheel_width}"/>
        </geometry>
      </collision>
      <inertial>
        <mass value="2.0"/>
        <inertia ixx="0.01" ixy="0.0" ixz="0.0" iyy="0.01" iyz="0.0" izz="0.01"/>
      </inertial>
    </link>

    <joint name="${prefix}_wheel_joint" type="continuous">
      <parent link="base_link"/>
      <child link="${prefix}_wheel"/>
      <origin xyz="${x} ${y} ${z}" rpy="0 0 0"/>
      <axis xyz="0 1 0"/>
      <dynamics damping="0.1" friction="1.0"/>
    </joint>
  </xacro:macro>

  <!-- Instantiate wheels -->
  <xacro:wheel prefix="front_left" x="0.3" y="0.25" z="0" yaw="0"/>
  <xacro:wheel prefix="front_right" x="0.3" y="-0.25" z="0" yaw="0"/>
  <xacro:wheel prefix="rear_left" x="-0.3" y="0.25" z="0" yaw="0"/>
  <xacro:wheel prefix="rear_right" x="-0.3" y="-0.25" z="0" yaw="0"/>

  <!-- Sensors -->
  <link name="imu_link">
    <visual>
      <geometry>
        <box size="0.05 0.05 0.05"/>
      </geometry>
    </visual>
  </link>

  <joint name="imu_joint" type="fixed">
    <parent link="base_link"/>
    <child link="imu_link"/>
    <origin xyz="0 0 0.2" rpy="0 0 0"/>
  </joint>

  <!-- Add camera mounts, GPS, etc. -->
</robot>
```

### Sensor Integration
```xml
<!-- Add to rover.urdf.xacro -->
<!-- GPS Sensor -->
<gazebo>
  <plugin name="gps_plugin" filename="libgazebo_ros_gps_sensor.so">
    <ros>
      <namespace>/rover</namespace>
      <remapping>~/gps:=gps/fix</remapping>
    </ros>
    <gps>
      <position_sensing>
        <horizontal>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>3.0</stddev>
          </noise>
        </horizontal>
        <vertical>
          <noise type="gaussian">
            <mean>0.0</mean>
            <stddev>6.0</stddev>
          </noise>
        </vertical>
      </position_sensing>
    </gps>
  </plugin>
</gazebo>

<!-- IMU Sensor -->
<gazebo>
  <plugin name="imu_plugin" filename="libgazebo_ros_imu_sensor.so">
    <ros>
      <namespace>/rover</namespace>
      <remapping>~/imu:=imu/data</remapping>
    </ros>
  </plugin>
</gazebo>

<!-- Camera -->
<gazebo>
  <plugin name="camera_plugin" filename="libgazebo_ros_camera.so">
    <ros>
      <namespace>/rover</namespace>
      <remapping>~/image_raw:=camera/image_raw</remapping>
    </ros>
  </plugin>
</gazebo>
```

## 4. Launch Files and Configuration

### Simulation Launch File
```xml
<!-- launch/simulation.launch.xml -->
<launch>
  <!-- Gazebo world -->
  <include file="$(find-pkg-share gazebo_ros)/launch/gzserver.launch.py">
    <arg name="world" value="$(find-pkg-share mars_simulation)/worlds/mars_desert.world"/>
  </include>

  <include file="$(find-pkg-share gazebo_ros)/launch/gzclient.launch.py">
  </include>

  <!-- Spawn rover -->
  <node pkg="gazebo_ros" exec="spawn_entity.py"
        args="-topic /rover_description -entity rover -x 0 -y 0 -z 0.5">
  </node>

  <!-- Robot state publisher -->
  <node pkg="robot_state_publisher" exec="robot_state_publisher"
        args="robot_description">
    <param name="robot_description" value="$(command 'xacro $(find-pkg-share rover_description)/urdf/rover.urdf.xacro')"/>
  </node>

  <!-- Joint state publisher -->
  <node pkg="joint_state_publisher_gui" exec="joint_state_publisher_gui">
  </node>
</launch>
```

### Autonomy System Launch
```xml
<!-- launch/autonomy.launch.xml -->
<launch>
  <!-- Simulation first -->
  <include file="simulation.launch.xml"/>

  <!-- SLAM -->
  <node pkg="slam_toolbox" exec="async_slam_toolbox_node">
    <param name="slam_params_file" value="$(find-pkg-share rover_navigation)/config/mapper_params_online_async.yaml"/>
  </node>

  <!-- Navigation -->
  <include file="$(find-pkg-share nav2_bringup)/launch/navigation_launch.py">
    <arg name="params_file" value="$(find-pkg-share rover_navigation)/config/nav2_params.yaml"/>
  </include>

  <!-- Computer Vision -->
  <node pkg="rover_vision" exec="object_detector_node">
    <param name="model_path" value="$(find-pkg-share rover_vision)/models/yolov5.pt"/>
  </node>

  <!-- State Management -->
  <node pkg="rover_state" exec="state_manager_node">
  </node>

  <!-- RViz for visualization -->
  <node pkg="rviz2" exec="rviz2"
        args="-d $(find-pkg-share rover_description)/rviz/navigation.rviz">
  </node>
</launch>
```

## 5. Testing and Validation

### Basic Functionality Tests
```bash
# Launch simulation
ros2 launch mars_simulation simulation.launch.xml

# Check topics
ros2 topic list

# Test sensor data
ros2 topic echo /rover/imu/data
ros2 topic echo /rover/gps/fix

# Test teleoperation
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### Autonomy Integration Tests
```bash
# Launch full autonomy system
ros2 launch mars_simulation autonomy.launch.xml

# Test navigation to waypoint
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "
pose:
  header:
    frame_id: 'map'
  pose:
    position:
      x: 10.0
      y: 5.0
      z: 0.0
    orientation:
      x: 0.0
      y: 0.0
      z: 0.0
      w: 1.0"
```

### Performance Benchmarking
```python
# performance_test.py
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import time

class PerformanceMonitor(Node):
    def __init__(self):
        super().__init__('performance_monitor')
        self.subscription = self.create_subscription(
            Image, '/rover/camera/image_raw', self.image_callback, 10)
        self.frame_count = 0
        self.start_time = time.time()

    def image_callback(self, msg):
        self.frame_count += 1
        elapsed = time.time() - self.start_time
        fps = self.frame_count / elapsed
        self.get_logger().info(f'FPS: {fps:.2f}')

def main():
    rclpy.init()
    node = PerformanceMonitor()
    rclpy.spin(node)
    rclpy.shutdown()
```

## 6. Advanced Simulation Features

### Terrain Generation
```python
# generate_terrain.py
import numpy as np
from PIL import Image

def generate_mars_terrain(size=1024, octaves=6):
    """Generate Mars-like terrain heightmap"""
    heightmap = np.zeros((size, size))

    # Add multiple octaves of noise
    for octave in range(octaves):
        scale = 2 ** octave
        amplitude = 1.0 / scale

        # Generate noise
        noise = np.random.rand(size // scale + 1, size // scale + 1)
        # Interpolate to full size
        # Add to heightmap

    # Normalize to 0-255 range
    heightmap = ((heightmap - heightmap.min()) /
                (heightmap.max() - heightmap.min()) * 255).astype(np.uint8)

    # Save as PNG
    img = Image.fromarray(heightmap, mode='L')
    img.save('mars_heightmap.png')

if __name__ == "__main__":
    generate_mars_terrain()
```

### Sensor Noise Modeling
```xml
<!-- Add realistic noise to sensors -->
<gazebo>
  <plugin name="camera_noise" filename="libgazebo_ros_camera.so">
    <noise>
      <type>gaussian</type>
      <mean>0.0</mean>
      <stddev>0.01</stddev>
    </noise>
  </plugin>
</gazebo>
```

### Weather Simulation
```xml
<!-- Add wind and dust effects -->
<plugin name="wind" filename="libWindPlugin.so">
  <wind_velocity>5.0</wind_velocity>
  <wind_direction>1.57</wind_direction> <!-- 90 degrees -->
</plugin>
```

## 7. CI/CD Integration

### Automated Testing
```yaml
# .github/workflows/simulation_tests.yml
name: Simulation Tests
on: [push, pull_request]

jobs:
  simulation:
    runs-on: ubuntu-22.04
    steps:
    - uses: actions/checkout@v2
    - name: Setup ROS
      uses: ros-tooling/setup-ros@v0.2
      with:
        required-ros-distributions: humble
    - name: Install Gazebo
      run: |
        sudo apt install gazebo
    - name: Build and test
      run: |
        source /opt/ros/humble/setup.bash
        colcon build
        # Run simulation tests
        ros2 launch mars_simulation test_simulation.launch.xml
```

## 8. Troubleshooting

### Common Issues

#### Gazebo won't start:
```bash
# Check GPU drivers
glxinfo | grep "OpenGL version"

# Run with software rendering
export LIBGL_ALWAYS_SOFTWARE=1
gazebo
```

#### ROS 2 communication issues:
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Verify node discovery
ros2 node list

# Check topic connectivity
ros2 topic info /rover/cmd_vel
```

#### Performance issues:
```bash
# Monitor system resources
htop

# Check GPU usage
nvidia-smi

# Reduce simulation complexity
# Lower physics update rate
# Reduce sensor update frequencies
```

## 9. Best Practices

### Simulation Fidelity
- **Match real-world conditions**: Calibrate simulation parameters to match actual hardware
- **Include environmental factors**: Wind, dust, lighting variations
- **Realistic sensor models**: Include noise, latency, and failure modes

### Development Workflow
- **Iterative testing**: Test in simulation before hardware integration
- **Version control**: Keep simulation worlds and models in git
- **Documentation**: Document simulation parameters and validation results

### Performance Optimization
- **Reduce visual complexity**: Use low-poly models for distant objects
- **Optimize physics**: Use simplified collision models where appropriate
- **Parallel execution**: Run simulation and processing on different cores

This simulation setup provides a realistic testing environment for developing and validating the autonomy system before field deployment.
