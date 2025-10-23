# 🔧 **Autonomy Setup Systems Guide: URC 2026 Rover**

## Overview

Beyond camera calibration, several critical setup systems must be properly configured for reliable autonomy operation. This guide covers all foundational setup procedures required for the URC 2026 rover's autonomy stack.

---

## 🎯 **Critical Setup Systems Matrix**

| Setup System | Priority | Timeline | Dependencies | Impact if Skipped |
|-------------|----------|----------|--------------|-------------------|
| **Sensor Calibration** | CRITICAL | Pre-deployment | Hardware assembly | Position errors, navigation failure |
| **Coordinate Frames (TF)** | CRITICAL | Week 1 | URDF model | Incorrect transformations, collision detection failure |
| **ROS2 Network Config** | HIGH | Week 1 | Hardware setup | Communication failures, system instability |
| **Motion Control Tuning** | HIGH | Week 2 | Sensor calibration | Poor path following, oscillations |
| **Time Synchronization** | MEDIUM | Week 2 | Network config | Sensor fusion errors, timing issues |
| **Health Monitoring** | MEDIUM | Week 3 | All sensors | Silent failures, undetected issues |
| **Environment Config** | LOW | Pre-competition | Mission planning | Boundary violations, unsafe operation |

---

## 📡 **1. Sensor Calibration Suite**

### **IMU Calibration (Inertial Measurement Unit)**
**Why Critical**: IMU provides primary orientation and acceleration data for navigation and SLAM.

#### **Accelerometer Calibration**
```python
# imu_tools calibration process
ros2 run imu_tools imu_calibration

# Collect data in all orientations
# Generate calibration file
```

#### **Gyroscope Bias Calibration**
```bash
# Static calibration (robot perfectly still)
ros2 run imu_filter_madgwick imu_filter_node \
  --ros-args -p use_mag:=false -p use_magnetic_field_msg:=false \
  --params-file imu_params.yaml
```

#### **Magnetometer Calibration (if used)**
```python
# Collect data in figure-8 pattern
# Calculate hard/soft iron offsets
# Apply correction matrix
```

### **GPS/GNSS Configuration**
**Why Critical**: GPS provides absolute positioning reference for long-distance navigation.

#### **RTK Setup (Real-Time Kinematic)**
```yaml
# gps_config.yaml
gps:
  device: "/dev/ttyACM0"
  baudrate: 115200
  rtk:
    enabled: true
    correction_source: "ntrip"  # or "base_station"
    mount_point: "RTCM3_MAX"
```

#### **Datum and Coordinate System**
```bash
# Set WGS84 datum
# Configure UTM zone for competition location
# Verify coordinate transformations
```

### **LiDAR Calibration**
**Why Critical**: LiDAR provides high-precision distance measurements for obstacle avoidance and mapping.

#### **Intrinsic Calibration**
```bash
# Velodyne calibration
ros2 run velodyne_pointcloud calibration_node

# Generate lookup tables for angle corrections
```

#### **Extrinsic Calibration (LiDAR ↔ IMU)**
```python
# Use calibration target or known landmarks
# Estimate rotation and translation between sensors
# Verify with ICP (Iterative Closest Point)
```

### **Wheel Encoder Calibration**
**Why Critical**: Wheel encoders provide odometry for dead-reckoning when GPS is unavailable.

#### **Wheel Diameter Calibration**
```python
# Measure actual wheel diameter
# Count encoder ticks per revolution
# Calculate meters per tick
wheel_diameter_m = measured_diameter
encoder_ticks_per_rev = 2048
meters_per_tick = (wheel_diameter_m * pi) / encoder_ticks_per_rev
```

#### **Wheel Base Calibration**
```python
# Measure distance between wheel centers
# Account for differential drive kinematics
wheel_base_m = measured_distance
```

---

## 🔗 **2. Coordinate Frame Setup (TF Tree)**

### **URDF Model Validation**
**Why Critical**: URDF defines the robot's physical structure and joint relationships.

#### **URDF Structure**
```xml
<!-- rover.urdf.xacro -->
<xacro:include filename="$(find rover_description)/urdf/sensors.xacro"/>
<xacro:include filename="$(find rover_description)/urdf/joints.xacro"/>

<robot name="rover">
  <!-- Base link -->
  <link name="base_link">
    <visual>...</visual>
    <collision>...</collision>
  </link>

  <!-- Sensor links -->
  <link name="imu_link"/>
  <link name="gps_link"/>
  <link name="camera_link"/>
  <link name="lidar_link"/>
</robot>
```

#### **Joint Definitions**
```xml
<!-- Static transforms between links -->
<joint name="imu_joint" type="fixed">
  <parent link="base_link"/>
  <child link="imu_link"/>
  <origin xyz="0.1 0 0.2" rpy="0 0 0"/>
</joint>
```

### **TF Tree Publishing**
```python
class TFPublisher(Node):
    def __init__(self):
        super().__init__('tf_publisher')

        # Static transforms
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        # Publish base_link → sensor transforms
        self.publish_static_transforms()

        # Dynamic transforms (odometry)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.odom_timer = self.create_timer(0.1, self.publish_odometry)

    def publish_static_transforms(self):
        # Camera transform
        camera_transform = TransformStamped()
        camera_transform.header.stamp = self.get_clock().now().to_msg()
        camera_transform.header.frame_id = 'base_link'
        camera_transform.child_frame_id = 'camera_link'
        # Set translation and rotation...
        self.tf_static_broadcaster.sendTransform(camera_transform)
```

### **Extrinsic Sensor Calibration**
**Camera ↔ LiDAR Calibration:**
```python
# Use calibration target visible to both sensors
# Estimate transform using PnP + ICP
# Verify with known checkerboard positions
```

---

## 🌐 **3. ROS2 Network Configuration**

### **DDS Configuration**
**Why Critical**: ROS2 uses DDS for real-time communication between distributed components.

#### **Domain ID Setup**
```bash
# Set unique domain ID for competition
export ROS_DOMAIN_ID=42

# Configure DDS settings
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
export CYCLONEDDS_URI=file://$(pwd)/cyclonedds_config.xml
```

#### **Quality of Service (QoS) Tuning**
```yaml
# ros2_params.yaml
/navigation/waypoint_follower:
  ros__parameters:
    qos_override:
      reliability: reliable
      durability: volatile
      history: keep_last
      depth: 10
```

### **Parameter Server Configuration**
```python
# Load system-wide parameters
self.declare_parameters(
    namespace='',
    parameters=[
        ('robot_name', 'rover_2026'),
        ('max_linear_velocity', 2.0),
        ('max_angular_velocity', 1.5),
        ('waypoint_tolerance', 1.0),
        ('obstacle_clearance', 0.5),
    ]
)
```

### **Topic/Service Remapping**
```bash
# Launch file remapping for testing
ros2 launch rover_bringup bringup.launch.py \
  camera_topic:=/test_camera/image_raw \
  imu_topic:=/test_imu/data
```

---

## 🎛️ **4. Motion Control Tuning**

### **PID Controller Tuning**
**Why Critical**: PID controllers govern all motion - poor tuning causes oscillations, slow response, or instability.

#### **Velocity Control PID**
```python
class VelocityController:
    def __init__(self):
        # Linear velocity PID
        self.linear_pid = PIDController(
            kp=1.0,    # Proportional gain
            ki=0.1,    # Integral gain
            kd=0.05    # Derivative gain
        )

        # Angular velocity PID
        self.angular_pid = PIDController(
            kp=2.0,    # Higher gain for rotation
            ki=0.2,
            kd=0.1
        )
```

#### **Tuning Methodology**
```
1. Start with Ki=Kd=0, gradually increase Kp until oscillation
2. Add Kd to dampen oscillations
3. Add Ki for steady-state error correction
4. Test under various load conditions
```

### **Feedforward Control**
```python
# Add feedforward terms for better performance
def compute_motor_command(self, target_velocity, current_velocity):
    # PID output
    pid_output = self.pid.compute(target_velocity - current_velocity)

    # Feedforward (accounts for known dynamics)
    feedforward = self.kinetic_friction + (self.viscous_friction * target_velocity)

    return pid_output + feedforward
```

### **Anti-windup Protection**
```python
# Prevent integral windup during saturation
def update_pid(self, error, dt):
    # Proportional term
    p_term = self.kp * error

    # Integral term with windup protection
    if abs(self.integral + error * dt) < self.integral_limit:
        self.integral += error * dt
    i_term = self.ki * self.integral

    # Derivative term
    d_term = self.kd * (error - self.prev_error) / dt
    self.prev_error = error

    return p_term + i_term + d_term
```

---

## ⏰ **5. Time Synchronization**

### **NTP Configuration**
**Why Critical**: All sensors and actuators must operate on the same time base for proper sensor fusion.

#### **NTP Server Setup**
```bash
# Install NTP
sudo apt-get install ntp

# Configure NTP server
# /etc/ntp.conf
server time.nist.gov iburst
server time.google.com iburst
```

#### **ROS2 Time Synchronization**
```python
# Use ROS2 time for all timestamps
msg.header.stamp = self.get_clock().now().to_msg()

# Synchronize with system clock
ros2 run time_sync time_sync_node
```

### **Sensor Time Offset Calibration**
```python
# Measure time offset between sensors
# GPS typically has accurate time
# IMU/camera may have offsets

# Calculate and store offsets
imu_time_offset = calibrate_time_offset(imu_data, gps_data)
camera_time_offset = calibrate_time_offset(camera_data, gps_data)
```

---

## 💊 **6. Health Monitoring Setup**

### **Diagnostic Aggregator Configuration**
**Why Critical**: Proactive detection of system issues prevents silent failures during competition.

#### **Diagnostic Tasks**
```yaml
# diagnostics_config.yaml
base_path: '/diagnostics'
analyzers:
  sensors:
    type: 'diagnostic_aggregator/AnalyzerGroup'
    path: 'Sensors'
    analyzers:
      imu:
        type: 'diagnostic_aggregator/GenericAnalyzer'
        path: 'IMU'
        timeout: 5.0
      gps:
        type: 'diagnostic_aggregator/GenericAnalyzer'
        path: 'GPS'
        timeout: 1.0
```

### **Health Thresholds**
```python
# Define acceptable operating ranges
HEALTH_THRESHOLDS = {
    'imu_temperature': {'min': 10, 'max': 60, 'unit': '°C'},
    'gps_satellites': {'min': 4, 'max': 20, 'unit': 'count'},
    'battery_voltage': {'min': 10.0, 'max': 12.6, 'unit': 'V'},
    'motor_current': {'min': 0, 'max': 5.0, 'unit': 'A'},
    'cpu_usage': {'min': 0, 'max': 80, 'unit': '%'},
}
```

### **Automated Health Checks**
```python
class HealthMonitor(Node):
    def __init__(self):
        super().__init__('health_monitor')

        # Subscribe to diagnostic messages
        self.diagnostic_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostic_callback, 10)

        # Emergency stop publisher
        self.emergency_pub = self.create_publisher(
            Bool, '/emergency_stop', 10)

        self.health_timer = self.create_timer(1.0, self.check_system_health)

    def check_system_health(self):
        # Check all critical systems
        critical_failures = []

        if self.imu_status != 'OK':
            critical_failures.append('IMU failure')
        if self.gps_satellites < 4:
            critical_failures.append('GPS signal weak')
        if self.battery_voltage < 10.5:
            critical_failures.append('Low battery')

        if critical_failures:
            self.emergency_pub.publish(Bool(data=True))
            self.get_logger().error(f'Emergency stop triggered: {critical_failures}')
```

---

## 🗺️ **7. Environment Configuration**

### **Map Setup**
**Why Critical**: Maps define drivable areas, obstacles, and mission boundaries.

#### **Occupancy Grid Generation**
```python
# Create 2D occupancy grid map
map_config = {
    'resolution': 0.05,  # 5cm per cell
    'width': 100,        # 5m width
    'height': 100,       # 5m height
    'origin_x': -2.5,    # Center the map
    'origin_y': -2.5,
}

# Load or generate map
occupancy_grid = load_occupancy_grid('competition_area.yaml')
```

#### **Waypoint Configuration**
```yaml
# mission_waypoints.yaml
waypoints:
  - name: "start_position"
    latitude: 37.7749
    longitude: -122.4194
    tolerance: 1.0
  - name: "checkpoint_1"
    latitude: 37.7849
    longitude: -122.4094
    tolerance: 2.0
  - name: "ar_tag_1"
    latitude: 37.7949
    longitude: -122.3994
    tolerance: 2.0
```

### **Safety Zones**
```python
# Define no-go zones
safety_zones = [
    {'type': 'polygon', 'points': [(0,0), (1,0), (1,1), (0,1)]},  # Keep-out area
    {'type': 'circle', 'center': (5,5), 'radius': 2.0},            # Danger zone
]

# Check if planned path intersects safety zones
def validate_path_safety(path, safety_zones):
    for zone in safety_zones:
        if path_intersects_zone(path, zone):
            return False
    return True
```

---

## 📊 **8. Performance Validation Setup**

### **Benchmarking Configuration**
**Why Critical**: Validates that the system meets competition requirements before deployment.

#### **Performance Metrics**
```python
PERFORMANCE_REQUIREMENTS = {
    'navigation': {
        'waypoint_accuracy': 1.0,  # meters
        'path_following_error': 0.5,  # meters
        'maximum_speed': 2.0,  # m/s
    },
    'computer_vision': {
        'aruco_detection_rate': 0.95,  # 95% success rate
        'pose_estimation_accuracy': 0.05,  # meters
        'processing_fps': 10,  # minimum frame rate
    },
    'slam': {
        'position_drift': 1.0,  # meters per minute
        'loop_closure_success': 0.90,  # 90% detection rate
    }
}
```

### **Logging Configuration**
```yaml
# logging_config.yaml
loggers:
  autonomy:
    level: INFO
    handlers: [file, console]
    format: '%(asctime)s - %(name)s - %(levelname)s - %(message)s'

  diagnostics:
    level: DEBUG
    handlers: [file]
    format: '%(timestamp)s,%(level)s,%(component)s,%(message)s'

# Log rotation
handlers:
  file:
    class: logging.handlers.RotatingFileHandler
    filename: 'logs/autonomy.log'
    maxBytes: 10485760  # 10MB
    backupCount: 5
```

### **Automated Testing Framework**
```python
class AutonomyTestSuite:
    def __init__(self):
        self.test_results = []

    def run_full_system_test(self):
        """Run complete autonomy system validation"""
        tests = [
            self.test_sensor_calibration,
            self.test_tf_tree,
            self.test_motion_control,
            self.test_navigation_accuracy,
            self.test_emergency_stop,
        ]

        for test in tests:
            result = test()
            self.test_results.append(result)

            if not result['passed']:
                self.logger.error(f"Test failed: {result['name']}")
                return False

        return True

    def test_sensor_calibration(self):
        """Verify all sensors are properly calibrated"""
        # Check calibration files exist
        # Validate calibration quality metrics
        # Test sensor data consistency
        pass
```

---

## 📅 **Setup Timeline & Dependencies**

### **Week 1: Foundation Setup**
```
Day 1: ROS2 network configuration
Day 2: URDF model and TF tree setup
Day 3: Sensor calibration (camera, IMU, GPS)
Day 4: Basic motion control tuning
Day 5: Time synchronization setup
```

### **Week 2: Integration Setup**
```
Day 1-2: Extrinsic calibration between sensors
Day 3-4: Health monitoring configuration
Day 5: Environment configuration (maps, waypoints)
```

### **Week 3: Validation & Optimization**
```
Day 1-2: Performance benchmarking
Day 3-4: Automated testing framework setup
Day 5: Final system validation
```

---

## 🔧 **Troubleshooting Setup Issues**

### **Common Setup Problems**

#### **TF Lookup Failures**
```
Symptoms: "TF lookup failed" errors
Solutions:
1. Verify URDF is loaded correctly
2. Check static transform publishers are running
3. Validate transform chain with tf2_tools
```

#### **Sensor Data Quality Issues**
```
Symptoms: Noisy or incorrect sensor readings
Solutions:
1. Recalibrate sensors
2. Check power supply stability
3. Verify sensor mounting (no vibration)
4. Update firmware/drivers
```

#### **Communication Timeouts**
```
Symptoms: Services/topics not responding
Solutions:
1. Check ROS_DOMAIN_ID consistency
2. Verify QoS settings match
3. Test network connectivity
4. Monitor system resources (CPU/memory)
```

---

## ✅ **Setup Completion Checklist**

### **Critical Systems (Must Pass)**
- [ ] All sensors calibrated and validated
- [ ] TF tree properly configured and verified
- [ ] ROS2 network stable and reliable
- [ ] Motion control responding correctly
- [ ] Emergency stop system functional

### **Performance Validation (Competition Ready)**
- [ ] Navigation accuracy within requirements
- [ ] Computer vision detection reliable
- [ ] SLAM drift within acceptable limits
- [ ] System stable under load
- [ ] Health monitoring alerting properly

### **Documentation & Maintenance**
- [ ] All calibration files backed up
- [ ] Setup procedures documented
- [ ] Maintenance schedule established
- [ ] Troubleshooting guides available

---

## 📚 **Related Documentation**

- [**CameraCalibrationGuide.md**](CameraCalibrationGuide.md) - Detailed camera calibration procedures
- [**CalibrationGuide.md**](CalibrationGuide.md) - Hardware-specific calibration for cameras
- [**SensorGuide.md**](SensorGuide.md) - Sensor specifications and integration
- [**PID_Control_Diagrams.md**](../../tasks/PID_Control_Diagrams.md) - Control system tuning details
- [**SystemArchitecture.md**](../../overview/SystemArchitecture.md) - Overall system design

---

**All autonomy setup systems must be completed and validated before competition deployment. Incomplete setup is the most common cause of autonomy system failures.** 🚀🤖
