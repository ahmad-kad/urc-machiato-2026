# Sensor Guide - URC 2026 Autonomy System

## Overview
This comprehensive guide covers all sensors required for the URC 2026 autonomy system, including existing recommendations, future upgrades, and complete integration details with ROS 2 libraries.

## 📊 Sensor Categories by Subsystem

### Navigation & Path Planning
**Primary Sensors:**
- **GNSS/GPS Receiver**: Global positioning and navigation
- **IMU (Inertial Measurement Unit)**: Orientation and acceleration
- **Wheel Encoders**: Odometry and velocity feedback
- **Magnetometer**: Heading reference (optional)

**Secondary Sensors:**
- **Barometric Altimeter**: Altitude estimation
- **Differential GPS**: Enhanced accuracy (future upgrade)

### SLAM & Localization
**Primary Sensors:**
- **2D LIDAR**: Environment mapping and obstacle detection
- **RGB-D Camera**: Visual odometry and depth sensing
- **IMU**: Motion compensation for sensor fusion

**Secondary Sensors:**
- **3D LIDAR**: High-resolution mapping (future upgrade)
- **Stereo Camera**: Depth estimation without structured light
- **Ultrasonic Sensors**: Close-range obstacle detection

### Computer Vision & Object Classification
**Primary Sensors:**
- **RGB Camera**: Object detection and classification
- **RGB-D Camera**: Depth-aware object recognition
- **Wide-Angle Camera**: Surround view and navigation

**Secondary Sensors:**
- **Thermal Camera**: Heat signature detection (future)
- **Multi-spectral Camera**: Enhanced object classification
- **High-speed Camera**: Motion analysis

### Autonomous Typing
**Primary Sensors:**
- **High-Resolution RGB Camera**: Keyboard detection and character recognition
- **RGB-D Camera**: 3D pose estimation for arm positioning
- **Force/Torque Sensor**: Tactile feedback for manipulation

**Secondary Sensors:**
- **Structured Light Scanner**: Precise 3D keyboard mapping
- **Tactile Sensors**: Surface contact detection

### State Management & Health Monitoring
**Primary Sensors:**
- **Voltage/Current Sensors**: Power monitoring
- **Temperature Sensors**: Thermal monitoring
- **IMU**: System orientation and vibration
- **GNSS**: Position validation

**Secondary Sensors:**
- **Strain Gauges**: Structural integrity monitoring
- **Vibration Sensors**: Mechanical health assessment
- **Environmental Sensors**: Dust, humidity monitoring

### LED Status Signaling
**Primary Components:**
- **High-Power LEDs**: Visual status indication
- **RGB LEDs**: Color-coded status messages
- **PWM Controllers**: Brightness and color control

**Secondary Components:**
- **LED Matrix Displays**: Text/status messaging
- **Laser Projectors**: Ground projection (future)

---

## 🔧 Sensor Specifications & Integration

### 📡 GPS/GNSS Receivers

#### **u-blox NEO-M8N - RECOMMENDED**
**INPUT:**
- Satellite signals: GPS, GLONASS, Galileo, BeiDou
- Update rate: Up to 10Hz
- Interface: UART/SPI/I2C (TTL levels)

**OUTPUT:**
- Position: Latitude, Longitude, Altitude (WGS84)
- Velocity: 3D velocity vector
- Time: GPS time with nanosecond precision
- Accuracy: 2.5m CEP (autonomous), <1m with DGPS

**WHY CHOOSE THIS:**
- Excellent outdoor performance in desert environments
- Multiple constellation support for better coverage
- RTK-ready for future centimeter accuracy
- Low power consumption (25mA @ 3.3V)

**SUMMARY:**
- **Cost:** $65
- **Size:** 12.2×16mm
- **Power:** 2.7-3.6V, 25mA
- **Best for:** Outdoor navigation, long-range missions
- **Alternatives:** Garmin GPS 18x LVC ($80), u-blox NEO-M8P ($85 RTK)

**ROS 2 INTEGRATION:**
```bash
# Install driver
sudo apt install ros-humble-nmea-navsat-driver

# Launch node
ros2 launch nmea_navsat_driver nmea_serial_driver.launch.xml \
  port:=/dev/ttyUSB0 \
  baud:=9600 \
  frame_id:=gps_link
```

---

### 🌀 IMU (Inertial Measurement Units)

#### **Bosch BMI088 - RECOMMENDED**
**INPUT:**
- Motion: Linear acceleration (3-axis)
- Rotation: Angular velocity (3-axis)
- Sample rate: Up to 1kHz
- Interface: SPI (primary), I2C (secondary)

**OUTPUT:**
- Acceleration: ±2g to ±16g range
- Gyroscope: ±125°/s to ±2000°/s range
- Resolution: 16-bit
- Filtered data: Digital low-pass filters available

**WHY CHOOSE THIS:**
- Excellent vibration rejection for rough terrain
- High dynamic range for rover movements
- SPI interface for high-speed data transfer
- Integrated temperature compensation

**SUMMARY:**
- **Cost:** $8
- **Size:** 3×4.5mm LGA
- **Power:** 2.4-3.6V, 950µA
- **Best for:** Motion sensing, sensor fusion
- **Alternatives:** MPU-6050 ($5), Bosch BMI160 ($12)

**ROS 2 INTEGRATION:**
```bash
# Install IMU tools
sudo apt install ros-humble-imu-tools

# Launch filter
ros2 run imu_filter_madgwick imu_filter_node \
  --ros-args -p use_mag:=false -p publish_tf:=false \
  -r imu/data_raw:=imu/data \
  -r imu/data:=imu/filtered
```

---

### 🌀 LIDAR Sensors

#### **RPLIDAR A2M8 - RECOMMENDED**
**INPUT:**
- Laser scanning: 360° horizontal field of view
- Range: 0.15-12 meters
- Sample rate: 8000 samples/second
- Motor speed: 10Hz rotation

**OUTPUT:**
- Point cloud: Distance measurements at 1° resolution
- Intensity data: Signal strength for each point
- Timestamp: Synchronized with system time
- Quality metrics: Signal-to-noise ratio

**WHY CHOOSE THIS:**
- Excellent outdoor performance in bright sunlight
- Wide field of view for comprehensive obstacle detection
- Low power consumption for battery operation
- Rugged design for rough terrain

**SUMMARY:**
- **Cost:** $399
- **Size:** 98.5mm diameter
- **Power:** 5V, 300mA
- **Best for:** 2D mapping, obstacle avoidance
- **Alternatives:** RPLIDAR A3 ($549), Velodyne VLP-16 ($4000)

**ROS 2 INTEGRATION:**
```bash
# Install RPLIDAR driver
sudo apt install ros-humble-rplidar-ros

# Launch LIDAR
ros2 launch rplidar_ros rplidar.launch.xml \
  serial_port:=/dev/ttyUSB0 \
  frame_id:=laser_link \
  angle_compensate:=true
```

---

### 📷 RGB-D Cameras

#### **Intel RealSense D435i - RECOMMENDED**
**INPUT:**
- RGB camera: 1920×1080 @ 30fps
- IR stereo: Dual infrared cameras for depth
- IMU: Integrated 6-DoF IMU
- Field of view: 69°×42° (RGB), 87°×58° (depth)

**OUTPUT:**
- RGB image: Color video stream
- Depth image: Distance to each pixel (0.2-10m range)
- Point cloud: 3D point cloud from depth data
- IMU data: Motion compensation for depth calculation

**WHY CHOOSE THIS:**
- Active stereo depth sensing works in sunlight
- Integrated IMU for motion compensation
- USB 3.0 interface for high bandwidth
- SDK support for advanced processing

**SUMMARY:**
- **Cost:** $179
- **Size:** 90×25×25mm
- **Power:** 1.6W typical
- **Best for:** Object detection, depth sensing
- **Alternatives:** Oak-D Pro ($249), Zed 2 ($449)

**ROS 2 INTEGRATION:**
```bash
# Install RealSense SDK (follow docs)
# Then launch camera
ros2 launch realsense2_camera rs_launch.py \
  camera_name:=camera \
  depth_module.depth_profile:=848x480x30 \
  rgb_camera.color_profile:=1280x720x30 \
  enable_gyro:=true \
  enable_accel:=true
```

---

### 🔄 Wheel Encoders

#### **Pololu Magnetic Encoder - RECOMMENDED**
**INPUT:**
- Magnetic field: Hall effect sensor
- Resolution: 12 CPR (48 counts/rev with quadrature)
- Shaft rotation: Bidirectional sensing

**OUTPUT:**
- Position: Absolute angular position
- Direction: Rotation direction (CW/CCW)
- Speed: Rotational velocity
- Resolution: 48 pulses per revolution

**WHY CHOOSE THIS:**
- Magnetic sensing immune to dust and dirt
- Quadrature encoding for direction sensing
- No mechanical contact (no wear)
- Easy mounting on wheel hubs

**SUMMARY:**
- **Cost:** $15 per encoder
- **Size:** Compact PCB module
- **Power:** 2.7-18V
- **Best for:** Odometry, velocity feedback
- **Alternatives:** AS5048A ($8), Optical encoders ($25)

**ROS 2 INTEGRATION:**
```python
# Custom encoder node
class EncoderNode(Node):
    def __init__(self):
        super().__init__('encoder_node')
        self.publisher = self.create_publisher(JointState, 'joint_states', 10)
        # Read encoder values and publish as joint states
```

---

### 💪 Force/Torque Sensors

#### **ATI Nano17 - RECOMMENDED**
**INPUT:**
- Force: 3D force vector (Fx, Fy, Fz)
- Torque: 3D torque vector (Tx, Ty, Tz)
- Sample rate: Up to 7kHz
- Interface: Ethernet TCP/IP

**OUTPUT:**
- Forces: ±8.9N to ±133N range
- Torques: ±0.08Nm to ±0.8Nm range
- Resolution: 1/16000 of full scale
- Calibrated data: Factory calibrated measurements

**WHY CHOOSE THIS:**
- High precision for delicate manipulation
- Ethernet interface for reliable data transfer
- Temperature compensated measurements
- NIST traceable calibration

**SUMMARY:**
- **Cost:** $3500
- **Size:** Compact sensor housing
- **Power:** 12-24V, 15W
- **Best for:** Robotic manipulation, force control
- **Alternatives:** Robotiq FT 300 ($2500), OnRobot HEX ($1800)

**ROS 2 INTEGRATION:**
```bash
# Install force/torque controller
sudo apt install ros-humble-force-torque-sensor-controller

# Launch sensor
ros2 run force_torque_sensor_controller force_torque_sensor_controller \
  --ros-args -p sensor_name:=wrist_ft_sensor
```

## 📊 Sensor Selection Matrix

| Sensor | Cost | Power | Best Use Case | ROS 2 Ready |
|--------|------|-------|----------------|-------------|
| **u-blox NEO-M8N** | $65 | 25mA | Outdoor navigation | ✅ Full |
| **Bosch BMI088** | $8 | 950µA | Motion sensing | ✅ Full |
| **RPLIDAR A2M8** | $399 | 300mA | 2D mapping | ✅ Full |
| **RealSense D435i** | $179 | 1.6W | Object detection | ✅ Full |
| **Pololu Encoder** | $15 | Low | Odometry | ⚠️ Custom |
| **ATI Nano17** | $3500 | 15W | Manipulation | ✅ Full |

## 🛠️ Integration Checklist

### Pre-Integration
- [ ] Power requirements verified for all sensors
- [ ] Interface compatibility checked (UART/I2C/SPI/USB)
- [ ] ROS 2 driver availability confirmed
- [ ] Mounting locations identified on rover
- [ ] Cable routing planned

### Electrical Integration
- [ ] Power distribution designed (12V, 5V, 3.3V rails)
- [ ] Fuse protection added for each sensor
- [ ] Ground connections verified (star grounding)
- [ ] EMI shielding considered for sensitive sensors

### Software Integration
- [ ] ROS 2 workspaces configured
- [ ] Sensor drivers installed
- [ ] Launch files created
- [ ] Topic naming standardized
- [ ] QoS profiles configured appropriately

### Testing & Validation
- [ ] Individual sensor testing completed
- [ ] Data quality verified (no noise, correct ranges)
- [ ] Timing synchronization checked
- [ ] Environmental testing done (temperature, vibration)
- [ ] Integration testing with other subsystems

## 🚀 Quick Start Integration

### 1. Choose Your Sensors
Based on your subsystem:
- **Navigation**: GPS + IMU + Encoders
- **SLAM**: LIDAR + IMU + Camera
- **Computer Vision**: RGB-D Camera
- **Autonomous Typing**: RGB-D Camera + Force/Torque
- **State Management**: IMU + GPS
- **LED Status**: GPIO interface

### 2. Order Sensors
Use the recommended sensors from this guide:
- Start with minimum viable set (~$800)
- Add advanced sensors as budget allows
- Source from reputable suppliers

### 3. Hardware Integration
- Mount sensors on rover frame
- Connect power and data cables
- Verify electrical connections
- Test power consumption

### 4. Software Setup
```bash
# Install ROS 2 drivers
sudo apt install ros-humble-[sensor-driver]

# Create launch files
ros2 launch [sensor_package] [sensor].launch.xml

# Test data publication
ros2 topic list | grep [sensor]
ros2 topic echo [sensor_topic]
```

### 5. System Integration
- Add sensors to main autonomy launch
- Configure sensor fusion (if applicable)
- Test end-to-end data flow
- Validate timing and synchronization

## 📞 Support & Resources

### Getting Help
1. **Check this guide** for your specific sensor
2. **ROS 2 documentation** for driver details
3. **Manufacturer datasheets** for technical specs
4. **Team discussion** for integration questions

### Useful Links
- **ROS 2 Sensors**: https://index.ros.org/r/sensor_drivers/
- **u-blox GPS**: https://www.u-blox.com/en/positioning-chips-and-modules
- **Intel RealSense**: https://www.intelrealsense.com/
- **RPLIDAR**: https://www.slamtec.com/en/Lidar

### Community Resources
- **ROS Answers**: https://answers.ros.org/
- **ROS Discourse**: https://discourse.ros.org/
- **GitHub Issues**: Check driver repositories

This sensor guide provides everything needed to select, integrate, and optimize sensors for the URC 2026 autonomy system. Each sensor includes practical INPUT→OUTPUT→WHY→SUMMARY information for informed decision making.
