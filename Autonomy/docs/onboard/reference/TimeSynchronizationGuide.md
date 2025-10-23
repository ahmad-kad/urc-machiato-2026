# Time Synchronization Guide - URC 2026 Rover

## Overview

Time synchronization is **critical for sensor fusion** in the URC 2026 rover system. Poor synchronization between IMU, LIDAR, camera, and GPS sensors can cause significant localization errors, failed sensor fusion, and navigation failures. This guide covers NTP/PTP network synchronization, ROS 2 clock management, and sensor timestamp alignment procedures.

## 📊 **Why Time Synchronization Matters**

### **Mission Impact**
- **Sensor Fusion Accuracy**: IMU + LIDAR + Camera data must be time-aligned for reliable SLAM
- **GPS/IMU Integration**: Precise timing enables accurate position/attitude estimation
- **Multi-Rate Processing**: Different sensor frequencies (10Hz GPS, 100Hz IMU, 30Hz camera) require synchronization
- **Data Association**: Correct temporal alignment of sensor measurements

### **Synchronization Consequences**
```
Poor Time Sync → Temporal Misalignment → Sensor Fusion Errors → Localization Failure
```

- **Position Errors**: 1-5m drift in GPS/IMU fusion with 100ms sync error
- **Velocity Errors**: Incorrect motion compensation in SLAM
- **Data Association Failures**: Features observed at different times treated as simultaneous
- **Filter Divergence**: Kalman filters fail with inconsistent timestamps

---

## 🔧 **Time Synchronization Fundamentals**

### **Core Concepts**

#### **Clock Types**
- **System Clock**: Linux system time (affected by NTP)
- **ROS Clock**: ROS 2 internal time source (can be external)
- **Sensor Clocks**: Hardware timestamps from IMU, cameras, etc.
- **GPS Time**: Precise time from GPS satellites

#### **Synchronization Methods**
- **NTP**: Network Time Protocol (accuracy: ~1-100ms)
- **PTP**: Precision Time Protocol (accuracy: ~1-100µs)
- **Hardware Sync**: Dedicated synchronization hardware
- **Software Sync**: Post-processing timestamp alignment

### **ROS 2 Time Architecture**
```python
# ROS 2 time management
from rclpy.time import Time, Duration
from rclpy.clock import Clock, ClockType

# Different clock sources
system_clock = Clock(clock_type=ClockType.SYSTEM_TIME)
ros_clock = Clock(clock_type=ClockType.ROS_TIME)
steady_clock = Clock(clock_type=ClockType.STEADY_TIME)
```

---

## 📋 **Time Synchronization Prerequisites**

### **Hardware Requirements**
- **GPS Receiver**: For accurate time reference (PPS signal preferred)
- **Ethernet Network**: For PTP synchronization
- **RTC Module**: Real-time clock for system startup
- **PPS Signal**: Pulse-per-second from GPS for microsecond accuracy

### **Software Requirements**
```bash
# NTP and PTP tools
sudo apt install ntp ntpdate
sudo apt install linuxptp  # PTP implementation

# ROS 2 time synchronization
sudo apt install ros-humble-ros2sync  # If available
sudo apt install ros-humble-chrony  # Alternative NTP implementation

# Time monitoring tools
pip install ntplib pytz
```

### **Network Configuration**
```bash
# PTP network interface configuration
# /etc/linuxptp/ptp4l.conf
[global]
slaveOnly 1
priority1 128
priority2 128
domainNumber 0

[eth0]  # Ethernet interface for PTP
```

---

## 🚀 **Time Synchronization Implementation**

### **Phase 1: System Clock Synchronization**

#### **NTP Configuration**
```bash
# Install and configure NTP
sudo apt install ntp

# Configure NTP servers (/etc/ntp.conf)
server 0.pool.ntp.org iburst
server 1.pool.ntp.org iburst
server 2.pool.ntp.org iburst
server 3.pool.ntp.org iburst

# GPS as NTP server (if available)
server 127.127.28.0 prefer  # SHM driver for GPS

# Start NTP service
sudo systemctl enable ntp
sudo systemctl start ntp
```

#### **Chrony Configuration (Alternative)**
```bash
# Install Chrony
sudo apt install chrony

# Configure Chrony (/etc/chrony/chrony.conf)
pool pool.ntp.org iburst
refclock SHM 0 refid GPS precision 1e-1 offset 0.0

# GPS PPS reference (if available)
refclock PPS /dev/pps0 refid PPS precision 1e-9

# Start Chrony
sudo systemctl enable chrony
sudo systemctl start chrony
```

#### **GPS-Based Time Synchronization**
```python
# gps_time_sync.py - GPS as time source
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger
import subprocess

class GPSTimeSync(Node):
    def __init__(self):
        super().__init__('gps_time_sync')

        # GPS subscriber
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 1
        )

        # Time sync service
        self.sync_service = self.create_service(
            Trigger, '/time_sync', self.sync_callback
        )

        self.last_gps_time = None

    def gps_callback(self, msg):
        """Store GPS time for synchronization"""
        if msg.status.status >= 0:  # Valid GPS fix
            self.last_gps_time = msg.header.stamp

    def sync_callback(self, request, response):
        """Synchronize system clock to GPS time"""
        if self.last_gps_time is None:
            response.success = False
            response.message = "No valid GPS time available"
            return response

        try:
            # Set system time (requires root)
            gps_time_seconds = self.last_gps_time.sec
            gps_time_nsec = self.last_gps_time.nanosec

            # Use date command to set system time
            cmd = f"sudo date -s '@{gps_time_seconds}.{gps_time_nsec}'"
            subprocess.run(cmd, shell=True, check=True)

            response.success = True
            response.message = f"System time synchronized to GPS time"

        except subprocess.CalledProcessError as e:
            response.success = False
            response.message = f"Time sync failed: {str(e)}"

        return response
```

### **Phase 2: PTP Precision Time Protocol**

#### **PTP Master Configuration**
```bash
# Configure PTP master (if rover is time master)
# /etc/linuxptp/ptp4l.conf
[global]
priority1 127  # Higher priority = preferred master
priority2 127
domainNumber 0
clockClass 6   # GPS-synchronized clock

[eth0]
network_transport L2  # Layer 2 transport
delay_mechanism P2P  # Peer-to-peer delay mechanism
```

#### **PTP Slave Configuration**
```bash
# Configure PTP slave (compute nodes)
# /etc/linuxptp/ptp4l.conf
[global]
slaveOnly 1
priority1 128
priority2 128
domainNumber 0

[eth0]
network_transport L2
delay_mechanism P2P
```

#### **PTP Service Management**
```bash
# Start PTP services
sudo systemctl enable ptp4l
sudo systemctl start ptp4l

# Check PTP status
sudo pmc -u -b 0 'GET CURRENT_DATA_SET'

# Monitor PTP synchronization
watch -n 1 pmc -u -b 0 'GET TIME_STATUS_NP'
```

### **Phase 3: ROS 2 Clock Management**

#### **ROS Clock Configuration**
```python
# ros_clock_config.py
class ROSClockManager(Node):
    def __init__(self):
        super().__init__('ros_clock_manager')

        # Use external time source if available
        self.use_sim_time = self.declare_parameter('use_sim_time', False).value

        if not self.use_sim_time:
            # Use system time synchronized via NTP/PTP
            self.clock_pub = self.create_publisher(Clock, '/clock', 1)

            # Publish clock at high frequency
            self.timer = self.create_timer(0.01, self.publish_clock)  # 100Hz

    def publish_clock(self):
        """Publish current ROS time"""
        clock_msg = Clock()
        clock_msg.clock = self.get_clock().now().to_msg()
        self.clock_pub.publish(clock_msg)
```

#### **Launch Configuration**
```python
# time_sync_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        # ROS Clock publisher
        Node(
            package='time_sync',
            executable='ros_clock_manager',
            name='ros_clock_manager'
        ),

        # Time synchronization monitor
        Node(
            package='time_sync',
            executable='time_sync_monitor',
            name='time_sync_monitor'
        ),

        # PTP daemon (if using PTP)
        ExecuteProcess(
            cmd=['sudo', 'ptp4l', '-f', '/etc/linuxptp/ptp4l.conf', '-i', 'eth0'],
            output='screen'
        )
    ])
```

### **Phase 4: Sensor Timestamp Alignment**

#### **Camera Timestamp Synchronization**
```python
# camera_time_sync.py
class CameraTimeSync(Node):
    def __init__(self):
        super().__init__('camera_time_sync')

        # Camera subscriber
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )

        # Synchronized publisher
        self.sync_image_pub = self.create_publisher(
            Image, '/camera/image_sync', 10
        )

        # Time offset estimation
        self.time_offsets = []
        self.max_samples = 100

    def image_callback(self, msg):
        """Synchronize camera timestamps"""

        # Get current ROS time
        ros_time = self.get_clock().now()

        # Camera timestamp (from driver)
        camera_time = Time.from_msg(msg.header.stamp)

        # Calculate offset
        offset = ros_time - camera_time
        self.time_offsets.append(offset.nanoseconds / 1e9)  # Convert to seconds

        # Keep only recent samples
        if len(self.time_offsets) > self.max_samples:
            self.time_offsets.pop(0)

        # Apply median filter to reduce outliers
        if len(self.time_offsets) >= 10:
            median_offset = sorted(self.time_offsets)[len(self.time_offsets)//2]

            # Correct timestamp
            corrected_time = camera_time + Duration(seconds=median_offset)
            msg.header.stamp = corrected_time.to_msg()

        # Publish synchronized image
        self.sync_image_pub.publish(msg)
```

#### **IMU Timestamp Synchronization**
```python
# imu_time_sync.py
class IMUTimeSync(Node):
    def __init__(self):
        super().__init__('imu_time_sync')

        # IMU subscriber
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_raw', self.imu_callback, 100
        )

        # Synchronized publisher
        self.sync_imu_pub = self.create_publisher(
            Imu, '/imu/data_sync', 100
        )

        # Hardware timestamp buffer
        self.hw_timestamps = []
        self.ros_timestamps = []

    def imu_callback(self, msg):
        """Synchronize IMU timestamps using hardware time"""

        # Get ROS time when message received
        ros_time = self.get_clock().now()

        # IMU hardware timestamp (if available from driver)
        # Assuming IMU provides hardware timestamps
        hw_time = Time.from_msg(msg.header.stamp)

        # Store timestamp pairs for offset estimation
        self.hw_timestamps.append(hw_time)
        self.ros_timestamps.append(ros_time)

        # Keep buffer size manageable
        if len(self.hw_timestamps) > 1000:
            self.hw_timestamps.pop(0)
            self.ros_timestamps.pop(0)

        # Estimate time offset using linear regression
        if len(self.hw_timestamps) >= 50:
            offset = self.estimate_time_offset()
            corrected_time = hw_time + offset
            msg.header.stamp = corrected_time.to_msg()

        self.sync_imu_pub.publish(msg)

    def estimate_time_offset(self):
        """Estimate time offset between hardware and ROS time"""

        if len(self.hw_timestamps) < 2:
            return Duration()

        # Convert to seconds for regression
        hw_times = [(t - self.hw_timestamps[0]).nanoseconds / 1e9 for t in self.hw_timestamps]
        ros_times = [(t - self.ros_timestamps[0]).nanoseconds / 1e9 for t in self.ros_timestamps]

        # Linear regression: ROS_time = slope * HW_time + offset
        try:
            from scipy import stats
            slope, intercept, r_value, p_value, std_err = stats.linregress(hw_times, ros_times)

            # Offset is the intercept
            offset_seconds = intercept
            return Duration(seconds=offset_seconds)

        except ImportError:
            # Fallback: simple average offset
            offsets = [ros - hw for ros, hw in zip(ros_times, hw_times)]
            avg_offset = sum(offsets) / len(offsets)
            return Duration(seconds=avg_offset)
```

#### **LIDAR Timestamp Synchronization**
```python
# lidar_time_sync.py
class LidarTimeSync(Node):
    def __init__(self):
        super().__init__('lidar_time_sync')

        # LIDAR subscriber
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Synchronized publisher
        self.sync_scan_pub = self.create_publisher(
            LaserScan, '/scan_sync', 10
        )

        # Scan timing compensation
        self.scan_period = 0.1  # 10Hz LIDAR
        self.points_per_scan = 360  # 1° resolution

    def scan_callback(self, msg):
        """Synchronize LIDAR scan timestamps"""

        # LIDAR scans are collected over time
        # Timestamp represents end of scan
        scan_end_time = Time.from_msg(msg.header.stamp)

        # Calculate start time of scan
        scan_start_time = scan_end_time - Duration(seconds=self.scan_period)

        # Interpolate timestamps for individual points
        # (Advanced: compensate for rotational motion)

        # For now, use scan midpoint time
        midpoint_time = scan_start_time + Duration(seconds=self.scan_period / 2.0)
        msg.header.stamp = midpoint_time.to_msg()

        self.sync_scan_pub.publish(msg)
```

---

## 🔍 **Time Synchronization Monitoring**

### **Time Sync Health Monitor**
```python
# time_sync_monitor.py
class TimeSyncMonitor(Node):
    def __init__(self):
        super().__init__('time_sync_monitor')

        # NTP offset monitoring
        self.ntp_timer = self.create_timer(10.0, self.check_ntp_offset)

        # PTP status monitoring
        self.ptp_timer = self.create_timer(5.0, self.check_ptp_status)

        # Sensor timestamp monitoring
        self.image_sub = self.create_subscription(
            Image, '/camera/image_sync', self.monitor_camera_time, 1
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_sync', self.monitor_imu_time, 10
        )

        # Time sync health publisher
        self.health_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 1
        )

        # Statistics
        self.camera_time_diffs = []
        self.imu_time_diffs = []
        self.ntp_offsets = []

    def check_ntp_offset(self):
        """Check NTP time offset"""
        try:
            import ntplib
            client = ntplib.NTPClient()
            response = client.request('pool.ntp.org', version=3)

            offset = response.offset
            self.ntp_offsets.append(offset)

            # Keep only recent samples
            if len(self.ntp_offsets) > 10:
                self.ntp_offsets.pop(0)

            # Check if offset is acceptable (< 100ms)
            if abs(offset) > 0.1:
                self.get_logger().warn(f'NTP offset too large: {offset:.3f}s')

        except Exception as e:
            self.get_logger().error(f'NTP check failed: {str(e)}')

    def check_ptp_status(self):
        """Check PTP synchronization status"""
        try:
            import subprocess
            result = subprocess.run(
                ['pmc', '-u', '-b', '0', 'GET TIME_STATUS_NP'],
                capture_output=True, text=True, timeout=5
            )

            # Parse PTP status
            if 'slave' in result.stdout.lower():
                self.get_logger().info('PTP: Synchronized as slave')
            else:
                self.get_logger().warn('PTP: Not synchronized')

        except subprocess.TimeoutExpired:
            self.get_logger().error('PTP status check timeout')
        except Exception as e:
            self.get_logger().error(f'PTP check failed: {str(e)}')

    def monitor_camera_time(self, msg):
        """Monitor camera timestamp health"""
        current_time = self.get_clock().now()
        camera_time = Time.from_msg(msg.header.stamp)

        time_diff = abs((current_time - camera_time).nanoseconds / 1e9)
        self.camera_time_diffs.append(time_diff)

        if len(self.camera_time_diffs) > 100:
            self.camera_time_diffs.pop(0)

        # Check for excessive delay
        if time_diff > 0.1:  # 100ms delay
            self.get_logger().warn('.3f')

    def monitor_imu_time(self, msg):
        """Monitor IMU timestamp health"""
        current_time = self.get_clock().now()
        imu_time = Time.from_msg(msg.header.stamp)

        time_diff = abs((current_time - imu_time).nanoseconds / 1e9)
        self.imu_time_diffs.append(time_diff)

        if len(self.imu_time_diffs) > 100:
            self.imu_time_diffs.pop(0)

        # IMU should have very low latency
        if time_diff > 0.01:  # 10ms delay
            self.get_logger().warn('.3f')

    def publish_health_status(self):
        """Publish comprehensive time sync health status"""

        diagnostics = DiagnosticArray()
        diagnostics.header.stamp = self.get_clock().now().to_msg()

        # NTP status
        ntp_status = DiagnosticStatus()
        ntp_status.name = 'NTP Synchronization'
        ntp_status.hardware_id = 'system_clock'

        if self.ntp_offsets:
            avg_offset = sum(self.ntp_offsets) / len(self.ntp_offsets)
            max_offset = max(abs(o) for o in self.ntp_offsets)

            if max_offset < 0.1:  # 100ms
                ntp_status.level = DiagnosticStatus.OK
                ntp_status.message = f'Good sync: {avg_offset:.3f}s average offset'
            else:
                ntp_status.level = DiagnosticStatus.WARN
                ntp_status.message = f'Poor sync: {max_offset:.3f}s max offset'
        else:
            ntp_status.level = DiagnosticStatus.ERROR
            ntp_status.message = 'NTP not available'

        # Camera sync status
        camera_status = DiagnosticStatus()
        camera_status.name = 'Camera Timestamp Sync'
        camera_status.hardware_id = 'camera'

        if self.camera_time_diffs:
            avg_delay = sum(self.camera_time_diffs) / len(self.camera_time_diffs)
            max_delay = max(self.camera_time_diffs)

            if max_delay < 0.1:
                camera_status.level = DiagnosticStatus.OK
                camera_status.message = f'Good sync: {avg_delay:.3f}s average delay'
            else:
                camera_status.level = DiagnosticStatus.WARN
                camera_status.message = f'High delay: {max_delay:.3f}s max delay'

        # IMU sync status
        imu_status = DiagnosticStatus()
        imu_status.name = 'IMU Timestamp Sync'
        imu_status.hardware_id = 'imu'

        if self.imu_time_diffs:
            avg_delay = sum(self.imu_time_diffs) / len(self.imu_time_diffs)
            max_delay = max(self.imu_time_diffs)

            if max_delay < 0.01:
                imu_status.level = DiagnosticStatus.OK
                imu_status.message = f'Good sync: {avg_delay:.3f}s average delay'
            else:
                imu_status.level = DiagnosticStatus.WARN
                imu_status.message = f'High delay: {max_delay:.3f}s max delay'

        diagnostics.status = [ntp_status, camera_status, imu_status]
        self.health_pub.publish(diagnostics)
```

---

## 📊 **Time Synchronization Validation**

### **Time Sync Accuracy Tests**
```python
# time_sync_validator.py
class TimeSyncValidator(Node):
    def __init__(self):
        super().__init__('time_sync_validator')

        # Multi-sensor timestamp collection
        self.imu_times = []
        self.camera_times = []
        self.lidar_times = []
        self.gps_times = []

        # Subscribers for validation
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data_sync', self.collect_imu_time, 100
        )
        self.camera_sub = self.create_subscription(
            Image, '/camera/image_sync', self.collect_camera_time, 10
        )
        self.lidar_sub = self.create_subscription(
            LaserScan, '/scan_sync', self.collect_lidar_time, 10
        )
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.collect_gps_time, 1
        )

        # Validation timer
        self.validation_timer = self.create_timer(30.0, self.validate_sync)

    def collect_imu_time(self, msg):
        self.imu_times.append(Time.from_msg(msg.header.stamp))
        self.limit_buffer(self.imu_times)

    def collect_camera_time(self, msg):
        self.camera_times.append(Time.from_msg(msg.header.stamp))
        self.limit_buffer(self.camera_times)

    def collect_lidar_time(self, msg):
        self.lidar_times.append(Time.from_msg(msg.header.stamp))
        self.limit_buffer(self.lidar_times)

    def collect_gps_time(self, msg):
        self.gps_times.append(Time.from_msg(msg.header.stamp))
        self.limit_buffer(self.gps_times)

    def limit_buffer(self, buffer, max_size=1000):
        """Keep buffer size manageable"""
        if len(buffer) > max_size:
            buffer.pop(0)

    def validate_sync(self):
        """Validate synchronization between all sensors"""

        # Check if we have sufficient data
        if (len(self.imu_times) < 50 or len(self.camera_times) < 5 or
            len(self.lidar_times) < 5 or len(self.gps_times) < 2):
            self.get_logger().info('Collecting more timestamp data...')
            return

        # Calculate pairwise time differences
        sync_errors = {}

        # IMU vs Camera
        imu_camera_errors = self.calculate_sync_errors(
            self.imu_times, self.camera_times, 100, 10  # Hz rates
        )
        sync_errors['IMU_Camera'] = imu_camera_errors

        # IMU vs LIDAR
        imu_lidar_errors = self.calculate_sync_errors(
            self.imu_times, self.lidar_times, 100, 10
        )
        sync_errors['IMU_LIDAR'] = imu_lidar_errors

        # Camera vs LIDAR
        camera_lidar_errors = self.calculate_sync_errors(
            self.camera_times, self.lidar_times, 10, 10
        )
        sync_errors['Camera_LIDAR'] = camera_lidar_errors

        # Report results
        for sensor_pair, errors in sync_errors.items():
            if errors:
                mean_error = sum(errors) / len(errors)
                max_error = max(abs(e) for e in errors)

                status = "GOOD" if max_error < 0.01 else "POOR"
                self.get_logger().info(
                    f'{sensor_pair} sync: {status} '
                    '.6f'
                )

                if max_error >= 0.01:
                    self.get_logger().warn(
                        f'{sensor_pair} synchronization needs improvement'
                    )

    def calculate_sync_errors(self, fast_sensor_times, slow_sensor_times,
                            fast_rate, slow_rate):
        """
        Calculate synchronization errors between sensors with different rates

        Uses interpolation to estimate what the slow sensor timestamp
        should be at fast sensor measurement times.
        """

        if len(fast_sensor_times) < 10 or len(slow_sensor_times) < 3:
            return []

        errors = []

        # For each fast sensor measurement
        for fast_time in fast_sensor_times[-100:]:  # Last 100 samples

            # Find slow sensor measurements around this time
            slow_times_array = [t.nanoseconds / 1e9 for t in slow_sensor_times]
            fast_time_sec = fast_time.nanoseconds / 1e9

            # Find bracketing slow sensor measurements
            slow_times_sorted = sorted(slow_times_array)

            # Simple interpolation (advanced version would use more sophisticated method)
            if len(slow_times_sorted) >= 2:
                # Find closest slow sensor measurement
                closest_idx = min(range(len(slow_times_sorted)),
                                key=lambda i: abs(slow_times_sorted[i] - fast_time_sec))

                closest_time = slow_times_sorted[closest_idx]

                # Error is difference between actual and expected
                error = fast_time_sec - closest_time
                errors.append(error)

        return errors
```

---

## 🐛 **Time Synchronization Troubleshooting**

### **Common Time Sync Issues**

#### **1. NTP Synchronization Failures**
```
Error: "NTP socket is in use" or "no server suitable for synchronization"
```
**Solutions**:
```bash
# Stop conflicting services
sudo systemctl stop systemd-timesyncd

# Check NTP configuration
sudo ntpq -p

# Force time sync
sudo ntpdate pool.ntp.org

# Restart NTP
sudo systemctl restart ntp
```

#### **2. PTP Synchronization Issues**
```
Error: "PTP clock not synchronized"
```
**Solutions**:
```bash
# Check PTP status
sudo pmc -u -b 0 'GET CURRENT_DATA_SET'

# Restart PTP services
sudo systemctl restart ptp4l

# Check network configuration
ethtool eth0  # Check link status
```

#### **3. ROS Clock Issues**
```
Warning: "Clock not synchronized"
```
**Solutions**:
```python
# Ensure proper clock initialization
import rclpy
rclpy.init()

# Use appropriate clock type
self.clock = self.get_clock()  # ROS clock
# or
self.clock = Clock(clock_type=ClockType.SYSTEM_TIME)  # System clock
```

#### **4. Sensor Timestamp Drift**
```
Error: Increasing timestamp offsets over time
```
**Solutions**:
- Implement continuous offset estimation
- Use hardware timestamping when available
- Regularly recalibrate time offsets
- Monitor and alert on offset changes

### **Time Sync Performance Monitoring**
```bash
# NTP monitoring
ntpq -p  # Peer status
ntpstat  # Synchronization status

# PTP monitoring
pmc -u -b 0 'GET TIME_STATUS_NP'  # Time status
phc_ctl /dev/ptp0 get  # Hardware clock status

# System clock monitoring
timedatectl status  # System time status
```

---

## 📈 **Time Synchronization Performance Metrics**

### **Accuracy Requirements**
- **NTP Sync**: < 100ms offset from reference time
- **PTP Sync**: < 100µs offset from master clock
- **Sensor Sync**: < 10ms inter-sensor timestamp alignment
- **ROS Clock**: < 1ms jitter from system time

### **Latency Requirements**
- **NTP Updates**: < 1 second convergence time
- **PTP Updates**: < 100ms convergence time
- **Sensor Processing**: < 5ms timestamp correction latency
- **ROS Time**: < 100µs resolution

### **Reliability Requirements**
- **Uptime**: > 99.9% synchronization maintained
- **Recovery Time**: < 30 seconds after network interruption
- **False Positives**: < 0.1% incorrect sync warnings
- **Monitoring Coverage**: 100% of critical time sources monitored

---

## 📚 **Advanced Time Synchronization Topics**

### **Hardware Timestamping**
```python
# PTP hardware clock (PHC) configuration
class HardwareTimestamping:
    def __init__(self):
        # Configure network interface for hardware timestamping
        self.configure_ptp_hardware()

    def configure_ptp_hardware(self):
        """Configure hardware for PTP timestamping"""
        import subprocess

        # Enable hardware timestamping
        cmds = [
            'sudo ethtool -T eth0',  # Check capabilities
            'sudo hwstamp_ctl -i eth0 -r 1',  # Enable RX timestamping
            'sudo hwstamp_ctl -i eth0 -t 1'   # Enable TX timestamping
        ]

        for cmd in cmds:
            try:
                subprocess.run(cmd, shell=True, check=True)
            except subprocess.CalledProcessError as e:
                self.get_logger().error(f'Hardware timestamping setup failed: {e}')
```

### **GPS PPS Integration**
```python
# GPS PPS time synchronization
class GPSPPSSync(Node):
    def __init__(self):
        super().__init__('gps_pps_sync')

        # Configure PPS device
        self.setup_pps_device()

        # PPS timestamping
        self.pps_timestamps = []

    def setup_pps_device(self):
        """Setup PPS device for precise time"""
        try:
            # Configure PPS kernel module
            import subprocess
            subprocess.run('sudo modprobe pps-gpio', shell=True, check=True)

            # Configure GPIO pin for PPS input
            # (Hardware-specific configuration)

        except subprocess.CalledProcessError as e:
            self.get_logger().error(f'PPS setup failed: {e}')

    def synchronize_to_pps(self, gps_time, pps_time):
        """Synchronize system time using GPS + PPS"""
        # Calculate precise offset using PPS signal
        # Set system time with microsecond accuracy
        pass
```

### **Distributed Time Synchronization**
```python
# Multi-robot time synchronization
class DistributedTimeSync(Node):
    def __init__(self):
        super().__init__('distributed_time_sync')

        # Inter-robot time sync service
        self.time_sync_service = self.create_service(
            TimeSync, '/distributed_time_sync', self.handle_time_sync_request
        )

        # Time offset storage
        self.robot_time_offsets = {}

    def handle_time_sync_request(self, request, response):
        """Handle time sync request from other robots"""
        # Implement distributed time sync algorithm
        # (e.g., average time consensus, leader election)
        pass
```

---

## ✅ **Time Synchronization Checklist**

### **System Clock Setup**
- [ ] NTP or Chrony configured with reliable servers
- [ ] GPS time source integrated (if available)
- [ ] System clock accuracy verified (< 100ms offset)
- [ ] Clock synchronization monitoring active

### **PTP Configuration (Optional)**
- [ ] PTP master/slave roles assigned
- [ ] Network interfaces configured for PTP
- [ ] PTP services running and synchronized
- [ ] PTP accuracy verified (< 100µs offset)

### **ROS 2 Time Management**
- [ ] ROS clock publishing configured
- [ ] Clock type appropriate for use case
- [ ] Time source properly selected
- [ ] ROS time synchronization verified

### **Sensor Timestamp Alignment**
- [ ] IMU timestamp correction implemented
- [ ] Camera timestamp synchronization active
- [ ] LIDAR scan timing compensation working
- [ ] GPS timestamp alignment verified

### **Monitoring & Validation**
- [ ] Time sync health monitoring deployed
- [ ] Sensor timestamp validation active
- [ ] Performance metrics being collected
- [ ] Automated alerts configured

### **Testing & Verification**
- [ ] Multi-sensor timestamp alignment tested
- [ ] Time sync recovery procedures validated
- [ ] Performance under network stress verified
- [ ] Integration with sensor fusion confirmed

---

**🎯 Success Criteria**: Time synchronization complete when all sensors maintain <10ms timestamp alignment, system clock stays within 100ms of reference time, and sensor fusion operates without temporal artifacts, enabling accurate multi-sensor localization for URC 2026 mission success.
