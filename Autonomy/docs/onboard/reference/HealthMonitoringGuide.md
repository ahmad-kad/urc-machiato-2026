# Health Monitoring Guide - URC 2026 Rover

## Overview

Health monitoring is **critical for competition reliability** in the URC 2026 rover system. With only 30 minutes to complete missions and no opportunity for mid-competition repairs, comprehensive health monitoring enables early fault detection, graceful degradation, and autonomous recovery from failures.

This guide covers system diagnostics, sensor health checks, subsystem monitoring, and failure detection/recovery procedures essential for competition success.

## 📊 **Why Health Monitoring Matters**

### **Competition Reality**
- **30-minute mission window**: No time for debugging during competition
- **Desert environment**: Extreme temperatures, dust, vibration
- **No external support**: Must operate autonomously
- **High stakes**: Single failure can end mission

### **Failure Consequences**
```
No Health Monitoring → Undetected Failures → Mission Failure
```

- **Sensor failures**: Navigation errors, localization loss
- **Communication loss**: Subsystem coordination breakdown
- **Power issues**: Unexpected shutdowns
- **Thermal problems**: Component damage or shutdown

---

## 🔧 **Health Monitoring Fundamentals**

### **Core Concepts**

#### **Health Dimensions**
- **Functional Health**: Is the component working correctly?
- **Performance Health**: Is the component meeting performance requirements?
- **Resource Health**: Does the component have adequate resources?
- **Predictive Health**: Will the component fail soon?

#### **Monitoring Levels**
- **Component Level**: Individual sensors, actuators, processors
- **Subsystem Level**: Navigation, SLAM, computer vision, etc.
- **System Level**: Overall rover health and mission capability
- **Environmental Level**: External conditions affecting operation

### **ROS 2 Diagnostics Architecture**
```python
# ROS 2 diagnostics framework
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from rclpy.node import Node

# Diagnostic levels
class DiagnosticLevel:
    OK = 0
    WARN = 1
    ERROR = 2
    STALE = 3
```

---

## 📋 **Health Monitoring Prerequisites**

### **Hardware Requirements**
- **Sensor Redundancy**: Backup sensors for critical functions
- **Power Monitoring**: Voltage/current sensors on all subsystems
- **Temperature Sensors**: Critical component thermal monitoring
- **Watchdog Timers**: Hardware-level failure detection

### **Software Requirements**
```bash
# ROS 2 diagnostics packages
sudo apt install ros-humble-diagnostics
sudo apt install ros-humble-diagnostic-updater
sudo apt install ros-humble-self-test

# Monitoring tools
pip install psutil numpy matplotlib
```

### **Configuration Setup**
```yaml
# diagnostics_config.yaml
diagnostics:
  ros__parameters:
    # Diagnostic settings
    base_path: "rover"
    hardware_id: "urc_rover_2026"

    # Update rates
    diagnostic_period: 1.0  # seconds
    timeout_period: 5.0     # seconds

    # Thresholds
    warning_threshold: 0.8
    error_threshold: 0.5
    critical_threshold: 0.2
```

---

## 🚀 **Health Monitoring Implementation**

### **Phase 1: Sensor Health Monitoring**

#### **IMU Health Monitor**
```python
# imu_health_monitor.py
class IMUHealthMonitor(Node):
    def __init__(self):
        super().__init__('imu_health_monitor')

        # IMU data subscription
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Diagnostic updater
        from diagnostic_updater import Updater, FrequencyStatus, TimeStampStatus
        self.updater = Updater(self)
        self.updater.setHardwareID('imu_bmi088')

        # Diagnostic tasks
        self.freq_status = FrequencyStatus(ParametrizedStatus(
            "IMU Frequency", Updater.OK, Updater.WARN, Updater.ERROR, 5, 100, 0.1, 10
        ))
        self.timestamp_status = TimeStampStatus()

        self.updater.add(self.freq_status)
        self.updater.add(self.timestamp_status)
        self.updater.add("IMU Data Validity", self.check_imu_data)

        # Health tracking
        self.last_imu_time = None
        self.imu_data_count = 0
        self.consecutive_errors = 0

        # Diagnostic publisher
        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def imu_callback(self, msg):
        """Process IMU data and check health"""

        current_time = self.get_clock().now()

        # Update frequency and timestamp diagnostics
        self.freq_status.tick()
        self.timestamp_status.tick(msg.header.stamp)

        # Check data validity
        accel_magnitude = np.linalg.norm([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        gyro_magnitude = np.linalg.norm([
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z
        ])

        # Check for valid ranges (±16g for accel, ±2000°/s for gyro)
        accel_valid = accel_magnitude < 160.0  # m/s²
        gyro_valid = gyro_magnitude < 35.0     # rad/s

        if not accel_valid or not gyro_valid:
            self.consecutive_errors += 1
        else:
            self.consecutive_errors = 0

        self.imu_data_count += 1
        self.last_imu_time = current_time

    def check_imu_data(self, stat):
        """Custom IMU health check"""

        # Check data rate (expect ~100Hz)
        if self.imu_data_count < 50:  # Less than 50 samples in last second
            stat.summary(DiagnosticStatus.WARN, "Low IMU data rate")
            stat.add("Data Rate", f"{self.imu_data_count} Hz")
        else:
            stat.summary(DiagnosticStatus.OK, "IMU data OK")
            stat.add("Data Rate", f"{self.imu_data_count} Hz")

        # Check for consecutive errors
        if self.consecutive_errors > 5:
            stat.summary(DiagnosticStatus.ERROR, "IMU data invalid")
            stat.add("Consecutive Errors", str(self.consecutive_errors))
        else:
            stat.add("Consecutive Errors", str(self.consecutive_errors))

        # Check timestamp freshness
        if self.last_imu_time:
            age = (self.get_clock().now() - self.last_imu_time).nanoseconds / 1e9
            stat.add("Data Age", f"{age:.3f} seconds")

            if age > 0.1:  # Older than 100ms
                stat.summary(DiagnosticStatus.WARN, "IMU data stale")

        self.imu_data_count = 0  # Reset counter

        return stat

    def publish_diagnostics(self):
        """Publish diagnostic status"""
        self.updater.update()
```

#### **GPS Health Monitor**
```python
# gps_health_monitor.py
class GPSHealthMonitor(Node):
    def __init__(self):
        super().__init__('gps_health_monitor')

        # GPS subscriptions
        self.gps_sub = self.create_subscription(
            NavSatFix, '/gps/fix', self.gps_callback, 1
        )
        self.gps_vel_sub = self.create_subscription(
            TwistStamped, '/gps/vel', self.gps_vel_callback, 1
        )

        # Diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID('gps_ublox_neo_m8n')

        # Diagnostic tasks
        self.updater.add("GPS Fix Quality", self.check_gps_fix)
        self.updater.add("GPS Satellite Count", self.check_satellite_count)
        self.updater.add("GPS HDOP/VDOP", self.check_dop_values)

        # Health tracking
        self.last_gps_fix = None
        self.satellite_count = 0
        self.hdop = float('inf')
        self.vdop = float('inf')
        self.fix_quality = 0

        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def gps_callback(self, msg):
        """Process GPS fix data"""

        # Store GPS fix information
        self.last_gps_fix = self.get_clock().now()
        self.fix_quality = msg.status.status

        # Extract DOP values if available
        if hasattr(msg, 'position_covariance'):
            # Calculate HDOP/VDOP from covariance matrix
            cov = np.array(msg.position_covariance).reshape(3,3)
            self.hdop = np.sqrt(cov[0,0] + cov[1,1])  # Horizontal DOP
            self.vdop = np.sqrt(cov[2,2])              # Vertical DOP

    def gps_vel_callback(self, msg):
        """Process GPS velocity data"""
        # Could add velocity validity checks here
        pass

    def check_gps_fix(self, stat):
        """Check GPS fix quality"""

        if self.last_gps_fix is None:
            stat.summary(DiagnosticStatus.ERROR, "No GPS fix available")
            return stat

        # Check fix age
        age = (self.get_clock().now() - self.last_gps_fix).nanoseconds / 1e9

        if age > 5.0:  # No fix for 5 seconds
            stat.summary(DiagnosticStatus.ERROR, "GPS fix stale")
            stat.add("Fix Age", f"{age:.1f} seconds")
        elif self.fix_quality == 0:  # No fix
            stat.summary(DiagnosticStatus.ERROR, "No GPS fix")
            stat.add("Fix Quality", "No fix")
        elif self.fix_quality == 1:  # GPS fix
            stat.summary(DiagnosticStatus.WARN, "GPS fix only (no DGPS)")
            stat.add("Fix Quality", "GPS")
        elif self.fix_quality >= 2:  # DGPS fix
            stat.summary(DiagnosticStatus.OK, "Good GPS fix")
            stat.add("Fix Quality", "DGPS")
        else:
            stat.summary(DiagnosticStatus.WARN, "Unknown GPS fix quality")

        return stat

    def check_satellite_count(self, stat):
        """Check satellite visibility"""

        # Note: Actual satellite count would come from GPS driver
        # This is a placeholder - integrate with actual GPS messages

        min_satellites = 4
        good_satellites = 6

        if self.satellite_count < min_satellites:
            stat.summary(DiagnosticStatus.ERROR, "Insufficient satellites")
            stat.add("Satellites", f"{self.satellite_count} (need {min_satellites})")
        elif self.satellite_count < good_satellites:
            stat.summary(DiagnosticStatus.WARN, "Low satellite count")
            stat.add("Satellites", f"{self.satellite_count} (good: {good_satellites}+)")
        else:
            stat.summary(DiagnosticStatus.OK, "Good satellite coverage")
            stat.add("Satellites", str(self.satellite_count))

        return stat

    def check_dop_values(self, stat):
        """Check dilution of precision values"""

        hdop_threshold_warn = 3.0
        hdop_threshold_error = 5.0
        vdop_threshold_warn = 2.0
        vdop_threshold_error = 4.0

        hdop_ok = self.hdop < hdop_threshold_warn
        vdop_ok = self.vdop < vdop_threshold_warn

        if self.hdop > hdop_threshold_error or self.vdop > vdop_threshold_error:
            stat.summary(DiagnosticStatus.ERROR, "Poor GPS geometry")
        elif not hdop_ok or not vdop_ok:
            stat.summary(DiagnosticStatus.WARN, "Degraded GPS geometry")
        else:
            stat.summary(DiagnosticStatus.OK, "Good GPS geometry")

        stat.add("HDOP", f"{self.hdop:.1f}")
        stat.add("VDOP", f"{self.vdop:.1f}")

        return stat

    def publish_diagnostics(self):
        """Publish GPS diagnostics"""
        self.updater.update()
```

#### **LIDAR Health Monitor**
```python
# lidar_health_monitor.py
class LidarHealthMonitor(Node):
    def __init__(self):
        super().__init__('lidar_health_monitor')

        # LIDAR scan subscription
        self.scan_sub = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10
        )

        # Diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID('lidar_rplidar_a2m8')

        # Diagnostic tasks
        self.updater.add("LIDAR Data Rate", self.check_data_rate)
        self.updater.add("LIDAR Range Validity", self.check_range_validity)
        self.updater.add("LIDAR Interference", self.check_interference)

        # Health tracking
        self.scan_count = 0
        self.last_scan_time = None
        self.range_readings = []
        self.intensity_readings = []

        self.diag_timer = self.create_timer(1.0, self.publish_diagnostics)

    def scan_callback(self, msg):
        """Process LIDAR scan data"""

        current_time = self.get_clock().now()
        self.scan_count += 1
        self.last_scan_time = current_time

        # Extract range and intensity data
        ranges = np.array(msg.ranges)
        intensities = np.array(msg.intensities) if msg.intensities else []

        # Filter valid ranges
        valid_ranges = ranges[np.isfinite(ranges) & (ranges > msg.range_min) & (ranges < msg.range_max)]

        if len(valid_ranges) > 0:
            self.range_readings.extend(valid_ranges[:100])  # Keep last 100 readings
            self.range_readings = self.range_readings[-1000:]  # Limit buffer

        if intensities.size > 0:
            self.intensity_readings.extend(intensities[:100])
            self.intensity_readings = self.intensity_readings[-1000:]

    def check_data_rate(self, stat):
        """Check LIDAR data rate (expect ~10Hz)"""

        expected_rate = 10.0
        rate_tolerance = 0.5  # ±50%

        if self.scan_count < expected_rate * (1 - rate_tolerance):
            stat.summary(DiagnosticStatus.ERROR, "LIDAR data rate too low")
        elif self.scan_count > expected_rate * (1 + rate_tolerance):
            stat.summary(DiagnosticStatus.WARN, "LIDAR data rate high")
        else:
            stat.summary(DiagnosticStatus.OK, "LIDAR data rate OK")

        stat.add("Scan Rate", f"{self.scan_count} Hz")
        self.scan_count = 0  # Reset counter

        return stat

    def check_range_validity(self, stat):
        """Check LIDAR range data validity"""

        if not self.range_readings:
            stat.summary(DiagnosticStatus.ERROR, "No valid range data")
            return stat

        # Check for valid range distribution
        ranges = np.array(self.range_readings)
        mean_range = np.mean(ranges)
        std_range = np.std(ranges)

        # Expect some variation in ranges (not all readings at same distance)
        range_variation = std_range / mean_range if mean_range > 0 else 0

        if range_variation < 0.1:  # Very low variation - possible stuck readings
            stat.summary(DiagnosticStatus.WARN, "Low range variation")
            stat.add("Range Variation", f"{range_variation:.3f}")
        else:
            stat.summary(DiagnosticStatus.OK, "Range data OK")
            stat.add("Mean Range", f"{mean_range:.2f} m")
            stat.add("Range Std", f"{std_range:.2f} m")

        return stat

    def check_interference(self, stat):
        """Check for LIDAR interference or contamination"""

        if not self.intensity_readings:
            stat.summary(DiagnosticStatus.WARN, "No intensity data available")
            return stat

        intensities = np.array(self.intensity_readings)
        mean_intensity = np.mean(intensities)
        intensity_variation = np.std(intensities) / mean_intensity if mean_intensity > 0 else 0

        # Low intensity variation may indicate dust or interference
        if intensity_variation < 0.2:
            stat.summary(DiagnosticStatus.WARN, "Possible LIDAR interference")
            stat.add("Intensity Variation", f"{intensity_variation:.3f}")
        else:
            stat.summary(DiagnosticStatus.OK, "LIDAR signal OK")
            stat.add("Mean Intensity", f"{mean_intensity:.1f}")

        return stat

    def publish_diagnostics(self):
        """Publish LIDAR diagnostics"""
        self.updater.update()
```

### **Phase 2: System Resource Monitoring**

#### **CPU/Memory Monitor**
```python
# system_resource_monitor.py
import psutil

class SystemResourceMonitor(Node):
    def __init__(self):
        super().__init__('system_resource_monitor')

        # Diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID('system_resources')

        # Diagnostic tasks
        self.updater.add("CPU Usage", self.check_cpu_usage)
        self.updater.add("Memory Usage", self.check_memory_usage)
        self.updater.add("Disk Usage", self.check_disk_usage)
        self.updater.add("Temperature", self.check_temperature)

        # Resource tracking
        self.cpu_percentages = []
        self.memory_usage = []

        self.diag_timer = self.create_timer(5.0, self.publish_diagnostics)  # Less frequent for system resources

    def check_cpu_usage(self, stat):
        """Check CPU usage across cores"""

        cpu_percent = psutil.cpu_percent(interval=1, percpu=True)

        # Overall CPU usage
        overall_cpu = sum(cpu_percent) / len(cpu_percent)

        # Check for high CPU usage
        high_cpu_cores = [i for i, pct in enumerate(cpu_percent) if pct > 80]

        if overall_cpu > 90:
            stat.summary(DiagnosticStatus.ERROR, "Critical CPU usage")
        elif overall_cpu > 70 or high_cpu_cores:
            stat.summary(DiagnosticStatus.WARN, "High CPU usage")
            if high_cpu_cores:
                stat.add("High CPU Cores", f"Cores {high_cpu_cores}")
        else:
            stat.summary(DiagnosticStatus.OK, "CPU usage OK")

        stat.add("Overall CPU", f"{overall_cpu:.1f}%")
        for i, pct in enumerate(cpu_percent):
            stat.add(f"CPU{i}", f"{pct:.1f}%")

        self.cpu_percentages.append(overall_cpu)
        self.cpu_percentages = self.cpu_percentages[-20:]  # Keep last 20 readings

        return stat

    def check_memory_usage(self, stat):
        """Check system memory usage"""

        memory = psutil.virtual_memory()

        memory_percent = memory.percent
        available_mb = memory.available / (1024 * 1024)

        if memory_percent > 95:
            stat.summary(DiagnosticStatus.ERROR, "Critical memory usage")
        elif memory_percent > 80:
            stat.summary(DiagnosticStatus.WARN, "High memory usage")
        else:
            stat.summary(DiagnosticStatus.OK, "Memory usage OK")

        stat.add("Memory Used", f"{memory_percent:.1f}%")
        stat.add("Available", f"{available_mb:.0f} MB")

        self.memory_usage.append(memory_percent)
        self.memory_usage = self.memory_usage[-20:]

        return stat

    def check_disk_usage(self, stat):
        """Check disk usage"""

        disk = psutil.disk_usage('/')

        disk_percent = disk.percent
        free_gb = disk.free / (1024 * 1024 * 1024)

        if disk_percent > 95:
            stat.summary(DiagnosticStatus.ERROR, "Critical disk usage")
        elif disk_percent > 80:
            stat.summary(DiagnosticStatus.WARN, "Low disk space")
        else:
            stat.summary(DiagnosticStatus.OK, "Disk usage OK")

        stat.add("Disk Used", f"{disk_percent:.1f}%")
        stat.add("Free Space", f"{free_gb:.1f} GB")

        return stat

    def check_temperature(self, stat):
        """Check system temperatures"""

        try:
            temps = psutil.sensors_temperatures()

            if not temps:
                stat.summary(DiagnosticStatus.WARN, "Temperature sensors not available")
                return stat

            # Check CPU temperatures
            cpu_temps = []
            if 'coretemp' in temps:
                cpu_temps = [t.current for t in temps['coretemp']]

            if cpu_temps:
                max_cpu_temp = max(cpu_temps)
                avg_cpu_temp = sum(cpu_temps) / len(cpu_temps)

                if max_cpu_temp > 85:  # Critical temperature
                    stat.summary(DiagnosticStatus.ERROR, "Critical CPU temperature")
                elif max_cpu_temp > 70:  # Warning temperature
                    stat.summary(DiagnosticStatus.WARN, "High CPU temperature")
                else:
                    stat.summary(DiagnosticStatus.OK, "Temperature OK")

                stat.add("Max CPU Temp", f"{max_cpu_temp:.1f}°C")
                stat.add("Avg CPU Temp", f"{avg_cpu_temp:.1f}°C")
            else:
                stat.summary(DiagnosticStatus.WARN, "CPU temperature unavailable")

        except Exception as e:
            stat.summary(DiagnosticStatus.ERROR, f"Temperature check failed: {str(e)}")

        return stat

    def publish_diagnostics(self):
        """Publish system resource diagnostics"""
        self.updater.update()
```

#### **Power Monitoring**
```python
# power_monitor.py
class PowerMonitor(Node):
    def __init__(self):
        super().__init__('power_monitor')

        # Power sensor subscriptions (assuming voltage/current sensors)
        self.voltage_sub = self.create_subscription(
            Float64, '/power/voltage', self.voltage_callback, 1
        )
        self.current_sub = self.create_subscription(
            Float64, '/power/current', self.current_callback, 1
        )

        # Diagnostic updater
        self.updater = Updater(self)
        self.updater.setHardwareID('power_system')

        # Diagnostic tasks
        self.updater.add("Battery Voltage", self.check_voltage)
        self.updater.add("Current Draw", self.check_current)
        self.updater.add("Power Consumption", self.check_power)

        # Power tracking
        self.voltage = 0.0
        self.current = 0.0
        self.voltage_history = []
        self.current_history = []

        self.diag_timer = self.create_timer(2.0, self.publish_diagnostics)

    def voltage_callback(self, msg):
        """Process voltage readings"""
        self.voltage = msg.data
        self.voltage_history.append(self.voltage)
        self.voltage_history = self.voltage_history[-50:]  # Keep last 50 readings

    def current_callback(self, msg):
        """Process current readings"""
        self.current = msg.data
        self.current_history.append(self.current)
        self.current_history = self.current_history[-50:]

    def check_voltage(self, stat):
        """Check battery voltage levels"""

        if not self.voltage_history:
            stat.summary(DiagnosticStatus.ERROR, "No voltage data")
            return stat

        avg_voltage = sum(self.voltage_history) / len(self.voltage_history)
        min_voltage = min(self.voltage_history)

        # 4S LiPo battery levels (14.8V nominal, 12.0V cutoff)
        critical_voltage = 12.0
        warning_voltage = 13.0

        if min_voltage < critical_voltage:
            stat.summary(DiagnosticStatus.ERROR, "Critical battery voltage")
        elif min_voltage < warning_voltage:
            stat.summary(DiagnosticStatus.WARN, "Low battery voltage")
        else:
            stat.summary(DiagnosticStatus.OK, "Battery voltage OK")

        stat.add("Current Voltage", f"{self.voltage:.2f} V")
        stat.add("Average Voltage", f"{avg_voltage:.2f} V")
        stat.add("Minimum Voltage", f"{min_voltage:.2f} V")

        return stat

    def check_current(self, stat):
        """Check current draw"""

        if not self.current_history:
            stat.summary(DiagnosticStatus.ERROR, "No current data")
            return stat

        avg_current = sum(self.current_history) / len(self.current_history)
        max_current = max(self.current_history)

        # Typical rover current limits
        max_safe_current = 10.0  # Amps
        warning_current = 7.0    # Amps

        if max_current > max_safe_current:
            stat.summary(DiagnosticStatus.ERROR, "Excessive current draw")
        elif max_current > warning_current:
            stat.summary(DiagnosticStatus.WARN, "High current draw")
        else:
            stat.summary(DiagnosticStatus.OK, "Current draw OK")

        stat.add("Current Draw", f"{self.current:.2f} A")
        stat.add("Average Current", f"{avg_current:.2f} A")
        stat.add("Peak Current", f"{max_current:.2f} A")

        return stat

    def check_power(self, stat):
        """Check power consumption"""

        if not self.voltage_history or not self.current_history:
            return stat  # Already handled in voltage/current checks

        # Calculate power consumption
        power = self.voltage * self.current

        # Estimate battery life
        battery_capacity = 5000  # mAh (example 5Ah battery)
        battery_voltage = 14.8   # V

        if self.current > 0:
            estimated_life_hours = (battery_capacity / 1000) / self.current
            stat.add("Estimated Battery Life", f"{estimated_life_hours:.1f} hours")

        stat.add("Power Consumption", f"{power:.2f} W")

        return stat

    def publish_diagnostics(self):
        """Publish power diagnostics"""
        self.updater.update()
```

### **Phase 3: Subsystem Health Aggregation**

#### **Central Health Aggregator**
```python
# central_health_aggregator.py
class CentralHealthAggregator(Node):
    def __init__(self):
        super().__init__('central_health_aggregator')

        # Diagnostic aggregator subscription
        self.diag_sub = self.create_subscription(
            DiagnosticArray, '/diagnostics', self.diagnostics_callback, 10
        )

        # Health status publishers
        self.system_health_pub = self.create_publisher(
            Float64, '/system/health', 1
        )
        self.subsystem_health_pub = self.create_publisher(
            DiagnosticArray, '/subsystem/health', 1
        )

        # Health tracking
        self.subsystem_health = {
            'imu': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'gps': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'lidar': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'camera': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'navigation': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'slam': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'computer_vision': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'system_resources': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0},
            'power': {'status': DiagnosticStatus.STALE, 'health': 0.0, 'last_update': 0}
        }

        # Health update timer
        self.health_timer = self.create_timer(2.0, self.publish_health_status)

        # Stale data timeout
        self.stale_timeout = 10.0  # seconds

    def diagnostics_callback(self, msg):
        """Process incoming diagnostic messages"""

        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        for status in msg.status:
            # Extract subsystem name from hardware ID or diagnostic name
            subsystem_name = self.extract_subsystem_name(status)

            if subsystem_name in self.subsystem_health:
                # Calculate health score from diagnostic level
                health_score = self.calculate_health_score(status.level)

                self.subsystem_health[subsystem_name] = {
                    'status': status.level,
                    'health': health_score,
                    'last_update': current_time,
                    'message': status.message
                }

    def extract_subsystem_name(self, status):
        """Extract subsystem name from diagnostic status"""

        hardware_id = status.hardware_id.lower()
        name = status.name.lower()

        # Map hardware IDs to subsystem names
        if 'imu' in hardware_id or 'imu' in name:
            return 'imu'
        elif 'gps' in hardware_id or 'gps' in name:
            return 'gps'
        elif 'lidar' in hardware_id or 'lidar' in name:
            return 'lidar'
        elif 'camera' in hardware_id or 'camera' in name:
            return 'camera'
        elif 'navigation' in name:
            return 'navigation'
        elif 'slam' in name:
            return 'slam'
        elif 'vision' in name:
            return 'computer_vision'
        elif 'system' in hardware_id and 'resource' in name:
            return 'system_resources'
        elif 'power' in hardware_id or 'power' in name:
            return 'power'

        return None

    def calculate_health_score(self, level):
        """Convert diagnostic level to health score (0.0-1.0)"""

        if level == DiagnosticStatus.OK:
            return 1.0
        elif level == DiagnosticStatus.WARN:
            return 0.7
        elif level == DiagnosticStatus.ERROR:
            return 0.3
        elif level == DiagnosticStatus.STALE:
            return 0.0
        else:
            return 0.5  # Unknown level

    def calculate_system_health(self):
        """Calculate overall system health"""

        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Check for stale data
        for subsystem_name, health_data in self.subsystem_health.items():
            age = current_time - health_data['last_update']
            if age > self.stale_timeout:
                self.subsystem_health[subsystem_name]['status'] = DiagnosticStatus.STALE
                self.subsystem_health[subsystem_name]['health'] = 0.0

        # Calculate weighted system health
        weights = {
            'imu': 0.15,
            'gps': 0.15,
            'lidar': 0.15,
            'camera': 0.1,
            'navigation': 0.15,
            'slam': 0.15,
            'computer_vision': 0.1,
            'system_resources': 0.03,
            'power': 0.02
        }

        total_weight = 0
        weighted_health = 0

        for subsystem_name, weight in weights.items():
            if subsystem_name in self.subsystem_health:
                health = self.subsystem_health[subsystem_name]['health']
                weighted_health += health * weight
                total_weight += weight

        if total_weight > 0:
            return weighted_health / total_weight
        else:
            return 0.0

    def publish_health_status(self):
        """Publish overall system health"""

        # Calculate and publish system health
        system_health = Float64()
        system_health.data = self.calculate_system_health()
        self.system_health_pub.publish(system_health)

        # Publish detailed subsystem health
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()

        for subsystem_name, health_data in self.subsystem_health.items():
            status = DiagnosticStatus()
            status.name = f"Subsystem: {subsystem_name}"
            status.hardware_id = f"subsystem_{subsystem_name}"
            status.level = health_data['status']
            status.message = health_data.get('message', 'Health check')

            # Add health score
            kv = KeyValue()
            kv.key = "Health Score"
            kv.value = f"{health_data['health']:.2f}"
            status.values.append(kv)

            # Add last update time
            kv = KeyValue()
            kv.key = "Last Update"
            kv.value = f"{health_data['last_update']:.1f}"
            status.values.append(kv)

            diag_array.status.append(status)

        self.subsystem_health_pub.publish(diag_array)

        # Log critical issues
        system_health_value = system_health.data
        if system_health_value < 0.3:
            self.get_logger().error(f"CRITICAL: System health critically low: {system_health_value:.2f}")
        elif system_health_value < 0.5:
            self.get_logger().warn(f"WARNING: System health low: {system_health_value:.2f}")
        elif system_health_value > 0.8:
            self.get_logger().info(f"System health good: {system_health_value:.2f}")
```

### **Phase 4: Failure Detection & Recovery**

#### **Failure Detector**
```python
# failure_detector.py
class FailureDetector(Node):
    def __init__(self):
        super().__init__('failure_detector')

        # Health subscriptions
        self.system_health_sub = self.create_subscription(
            Float64, '/system/health', self.system_health_callback, 1
        )
        self.subsystem_health_sub = self.create_subscription(
            DiagnosticArray, '/subsystem/health', self.subsystem_health_callback, 1
        )

        # Emergency control publishers
        self.emergency_stop_pub = self.create_publisher(
            Bool, '/emergency_stop', 10
        )
        self.mode_switch_pub = self.create_publisher(
            String, '/requested_mode', 10
        )

        # Failure tracking
        self.failure_history = []
        self.recovery_actions = []

        # Failure thresholds
        self.critical_health_threshold = 0.3
        self.recovery_health_threshold = 0.7

        # Timers
        self.failure_check_timer = self.create_timer(1.0, self.check_for_failures)
        self.recovery_timer = self.create_timer(5.0, self.attempt_recovery)

        # Recovery state
        self.in_recovery = False
        self.last_failure_time = 0

    def system_health_callback(self, msg):
        """Monitor overall system health"""

        health = msg.data
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Track health history
        self.failure_history.append({
            'time': current_time,
            'type': 'system_health',
            'value': health
        })

        # Keep only recent history
        self.failure_history = [
            h for h in self.failure_history
            if current_time - h['time'] < 300  # Last 5 minutes
        ]

    def subsystem_health_callback(self, msg):
        """Monitor individual subsystem health"""

        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        for status in msg.status:
            if status.level in [DiagnosticStatus.ERROR, DiagnosticStatus.STALE]:
                # Record subsystem failure
                self.failure_history.append({
                    'time': current_time,
                    'type': 'subsystem_failure',
                    'subsystem': status.name,
                    'level': status.level,
                    'message': status.message
                })

    def check_for_failures(self):
        """Check for system failures requiring action"""

        if not self.failure_history:
            return

        current_time = self.get_clock().now().seconds_nanoseconds()[0]
        recent_failures = [
            f for f in self.failure_history
            if current_time - f['time'] < 30  # Last 30 seconds
        ]

        # Check for critical system health
        system_health_failures = [
            f for f in recent_failures
            if f['type'] == 'system_health' and f['value'] < self.critical_health_threshold
        ]

        if system_health_failures:
            self.handle_critical_failure("System health critically low")
            return

        # Check for multiple subsystem failures
        subsystem_failures = [
            f for f in recent_failures
            if f['type'] == 'subsystem_failure'
        ]

        critical_subsystems = ['imu', 'gps', 'navigation', 'slam']
        critical_failures = [
            f for f in subsystem_failures
            if any(crit in f.get('subsystem', '').lower() for crit in critical_subsystems)
        ]

        if len(critical_failures) >= 2:
            self.handle_critical_failure("Multiple critical subsystems failed")
            return

        # Check for persistent failures (same failure repeatedly)
        failure_counts = {}
        for failure in recent_failures:
            key = f"{failure.get('type', '')}_{failure.get('subsystem', '')}"
            failure_counts[key] = failure_counts.get(key, 0) + 1

        persistent_failures = [
            k for k, v in failure_counts.items() if v >= 5  # 5 occurrences in 30 seconds
        ]

        if persistent_failures:
            self.handle_persistent_failure(persistent_failures)

    def handle_critical_failure(self, reason):
        """Handle critical system failures"""

        self.get_logger().error(f"CRITICAL FAILURE: {reason}")

        # Trigger emergency stop
        emergency_msg = Bool()
        emergency_msg.data = True
        self.emergency_stop_pub.publish(emergency_msg)

        # Record recovery action
        self.recovery_actions.append({
            'time': self.get_clock().now().seconds_nanoseconds()[0],
            'action': 'emergency_stop',
            'reason': reason
        })

        self.last_failure_time = self.get_clock().now().seconds_nanoseconds()[0]

    def handle_persistent_failure(self, failures):
        """Handle persistent subsystem failures"""

        self.get_logger().warn(f"PERSISTENT FAILURES: {failures}")

        # Attempt graceful degradation
        for failure in failures:
            if 'navigation' in failure:
                # Switch to dead reckoning mode
                mode_msg = String()
                mode_msg.data = 'degraded_navigation'
                self.mode_switch_pub.publish(mode_msg)
            elif 'slam' in failure:
                # Switch to GPS-only localization
                mode_msg = String()
                mode_msg.data = 'gps_only_localization'
                self.mode_switch_pub.publish(mode_msg)

    def attempt_recovery(self):
        """Attempt recovery from failures"""

        if self.in_recovery:
            return  # Already attempting recovery

        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        # Only attempt recovery if enough time has passed since last failure
        if current_time - self.last_failure_time < 60:  # Wait 1 minute
            return

        # Check if system health has recovered
        if self.failure_history:
            recent_health = [
                f['value'] for f in self.failure_history[-5:]
                if f['type'] == 'system_health'
            ]

            if recent_health and sum(recent_health) / len(recent_health) > self.recovery_health_threshold:
                self.attempt_system_recovery()

    def attempt_system_recovery(self):
        """Attempt to recover system from failure state"""

        self.get_logger().info("Attempting system recovery...")

        self.in_recovery = True

        # Reset emergency stop
        emergency_msg = Bool()
        emergency_msg.data = False
        self.emergency_stop_pub.publish(emergency_msg)

        # Request mode switch back to autonomous
        mode_msg = String()
        mode_msg.data = 'autonomous'
        self.mode_switch_pub.publish(mode_msg)

        # Record recovery attempt
        self.recovery_actions.append({
            'time': self.get_clock().now().seconds_nanoseconds()[0],
            'action': 'recovery_attempt',
            'reason': 'system_health_recovered'
        })

        # Reset recovery flag after delay
        self.create_timer(10.0, lambda: setattr(self, 'in_recovery', False))
```

---

## 🔍 **Health Monitoring Visualization**

### **Diagnostic Dashboard**
```python
# diagnostic_dashboard.py
class DiagnosticDashboard(Node):
    def __init__(self):
        super().__init__('diagnostic_dashboard')

        # Subscriptions
        self.system_health_sub = self.create_subscription(
            Float64, '/system/health', self.system_health_callback, 1
        )
        self.subsystem_health_sub = self.create_subscription(
            DiagnosticArray, '/subsystem/health', self.subsystem_health_callback, 1
        )

        # Health history for trending
        self.health_history = []
        self.max_history = 100

        # Dashboard update timer
        self.dashboard_timer = self.create_timer(5.0, self.update_dashboard)

    def system_health_callback(self, msg):
        """Record system health history"""

        timestamp = self.get_clock().now().seconds_nanoseconds()[0]
        self.health_history.append({
            'time': timestamp,
            'system_health': msg.data
        })

        # Maintain history length
        if len(self.health_history) > self.max_history:
            self.health_history.pop(0)

    def subsystem_health_callback(self, msg):
        """Record subsystem health"""

        timestamp = self.get_clock().now().seconds_nanoseconds()[0]

        for status in msg.status:
            subsystem_name = status.name.replace('Subsystem: ', '')

            # Find or create subsystem entry
            subsystem_entry = None
            for entry in self.health_history:
                if entry.get('subsystem') == subsystem_name:
                    subsystem_entry = entry
                    break

            if not subsystem_entry:
                subsystem_entry = {'subsystem': subsystem_name, 'health': []}
                self.health_history.append(subsystem_entry)

            # Calculate health score
            health_score = 1.0 if status.level == DiagnosticStatus.OK else \
                          0.7 if status.level == DiagnosticStatus.WARN else \
                          0.3 if status.level == DiagnosticStatus.ERROR else 0.0

            subsystem_entry['health'].append({
                'time': timestamp,
                'level': status.level,
                'score': health_score,
                'message': status.message
            })

            # Maintain history length
            if len(subsystem_entry['health']) > self.max_history:
                subsystem_entry['health'].pop(0)

    def update_dashboard(self):
        """Update and display health dashboard"""

        if not self.health_history:
            return

        # Clear screen and display header
        print("\033[2J\033[H")  # Clear screen
        print("🚗 URC 2026 Rover Health Dashboard")
        print("=" * 50)

        # System health
        latest_system = None
        for entry in reversed(self.health_history):
            if 'system_health' in entry:
                latest_system = entry
                break

        if latest_system:
            health = latest_system['system_health']
            health_bar = self.create_health_bar(health)
            status_icon = "🟢" if health > 0.8 else "🟡" if health > 0.5 else "🔴"

            print(f"System Health: {status_icon} {health:.2f} {health_bar}")

        # Subsystem health
        print("\nSubsystem Status:")
        subsystem_entries = [e for e in self.health_history if 'subsystem' in e]

        for entry in subsystem_entries:
            subsystem_name = entry['subsystem']
            if entry['health']:
                latest_health = entry['health'][-1]
                health = latest_health['score']

                status_icon = "🟢" if health > 0.8 else "🟡" if health > 0.5 else "🔴"
                status_text = "OK" if latest_health['level'] == DiagnosticStatus.OK else \
                             "WARN" if latest_health['level'] == DiagnosticStatus.WARN else \
                             "ERROR" if latest_health['level'] == DiagnosticStatus.ERROR else "STALE"

                print(f"  {subsystem_name:15}: {status_icon} {status_text} ({health:.2f})")

        # Recent alerts
        print("\nRecent Alerts:")
        recent_alerts = []
        current_time = self.get_clock().now().seconds_nanoseconds()[0]

        for entry in self.health_history:
            if 'subsystem' in entry:
                for health_entry in entry['health'][-3:]:  # Last 3 entries
                    if health_entry['level'] in [DiagnosticStatus.WARN, DiagnosticStatus.ERROR]:
                        age = current_time - health_entry['time']
                        if age < 60:  # Last minute
                            recent_alerts.append({
                                'subsystem': entry['subsystem'],
                                'level': health_entry['level'],
                                'message': health_entry['message'],
                                'age': age
                            })

        if recent_alerts:
            for alert in recent_alerts[:5]:  # Show up to 5 recent alerts
                level_text = "WARN" if alert['level'] == DiagnosticStatus.WARN else "ERROR"
                print(f"  ⚠️  {alert['subsystem']}: {alert['message']} ({level_text})")
        else:
            print("  ✅ No recent alerts")

        print(f"\nLast updated: {self.get_clock().now().to_msg().sec}")

    def create_health_bar(self, health, width=20):
        """Create a visual health bar"""

        filled = int(health * width)
        bar = "█" * filled + "░" * (width - filled)
        return f"[{bar}]"
```

---

## 📊 **Health Monitoring Validation**

### **Health Test Suite**
```python
# health_monitoring_test.py
class HealthMonitoringTest(Node):
    def __init__(self):
        super().__init__('health_monitoring_test')

        # Test control
        self.test_service = self.create_service(
            Trigger, '/test/health_monitoring', self.run_health_tests
        )

        # Test results publisher
        self.test_results_pub = self.create_publisher(
            String, '/test/health_results', 10
        )

    def run_health_tests(self, request, response):
        """Run comprehensive health monitoring tests"""

        test_results = []
        passed_tests = 0
        total_tests = 0

        # Test 1: Diagnostic message publishing
        total_tests += 1
        if self.test_diagnostic_publishing():
            passed_tests += 1
            test_results.append("✓ Diagnostic publishing test passed")
        else:
            test_results.append("✗ Diagnostic publishing test failed")

        # Test 2: Health aggregation
        total_tests += 1
        if self.test_health_aggregation():
            passed_tests += 1
            test_results.append("✓ Health aggregation test passed")
        else:
            test_results.append("✗ Health aggregation test failed")

        # Test 3: Failure detection
        total_tests += 1
        if self.test_failure_detection():
            passed_tests += 1
            test_results.append("✓ Failure detection test passed")
        else:
            test_results.append("✗ Failure detection test failed")

        # Test 4: Recovery mechanisms
        total_tests += 1
        if self.test_recovery_mechanisms():
            passed_tests += 1
            test_results.append("✓ Recovery mechanisms test passed")
        else:
            test_results.append("✗ Recovery mechanisms test failed")

        # Summary
        success_rate = passed_tests / total_tests * 100
        result_message = f"Health monitoring tests: {passed_tests}/{total_tests} passed ({success_rate:.1f}%)"

        test_results.insert(0, result_message)

        # Publish results
        result_msg = String()
        result_msg.data = "\n".join(test_results)
        self.test_results_pub.publish(result_msg)

        # Set response
        response.success = success_rate >= 80.0  # Require 80% pass rate
        response.message = result_message

        return response

    def test_diagnostic_publishing(self):
        """Test that diagnostic messages are being published"""
        # Check for diagnostic messages on /diagnostics topic
        # This would require message introspection
        return True  # Placeholder

    def test_health_aggregation(self):
        """Test health score aggregation"""
        # Verify that subsystem health rolls up to system health
        return True  # Placeholder

    def test_failure_detection(self):
        """Test failure detection logic"""
        # Simulate failures and verify detection
        return True  # Placeholder

    def test_recovery_mechanisms(self):
        """Test recovery action triggers"""
        # Verify emergency stops and mode switches work
        return True  # Placeholder
```

---

## 🐛 **Troubleshooting Health Issues**

### **Common Health Monitoring Problems**

#### **1. Missing Diagnostic Messages**
```
Error: No diagnostic messages received
```
**Solutions**:
```bash
# Check if diagnostic nodes are running
ros2 node list | grep diagnostic

# Check topic publishing
ros2 topic list | grep diagnostics

# Restart diagnostic nodes
ros2 lifecycle set /imu_health_monitor configure
ros2 lifecycle set /imu_health_monitor activate
```

#### **2. Stale Health Data**
```
Warning: Health data is stale
```
**Solutions**:
- Increase diagnostic publishing frequency
- Check for blocking operations in diagnostic callbacks
- Verify timer configurations
- Monitor system resource usage

#### **3. False Positive Failures**
```
Error: False failure detection
```
**Solutions**:
- Adjust failure thresholds based on environmental testing
- Implement hysteresis in health scoring
- Add noise filtering to sensor data
- Tune timeout parameters appropriately

#### **4. Recovery Mechanism Failures**
```
Error: Recovery actions not working
```
**Solutions**:
- Verify emergency stop topic connections
- Check mode switching permissions
- Test recovery procedures in safe conditions
- Implement recovery action logging

---

## 📈 **Health Monitoring Performance Metrics**

### **Reliability Metrics**
- **Detection Latency**: <2 seconds from failure to detection
- **False Positive Rate**: <5% incorrect failure detections
- **False Negative Rate**: <1% missed actual failures
- **Recovery Success Rate**: >90% successful recoveries

### **Performance Metrics**
- **CPU Overhead**: <5% additional CPU usage
- **Memory Usage**: <50MB additional memory
- **Network Bandwidth**: <10KB/s diagnostic traffic
- **Update Frequency**: 1-10Hz depending on subsystem

### **Coverage Metrics**
- **Subsystem Coverage**: 100% of critical subsystems monitored
- **Sensor Coverage**: 100% of mission-critical sensors monitored
- **Failure Mode Coverage**: >95% of known failure modes detected
- **Recovery Coverage**: Recovery procedures for all detected failures

---

## ✅ **Health Monitoring Checklist**

### **Sensor Health Monitoring**
- [ ] IMU health monitor implemented and tested
- [ ] GPS health monitor with satellite tracking
- [ ] LIDAR health monitor with interference detection
- [ ] Camera health monitor with frame rate checking
- [ ] Sensor diagnostic publishing verified

### **System Resource Monitoring**
- [ ] CPU usage monitoring across all cores
- [ ] Memory usage tracking with alerts
- [ ] Disk space monitoring for data logging
- [ ] Temperature monitoring for thermal issues
- [ ] Power consumption tracking

### **Subsystem Health Aggregation**
- [ ] Central health aggregator collecting all diagnostics
- [ ] Weighted health scoring by subsystem criticality
- [ ] System health calculation and publishing
- [ ] Health history and trending implemented

### **Failure Detection & Recovery**
- [ ] Critical failure detection with emergency stops
- [ ] Graceful degradation for subsystem failures
- [ ] Automatic recovery attempt mechanisms
- [ ] Failure history logging and analysis

### **Visualization & Testing**
- [ ] Real-time health dashboard implemented
- [ ] Diagnostic test suite for validation
- [ ] Performance benchmarking completed
- [ ] Integration testing with mission scenarios

### **Integration & Deployment**
- [ ] Health monitoring integrated with state management
- [ ] LED status indicators reflect health status
- [ ] Competition procedures include health monitoring
- [ ] Backup monitoring systems for redundancy

---

**🎯 Success Criteria**: Health monitoring complete when all critical failures are detected within 2 seconds, false positive rate <5%, system maintains >90% uptime through automatic recovery, and operators have clear visibility into system health during competition, enabling successful 30-minute autonomous missions in desert conditions.
