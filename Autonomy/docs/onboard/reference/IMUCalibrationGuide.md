# IMU Calibration Guide - URC 2026 Rover

## Overview

IMU (Inertial Measurement Unit) calibration is **critical for accurate navigation** in the URC 2026 rover system. Poor IMU calibration can cause significant drift in orientation estimation, leading to navigation errors that violate the 3m accuracy requirements for GNSS-only waypoints and 2m accuracy for AR-tagged posts.

This guide covers comprehensive calibration procedures for the Bosch BMI088 IMU, including accelerometer bias/scale calibration, gyroscope bias calibration, and magnetometer calibration (if equipped).

## 📊 **Why IMU Calibration Matters**

### **Mission Impact**
- **Navigation Accuracy**: IMU provides orientation data for dead reckoning when GPS is unavailable
- **SLAM Performance**: IMU data compensates for motion blur in LIDAR scans
- **Sensor Fusion**: IMU data improves GPS accuracy through Kalman filtering
- **Autonomous Typing**: IMU helps maintain orientation reference for arm manipulation

### **Calibration Consequences**
```
Uncalibrated IMU → Orientation Drift → Navigation Errors → Mission Failure
```

- **Heading Errors**: 5-15° drift over 5 minutes without proper calibration
- **Tilt Errors**: 2-5° pitch/roll errors affecting terrain assessment
- **Acceleration Bias**: False acceleration readings causing position drift
- **Gyroscope Bias**: Constant rotation bias accumulating over time

---

## 🔧 **IMU Hardware Overview**

### **Bosch BMI088 Specifications**
- **Accelerometer**: ±2g to ±16g range, 16-bit resolution
- **Gyroscope**: ±125°/s to ±2000°/s range, 16-bit resolution
- **Interface**: SPI (primary), I2C (secondary)
- **Sample Rate**: Up to 1kHz
- **Power**: 2.7-3.6V, <1mA active current

### **ROS 2 Integration**
```bash
# Install IMU drivers and tools
sudo apt install ros-humble-imu-tools
pip install imu-calibration  # For offline calibration
```

---

## 📋 **Calibration Prerequisites**

### **Hardware Requirements**
- **Calibration Surface**: Level, stable platform (marble table or precision level)
- **Mounting**: IMU securely mounted in final rover position
- **Power Supply**: Stable 3.3V supply during calibration
- **Temperature**: Controlled environment (20-25°C)

### **Software Requirements**
```bash
# Required packages
sudo apt install ros-humble-imu-tools
sudo apt install python3-numpy python3-scipy python3-matplotlib
pip install imu-calibration tqdm

# Optional: Advanced calibration tools
pip install allantools  # Allan variance analysis
```

### **Data Collection Setup**
```bash
# Create calibration workspace
mkdir -p ~/imu_calibration && cd ~/imu_calibration

# Launch IMU data collection
ros2 launch imu_tools imu.launch.py \
  imu_name:=bmi088 \
  port:=/dev/ttyIMU \
  baudrate:=115200
```

---

## 🧭 **Calibration Procedures**

### **Phase 1: Accelerometer Calibration**

#### **Theory: Accelerometer Calibration**
Accelerometers measure specific force (gravity + acceleration). When stationary, they should measure gravity (9.81 m/s² downward).

**Calibration Parameters:**
- **Bias (Offset)**: Zero-point error
- **Scale Factor**: Sensitivity error per axis
- **Cross-axis Coupling**: Misalignment between axes

#### **Static Calibration Procedure**
1. **Setup IMU Level**
   ```bash
   # Mount IMU perfectly level using precision level
   # Verify all axes aligned with gravity
   ```

2. **Collect Static Data (6 Positions)**
   ```python
   # collect_static_data.py
   import rclpy
   from sensor_msgs.msg import Imu
   import numpy as np

   class IMUCalibrator:
       def __init__(self):
           self.data = {'x+': [], 'x-': [], 'y+': [], 'y-': [], 'z+': [], 'z-': []}
           self.current_position = 'x+'

       def collect_position_data(self, position, duration=30):
           """Collect 30 seconds of data per position"""
           print(f"Collecting data for {position} position...")
           # Rotate IMU to each orientation and collect data
   ```

3. **Position Sequence**
   - **X+**: X-axis pointing up (against gravity)
   - **X-**: X-axis pointing down (with gravity)
   - **Y+**: Y-axis pointing up
   - **Y-**: Y-axis pointing down
   - **Z+**: Z-axis pointing up (normal orientation)
   - **Z-**: Z-axis pointing down

4. **Calculate Calibration Parameters**
   ```python
   # accelerometer_calibration.py
   import numpy as np
   from scipy.optimize import minimize

   def calibrate_accelerometer(data_dict, g=9.80665):
       """
       Calculate accelerometer calibration matrix

       Returns:
       - bias: 3x1 offset vector
       - scale: 3x3 diagonal scale matrix
       - cross_coupling: 3x3 cross-axis coupling matrix
       """

       # Expected readings for each position
       expected = {
           'x+': np.array([-g, 0, 0]),
           'x-': np.array([g, 0, 0]),
           'y+': np.array([0, -g, 0]),
           'y-': np.array([0, g, 0]),
           'z+': np.array([0, 0, -g]),
           'z-': np.array([0, 0, g])
       }

       # Extract measured data
       measurements = []
       references = []

       for pos, expected_vec in expected.items():
           if pos in data_dict and data_dict[pos]:
               measurements.append(np.mean(data_dict[pos], axis=0))
               references.append(expected_vec)

       measurements = np.array(measurements)
       references = np.array(references)

       # Solve for calibration parameters using least squares
       # A * params = b, where params = [scale_x, scale_y, scale_z, bias_x, bias_y, bias_z]
       A = np.zeros((len(measurements)*3, 9))
       b = np.zeros(len(measurements)*3)

       for i, (meas, ref) in enumerate(zip(measurements, references)):
           for j in range(3):
               A[i*3 + j, j] = meas[j]          # scale factor
               A[i*3 + j, j+3] = 1              # bias
               A[i*3 + j, j+6] = meas[(j+1)%3]  # cross-coupling (simplified)
               b[i*3 + j] = ref[j]

       # Solve calibration parameters
       params, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)

       # Extract parameters
       scale_factors = params[:3]
       biases = params[3:6]
       cross_coupling = params[6:]

       return scale_factors, biases, cross_coupling
   ```

### **Phase 2: Gyroscope Calibration**

#### **Theory: Gyroscope Calibration**
Gyroscopes measure angular velocity. When stationary, they should read zero.

**Calibration Parameters:**
- **Bias**: Constant offset when stationary
- **Scale Factor**: Sensitivity error
- **Temperature Compensation**: Bias variation with temperature

#### **Stationary Calibration Procedure**
1. **Setup IMU Stationary**
   ```bash
   # Ensure IMU is completely stationary
   # No vibrations, thermal changes
   ```

2. **Collect Stationary Data**
   ```python
   # collect_gyro_bias.py
   import rclpy
   from sensor_msgs.msg import Imu
   import numpy as np

   def collect_gyro_bias(duration=300):  # 5 minutes
       """Collect gyroscope bias data"""
       gyro_data = []

       def imu_callback(msg):
           gyro_data.append([
               msg.angular_velocity.x,
               msg.angular_velocity.y,
               msg.angular_velocity.z
           ])

       # Collect data for specified duration
       # Calculate bias as mean of stationary readings

       bias = np.mean(gyro_data, axis=0)
       bias_std = np.std(gyro_data, axis=0)

       return bias, bias_std
   ```

3. **Temperature Compensation (Optional)**
   ```python
   def temperature_compensation(gyro_data, temp_data):
       """Model bias variation with temperature"""
       from scipy.stats import linregress

       bias_temp_corr = {}
       for axis in range(3):
           slope, intercept, r_value, p_value, std_err = linregress(
               temp_data, gyro_data[:, axis]
           )
           bias_temp_corr[f'axis_{axis}'] = {
               'slope': slope,
               'intercept': intercept,
               'correlation': r_value
           }

       return bias_temp_corr
   ```

### **Phase 3: Magnetometer Calibration (Optional)**

#### **Hard Iron and Soft Iron Calibration**
```python
# magnetometer_calibration.py
import numpy as np
from scipy.optimize import minimize

def calibrate_magnetometer(mag_data):
    """
    Calculate magnetometer calibration parameters

    Hard iron: Constant offset (permanent magnets)
    Soft iron: Scale and rotation (ferrous materials)
    """

    def ellipsoid_error(params, points):
        """Minimize ellipsoid fit error"""
        cx, cy, cz, rx, ry, rz, sx, sy, sz = params

        # Ellipsoid equation: (x-cx)^2/rx^2 + (y-cy)^2/ry^2 + (z-cz)^2/rz^2 = 1
        errors = []
        for point in points:
            x, y, z = point
            error = ((x-cx)/rx)**2 + ((y-cy)/ry)**2 + ((z-cz)/rz)**2 - 1
            errors.append(error**2)

        return np.sum(errors)

    # Initial guess: center at mean, radius based on std
    center = np.mean(mag_data, axis=0)
    radius = np.std(mag_data, axis=0)
    scale = np.ones(3)

    initial_params = np.concatenate([center, radius, scale])

    # Optimize ellipsoid fit
    result = minimize(
        ellipsoid_error,
        initial_params,
        args=(mag_data,),
        method='L-BFGS-B'
    )

    cx, cy, cz, rx, ry, rz, sx, sy, sz = result.x

    return {
        'hard_iron': np.array([cx, cy, cz]),
        'soft_iron': np.array([[sx, 0, 0], [0, sy, 0], [0, 0, sz]]),
        'scale': np.array([rx, ry, rz])
    }
```

---

## 🔧 **ROS 2 IMU Tools Integration**

### **imu_filter_madgwick Configuration**
```yaml
# imu_filter_config.yaml
imu_filter_madgwick:
  ros__parameters:
    # IMU data
    imu_topic: "/imu/data_raw"
    output_topic: "/imu/data"

    # Filter parameters
    gain: 0.1  # Filter gain
    zeta: 0.0  # Gyro bias estimation

    # Coordinate frame
    world_frame: "enu"  # East-North-Up
    imu_frame: "imu_link"

    # Processing
    publish_tf: true
    publish_debug_topics: false
```

### **imu_transformer Setup**
```yaml
# imu_transformer_config.yaml
imu_transformer:
  ros__parameters:
    # Input/output topics
    input_topic: "/imu/data"
    output_topic: "/imu/data_transformed"

    # Transform configuration
    target_frame: "base_link"
    source_frame: "imu_link"

    # Calibration parameters (from our calibration)
    accelerometer_bias: [0.01, -0.02, 0.005]
    gyroscope_bias: [0.001, -0.001, 0.002]
    accelerometer_scale: [1.01, 0.99, 1.005]
```

---

## 📊 **Calibration Validation**

### **Static Validation Tests**
```python
# validate_calibration.py
import rclpy
from sensor_msgs.msg import Imu
import numpy as np

class CalibrationValidator:
    def __init__(self):
        self.accel_data = []
        self.gyro_data = []

    def validate_static_performance(self):
        """Validate IMU performance when stationary"""

        # Collect 1 minute of data
        accel_bias = np.mean(self.accel_data, axis=0)
        gyro_bias = np.mean(self.gyro_data, axis=0)

        accel_std = np.std(self.accel_data, axis=0)
        gyro_std = np.std(self.gyro_data, axis=0)

        # Check specifications
        print(f"Accelerometer bias: {accel_bias} m/s² (should be ~[0,0,-9.81])")
        print(f"Gyroscope bias: {gyro_bias} rad/s (should be ~[0,0,0])")
        print(f"Accelerometer noise: {accel_std} m/s²")
        print(f"Gyroscope noise: {gyro_std} rad/s")

        # Performance criteria
        accel_bias_limit = 0.1  # m/s²
        gyro_bias_limit = 0.01  # rad/s

        accel_ok = np.all(np.abs(accel_bias - np.array([0,0,-9.81])) < accel_bias_limit)
        gyro_ok = np.all(np.abs(gyro_bias) < gyro_bias_limit)

        return accel_ok and gyro_ok
```

### **Dynamic Validation Tests**
```python
def validate_dynamic_response():
    """Validate IMU response to known motions"""

    # Rotate 90° around each axis
    # Check if integrated angle matches expected rotation
    # Validate accelerometer during linear motion

    pass  # Implementation for dynamic testing
```

---

## 🚀 **Integration with Navigation Stack**

### **EKF Configuration**
```yaml
# ekf_config.yaml
ekf_filter_node:
  ros__parameters:
    frequency: 30.0
    sensor_timeout: 0.1
    two_d_mode: false

    # IMU configuration
    imu0: /imu/data
    imu0_config: [false, false, false,
                  true,  true,  true,
                  false, false, false,
                  true,  true,  true,
                  true,  true,  true]
    imu0_differential: false
    imu0_relative: false

    # Process noise
    process_noise_covariance: [0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0.05, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0.03, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0.06, 0, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0.025, 0, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0.04, 0, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.02, 0, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.01, 0,
                               0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.015]
```

---

## 🐛 **Troubleshooting & Common Issues**

### **Common Calibration Problems**
1. **Vibrations During Static Calibration**
   - **Symptom**: High standard deviation in measurements
   - **Solution**: Use vibration isolation, collect longer datasets

2. **Temperature Drift**
   - **Symptom**: Calibration parameters change with temperature
   - **Solution**: Calibrate at operating temperature, implement temperature compensation

3. **Mounting Errors**
   - **Symptom**: Inconsistent calibration results
   - **Solution**: Verify IMU mounting alignment, use precision fixtures

4. **Power Supply Noise**
   - **Symptom**: Erratic readings, high noise floor
   - **Solution**: Use clean power supply, add decoupling capacitors

### **Performance Monitoring**
```python
# imu_monitor.py
class IMUHealthMonitor:
    def __init__(self):
        self.bias_history = []
        self.calibration_date = None

    def check_calibration_health(self, current_bias):
        """Monitor IMU calibration stability"""
        self.bias_history.append(current_bias)

        # Check if bias has drifted significantly
        if len(self.bias_history) > 10:
            recent_bias = np.mean(self.bias_history[-10:], axis=0)
            initial_bias = np.mean(self.bias_history[:10], axis=0)

            drift = np.abs(recent_bias - initial_bias)
            max_allowed_drift = np.array([0.05, 0.05, 0.05])  # rad/s

            if np.any(drift > max_allowed_drift):
                print("WARNING: IMU calibration drift detected!")
                return False

        return True
```

---

## 📈 **Performance Metrics & Acceptance Criteria**

### **Calibration Quality Metrics**
- **Accelerometer Bias**: <0.1 m/s² from expected gravity vector
- **Gyroscope Bias**: <0.01 rad/s when stationary
- **Scale Factor Accuracy**: <1% error per axis
- **Cross-axis Coupling**: <1% coupling between axes

### **Runtime Performance Metrics**
- **Orientation Accuracy**: <1° RMS over 5 minutes
- **Drift Rate**: <0.1°/minute when stationary
- **Update Rate**: 100Hz minimum
- **Latency**: <10ms from sensor to filtered output

---

## 📚 **Additional Resources**

### **Reference Documentation**
- [Bosch BMI088 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmi088-ds001.pdf)
- [ROS 2 IMU Tools](https://github.com/CCNYRoboticsLab/imu_tools)
- [imu_filter_madgwick](https://github.com/CCNYRoboticsLab/imu_tools/tree/ros2/imu_filter_madgwick)

### **Advanced Topics**
- **Allan Variance Analysis**: Long-term stability characterization
- **Kalman Filter Tuning**: Optimal sensor fusion parameters
- **GPS/IMU Integration**: RTK-GPS with IMU for precise navigation

---

## ✅ **Calibration Checklist**

### **Pre-Calibration**
- [ ] IMU securely mounted in final position
- [ ] Power supply stable and noise-free
- [ ] Environment temperature controlled
- [ ] Calibration tools installed and tested

### **Accelerometer Calibration**
- [ ] Static data collected for all 6 positions
- [ ] Bias and scale parameters calculated
- [ ] Cross-axis coupling characterized
- [ ] Validation tests passed

### **Gyroscope Calibration**
- [ ] Stationary bias measured
- [ ] Temperature compensation (if required)
- [ ] Noise characteristics analyzed
- [ ] Validation tests passed

### **System Integration**
- [ ] ROS 2 configuration updated
- [ ] EKF parameters set
- [ ] Navigation stack tested
- [ ] Performance metrics verified

### **Documentation**
- [ ] Calibration parameters recorded
- [ ] Validation results documented
- [ ] Procedures archived for future recalibration

---

**🎯 Success Criteria**: IMU calibration complete when orientation drift <0.1°/minute and accelerometer bias <0.1 m/s², enabling the required navigation accuracy for URC 2026 mission success.
