# Camera Calibration Guide - University Rover Challenge 2026

## Overview
Camera calibration is critical for accurate computer vision, SLAM, and autonomous navigation. This guide covers calibration procedures for all camera types in the BOM and recommended upgrades, including data management and system integration.

> **📖 For comprehensive camera calibration concepts, procedures, and system integration, see: [`CameraCalibrationGuide.md`](CameraCalibrationGuide.md)** - This dedicated guide covers fundamental concepts, quality metrics, troubleshooting, and integration with mission tasks.

## Camera Types in System

### 1. RGB-D Camera (Oak-D)
### 2. RGB-D Camera (Intel RealSense D435i)
### 3. Raspberry Pi AI Camera (with 360° Gimbal)
### 4. Additional RGB Cameras (Recommended Upgrades)

## Prerequisites

### Hardware Requirements
- **Calibration Target**: Chessboard pattern (8x6 or 9x7 squares, 20-50mm squares)
- **Calibration Stand**: Stable mount for precise positioning
- **Measuring Tools**: Calipers for accurate target measurements
- **Lighting**: Even, diffuse lighting without harsh shadows

### Software Requirements
```bash
# OpenCV calibration tools
sudo apt-get install python3-opencv
pip install opencv-contrib-python

# Intel RealSense SDK (for D435i)
# Install from Intel website or:
sudo apt-get install librealsense2-dev

# Oak-D DepthAI
pip install depthai

# ROS calibration tools (optional)
sudo apt-get install ros-humble-camera-calibration
```

## 1. Oak-D RGB-D Camera Calibration

### Intrinsic Parameters Calibration

#### Step-by-Step Process:
1. **Setup Environment**
   ```bash
   # Create calibration directory
   mkdir -p ~/calibration/oak_d && cd ~/calibration/oak_d

   # Print or display chessboard pattern
   # Ensure pattern is flat and well-lit
   ```

2. **Capture Calibration Images**
   ```python
   import cv2
   import depthai as dai

   # Initialize Oak-D pipeline
   pipeline = dai.Pipeline()

   # Create RGB camera
   camRgb = pipeline.create(dai.node.ColorCamera)
   camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
   camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)

   # Capture 20-30 images of chessboard at different angles/orientations
   # Ensure full coverage of field of view
   ```

3. **Run OpenCV Calibration**
   ```python
   import cv2
   import numpy as np
   import glob

   # Chessboard dimensions
   CHECKERBOARD = (9, 6)  # 9x6 internal corners
   SQUARE_SIZE = 25  # mm

   # Prepare object points
   objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
   objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
   objp *= SQUARE_SIZE

   # Run calibration
   ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
       objpoints, imgpoints, gray.shape[::-1], None, None)
   ```

4. **Validate Calibration**
   ```python
   # Calculate reprojection error
   mean_error = 0
   for i in range(len(objpoints)):
       imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i],
                                       camera_matrix, dist_coeffs)
       error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) / len(imgpoints2)
       mean_error += error

   print(f"Total reprojection error: {mean_error/len(objpoints)}")
   # Should be < 1.0 pixels for good calibration
   ```

### Depth Calibration (Oak-D Specific)

#### Process:
1. **Capture Depth-Color Pairs**
   ```python
   # Use DepthAI calibration script
   # Capture synchronized RGB and depth frames
   # Ensure proper lighting for depth accuracy
   ```

2. **Depth-RGB Alignment**
   ```python
   # Oak-D has built-in depth-RGB alignment
   # Verify alignment quality
   # Check depth accuracy at various distances
   ```

3. **Depth Validation**
   - Measure known distances
   - Check depth noise levels
   - Validate depth-color pixel correspondence

## 2. Intel RealSense D435i Calibration

### Intrinsic Parameters Calibration

#### Step-by-Step Process:
1. **Install RealSense SDK**
   ```bash
   # Add Intel repository
   sudo apt-key adv --keyserver keyserver.ubuntu.com --recv-key F6E65AC044F831AC80A06380C8B3A55A6F3EFCDE
   sudo add-apt-repository "deb https://librealsense.intel.com/Debian/apt-repo focal main" -u

   # Install SDK
   sudo apt-get install librealsense2-dev librealsense2-utils
   ```

2. **Capture Calibration Images**
   ```python
   import pyrealsense2 as rs
   import cv2
   import numpy as np

   # Configure streams
   pipeline = rs.pipeline()
   config = rs.config()
   config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
   config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)

   # Start streaming and capture images
   pipeline.start(config)
   ```

3. **Run Calibration**
   ```bash
   # Use Intel RealSense calibration tool
   rs-enumerate-devices  # Check device connection
   rs-depth-quality      # Test depth quality
   ```

### Extrinsic Calibration (Depth to Color)

#### Process:
1. **Hardware Sync Check**
   ```python
   # Verify depth-color synchronization
   # Check timestamp alignment
   frames = pipeline.wait_for_frames()
   color_frame = frames.get_color_frame()
   depth_frame = frames.get_depth_frame()
   ```

2. **Extrinsic Parameter Estimation**
   ```python
   # Use stereo calibration if needed
   # RealSense D435i has factory-calibrated extrinsics
   # Verify and document values
   ```

## 3. Raspberry Pi AI Camera Calibration

### Standard RGB Camera Calibration

#### Step-by-Step Process:
1. **Camera Setup**
   ```bash
   # Enable camera in Raspberry Pi
   sudo raspi-config  # Enable camera interface

   # Install picamera library
   pip install picamera[array]
   ```

2. **Capture Calibration Images**
   ```python
   from picamera import PiCamera
   from time import sleep
   import cv2

   camera = PiCamera()
   camera.resolution = (1920, 1080)

   # Capture images at different positions
   for i in range(30):
       camera.capture(f'calibration_image_{i:02d}.jpg')
       sleep(2)  # Allow time for positioning
   ```

3. **Run OpenCV Calibration**
   - Use same OpenCV process as Oak-D
   - Ensure sufficient corner detection
   - Validate reprojection error

### Gimbal Calibration (360° Mast Camera)

#### Additional Steps:
1. **Gimbal Zero Position**
   - Find and record gimbal's mechanical zero
   - Calibrate encoder offsets
   - Test full rotation range

2. **Pan-Tilt Calibration**
   ```python
   # Calibrate pan-tilt angles
   # Record encoder values at known positions
   # Create lookup table for angle-to-encoder conversion
   ```

3. **Dynamic Calibration**
   - Calibrate while moving (if gimbal is on moving rover)
   - Account for vibrations and motion blur
   - Validate calibration stability

## 4. Multi-Camera System Calibration

### Stereo Camera Calibration (If Using Multiple Cameras)

#### Process:
1. **Individual Camera Calibration**
   - Calibrate each camera separately first
   - Ensure all cameras use same chessboard size

2. **Stereo Calibration**
   ```python
   # Use OpenCV stereo calibration
   ret, CM1, dist1, CM2, dist2, R, T, E, F = cv2.stereoCalibrate(
       objpoints, imgpoints_left, imgpoints_right,
       camera_matrix_left, dist_coeffs_left,
       camera_matrix_right, dist_coeffs_right,
       image_size)
   ```

3. **Rectification Setup**
   ```python
   # Compute rectification transforms
   R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
       CM1, dist1, CM2, dist2, image_size, R, T)
   ```

### Camera-to-Robot Coordinate Frame Calibration

#### Hand-Eye Calibration:
1. **Capture Poses**
   - Move robotic arm to known positions
   - Capture camera images at each position
   - Record arm joint angles/positions

2. **Solve Hand-Eye Problem**
   ```python
   # Use OpenCV solvePnP or custom hand-eye calibration
   # Compute transformation between camera and arm base
   ```

## Data Storage and Management

### Calibration Data Structure
```
calibration_data/
├── oak_d/
│   ├── intrinsics.yaml
│   ├── extrinsics.yaml
│   ├── calibration_images/
│   └── validation_results.txt
├── realsense_d435i/
│   ├── camera_matrix.npy
│   ├── dist_coeffs.npy
│   ├── depth_scale.txt
│   └── calibration_log.txt
├── pi_camera/
│   ├── camera_params.json
│   ├── gimbal_calibration.csv
│   └── lens_distortion.npy
└── system/
    ├── camera_poses.yaml      # Relative camera positions
    ├── timestamp_offsets.txt  # Synchronization data
    └── calibration_report.pdf # Summary report
```

### YAML Format Example (Oak-D)
```yaml
%YAML:1.0
---
camera_matrix: !!opencv-matrix
  rows: 3
  cols: 3
  dt: d
  data: [ fx, 0, cx,
          0, fy, cy,
          0, 0, 1 ]
distortion_coefficients: !!opencv-matrix
  rows: 1
  cols: 5
  dt: d
  data: [ k1, k2, p1, p2, k3 ]
image_width: 1920
image_height: 1080
calibration_date: "2025-10-16"
reprojection_error: 0.85
```

### Version Control and Tracking
```bash
# Git-based calibration tracking
cd calibration_data
git init
git add .
git commit -m "Initial calibration for Oak-D camera"

# Tag important calibration versions
git tag -a "competition_ready" -m "Calibration validated for competition"
```

## Validation and Maintenance

### Calibration Quality Metrics
- **Reprojection Error**: < 1.0 pixels (good), < 0.5 pixels (excellent)
- **Depth Accuracy**: Within 2% of measured distance
- **Temporal Stability**: Calibration valid for >30 days
- **Thermal Stability**: <5% parameter change across temperature range

### Regular Validation Checks
```python
# Automated validation script
def validate_calibration(camera_matrix, dist_coeffs, test_images):
    """Validate calibration quality on test images"""
    total_error = 0
    for img in test_images:
        # Detect chessboard corners
        # Calculate reprojection error
        # Accumulate statistics
    return total_error / len(test_images)
```

### Recalibration Triggers
- **Performance Degradation**: Detection accuracy drops >10%
- **Physical Changes**: Camera replacement or mounting changes
- **Environmental Changes**: Extreme temperature exposure
- **Time-based**: Annual recalibration schedule

## Integration with Autonomy System

### ROS Integration
```bash
# Create camera calibration files for ROS
rosrun camera_calibration cameracalibrator.py
# Save calibration data to ROS parameter server
rosparam set /camera/camera_info [calibration_data]
```

### Configuration Management
```python
# Load calibration data in autonomy code
import yaml

def load_camera_calibration(camera_name):
    with open(f'calibration_data/{camera_name}/intrinsics.yaml') as f:
        calib = yaml.safe_load(f)
    return calib['camera_matrix']['data'], calib['distortion_coefficients']['data']
```

### Runtime Calibration Updates
- **Online Calibration**: Continuous parameter refinement during operation
- **Adaptive Parameters**: Temperature-compensated calibration
- **Multi-session Learning**: Combine data from multiple calibration sessions

## Troubleshooting Common Issues

### Poor Corner Detection
- **Cause**: Insufficient lighting or motion blur
- **Solution**: Improve lighting, stabilize camera, use larger chessboard

### High Reprojection Error
- **Cause**: Poor image quality or insufficient coverage
- **Solution**: Capture more images, ensure even angle distribution

### Depth Alignment Issues
- **Cause**: Factory calibration drift or temperature effects
- **Solution**: Recalibrate extrinsics, check for hardware damage

### Temporal Drift
- **Cause**: Mechanical stress or thermal expansion
- **Solution**: Implement online recalibration, improve mounting stability

## Best Practices

### Calibration Environment
- **Controlled Lighting**: Diffuse, even illumination
- **Stable Setup**: Vibration-free mounting
- **Temperature Control**: Consistent ambient temperature
- **Clean Optics**: Dust-free lenses and sensors

### Data Collection
- **Comprehensive Coverage**: Images from all parts of field of view
- **Angle Variation**: ±30° pitch/yaw rotations
- **Distance Variation**: 0.5m to 3m from target
- **Multiple Sessions**: 3-5 calibration runs for averaging

### Quality Assurance
- **Automated Validation**: Script-based quality checks
- **Human Verification**: Visual inspection of results
- **Performance Metrics**: Track calibration KPIs over time
- **Documentation**: Detailed logs of calibration conditions

## Maintenance Schedule

### Daily Checks
- Visual inspection for dust/debris
- Quick depth accuracy test
- Basic functionality verification

### Weekly Maintenance
- Clean camera lenses
- Check mounting stability
- Validate calibration parameters

### Monthly Calibration
- Full recalibration procedure
- Performance benchmarking
- Update calibration database

### Annual Overhaul
- Complete teardown and inspection
- Factory-level recalibration
- Component replacement as needed

---

*Proper camera calibration is essential for accurate computer vision and SLAM performance. Follow this guide meticulously and maintain detailed records of all calibration sessions.*
