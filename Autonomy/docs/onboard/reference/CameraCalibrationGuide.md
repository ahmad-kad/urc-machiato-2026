# 📷 **Camera Calibration Guide: URC 2026 Rover**

## Overview

Camera calibration is a critical process that establishes the mathematical relationship between 3D world coordinates and 2D image coordinates. For the URC 2026 rover, proper camera calibration ensures accurate computer vision performance across all mission tasks.

> **🔧 For hardware-specific calibration procedures** for Oak-D, Intel RealSense, and Raspberry Pi cameras, see: [`CalibrationGuide.md`](CalibrationGuide.md) - This guide contains camera-specific setup, intrinsic/extrinsic calibration, and depth calibration procedures.

> **🛠️ For ready-to-use calibration tools**, see: [`calibration/`](../../calibration/) - Complete toolkit with ChArUco board generation, camera calibration processing, validation, and testing scripts.

---

## 🎯 **Why Camera Calibration is Critical**

### **Mission Impact**
Camera calibration directly affects the success of several URC 2026 mission components:

#### **1.f Navigation (Autonomous Navigation)**
- **AR Tag Detection**: Precise pose estimation of AR-tagged posts (±2m accuracy requirement)
- **Visual Markers**: 3-sided markers with 20 x 20 cm faces, 2.5 cm cells (ArUco 4x4_50)
- **Visual Odometry**: Fallback positioning when GPS is unavailable
- **Obstacle Detection**: Accurate distance measurements for safe navigation

#### **1.e Equipment Servicing (Autonomous Typing)**
- **Keyboard Detection**: Precise localization using 2 x 2 cm ArUco markers at corners
- **USB Slot Detection**: Precise localization using 1 x 1 cm ArUco markers at corners
- **Arm Coordination**: Accurate hand-eye calibration for robotic manipulation
- **Character Recognition**: Reliable OCR performance

#### **1.d Delivery (Science Tasks)**
- **Sample Collection**: Precise positioning for scientific instruments
- **Terrain Analysis**: Accurate depth perception for traversability assessment

### **Why ChArUco Boards?**
ChArUco boards combine ArUco markers with chessboard corners for superior calibration:

- **Robust Detection**: ArUco markers provide unique IDs, preventing detection confusion
- **Better Pose Estimation**: Chessboard corners enhanced by marker tracking
- **Motion Tolerance**: Works better with camera movement and varying distances
- **Self-Identifying**: No need to specify board size during detection

---

## 🔍 **ChArUco vs ArUco: Understanding the Difference**

### **ChArUco Boards (For Camera Calibration)**
- **Purpose**: Camera intrinsic calibration and hand-eye calibration
- **Location**: `calibration/charuco_board/`
- **Tools**: `generate_chessboard.py`, `camera_calibrator.py`, `hand_eye_calibration.py`
- **Characteristics**: Hybrid checkerboard + ArUco markers, fixed patterns for precise calibration
- **Usage**: One-time calibration procedure, stored as camera parameters

### **ArUco Tags (For Environment Setup)**
- **Purpose**: Navigation landmarks, equipment markers, autonomous typing targets
- **Location**: `calibration/aruco_tags/`
- **Tools**: `generate_aruco_tag.py`, `aruco_sheets.py`
- **Characteristics**: Individual or grouped markers with unique IDs for identification
- **Usage**: Deployed throughout competition environment for real-time localization

### **Key Distinction**
```
ChArUco Boards → Camera Calibration (Intrinsics + Hand-Eye)
ArUco Tags → Environment Markers (Navigation + Equipment Detection)
```

**Never use ArUco tags for camera calibration!** They lack the precision patterns needed for accurate intrinsic parameter estimation.

### **Technical Consequences of Poor Calibration**
```
Uncalibrated Camera → Distorted Measurements → Navigation Errors → Mission Failure
```

- **Position Errors**: 10-50cm errors in depth perception
- **Angle Errors**: 2-5° errors in orientation estimation
- **Scale Errors**: Inconsistent object sizing across the image
- **Motion Blur**: Incorrect exposure compensation

---

## 🔑 **Key Concepts**

### **Intrinsic Parameters (Internal Camera Properties)**
```python
# Camera Matrix (3x3)
K = [[fx, 0, cx],
     [0, fy, cy],
     [0,  0,  1]]

# Where:
# fx, fy: Focal length in pixels (horizontal/vertical)
# cx, cy: Principal point coordinates (image center)
```

### **Distortion Coefficients (Lens Imperfections)**
```python
# Radial distortion (barrel/pincushion effect)
k1, k2, k3 = radial_distortion_coefficients

# Tangential distortion (lens decentering)
p1, p2 = tangential_distortion_coefficients
```

### **Extrinsic Parameters (Camera Position/Orientation)**
```python
# Rotation matrix (3x3) and translation vector (3x1)
[R|t] = camera_pose_in_world_coordinates
```

### **Calibration Target Types**
- **ChArUco Boards**: **RECOMMENDED** - AR tag + chessboard hybrid for superior pose estimation
- **Chessboard Pattern**: Traditional method, provides corner detection
- **Circle Grids**: Alternative to chessboard with different detection algorithms

---

## 🛠️ **Required Materials & Setup**

### **Hardware Requirements**
- **Camera System**: Intel D435i RGB-D camera (competition hardware)
- **Calibration Target**: ChArUco board (4x6 squares, 3x5 markers recommended for US Letter paper)
- **Mounting Hardware**: Rigid camera mount for stable positioning
- **Tripod/Light Stand**: For target positioning during calibration
- **Measuring Tools**: Calipers for precise square/marker size measurement

### **Software Requirements**
- **ROS2 Humble**: Core framework
- **OpenCV 4.0+**: Computer vision library with ChArUco and calibration tools
- **camera_calibration Package**: ROS2 camera calibration tools
- **Python Libraries**: numpy, cv2, yaml for configuration

### **ChArUco Board Generation Library**
**OpenCV (cv2.aruco.CharucoBoard)** - The standard library for generating and detecting ChArUco boards:

- **Features**: Automatic board generation, robust detection, pose estimation
- **Advantages**: Industry standard, well-documented, actively maintained
- **Integration**: Works seamlessly with ROS2 and computer vision pipelines
- **Alternatives**: Custom implementations possible but OpenCV is recommended

### **Environment Setup**
```bash
# Install ROS2 camera calibration package
sudo apt-get install ros-humble-camera-calibration

# Verify OpenCV installation
python3 -c "import cv2; print('OpenCV version:', cv2.__version__)"
```

---

## 📋 **Calibration Procedure**

### **Phase 1: Pre-Calibration Setup**

#### **1. Target Preparation**
```bash
# Create ChArUco calibration target
# Use the provided calibration toolkit
python calibration/charuco_board/generate_chessboard.py --squares 4x6 --markers 3x5 --square-size 20 --marker-size 16 --output charuco_target.pdf

# Print on high-quality matte paper
# Measure square/marker sizes precisely
# Mount on rigid, flat surface for stability
```

#### **2. Camera Configuration**
```python
# camera_config.yaml
camera_info:
  width: 640
  height: 480
  distortion_model: "plumb_bob"
  D: [0.0, 0.0, 0.0, 0.0, 0.0]  # Initial distortion coefficients
  K: [600.0, 0.0, 320.0,  # fx, skew, cx
      0.0, 600.0, 240.0,  # skew, fy, cy
      0.0, 0.0, 1.0]       # 0, 0, 1
  R: [1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0]
  P: [600.0, 0.0, 320.0, 0.0,
      0.0, 600.0, 240.0, 0.0,
      0.0, 0.0, 1.0, 0.0]
```

#### **3. Camera Mounting**
```bash
# Mount camera rigidly to prevent vibration
# Ensure camera is level and square to target
# Mark camera position for consistent placement
# Use stable mounting surface
```

### **Phase 2: Data Collection**

#### **1. Launch Camera Node**
```bash
# Start camera driver
ros2 launch realsense2_camera rs_launch.py

# Verify camera publishing
ros2 topic list | grep camera
ros2 topic hz /camera/color/image_raw
```

#### **2. Launch Calibration Tool**
```bash
# Start ROS2 camera calibration
ros2 run camera_calibration cameracalibrator \
  --size 8x6 \
  --square 0.025 \
  image:=/camera/color/image_raw \
  camera:=/camera
```

#### **3. Capture Calibration Images**
```
Movement Requirements:
• 40-50 images minimum
• Target should cover entire image frame
• Vary distance: 0.5m to 2.0m from camera
• Vary angle: ±30° rotation in all axes
• Vary position: Move target across entire field of view
• Include all orientations: portrait/landscape rotations
```

#### **4. Quality Indicators**
```
✅ Good Capture Indicators:
• ChArUco corners and markers clearly detected
• ArUco markers show unique IDs (not just corners)
• Target covers significant portion of image
• Even lighting without glare on markers

❌ Poor Capture Indicators:
• Blurry images or motion blur
• Insufficient target movement/variation
• Poor lighting or glare on ArUco markers
• Target too small or too close to image edges
```

### **Phase 3: Parameter Estimation**

#### **1. Run Calibration Algorithm**
```python
# OpenCV ChArUco calibration process
import cv2
import numpy as np

# Create ChArUco board
board = cv2.aruco.CharucoBoard((4, 6), square_size, marker_size, cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50))

# Arrays to store detected corners and IDs
all_corners = []
all_ids = []

# Detect ChArUco board (automatic detection)
charuco_corners, charuco_ids, marker_corners, marker_ids = cv2.aruco.ArucoDetector(board.dictionary).detectBoard(image)

# Calibrate camera using ChArUco
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
    all_corners, all_ids, board, image_size, None, None)
```

#### **2. Validate Results**
```python
# Check reprojection error
mean_error = 0
for i in range(len(objpoints)):
    imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
    error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2)/len(imgpoints2)
    mean_error += error

print(f"Total reprojection error: {mean_error/len(objpoints)}")
```

#### **3. Save Calibration Data**
```yaml
# camera_calibration.yaml
image_width: 640
image_height: 480
camera_name: rgb_camera
camera_matrix:
  rows: 3
  cols: 3
  data: [615.0, 0.0, 310.0, 0.0, 615.0, 236.0, 0.0, 0.0, 1.0]
distortion_coefficients:
  rows: 1
  cols: 5
  data: [-0.25, 0.12, 0.0, 0.0, -0.05]
```

### **Phase 4: Validation & Testing**

#### **1. Undistortion Test**
```python
# Test distortion correction
img = cv2.imread('test_image.jpg')
h, w = img.shape[:2]

# Get optimal camera matrix
newcameramtx, roi = cv2.getOptimalNewCameraMatrix(mtx, dist, (w,h), 1, (w,h))

# Undistort
undistorted = cv2.undistort(img, mtx, dist, None, newcameramtx)

# Display comparison
cv2.imshow('Original', img)
cv2.imshow('Undistorted', undistorted)
```

#### **2. Pose Estimation Test**
```python
# Test with known target position
# Measure actual vs estimated distance
# Verify angular accuracy
# Check consistency across different viewpoints
```

---

## 🔗 **Integration with Other Systems**

### **SLAM Integration**
```python
# camera_calibration.yaml used by SLAM (ChArUco calibrated)
slam_config:
  camera_calibration_file: camera_calibration.yaml
  calibration_method: charuco  # Indicates ChArUco calibration
  feature_extraction:
    calibration_dependent: true  # Uses calibrated focal length
  visual_odometry:
    intrinsics: calibrated_K_matrix
    board_type: charuco_4x6  # For reference
```

### **Computer Vision Pipeline**
```python
class CalibratedVisionNode(Node):
    def __init__(self):
        # Load calibration parameters
        self.camera_matrix = load_camera_matrix()
        self.dist_coeffs = load_distortion_coeffs()

        # Undistort images before processing
        self.undistort_images = True

    def process_image(self, image_msg):
        # Apply undistortion
        undistorted = cv2.undistort(image, self.camera_matrix, self.dist_coeffs)

        # Proceed with vision algorithms
        detections = self.detect_objects(undistorted)
```

### **Navigation Integration**
```python
# Camera calibration affects:
# - Visual odometry accuracy
# - Depth perception for obstacle avoidance
# - AR tag pose estimation
# - Terrain analysis for path planning
```

### **Mission Coordination**
```yaml
# Mission configuration includes calibration status
mission_requirements:
  camera_calibrated: true  # Prerequisite check
  calibration_method: charuco  # ChArUco calibration
  board_configuration: "4x6_squares_3x5_markers"  # For reference
  calibration_date: "2024-01-15"
  calibration_quality: "rms_error < 0.5"
```

---

## 📊 **Calibration Quality Metrics**

### **Reprojection Error**
```
Acceptable: < 1.0 pixels
Good: < 0.5 pixels
Excellent: < 0.3 pixels
```

### **Coverage Analysis**
```
Field of View: > 80% of sensor covered
Distance Range: 0.3m - 3.0m tested
Angle Range: ±45° in all axes
```

### **Temporal Stability**
```
Calibration valid for: 6 months (typical)
Re-check after: Major temperature changes, physical shock
```

---

## 🔧 **Troubleshooting Common Issues**

### **Poor Corner Detection**
```python
# Solutions:
# 1. Improve lighting (even, diffuse)
# 2. Clean calibration target
# 3. Reduce camera exposure
# 4. Use larger target squares
# 5. Ensure target is flat and rigid
```

### **High Reprojection Error**
```python
# Causes:
# - Insufficient image variety
# - Camera movement during capture
# - Poor target quality
# - Incorrect square size measurement

# Solutions:
# - Capture more images (60-80 minimum)
# - Use more stable mounting
# - Recreate target with precise measurements
# - Include more extreme viewing angles
```

### **Lens Distortion Issues**
```python
# Fish-eye lenses need different distortion model
# Use "equidistant" instead of "plumb_bob" for wide-angle lenses
distortion_model: "equidistant"
```

---

## 📅 **Maintenance Schedule**

### **Pre-Competition**
- **Daily**: Verify calibration file presence
- **Weekly**: Quick undistortion test
- **Monthly**: Full re-calibration if RMS > 1.0

### **During Competition**
- **Pre-mission**: Load calibration parameters
- **Mid-mission**: Monitor for calibration drift
- **Post-mission**: Archive calibration data

### **Post-Competition**
- **Analysis**: Compare calibration quality vs mission performance
- **Updates**: Improve calibration procedure based on lessons learned

---

## 🎯 **Success Checklist**

### **Calibration Quality**
- [ ] RMS reprojection error < 1.0 pixels
- [ ] 50+ calibration images captured
- [ ] Full field-of-view coverage
- [ ] Multiple distances and angles tested

### **System Integration**
- [ ] Calibration file in correct ROS2 parameter location
- [ ] Vision nodes loading calibration parameters
- [ ] SLAM system using calibrated intrinsics
- [ ] Mission planner checking calibration status

### **Validation**
- [ ] Undistortion test successful
- [ ] Pose estimation accuracy verified
- [ ] Depth perception measurements accurate
- [ ] AR tag detection working reliably

---

## 📚 **Additional Resources**

### **Reference Documents**
- [OpenCV Camera Calibration Tutorial](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)
- [ROS2 Camera Calibration Package](https://index.ros.org/p/camera_calibration/)
- [Intel D435i Datasheet](https://www.intelrealsense.com/depth-camera-d435i/)

### **Advanced Topics**
- **Multi-camera Calibration**: Stereo and RGB-D calibration
- **Online Calibration**: Real-time parameter refinement
- **Thermal Calibration**: Temperature-dependent parameter compensation

---

## ⚠️ **Critical Notes**

**Camera calibration with ChArUco boards is NOT optional** - it's a fundamental requirement for reliable computer vision performance. ChArUco provides superior accuracy compared to traditional chessboard methods. Poor calibration will result in:
- Failed AR tag detection during navigation
- Inaccurate depth perception for obstacle avoidance
- Unreliable autonomous typing performance
- Potential mission failure

**Always use ChArUco boards for calibration** - they provide better robustness and accuracy than plain chessboards. Always verify calibration quality before deploying to competition hardware and re-calibrate if any camera components are replaced or moved.
