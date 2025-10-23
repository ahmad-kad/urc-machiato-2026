# Libraries Guide - URC 2026 Autonomy System

## Overview
Comprehensive guide to all software libraries required for the URC 2026 autonomy system, including installation, usage patterns, and ROS 2 integration.

## 🐧 ROS 2 Humble Core Libraries

### **Installation & Setup**
```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture)] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop python3-argcomplete

# Setup environment (add to ~/.bashrc)
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

### **Core Navigation Stack**
```bash
# Install Navigation2
sudo apt install ros-humble-navigation2 ros-humble-nav2-bringup

# Launch navigation stack
ros2 launch nav2_bringup tb3_simulation_launch.py
```

### **SLAM Libraries**
```bash
# Install SLAM Toolbox
sudo apt install ros-humble-slam-toolbox

# Launch SLAM
ros2 launch slam_toolbox online_async_launch.py
```

### **Computer Vision Bridge**
```bash
# Install CV Bridge
sudo apt install ros-humble-cv-bridge ros-humble-vision-msgs

# Python usage
import cv_bridge
bridge = cv_bridge.CvBridge()
cv_image = bridge.imgmsg_to_cv2(ros_image, "bgr8")
```

## 📷 Computer Vision Libraries

### **OpenCV 4.5+**
```bash
# Installation
sudo apt install python3-opencv libopencv-dev

# Core imports
import cv2
import numpy as np
from cv2 import aruco

# ArUco marker detection (for autonomous typing)
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

corners, ids, rejected = detector.detectMarkers(image)

# Pose estimation
if ids is not None:
    rvec, tvec, _ = aruco.estimatePoseSingleMarkers(corners, 0.05, camera_matrix, dist_coeffs)
```

### **PyTorch & TorchVision**
```bash
# Installation
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu

# Object detection with pre-trained model
import torch
from torchvision.models.detection import fasterrcnn_resnet50_fpn

model = fasterrcnn_resnet50_fpn(pretrained=True)
model.eval()

# Inference
with torch.no_grad():
    predictions = model(images)
```

### **YOLOv5 (Ultralytics)**
```bash
# Installation
pip3 install ultralytics

# Usage
from ultralytics import YOLO

# Load model
model = YOLO('yolov5s.pt')  # or custom trained model

# Inference
results = model(image)

# Process results
for result in results:
    boxes = result.boxes  # bounding boxes
    masks = result.masks  # segmentation masks
    probs = result.probs  # classification probabilities
```

## 🔍 Point Cloud Processing

### **PCL (Point Cloud Library)**
```bash
# Installation
sudo apt install libpcl-dev python3-pcl

# Basic usage
import pcl
import numpy as np

# Load point cloud
cloud = pcl.load('scan.pcd')

# Voxel grid filtering (downsampling)
vgf = cloud.make_voxel_grid_filter()
vgf.set_leaf_size(0.01, 0.01, 0.01)
cloud_filtered = vgf.filter()

# Statistical outlier removal
sor = cloud_filtered.make_statistical_outlier_filter()
sor.set_mean_k(50)
sor.set_std_dev_mul_thresh(1.0)
cloud_cleaned = sor.filter()
```

### **Open3D**
```bash
# Installation
pip3 install open3d

# Usage
import open3d as o3d

# Load and visualize
pcd = o3d.io.read_point_cloud("scan.pcd")
o3d.visualization.draw_geometries([pcd])

# Point cloud registration (ICP)
source = o3d.io.read_point_cloud("source.pcd")
target = o3d.io.read_point_cloud("target.pcd")

icp_result = o3d.pipelines.registration.registration_icp(
    source, target, 0.05, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint())
```

## 📊 Optimization Libraries

### **GTSAM (Georgia Tech Smoothing and Mapping)**
```bash
# Installation
sudo apt install libgtsam-dev python3-gtsam

# Factor graph optimization
import gtsam
from gtsam import symbol_shorthand

# Create symbols
X = symbol_shorthand.X  # Pose symbols
L = symbol_shorthand.L  # Landmark symbols

# Build factor graph
graph = gtsam.NonlinearFactorGraph()
initial = gtsam.Values()

# Add factors and optimize
params = gtsam.LevenbergMarquardtParams()
optimizer = gtsam.LevenbergMarquardtOptimizer(graph, initial, params)
result = optimizer.optimize()
```

### **Ceres Solver**
```bash
# Installation
sudo apt install libceres-dev

# Non-linear least squares
import ceres
from ceres import Problem, AutoDiffCostFunction

# Define cost function
class SimpleCostFunction:
    def __init__(self, observed_x, observed_y):
        self.observed_x = observed_x
        self.observed_y = observed_y

    def __call__(self, parameters, residuals, jacobians):
        m = parameters[0]
        c = parameters[1]
        residuals[0] = self.observed_y - (m * self.observed_x + c)
        if jacobians:
            jacobians[0][0] = -self.observed_x
            jacobians[0][1] = -1.0
        return True

# Solve
problem = Problem()
cost_function = AutoDiffCostFunction(SimpleCostFunction(1.0, 2.0), 1, 2)
problem.AddResidualBlock(cost_function, None, [2.0, 1.0])

options = ceres.SolverOptions()
options.linear_solver_type = ceres.DENSE_QR
options.minimizer_progress_to_stdout = True
summary = ceres.Summary()
ceres.Solve(options, problem, summary)
```

## 📡 Communication Libraries

### **Serial Communication (PySerial)**
```bash
# Installation
pip3 install pyserial

# GPS NMEA parsing
import serial
import pynmea2

ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)

while True:
    line = ser.readline().decode('ascii', errors='replace')
    if line.startswith('$'):
        try:
            msg = pynmea2.parse(line)
            if hasattr(msg, 'latitude'):
                print(f"Lat: {msg.latitude}, Lon: {msg.longitude}")
        except:
            pass
```

### **I2C/SPI Communication**
```bash
# Installation
sudo apt install python3-smbus python3-spidev

# I2C IMU reading
import smbus2 as smbus

bus = smbus.SMBus(1)
address = 0x68  # MPU-6050

# Read accelerometer
accel_x = bus.read_word_data(address, 0x3B)
accel_y = bus.read_word_data(address, 0x3D)
accel_z = bus.read_word_data(address, 0x3F)

# Convert to signed values
accel_x = accel_x if accel_x < 0x8000 else accel_x - 0x10000
```

### **Socket Communication**
```python
# TCP client for sensor data
import socket
import struct

class TCPSensorClient:
    def __init__(self, host, port):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.connect((host, port))

    def send_data(self, data):
        packed_data = struct.pack('!f', data)  # Float32
        self.sock.sendall(packed_data)

    def receive_data(self):
        data = self.sock.recv(4)
        return struct.unpack('!f', data)[0]
```

## 🎛️ Control Libraries

### **PID Control**
```python
# Simple PID controller
class PIDController:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0

    def update(self, setpoint, measurement, dt):
        error = setpoint - measurement
        self.integral += error * dt
        derivative = (error - self.prev_error) / dt

        output = (self.kp * error +
                 self.ki * self.integral +
                 self.kd * derivative)

        self.prev_error = error
        return output

# Usage
pid = PIDController(kp=1.0, ki=0.1, kd=0.05)
control_signal = pid.update(target_velocity, current_velocity, dt=0.1)
```

### **Trajectory Planning**
```python
# Cubic spline trajectory
import numpy as np
from scipy.interpolate import CubicSpline

class TrajectoryPlanner:
    def __init__(self):
        pass

    def plan_trajectory(self, waypoints, times):
        # waypoints: list of [x, y] positions
        # times: corresponding timestamps

        x_coords = [wp[0] for wp in waypoints]
        y_coords = [wp[1] for wp in waypoints]

        cs_x = CubicSpline(times, x_coords)
        cs_y = CubicSpline(times, y_coords)

        return cs_x, cs_y

    def get_position(self, cs_x, cs_y, t):
        return cs_x(t), cs_y(t)

    def get_velocity(self, cs_x, cs_y, t):
        return cs_x(t, 1), cs_y(t, 1)  # First derivative
```

## 🔄 State Estimation Libraries

### **Kalman Filtering**
```python
# Extended Kalman Filter for localization
import numpy as np

class ExtendedKalmanFilter:
    def __init__(self, dim_x, dim_z):
        self.dim_x = dim_x  # State dimension
        self.dim_z = dim_z  # Measurement dimension

        self.x = np.zeros((dim_x, 1))  # State vector
        self.P = np.eye(dim_x)          # State covariance
        self.Q = np.eye(dim_x)          # Process noise
        self.R = np.eye(dim_z)          # Measurement noise

    def predict(self, F, u=None, B=None):
        # State prediction
        if u is not None and B is not None:
            self.x = np.dot(F, self.x) + np.dot(B, u)
        else:
            self.x = np.dot(F, self.x)

        # Covariance prediction
        self.P = np.dot(np.dot(F, self.P), F.T) + self.Q

    def update(self, z, H, R=None):
        if R is not None:
            self.R = R

        # Kalman gain
        S = np.dot(np.dot(H, self.P), H.T) + self.R
        K = np.dot(np.dot(self.P, H.T), np.linalg.inv(S))

        # State update
        y = z - np.dot(H, self.x)
        self.x = self.x + np.dot(K, y)

        # Covariance update
        I = np.eye(self.dim_x)
        self.P = np.dot((I - np.dot(K, H)), self.P)
```

### **Particle Filter**
```python
# Particle filter for localization
import numpy as np

class ParticleFilter:
    def __init__(self, num_particles, state_dim):
        self.num_particles = num_particles
        self.particles = np.random.randn(num_particles, state_dim)
        self.weights = np.ones(num_particles) / num_particles

    def predict(self, motion_model, control_input):
        # Apply motion model to all particles
        for i in range(self.num_particles):
            self.particles[i] = motion_model(self.particles[i], control_input)

        # Add process noise
        self.particles += np.random.normal(0, 0.1, self.particles.shape)

    def update(self, measurement_model, measurement, measurement_noise):
        # Update weights based on measurement likelihood
        for i in range(self.num_particles):
            predicted_measurement = measurement_model(self.particles[i])
            likelihood = self.gaussian_likelihood(
                measurement, predicted_measurement, measurement_noise)
            self.weights[i] *= likelihood

        # Normalize weights
        self.weights /= np.sum(self.weights)

        # Resample if needed
        if self.effective_sample_size() < self.num_particles / 2:
            self.resample()

    def gaussian_likelihood(self, actual, predicted, noise_cov):
        diff = actual - predicted
        return np.exp(-0.5 * np.dot(np.dot(diff.T, np.linalg.inv(noise_cov)), diff))

    def effective_sample_size(self):
        return 1.0 / np.sum(self.weights ** 2)

    def resample(self):
        # Systematic resampling
        cumulative_sum = np.cumsum(self.weights)
        cumulative_sum[-1] = 1.0  # Avoid floating point errors

        indices = np.zeros(self.num_particles, dtype=int)
        u0 = np.random.random() / self.num_particles

        for i in range(self.num_particles):
            u = u0 + i / self.num_particles
            indices[i] = np.searchsorted(cumulative_sum, u)

        self.particles = self.particles[indices]
        self.weights = np.ones(self.num_particles) / self.num_particles
```

## 🧪 Testing & Simulation Libraries

### **Pytest for Unit Testing**
```bash
# Installation
pip3 install pytest pytest-cov

# Basic test structure
import pytest
import numpy as np
from autonomy.navigation.path_planner import PathPlanner

class TestPathPlanner:
    def setup_method(self):
        self.planner = PathPlanner()

    def test_straight_line_path(self):
        start = np.array([0, 0])
        goal = np.array([10, 0])

        path = self.planner.plan_path(start, goal)

        assert len(path) > 0
        assert np.allclose(path[0], start)
        assert np.allclose(path[-1], goal)

    def test_obstacle_avoidance(self):
        start = np.array([0, 0])
        goal = np.array([10, 0])
        obstacles = [np.array([5, 0])]

        path = self.planner.plan_path(start, goal, obstacles)

        # Path should avoid obstacle
        for point in path:
            assert not np.any([np.linalg.norm(point - obs) < 1.0 for obs in obstacles])
```

### **Gazebo Simulation**
```bash
# Install Gazebo
sudo apt install gazebo ros-humble-gazebo-ros-pkgs

# Launch simulation
ros2 launch gazebo_ros gazebo.launch.py

# Load robot model
ros2 run gazebo_ros spawn_entity.py \
  -entity rover \
  -file /path/to/rover.urdf \
  -x 0 -y 0 -z 0
```

### **ROS 2 Testing Tools**
```bash
# Install testing tools
sudo apt install ros-humble-ros-testing

# Launch testing
ros2 test autonomy_navigation path_planner.test

# Integration testing
ros2 launch autonomy_system integration_test.launch.xml
```

## 📦 Package Management

### **Requirements.txt**
```
# Core dependencies
numpy>=1.21.0
scipy>=1.7.0
matplotlib>=3.5.0

# Computer vision
opencv-python>=4.5.0
torch>=1.12.0
torchvision>=0.13.0
ultralytics>=8.0.0

# Robotics
pyserial>=3.5
smbus2>=0.4.0
pynmea2>=1.18.0

# Testing
pytest>=7.0.0
pytest-cov>=4.0.0
```

### **Docker Dependencies**
```dockerfile
# Python packages
RUN pip3 install --no-cache-dir \
    torch==1.12.0+cpu \
    torchvision==0.13.0+cpu \
    opencv-python==4.5.5.64 \
    ultralytics==8.0.0 \
    pyserial==3.5 \
    pytest==7.1.2

# ROS 2 packages
RUN apt-get update && apt-get install -y \
    ros-humble-navigation2 \
    ros-humble-slam-toolbox \
    ros-humble-cv-bridge \
    && rm -rf /var/lib/apt/lists/*
```

## 🔧 Development Tools

### **Code Quality**
```bash
# Install linting tools
pip3 install flake8 black isort mypy

# Code formatting
black autonomy/
isort autonomy/

# Type checking
mypy autonomy/ --ignore-missing-imports

# Linting
flake8 autonomy/ --max-line-length=88
```

### **Version Control**
```bash
# Git LFS for large files
git lfs install
git lfs track "*.pcd" "*.bag" "*.h5" "*.pt"

# Pre-commit hooks
pip3 install pre-commit
pre-commit install

# Commit message format
# feat: add new sensor integration
# fix: resolve IMU calibration issue
# docs: update sensor guide
```

This libraries guide covers all the software components needed for the URC 2026 autonomy system, from low-level sensor drivers to high-level AI algorithms.
