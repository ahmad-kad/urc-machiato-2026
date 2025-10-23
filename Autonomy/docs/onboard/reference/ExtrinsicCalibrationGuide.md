# Extrinsic Calibration Guide - URC 2026 Rover

## Overview

Extrinsic calibration establishes the spatial relationships between different sensors on the URC 2026 rover, enabling accurate multi-sensor fusion for SLAM, object detection, and autonomous navigation. Poor extrinsic calibration can cause significant localization errors, failed sensor fusion, and mission-critical failures in autonomous operations.

This guide covers Camera-LiDAR calibration, hand-eye calibration for manipulation, and multi-sensor extrinsic calibration procedures essential for accurate sensor fusion.

## 📊 **Why Extrinsic Calibration Matters**

### **Mission Impact**
- **SLAM Accuracy**: Camera-LiDAR fusion requires precise extrinsic calibration for accurate 3D mapping
- **Object Detection**: Ground truth objects must be correctly localized across all sensors
- **Autonomous Typing**: Hand-eye calibration ensures precise arm positioning relative to camera
- **Multi-Sensor Fusion**: All sensors must be calibrated to a common reference frame

### **Calibration Consequences**
```
Poor Extrinsic Calibration → Sensor Misalignment → Fusion Errors → Mission Failure
```

- **Position Errors**: 10-50cm object localization errors
- **Orientation Errors**: 2-10° misalignment between sensor frames
- **Fusion Failures**: Inconsistent data preventing reliable SLAM
- **Manipulation Errors**: Incorrect arm positioning for typing tasks

### **Primary Method: Hand-Eye Calibration with ChArUco Boards**
For URC 2026, the primary extrinsic calibration method is **hand-eye calibration using ChArUco boards**:

- **Camera Intrinsics**: Calibrated using ChArUco boards (`calibration/charuco_board/camera_calibrator.py`)
- **Hand-Eye Calibration**: Robot-camera transformation using ChArUco board detection (`calibration/hand_eye/hand_eye_calibration.py`)
- **Why ChArUco?**: Superior pose estimation accuracy compared to traditional methods
- **Data Collection**: Capture images of ChArUco board from multiple robot poses
- **Result**: Precise transformation between robot base and camera coordinate frames

---

## 🔧 **Extrinsic Calibration Fundamentals**

### **Core Concepts**

#### **Extrinsic Parameters**
- **Rotation Matrix (R)**: 3x3 rotation matrix describing sensor orientation
- **Translation Vector (t)**: 3x1 vector describing sensor position
- **Transformation Matrix (T)**: 4x4 homogeneous transformation matrix

#### **Calibration Methods**
- **Target-Based**: Using known calibration targets (chessboards, ArUco markers)
- **Target-Less**: Motion-based calibration using sensor data alone
- **Hybrid**: Combining target-based accuracy with motion-based robustness

### **ROS 2 Calibration Tools**
```python
# ROS 2 calibration packages
from tf2_ros import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
import tf2_geometry_msgs
```

---

## 📋 **Extrinsic Calibration Prerequisites**

### **Hardware Requirements**
- **Calibration Targets**: Large chessboard patterns (1m x 1m minimum)
- **ArUco Markers**: Known-size markers for automated detection
- **Calibration Rig**: Precise mounting fixture for sensor alignment
- **Motion Platform**: For motion-based calibration (optional)

### **Software Requirements**
```bash
# Calibration tools
sudo apt install ros-humble-calibration-tools
sudo apt install ros-humble-camera-calibration
pip install opencv-contrib-python
pip install scipy scikit-learn

# Point cloud processing
pip install open3d pclpy
```

### **Data Collection Setup**
```python
# Extrinsic calibration data collection
class CalibrationDataCollector(Node):
    def __init__(self):
        super().__init__('calibration_data_collector')

        # Sensor subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.pointcloud_callback, 10
        )

        # Data storage
        self.image_data = []
        self.pointcloud_data = []
        self.timestamps = []

        # Synchronization
        self.last_image_time = None
        self.last_pc_time = None
        self.time_threshold = 0.05  # 50ms sync threshold
```

---

## 🚀 **Camera-LiDAR Extrinsic Calibration**

### **Phase 1: Target-Based Calibration**

#### **Chessboard Target Preparation**
```python
# generate_calibration_target.py
import cv2
import numpy as np

def generate_large_chessboard(square_size=50, rows=10, cols=14, output_path="calibration_target.png"):
    """
    Generate large chessboard calibration target for Camera-LiDAR calibration

    Args:
        square_size: Size of each square in mm
        rows: Number of internal corners (rows-1 squares)
        cols: Number of internal corners (cols-1 squares)
    """

    # Create large image (A2 size: 420x594mm at 300 DPI = 4961x7016 pixels)
    dpi = 300
    width_mm = cols * square_size
    height_mm = rows * square_size
    width_px = int(width_mm * dpi / 25.4)  # Convert mm to pixels
    height_px = int(height_mm * dpi / 25.4)

    # Create chessboard pattern
    pattern = np.zeros((height_px, width_px, 3), dtype=np.uint8)

    for i in range(rows):
        for j in range(cols):
            if (i + j) % 2 == 0:
                color = (255, 255, 255)  # White
            else:
                color = (0, 0, 0)       # Black

            start_x = j * int(square_size * dpi / 25.4)
            start_y = i * int(square_size * dpi / 25.4)
            end_x = start_x + int(square_size * dpi / 25.4)
            end_y = start_y + int(square_size * dpi / 25.4)

            cv2.rectangle(pattern, (start_x, start_y), (end_x, end_y), color, -1)

    # Add coordinate markers for LiDAR alignment
    font = cv2.FONT_HERSHEY_SIMPLEX
    cv2.putText(pattern, "X+", (width_px//2 + 100, height_px//2),
                font, 2, (255, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(pattern, "Y+", (width_px//2, height_px//2 - 100),
                font, 2, (0, 255, 0), 3, cv2.LINE_AA)

    cv2.imwrite(output_path, pattern)
    print(f"Calibration target saved to {output_path}")
    print(f"Physical size: {width_mm}x{height_mm}mm")
    print(f"Internal corners: {cols}x{rows}")

    return pattern

# Generate target for Camera-LiDAR calibration
generate_large_chessboard(square_size=75, rows=8, cols=11)
```

#### **Data Collection Procedure**
```python
# collect_calibration_data.py
class CameraLidarCalibrationCollector(Node):
    def __init__(self):
        super().__init__('camera_lidar_calibration_collector')

        # Sensor subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.pointcloud_callback, 10
        )

        # Data storage
        self.calibration_data = []
        self.collection_active = False

        # Services
        self.start_collection_service = self.create_service(
            Trigger, '/calibration/start_collection', self.start_collection
        )
        self.stop_collection_service = self.create_service(
            Trigger, '/calibration/stop_collection', self.stop_collection
        )
        self.save_data_service = self.create_service(
            Trigger, '/calibration/save_data', self.save_data
        )

        # Chessboard parameters
        self.chessboard_size = (11, 8)  # Internal corners
        self.square_size = 0.075  # 75mm squares

        self.get_logger().info('Camera-LiDAR calibration data collector initialized')

    def start_collection(self, request, response):
        """Start collecting calibration data"""
        self.calibration_data = []
        self.collection_active = True

        response.success = True
        response.message = "Calibration data collection started"
        self.get_logger().info('Started calibration data collection')

        return response

    def stop_collection(self, request, response):
        """Stop collecting calibration data"""
        self.collection_active = False

        response.success = True
        response.message = f"Calibration data collection stopped. Collected {len(self.calibration_data)} frames"
        self.get_logger().info(f'Stopped calibration data collection. Collected {len(self.calibration_data)} frames')

        return response

    def image_callback(self, msg):
        """Process camera images for chessboard detection"""
        if not self.collection_active:
            return

        try:
            # Convert ROS image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Convert to grayscale
            gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, self.chessboard_size,
                cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
            )

            if ret:
                # Refine corner positions
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

                # Store image data with detected corners
                image_data = {
                    'timestamp': msg.header.stamp,
                    'image': cv_image.copy(),
                    'corners': corners_refined,
                    'chessboard_found': True
                }

                # Match with nearest point cloud
                self.match_image_to_pointcloud(image_data)

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def pointcloud_callback(self, msg):
        """Store point cloud data"""
        if not self.collection_active:
            return

        try:
            # Convert ROS PointCloud2 to numpy array
            pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(pc_data))

            pointcloud_data = {
                'timestamp': msg.header.stamp,
                'points': points
            }

            # Store for matching with images
            self.pending_pointclouds.append(pointcloud_data)

            # Keep only recent point clouds
            current_time = self.get_clock().now().to_msg()
            self.pending_pointclouds = [
                pc for pc in self.pending_pointclouds
                if abs((current_time - pc['timestamp']).nanoseconds) < 1e9  # Within 1 second
            ]

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def match_image_to_pointcloud(self, image_data):
        """Match image with closest point cloud in time"""
        if not self.pending_pointclouds:
            return

        # Find closest point cloud in time
        image_time = image_data['timestamp']
        closest_pc = min(self.pending_pointclouds,
                        key=lambda pc: abs((image_time - pc['timestamp']).nanoseconds))

        time_diff = abs((image_time - closest_pc['timestamp']).nanoseconds / 1e9)

        if time_diff < 0.1:  # Within 100ms
            # Create calibration data pair
            calibration_pair = {
                'image': image_data,
                'pointcloud': closest_pc,
                'time_diff': time_diff
            }

            self.calibration_data.append(calibration_pair)
            self.get_logger().info(f'Collected calibration pair {len(self.calibration_data)} (time diff: {time_diff:.3f}s)')

            # Remove used point cloud
            self.pending_pointclouds.remove(closest_pc)

    def save_data(self, request, response):
        """Save collected calibration data"""
        try:
            import pickle

            data_file = f'/tmp/camera_lidar_calibration_{int(self.get_clock().now().to_msg().sec)}.pkl'

            with open(data_file, 'wb') as f:
                pickle.dump(self.calibration_data, f)

            response.success = True
            response.message = f"Saved {len(self.calibration_data)} calibration pairs to {data_file}"

        except Exception as e:
            response.success = False
            response.message = f"Failed to save data: {str(e)}"

        return response
```

#### **Calibration Computation**
```python
# camera_lidar_calibration.py
import cv2
import numpy as np
from scipy.optimize import minimize
import open3d as o3d

class CameraLidarCalibrator:
    def __init__(self, chessboard_size=(11, 8), square_size=0.075):
        self.chessboard_size = chessboard_size
        self.square_size = square_size

        # Prepare object points (chessboard corners in 3D space)
        self.obj_points = self.create_object_points()

        # Camera intrinsic parameters (from camera calibration)
        self.camera_matrix = None
        self.dist_coeffs = None

        # Extrinsic parameters to estimate
        self.R = None  # Rotation matrix
        self.t = None  # Translation vector

    def create_object_points(self):
        """Create 3D object points for chessboard corners"""
        obj_points = []

        for i in range(self.chessboard_size[1]):
            for j in range(self.chessboard_size[0]):
                obj_points.append([
                    j * self.square_size,
                    i * self.square_size,
                    0
                ])

        return np.array(obj_points, dtype=np.float32)

    def set_camera_intrinsics(self, camera_matrix, dist_coeffs):
        """Set camera intrinsic parameters"""
        self.camera_matrix = camera_matrix
        self.dist_coeffs = dist_coeffs

    def extract_chessboard_plane(self, pointcloud, chessboard_corners_2d):
        """
        Extract chessboard plane from point cloud using 2D image corners

        This is a simplified version. Real implementation would use more sophisticated
        plane fitting and corner correspondence methods.
        """

        # Project 2D chessboard corners to 3D using known plane equation
        # For simplicity, assume chessboard is in XY plane at Z=0

        # In practice, you would:
        # 1. Detect edges/corners in point cloud
        # 2. Use RANSAC to find dominant plane
        # 3. Extract corner points from plane
        # 4. Match with image corners

        # Placeholder implementation
        corners_3d = []

        for corner_2d in chessboard_corners_2d:
            # Undistort point
            undistorted = cv2.undistortPoints(
                corner_2d.reshape(1, 1, 2),
                self.camera_matrix,
                self.dist_coeffs
            )

            # Assume chessboard is perpendicular to camera at known distance
            # This is a simplification - real implementation needs proper 3D reconstruction
            x_3d = (undistorted[0, 0, 0] - self.camera_matrix[0, 2]) * 2.0 / self.camera_matrix[0, 0]
            y_3d = (undistorted[0, 0, 1] - self.camera_matrix[1, 2]) * 2.0 / self.camera_matrix[1, 1]
            z_3d = 2.0  # Assumed distance

            corners_3d.append([x_3d, y_3d, z_3d])

        return np.array(corners_3d)

    def calibrate_extrinsics(self, calibration_data):
        """
        Perform Camera-LiDAR extrinsic calibration

        Args:
            calibration_data: List of dicts with 'image' and 'pointcloud' keys
        """

        print(f"Calibrating with {len(calibration_data)} data pairs...")

        # Collect corresponding points
        image_points_all = []
        object_points_all = []

        for data_pair in calibration_data:
            image_data = data_pair['image']
            pointcloud_data = data_pair['pointcloud']

            # Get 2D chessboard corners
            corners_2d = image_data['corners']

            # Extract corresponding 3D points from point cloud
            corners_3d = self.extract_chessboard_plane(
                pointcloud_data['points'], corners_2d
            )

            if len(corners_3d) == len(self.obj_points):
                image_points_all.append(corners_2d)
                object_points_all.append(corners_3d)

        if len(image_points_all) < 5:
            raise ValueError("Need at least 5 valid chessboard detections for calibration")

        # Solve for extrinsic parameters using PnP
        success, rvec, tvec = cv2.solvePnP(
            np.array(object_points_all[0], dtype=np.float32),
            np.array(image_points_all[0], dtype=np.float32),
            self.camera_matrix,
            self.dist_coeffs,
            flags=cv2.SOLVEPNP_ITERATIVE
        )

        if success:
            # Convert rotation vector to matrix
            self.R, _ = cv2.Rodrigues(rvec)
            self.t = tvec.flatten()

            print("Camera-LiDAR calibration successful!")
            print(f"Rotation matrix:\n{self.R}")
            print(f"Translation vector: {self.t}")

            return self.R, self.t
        else:
            raise RuntimeError("Camera-LiDAR calibration failed")

    def validate_calibration(self, test_data):
        """Validate calibration quality"""

        if self.R is None or self.t is None:
            raise RuntimeError("Calibration not performed yet")

        total_error = 0
        point_count = 0

        for data_pair in test_data:
            image_data = data_pair['image']
            pointcloud_data = data_pair['pointcloud']

            # Get 2D chessboard corners
            corners_2d = image_data['corners']

            # Extract corresponding 3D points
            corners_3d = self.extract_chessboard_plane(
                pointcloud_data['points'], corners_2d
            )

            # Project 3D points to 2D using calibration
            points_3d_homo = np.column_stack((corners_3d, np.ones(len(corners_3d))))
            projected_2d, _ = cv2.projectPoints(
                points_3d_homo, self.R, self.t, self.camera_matrix, self.dist_coeffs
            )

            # Calculate reprojection error
            projected_2d = projected_2d.reshape(-1, 2)
            error = np.linalg.norm(corners_2d - projected_2d, axis=1)
            total_error += np.sum(error)
            point_count += len(error)

        mean_error = total_error / point_count
        print(f"Mean reprojection error: {mean_error:.3f} pixels")

        # Acceptable error threshold
        if mean_error < 5.0:  # Less than 5 pixels
            print("Calibration validation PASSED")
            return True
        else:
            print("Calibration validation FAILED - error too high")
            return False
```

### **Phase 2: Motion-Based Calibration**

#### **Continuous Motion Calibration**
```python
# motion_based_calibration.py
class MotionBasedCalibrator(Node):
    def __init__(self):
        super().__init__('motion_based_calibrator')

        # Sensor subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.pointcloud_sub = self.create_subscription(
            PointCloud2, '/lidar/points', self.pointcloud_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Motion data storage
        self.image_trajectory = []
        self.lidar_trajectory = []
        self.imu_trajectory = []

        # Calibration parameters
        self.T_camera_lidar = np.eye(4)  # Initial guess
        self.optimization_window = 100   # Frames to use for optimization

    def image_callback(self, msg):
        """Process camera images for feature tracking"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Detect features (ORB, SIFT, etc.)
            features = self.detect_features(cv_image)

            motion_data = {
                'timestamp': msg.header.stamp,
                'features': features,
                'image': cv_image
            }

            self.image_trajectory.append(motion_data)
            self.maintain_trajectory_length()

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def pointcloud_callback(self, msg):
        """Process LIDAR point clouds"""
        try:
            pc_data = point_cloud2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            points = np.array(list(pc_data))

            motion_data = {
                'timestamp': msg.header.stamp,
                'points': points
            }

            self.lidar_trajectory.append(motion_data)
            self.maintain_trajectory_length()

        except Exception as e:
            self.get_logger().error(f'Error processing point cloud: {str(e)}')

    def imu_callback(self, msg):
        """Process IMU data for motion estimation"""
        motion_data = {
            'timestamp': msg.header.stamp,
            'angular_velocity': [
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ],
            'linear_acceleration': [
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ]
        }

        self.imu_trajectory.append(motion_data)
        self.maintain_trajectory_length()

    def maintain_trajectory_length(self):
        """Keep trajectory buffers at manageable size"""
        max_length = 200

        if len(self.image_trajectory) > max_length:
            self.image_trajectory.pop(0)
        if len(self.lidar_trajectory) > max_length:
            self.lidar_trajectory.pop(0)
        if len(self.imu_trajectory) > max_length:
            self.imu_trajectory.pop(0)

        # Trigger calibration when we have enough data
        if (len(self.image_trajectory) >= self.optimization_window and
            len(self.lidar_trajectory) >= self.optimization_window and
            len(self.imu_trajectory) >= self.optimization_window):

            self.perform_motion_calibration()

    def perform_motion_calibration(self):
        """Perform motion-based extrinsic calibration"""

        try:
            # Synchronize trajectories by timestamp
            synced_data = self.synchronize_trajectories()

            if len(synced_data) < 10:
                return  # Not enough synchronized data

            # Estimate motion from IMU
            motion_estimates = self.estimate_motion_from_imu(synced_data)

            # Optimize Camera-LiDAR transform
            optimized_transform = self.optimize_extrinsics(
                synced_data, motion_estimates, self.T_camera_lidar
            )

            # Update calibration
            self.T_camera_lidar = optimized_transform

            # Publish updated transform
            self.publish_calibration_transform()

            self.get_logger().info('Motion-based calibration updated')

        except Exception as e:
            self.get_logger().error(f'Motion calibration failed: {str(e)}')

    def synchronize_trajectories(self):
        """Synchronize image, LIDAR, and IMU trajectories by timestamp"""

        synced_data = []

        for img_data in self.image_trajectory:
            img_time = img_data['timestamp']

            # Find closest LIDAR data
            lidar_match = min(self.lidar_trajectory,
                            key=lambda x: abs((img_time - x['timestamp']).nanoseconds))

            # Find closest IMU data
            imu_match = min(self.imu_trajectory,
                          key=lambda x: abs((img_time - x['timestamp']).nanoseconds))

            time_diff_lidar = abs((img_time - lidar_match['timestamp']).nanoseconds / 1e9)
            time_diff_imu = abs((img_time - imu_match['timestamp']).nanoseconds / 1e9)

            # Only use well-synchronized data
            if time_diff_lidar < 0.05 and time_diff_imu < 0.05:  # 50ms threshold
                synced_data.append({
                    'image': img_data,
                    'lidar': lidar_match,
                    'imu': imu_match,
                    'time_diff': max(time_diff_lidar, time_diff_imu)
                })

        return synced_data

    def estimate_motion_from_imu(self, synced_data):
        """Estimate motion between frames using IMU"""

        motion_estimates = []

        for i in range(1, len(synced_data)):
            prev_imu = synced_data[i-1]['imu']
            curr_imu = synced_data[i]['imu']

            # Integrate angular velocity for rotation
            dt = (curr_imu['timestamp'] - prev_imu['timestamp']).nanoseconds / 1e9

            # Simplified motion estimation (real implementation would use proper IMU integration)
            rotation = np.array(curr_imu['angular_velocity']) * dt
            translation = np.array(curr_imu['linear_acceleration']) * dt * dt

            motion_estimates.append({
                'rotation': rotation,
                'translation': translation,
                'dt': dt
            })

        return motion_estimates

    def optimize_extrinsics(self, synced_data, motion_estimates, initial_transform):
        """Optimize extrinsic parameters using motion data"""

        def objective_function(params):
            """Objective function for extrinsic optimization"""
            # Extract parameters (6 DOF: 3 rotation, 3 translation)
            rx, ry, rz, tx, ty, tz = params

            # Create transform matrix
            R = self.rotation_matrix_from_euler(rx, ry, rz)
            t = np.array([tx, ty, tz])

            T = np.eye(4)
            T[:3, :3] = R
            T[:3, 3] = t

            # Calculate consistency error across motion
            total_error = 0

            for i in range(len(motion_estimates)):
                # This would compute how well the transform explains the motion
                # between image and LIDAR frames
                error = self.compute_motion_consistency_error(
                    synced_data[i], synced_data[i+1], T, motion_estimates[i]
                )
                total_error += error

            return total_error

        # Initial parameters from current transform
        R_init = initial_transform[:3, :3]
        t_init = initial_transform[:3, 3]
        rx, ry, rz = self.euler_from_rotation_matrix(R_init)

        initial_params = [rx, ry, rz, t_init[0], t_init[1], t_init[2]]

        # Optimize
        result = minimize(objective_function, initial_params, method='BFGS')

        # Reconstruct optimized transform
        rx, ry, rz, tx, ty, tz = result.x
        R_opt = self.rotation_matrix_from_euler(rx, ry, rz)
        t_opt = np.array([tx, ty, tz])

        T_opt = np.eye(4)
        T_opt[:3, :3] = R_opt
        T_opt[:3, 3] = t_opt

        return T_opt

    def publish_calibration_transform(self):
        """Publish the calibrated Camera-LiDAR transform"""

        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'lidar_link'
        transform.child_frame_id = 'camera_link'

        # Extract rotation and translation
        R = self.T_camera_lidar[:3, :3]
        t = self.T_camera_lidar[:3, 3]

        # Convert to quaternion
        quaternion = self.quaternion_from_rotation_matrix(R)

        transform.transform.translation.x = t[0]
        transform.transform.translation.y = t[1]
        transform.transform.translation.z = t[2]
        transform.transform.rotation.w = quaternion[0]
        transform.transform.rotation.x = quaternion[1]
        transform.transform.rotation.y = quaternion[2]
        transform.transform.rotation.z = quaternion[3]

        # Publish static transform
        self.tf_broadcaster.sendTransform(transform)

    # Utility functions for rotation matrices, quaternions, etc.
    # (Implementation details omitted for brevity)
```

---

## 🤖 **Hand-Eye Calibration for Manipulation**

### **Eye-in-Hand vs Eye-to-Hand Calibration**

#### **Eye-in-Hand Calibration (Camera on Arm)**
```python
# eye_in_hand_calibration.py
class EyeInHandCalibrator(Node):
    def __init__(self):
        super().__init__('eye_in_hand_calibrator')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_callback, 10
        )

        # Calibration data
        self.calibration_poses = []
        self.marker_detections = []

        # Known marker in calibration target frame
        self.marker_in_target = np.array([0.1, 0.1, 0.0])  # Marker position

    def collect_calibration_data(self):
        """Collect arm poses and marker detections"""

        # Move arm to different poses
        calibration_poses = self.generate_calibration_poses()

        for pose in calibration_poses:
            # Move arm to pose
            self.move_arm_to_pose(pose)

            # Wait for settling
            time.sleep(2.0)

            # Capture image and detect marker
            marker_in_camera = self.detect_marker_in_image()

            if marker_in_camera is not None:
                # Store arm pose and marker detection
                arm_pose = self.get_current_arm_pose()

                self.calibration_poses.append(arm_pose)
                self.marker_detections.append(marker_in_camera)

                self.get_logger().info(f'Collected calibration data {len(self.calibration_poses)}')

    def compute_hand_eye_transform(self):
        """Compute hand-eye transformation using Tsai's method"""

        if len(self.calibration_poses) < 4:
            self.get_logger().error("Need at least 4 calibration poses")
            return None

        # Implement Tsai's hand-eye calibration algorithm
        # This solves: AX = XB where A is arm motion, B is camera motion, X is hand-eye transform

        A_matrices = []  # Arm transformations between poses
        B_matrices = []  # Camera transformations between poses

        for i in range(1, len(self.calibration_poses)):
            # Arm transformation from pose i-1 to pose i
            T_arm_prev = self.pose_to_transform(self.calibration_poses[i-1])
            T_arm_curr = self.pose_to_transform(self.calibration_poses[i])
            T_arm = np.linalg.inv(T_arm_prev) @ T_arm_curr
            A_matrices.append(T_arm)

            # Camera transformation from pose i-1 to pose i
            marker_prev = self.marker_detections[i-1]
            marker_curr = self.marker_detections[i]

            # For eye-in-hand, we need to compute camera motion that would
            # explain the marker motion if the marker were stationary
            T_marker_prev = self.marker_pose_to_transform(marker_prev)
            T_marker_curr = self.marker_pose_to_transform(marker_curr)
            T_marker = np.linalg.inv(T_marker_prev) @ T_marker_curr

            # Since marker is fixed in world, camera motion B = T_marker
            B_matrices.append(T_marker)

        # Solve AX = XB for X using Tsai's method
        X = self.solve_tsai_hand_eye(A_matrices, B_matrices)

        return X

    def solve_tsai_hand_eye(self, A_list, B_list):
        """Solve hand-eye calibration using Tsai's method"""

        # Simplified implementation of Tsai's algorithm
        # Real implementation would use the full mathematical formulation

        # Collect rotation and translation constraints
        alpha_list = []
        beta_list = []

        for A, B in zip(A_list, B_list):
            # Extract rotation and translation
            R_a = A[:3, :3]
            t_a = A[:3, 3]
            R_b = B[:3, :3]
            t_b = B[:3, 3]

            # Build constraint matrices for rotation estimation
            alpha = self.rotation_to_axis_angle(R_a @ R_b.T)
            beta = self.rotation_to_axis_angle(R_b @ R_a.T)

            alpha_list.append(alpha)
            beta_list.append(beta)

        # Estimate rotation part using least squares
        # (Detailed implementation would follow Tsai's paper)

        # For now, return identity (placeholder)
        return np.eye(4)

    def validate_hand_eye_calibration(self, test_poses):
        """Validate hand-eye calibration accuracy"""

        errors = []

        for pose in test_poses:
            # Move arm to test pose
            self.move_arm_to_pose(pose)

            # Get expected marker position from forward kinematics + calibration
            expected_marker = self.compute_expected_marker_position(pose)

            # Detect actual marker position
            actual_marker = self.detect_marker_in_image()

            if actual_marker is not None and expected_marker is not None:
                # Compute position error
                error = np.linalg.norm(actual_marker - expected_marker)
                errors.append(error)

        if errors:
            mean_error = np.mean(errors)
            std_error = np.std(errors)

            self.get_logger().info(
                f'Hand-eye calibration validation: '
                f'mean error = {mean_error:.3f}m, '
                f'std = {std_error:.3f}m'
            )

            # Acceptable error < 5mm
            return mean_error < 0.005
        else:
            return False
```

#### **Eye-to-Hand Calibration (Fixed Camera)**
```python
# eye_to_hand_calibration.py
class EyeToHandCalibrator(Node):
    def __init__(self):
        super().__init__('eye_to_hand_calibrator')

        # For eye-to-hand, camera is fixed and we move calibration target with arm
        # This is simpler than eye-in-hand as target pose is directly measured

        # Known target geometry
        self.target_points = self.define_target_geometry()

    def calibrate_eye_to_hand(self):
        """Perform eye-to-hand calibration"""

        # Collect multiple arm poses with target visible
        calibration_data = self.collect_eye_to_hand_data()

        # Estimate camera-to-base transform
        T_camera_base = self.estimate_camera_to_base(calibration_data)

        # Compute arm-to-camera transform
        # T_arm_camera = T_base_camera * T_camera_arm (from known arm kinematics)
        # For eye-to-hand, we directly estimate T_camera_base

        return T_camera_base

    def collect_eye_to_hand_data(self):
        """Collect arm poses and corresponding target detections"""

        data = []

        # Move arm through calibration poses
        poses = self.generate_calibration_poses()

        for pose in poses:
            self.move_arm_to_target_pose(pose)

            # Detect target in camera
            target_detection = self.detect_calibration_target()

            if target_detection:
                data.append({
                    'arm_pose': pose,
                    'target_detection': target_detection
                })

        return data

    def estimate_camera_to_base(self, calibration_data):
        """Estimate camera-to-base transform from calibration data"""

        # For each calibration pose, we know:
        # - Arm pose (from joint encoders) -> T_base_target
        # - Target detection in camera -> T_camera_target

        # Therefore: T_camera_base = T_camera_target * T_target_base
        # Where T_target_base = inv(T_base_target)

        transforms = []

        for data_point in calibration_data:
            arm_pose = data_point['arm_pose']
            target_detection = data_point['target_detection']

            # Get arm-to-target transform from forward kinematics
            T_base_target = self.forward_kinematics(arm_pose)

            # Get camera-to-target transform from detection
            T_camera_target = self.detection_to_transform(target_detection)

            # Compute camera-to-base transform
            T_target_base = np.linalg.inv(T_base_target)
            T_camera_base = T_camera_target @ T_target_base

            transforms.append(T_camera_base)

        # Average transforms (or use more sophisticated method)
        T_camera_base_avg = self.average_transforms(transforms)

        return T_camera_base_avg
```

---

## 🔗 **Multi-Sensor Extrinsic Calibration**

### **Phase 1: Pairwise Calibration**

#### **Camera-IMU Calibration**
```python
# camera_imu_calibration.py
class CameraIMUCalibrator(Node):
    def __init__(self):
        super().__init__('camera_imu_calibration')

        # Subscriptions
        self.image_sub = self.create_subscription(
            Image, '/camera/color/image_raw', self.image_callback, 10
        )
        self.imu_sub = self.create_subscription(
            Imu, '/imu/data', self.imu_callback, 10
        )

        # Use IMU for motion estimation, camera for visual features
        # Similar to motion-based calibration but using visual odometry

    def calibrate_camera_imu(self):
        """Calibrate Camera-IMU extrinsics using motion"""

        # Collect data during motion
        motion_data = self.collect_motion_data()

        # Estimate transform using visual-inertial methods
        T_camera_imu = self.estimate_camera_imu_transform(motion_data)

        return T_camera_imu
```

#### **LIDAR-IMU Calibration**
```python
# lidar_imu_calibration.py
class LidarIMUCalibrator(Node):
    def __init__(self):
        super().__init__('lidar_imu_calibration')

        # Use LIDAR odometry and IMU measurements
        # Align LIDAR motion estimates with IMU measurements

    def calibrate_lidar_imu(self):
        """Calibrate LIDAR-IMU extrinsics"""

        # This would use continuous-time optimization
        # to align LIDAR scan matching with IMU measurements

        return T_lidar_imu
```

### **Phase 2: Multi-Sensor Graph Optimization**

#### **Global Extrinsic Optimization**
```python
# multi_sensor_calibrator.py
class MultiSensorCalibrator:
    def __init__(self):
        # All sensor transforms
        self.transforms = {
            'T_camera_base': None,
            'T_lidar_base': None,
            'T_imu_base': None,
            'T_gps_base': None
        }

        # Collected measurements between sensors
        self.measurements = []

    def add_measurement(self, sensor_a, sensor_b, transform_a_b, covariance):
        """Add measurement between two sensors"""

        self.measurements.append({
            'sensor_a': sensor_a,
            'sensor_b': sensor_b,
            'transform': transform_a_b,
            'covariance': covariance
        })

    def optimize_extrinsics(self):
        """Perform global optimization of all extrinsic parameters"""

        # Use graph-based optimization (similar to SLAM)
        # Each sensor becomes a node, measurements become constraints

        # Define cost function
        def cost_function(sensor_transforms):
            total_cost = 0

            for measurement in self.measurements:
                # Extract transforms
                T_a = self.get_transform_from_params(sensor_transforms, measurement['sensor_a'])
                T_b = self.get_transform_from_params(sensor_transforms, measurement['sensor_b'])

                # Predicted measurement: T_a_b_predicted = inv(T_a) @ T_b
                T_a_b_predicted = np.linalg.inv(T_a) @ T_b

                # Compare with actual measurement
                error = self.compute_transform_error(
                    T_a_b_predicted, measurement['transform'], measurement['covariance']
                )

                total_cost += error

            return total_cost

        # Optimize using nonlinear least squares
        initial_params = self.flatten_transforms(self.transforms)
        result = minimize(cost_function, initial_params, method='LM')

        # Extract optimized transforms
        optimized_transforms = self.unflatten_transforms(result.x)

        return optimized_transforms

    def compute_transform_error(self, T_pred, T_meas, covariance):
        """Compute error between predicted and measured transforms"""

        # Convert to log space for error computation
        log_T_pred = self.se3_log(T_pred)
        log_T_meas = self.se3_log(T_meas)

        # Error in se(3)
        error = log_T_pred - log_T_meas

        # Weighted by covariance
        weighted_error = error.T @ np.linalg.inv(covariance) @ error

        return weighted_error
```

---

## 📊 **Extrinsic Calibration Validation**

### **Calibration Quality Metrics**
```python
# calibration_validator.py
class ExtrinsicCalibrationValidator:
    def __init__(self):
        self.validation_metrics = {}

    def validate_calibration(self, calibration_data, transforms):
        """Comprehensive calibration validation"""

        # Reprojection error
        reprojection_error = self.compute_reprojection_error(calibration_data)

        # Motion consistency
        motion_consistency = self.check_motion_consistency(calibration_data)

        # Multi-sensor consistency
        multi_sensor_consistency = self.check_multi_sensor_consistency()

        # Store metrics
        self.validation_metrics = {
            'reprojection_error': reprojection_error,
            'motion_consistency': motion_consistency,
            'multi_sensor_consistency': multi_sensor_consistency
        }

        # Overall quality score
        quality_score = self.compute_quality_score()

        return quality_score > 0.8  # Acceptable calibration

    def compute_reprojection_error(self, calibration_data):
        """Compute reprojection error for calibrated transforms"""

        total_error = 0
        point_count = 0

        for data_pair in calibration_data:
            # Project points from one sensor to another using calibration
            # Compute error between projected and measured points

            error = self.compute_pairwise_reprojection_error(data_pair)
            total_error += error
            point_count += 1

        return total_error / point_count if point_count > 0 else float('inf')

    def check_motion_consistency(self, motion_data):
        """Check if transforms are consistent with motion"""

        # Verify that transforms explain observed motion correctly
        # This detects incorrect rotation directions, scale errors, etc.

        consistency_score = 0.0

        for motion_segment in motion_data:
            # Check if calibrated transforms predict motion correctly
            predicted_motion = self.predict_motion_from_transforms(motion_segment)
            actual_motion = motion_segment['actual_motion']

            motion_error = self.compute_motion_error(predicted_motion, actual_motion)
            consistency_score += motion_error

        return consistency_score / len(motion_data) if motion_data else float('inf')

    def compute_quality_score(self):
        """Compute overall calibration quality score"""

        metrics = self.validation_metrics

        # Weighted combination of metrics
        weights = {
            'reprojection_error': 0.4,
            'motion_consistency': 0.4,
            'multi_sensor_consistency': 0.2
        }

        score = 0

        # Normalize and weight each metric
        for metric_name, weight in weights.items():
            value = metrics.get(metric_name, float('inf'))

            if metric_name == 'reprojection_error':
                # Lower is better, max acceptable error = 10 pixels
                normalized = max(0, 1 - value / 10.0)
            elif metric_name.endswith('_consistency'):
                # Lower is better, perfect = 0
                normalized = max(0, 1 - value)

            score += weight * normalized

        return score
```

---

## 🚀 **ROS 2 Integration and Deployment**

### **Calibration Parameter Management**
```python
# calibration_manager.py
class CalibrationManager(Node):
    def __init__(self):
        super().__init__('calibration_manager')

        # Load calibration parameters
        self.load_calibration_parameters()

        # Publish static transforms
        self.publish_static_transforms()

        # Calibration update services
        self.update_calibration_service = self.create_service(
            SetTransform, '/calibration/update_transform', self.update_transform
        )

        # Calibration saving/loading
        self.save_calibration_service = self.create_service(
            Trigger, '/calibration/save', self.save_calibration
        )
        self.load_calibration_service = self.create_service(
            Trigger, '/calibration/load', self.load_calibration
        )

    def load_calibration_parameters(self):
        """Load extrinsic calibration from parameter server or file"""

        # Camera-LiDAR transform
        self.T_camera_lidar = self.load_transform_from_params('camera_lidar')

        # Camera-IMU transform
        self.T_camera_imu = self.load_transform_from_params('camera_imu')

        # LIDAR-IMU transform
        self.T_lidar_imu = self.load_transform_from_params('lidar_imu')

        # Camera-base transform
        self.T_camera_base = self.load_transform_from_params('camera_base')

    def publish_static_transforms(self):
        """Publish calibrated static transforms"""

        transforms = [
            ('base_link', 'camera_link', self.T_camera_base),
            ('camera_link', 'lidar_link', self.compute_relative_transform(
                self.T_camera_base, self.T_lidar_base
            )),
            # Add other transforms...
        ]

        for parent, child, transform in transforms:
            self.publish_static_transform(parent, child, transform)

    def update_transform(self, request, response):
        """Update a calibration transform"""

        transform_name = request.transform_name
        new_transform = self.transform_msg_to_matrix(request.transform)

        # Update internal storage
        setattr(self, f'T_{transform_name}', new_transform)

        # Republish transforms
        self.publish_static_transforms()

        response.success = True
        response.message = f"Updated transform {transform_name}"

        return response
```

---

## 📋 **Extrinsic Calibration Checklist**

### **Camera-LiDAR Calibration**
- [ ] Large chessboard target prepared (minimum 1m x 0.75m)
- [ ] Data collection procedure executed (10+ poses)
- [ ] Target-based calibration computed
- [ ] Motion-based refinement applied
- [ ] Reprojection error validated (< 5 pixels)
- [ ] Transform published and tested

### **Hand-Eye Calibration**
- [ ] Calibration target with known markers prepared
- [ ] Arm moved through sufficient calibration poses (10+)
- [ ] Target detections collected for each pose
- [ ] Hand-eye transform computed using appropriate method
- [ ] Position accuracy validated (< 5mm error)
- [ ] Orientation accuracy validated (< 1° error)

### **Multi-Sensor Calibration**
- [ ] Pairwise calibrations completed for all sensor pairs
- [ ] Motion data collected for consistency validation
- [ ] Global optimization performed
- [ ] All transforms consistent with motion data
- [ ] Multi-sensor fusion tested and validated

### **Integration & Validation**
- [ ] Transforms integrated with TF tree
- [ ] SLAM system uses calibrated transforms
- [ ] Computer vision detections properly transformed
- [ ] Autonomous typing uses accurate hand-eye calibration
- [ ] Overall system performance validated

### **Maintenance & Monitoring**
- [ ] Calibration monitoring system deployed
- [ ] Drift detection implemented
- [ ] Recalibration procedures documented
- [ ] Calibration parameters backed up
- [ ] Validation tests automated

---

**🎯 Success Criteria**: Extrinsic calibration complete when all sensor transforms are accurate (<5mm, <1° error), multi-sensor fusion operates without temporal/spatial inconsistencies, and SLAM/localization performance meets URC 2026 accuracy requirements (<3m position error for GNSS waypoints, <2m for AR-tagged posts).
