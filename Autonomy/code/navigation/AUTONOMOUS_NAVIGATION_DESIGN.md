# Autonomous Navigation System - Detailed Design

**URC 2026 Multi-Mode Navigation with Rear-Facing RGB-D SLAM**

---

## 1. System Overview

### Unique Constraint: Rear-Facing Camera
- **Single RGB-D camera**: Mounted on rear of rover, facing backwards
- **SLAM reference frame**: `camera_link` fixed to rover rear
- **Navigation paradigm**: All movement in reverse (backing up)
- **Coordinate system**: Robot moves in negative X direction (backwards)

### Navigation Modes (State Machine)
```
┌─────────────┐
│    IDLE     │ ← System ready, awaiting command
└──────┬──────┘
       │ mission_command received
       ↓
┌────────────────────────────────────────────────┐
│           GPS_NAVIGATE (PRIMARY)               │
│  - Book it to GPS waypoint                     │
│  - Avoid obstacles (SLAM depth)                │
│  - If ArUco detected → APPROACH_TARGET         │
│  - Speed: 1.0 m/s (backing up)                │
└────────────────────────────────────────────────┘
       ↓ ArUco detected within FOV
┌────────────────────────────────────────────────┐
│      APPROACH_TARGET (VISION GUIDANCE)         │
│  - Use ArUco pose for precision approach       │
│  - Center tag in frame, approach               │
│  - Speed: 0.2 m/s (slow precision)            │
│  - On success → IDLE                          │
└────────────────────────────────────────────────┘

       ↓ GPS target not found in radius limit
┌────────────────────────────────────────────────┐
│       EXPLORE_AREA (SEARCH & MAP)              │
│  - Spiral search pattern (expanding radius)    │
│  - Continuous SLAM mapping                     │
│  - ArUco detection running                     │
│  - Radius limit: 50m from start                │
│  - If ArUco found → APPROACH_TARGET            │
│  - If timeout (30min) → EXIT_EXPLORE           │
└────────────────────────────────────────────────┘

       ↓ Remote signal lost (heartbeat timeout)
┌────────────────────────────────────────────────┐
│       SIGNAL_LOST (FAILSAFE RETREAT)           │
│  - Reverse at 0.5 m/s to safe zone             │
│  - Use SLAM to navigate back                   │
│  - Rear camera for local obstacle detection    │
│  - Track breadcrumb trail back to origin       │
│  - On signal recover → IDLE                    │
│  - OR reach origin → IDLE                      │
└────────────────────────────────────────────────┘

       ↓ Critical safety issue
       EMERGENCY_STOP (stop all motion)
```

---

## 2. Coordinate System & Kinematics

### Rear-Facing Camera Implications

```
Rover Frame Definition:
┌─────────────────────────────────────────┐
│  Front of Rover (BLOCKED)               │
│                                         │
│  +Y (left)                              │
│   ^                                     │
│   |                                     │
│   |---+X (RIGHT-FACING) →               │
│       |                                 │
│       v -X (backwards, navigation dir)  │
│                                         │
│  ┌─────────────────────┐                │
│  │  REAR RGB-D Camera  │ (facing back)  │
│  │  (backwards, -X)    │                │
│  └─────────────────────┘                │
└─────────────────────────────────────────┘

SLAM Frame (Camera): Points forward (-X from body center)
Navigation Frame (Body): Points forward (+X), BUT rover backs up (-X)

Velocity Commands:
- Reverse speed 1.0 m/s = vx = -1.0 m/s (body frame)
- Turn left = ωz = +0.5 rad/s
- Turn right = ωz = -0.5 rad/s
```

### Pose Transformation
```
camera_link → camera_optical_frame (90° rotation)
camera_optical_frame → base_link (rear mount offset)
base_link → map (SLAM output)

When navigating in reverse:
- map frame: standard (North-East-Down)
- base_link: rover body (forward = +X)
- camera_link: rear-mounted, looking backwards
- Poses from SLAM will be rotated (camera sees behind)
```

---

## 3. State Definitions & Transitions

### State: IDLE
```yaml
Entry:
  - Stop all motors
  - Clear navigation goal
  - Reset state variables
  
Transitions:
  - mission_command (GPS waypoint) → GPS_NAVIGATE
  - explore_command → EXPLORE_AREA
  - [Always monitoring heartbeat]
```

### State: GPS_NAVIGATE (Book It to Target)
```yaml
Entry:
  - Store target GPS location
  - Convert to local coordinates (map frame)
  - Initialize path planner
  - Start heartbeat watchdog (2 second timeout)

Execution:
  1. Get current pose from /slam/pose/fused
  2. Calculate vector to target
  3. Plan path (avoid obstacles from SLAM depth)
  4. Command velocities:
     - vx = -1.0 m/s (back up at 1.0 m/s)
     - ωz = proportional to bearing error
  5. Run ArUco detector on RGB frames
     - If ArUco detected in frame → APPROACH_TARGET
     - If confidence > threshold and distance < 5m
  6. Check heartbeat: if timeout → SIGNAL_LOST
  7. If within 1m of target → IDLE (success)

Obstacle Avoidance:
  - Use SLAM depth map (rear camera)
  - If obstacle detected within 0.5m → stop and re-plan
  - Update local costmap from /slam/depth/processed
  - DWA (Dynamic Window Approach) planner

Exit:
  - On target reached
  - On critical obstacle
  - On heartbeat timeout
  - On signal loss
```

### State: APPROACH_TARGET (ArUco Vision Guidance)
```yaml
Entry:
  - Store ArUco pose (from vision detector)
  - Switch to precision mode (slow speed)
  - Initialize vision-based guidance

Execution:
  1. Get ArUco detection: position, orientation, distance
  2. Calculate approach vector:
     - If ArUco off-center → adjust heading
     - If distance > 0.3m → back up slowly
     - If distance < 0.3m → stop (target reached!)
  3. Command velocities:
     - vx = -0.2 m/s (slow precision backing)
     - ωz = (error_angle * Kp) for centering
  4. Continuous ArUco tracking
  5. Check safety: maintain obstacle avoidance

Exit:
  - Distance < 0.3m → SUCCESS (go to IDLE)
  - ArUco lost → return to GPS_NAVIGATE
  - Obstacle detected → stop and re-plan
  - Heartbeat timeout → SIGNAL_LOST
```

### State: EXPLORE_AREA (Search & Map)
```yaml
Entry:
  - Store current pose as origin
  - Set max radius = 50m
  - Initialize spiral search pattern
  - Start exploration timer (30 min max)

Execution:
  1. Generate waypoints along expanding spiral:
     - Start at origin, spiral outward
     - Ring radius: 5m → 10m → 15m → 20m → 25m → 50m
     - Each ring: 8 waypoints (45° spacing)
     - Distance between rings: 5m
  
  2. For each waypoint:
     a. Navigate there (same as GPS_NAVIGATE)
     b. Scan 360° (rotate in place)
     c. Run ArUco detector
        - If ArUco found → APPROACH_TARGET
     d. Add to SLAM map
  
  3. Local mapping:
     - Continuous SLAM from rear RGB-D
     - Building local map of area
     - Detecting features, mapping terrain
  
  4. Termination conditions:
     - ArUco found → APPROACH_TARGET
     - Exploration timeout (30 min) → EXIT_EXPLORE → IDLE
     - Heartbeat timeout → SIGNAL_LOST
     - All waypoints visited → EXIT_EXPLORE → IDLE

Waypoint Generation (Spiral):
```python
def generate_spiral_waypoints(origin, max_radius=50, ring_spacing=5):
    waypoints = []
    ring = 0
    while ring * ring_spacing <= max_radius:
        radius = ring * ring_spacing
        num_points = 8  # 8 waypoints per ring (45° spacing)
        for i in range(num_points):
            angle = (i / num_points) * 2 * π
            x = origin.x + radius * cos(angle)
            y = origin.y + radius * sin(angle)
            waypoints.append((x, y))
        ring += 1
    return waypoints
```
```

### State: SIGNAL_LOST (Failsafe Retreat)
```yaml
Entry:
  - Stop new navigation commands
  - Activate retreat protocol
  - Start beacon (if available)
  - Log event with timestamp

Execution:
  1. Use SLAM to navigate back:
     - Reverse the path taken (breadcrumb trail)
     - OR use SLAM map to navigate to known safe zone
     - OR simple straight-line retreat toward origin
  
  2. Navigation logic:
     - Maintain distance from obstacles (0.5m clearance)
     - Speed = 0.5 m/s (cautious)
     - Keep heading away from explored area
  
  3. Retreat modes:
     a. Breadcrumb Trail (if available)
        - Follow inverse of forward path
        - Use SLAM loop closure to confirm location
     
     b. Origin Retreat
        - Calculate bearing to origin from SLAM pose
        - Back up in that direction
        - Track progress
     
     c. Obstacle Avoidance
        - Keep rear camera obstacle detection active
        - Stop if obstacle within 0.3m

  4. Monitoring:
     - Check for signal recovery
       - If signal returns within timeout (10 sec) → IDLE
     - Check for reaching safe zone
       - If distance_to_origin < 2m → IDLE
     - Check exploration timeout
       - If > 60 min in this state → force IDLE

Exit:
  - Signal recovered → IDLE
  - Reached safe zone → IDLE
  - Critical emergency → EMERGENCY_STOP
```

---

## 4. ArUco Tag Detection System

### Integration with SLAM

```
RGB from rear camera:
  ├─ SLAM feature extraction (ORB for mapping)
  ├─ ArUco detection (parallel process)
  └─ Frame published for logging

ArUco Detection Pipeline:
  1. Input: RGB frames from /slam/rgb/image
  2. Detection:
     - Detect ArUco markers
     - Estimate pose using camera intrinsics
     - Calculate distance and bearing
  3. Output: ArUco pose + confidence
```

### ArUco Detection Node

```python
class ArUcoDetector(Node):
    def __init__(self):
        self.detector = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()
        self.camera_matrix = load_calibration()  # From calibration subsystem
        self.dist_coeffs = load_distortion()
        
        self.aruco_pub = self.create_publisher(ArUcoDetection, '/aruco/detection', 10)
        self.rgb_sub = self.create_subscription(Image, '/slam/rgb/image', self.detect_aruco, 10)
        self.camera_info_sub = self.create_subscription(CameraInfo, '/slam/depth/camera_info', 
                                                         self.on_camera_info, 10)

    def detect_aruco(self, rgb_msg):
        rgb_cv = cv2_bridge.imgmsg_to_cv2(rgb_msg, 'bgr8')
        detector = cv2.aruco.ArucoDetector(self.detector, self.parameters)
        corners, ids, rejected = detector.detectMarkers(rgb_cv)
        
        if ids is None:
            return  # No tags found
        
        # Estimate poses
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, marker_size=0.05, cameraMatrix=self.camera_matrix,
            distCoeffs=self.dist_coeffs
        )
        
        for i, tag_id in enumerate(ids):
            # Convert to ROS message
            detection = ArUcoDetection()
            detection.tag_id = int(tag_id)
            detection.distance = np.linalg.norm(tvecs[i])
            detection.bearing = np.arctan2(tvecs[i][0, 0], tvecs[i][0, 2])
            detection.confidence = 0.95  # High confidence if detected
            
            # Store pose for approach_target
            detection.pose.position.x = tvecs[i][0, 0]
            detection.pose.position.y = tvecs[i][0, 1]
            detection.pose.position.z = tvecs[i][0, 2]
            
            self.aruco_pub.publish(detection)
```

### Custom Message: ArUcoDetection

```yaml
# autonomy_interfaces/msg/ArUcoDetection.msg
int32 tag_id                      # ID of detected tag
float32 distance                  # Distance from camera (meters)
float32 bearing                   # Bearing in camera frame (radians)
float32 confidence                # Detection confidence [0-1]
geometry_msgs/PoseStamped pose    # 3D pose in camera frame
```

---

## 5. Path Planning & Obstacle Avoidance

### Dynamic Window Approach (DWA) for Reverse Navigation

```python
class ReverseNavigationPlanner(Node):
    def __init__(self):
        self.max_speed = 1.0  # m/s backward
        self.max_angular = 0.8  # rad/s
        self.prediction_horizon = 1.0  # seconds
        
    def plan_reverse_motion(self, current_pose, target, obstacles):
        """
        Plan motion backward from current_pose toward target
        while avoiding obstacles.
        
        Args:
            current_pose: (x, y, theta) in map frame
            target: (x, y) in map frame
            obstacles: list of obstacle positions from SLAM depth
        
        Returns:
            (vx, ωz) - velocity commands (negative vx for backup)
        """
        # Calculate bearing to target (in reverse)
        dx = target[0] - current_pose[0]
        dy = target[1] - current_pose[1]
        bearing = np.arctan2(dy, dx)
        
        # Current heading (looking backwards, so add π)
        current_heading = current_pose[2] + np.pi
        
        # Angle error (keep in [-π, π])
        angle_error = self._normalize_angle(bearing - current_heading)
        
        # Calculate velocities
        vx = -1.0  # Back up (negative in body frame)
        ωz = self._calculate_angular_velocity(angle_error)
        
        # Check for obstacles
        if self._collision_imminent(current_pose, vx, ωz, obstacles):
            vx = 0.0  # Stop
            ωz = self._plan_escape(current_pose, obstacles)
        
        return vx, ωz
    
    def _collision_imminent(self, pose, vx, ωz, obstacles, horizon=1.0):
        """Check if trajectory will collide within prediction horizon"""
        # Simulate motion forward
        steps = int(horizon / 0.01)
        for step in range(steps):
            future_pose = self._integrate_motion(pose, vx, ωz, 0.01)
            
            # Check distance to obstacles
            for obs in obstacles:
                dist = np.sqrt((future_pose[0] - obs[0])**2 + 
                              (future_pose[1] - obs[1])**2)
                if dist < 0.3:  # 30cm safety margin
                    return True
        
        return False
    
    def _calculate_angular_velocity(self, angle_error, kp=1.0):
        """Calculate angular velocity to correct heading"""
        return np.clip(angle_error * kp, -self.max_angular, self.max_angular)
    
    @staticmethod
    def _normalize_angle(angle):
        """Normalize angle to [-π, π]"""
        while angle > np.pi:
            angle -= 2 * np.pi
        while angle < -np.pi:
            angle += 2 * np.pi
        return angle
```

### Obstacle Detection from SLAM Depth

```python
class ObstacleDetector(Node):
    def __init__(self):
        self.depth_sub = self.create_subscription(Image, '/slam/depth/processed', 
                                                   self.on_depth, 10)
        self.cam_info_sub = self.create_subscription(CameraInfo, '/slam/depth/camera_info',
                                                      self.on_camera_info, 10)
        self.obstacles_pub = self.create_publisher(PointCloud2, '/obstacles', 10)
    
    def on_depth(self, depth_msg):
        """Convert depth image to obstacle list"""
        depth_cv = cv2_bridge.imgmsg_to_cv2(depth_msg)
        
        # Find pixels with obstacles (depth < 0.5m, near rear of rover)
        obstacle_mask = (depth_cv > 50) & (depth_cv < 500)  # mm
        
        # Convert to point cloud
        obstacles = []
        for v, u in np.argwhere(obstacle_mask):
            # Back-project to 3D using camera intrinsics
            Z = depth_cv[v, u] / 1000.0  # Convert mm to meters
            X = (u - self.cx) * Z / self.fx
            Y = (v - self.cy) * Z / self.fy
            
            # Transform from camera frame to base_link
            # (rear camera looks backward)
            point_base = self.transform_camera_to_base(X, Y, Z)
            obstacles.append(point_base[:2])  # (x, y) in base frame
        
        # Publish for navigation
        self.publish_obstacles(obstacles)
```

---

## 6. Navigation Node Architecture

### Main Navigation Node

```python
#!/usr/bin/env python3
"""
Autonomous Navigation Controller

Multi-state navigation with:
- GPS waypoint following
- ArUco tag detection and approach
- Exploration mode with spiral search
- Signal loss failsafe
- Obstacle avoidance
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from enum import Enum
import numpy as np
from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import Image
from std_msgs.msg import String
from nav_msgs.msg import Path
import tf2_ros
from tf2_ros import TransformListener, TransformBroadcaster
import time

class NavigationState(Enum):
    IDLE = 0
    GPS_NAVIGATE = 1
    APPROACH_TARGET = 2
    EXPLORE_AREA = 3
    SIGNAL_LOST = 4
    EMERGENCY_STOP = 5

class AutonomousNavigationController(Node):
    def __init__(self):
        super().__init__('autonomous_navigation')
        
        self.callback_group = ReentrantCallbackGroup()
        
        # State management
        self.state = NavigationState.IDLE
        self.previous_state = NavigationState.IDLE
        
        # Heartbeat monitoring
        self.last_heartbeat = time.time()
        self.heartbeat_timeout = 2.0  # seconds
        
        # Navigation goals
        self.gps_target = None
        self.aruco_target = None
        self.explore_origin = None
        self.explore_radius = 50.0
        self.explore_waypoints = []
        self.current_waypoint_idx = 0
        
        # Controllers
        self.planner = ReverseNavigationPlanner()
        self.aruco_detector = None
        
        # TF2
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.nav_status_pub = self.create_publisher(String, '/nav/status', 10)
        self.path_pub = self.create_publisher(Path, '/nav/path', 10)
        
        # Subscribers
        self.mission_sub = self.create_subscription(
            PoseStamped, '/nav/mission_target', self.on_mission_target, 10,
            callback_group=self.callback_group)
        
        self.aruco_sub = self.create_subscription(
            ArUcoDetection, '/aruco/detection', self.on_aruco_detection, 10,
            callback_group=self.callback_group)
        
        self.heartbeat_sub = self.create_subscription(
            String, '/heartbeat', self.on_heartbeat, 10,
            callback_group=self.callback_group)
        
        self.slam_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, '/slam/pose/fused', self.on_slam_pose, 10,
            callback_group=self.callback_group)
        
        # Main control loop
        self.control_timer = self.create_timer(0.1, self.control_loop, 
                                              callback_group=self.callback_group)
        
        # Heartbeat watchdog
        self.watchdog_timer = self.create_timer(0.5, self.heartbeat_watchdog,
                                               callback_group=self.callback_group)
        
        self.get_logger().info('Navigation controller initialized')
    
    def control_loop(self):
        """Main control loop - 10 Hz"""
        
        # Check for state transitions
        self._check_state_transitions()
        
        # Execute current state
        if self.state == NavigationState.IDLE:
            self._execute_idle()
        elif self.state == NavigationState.GPS_NAVIGATE:
            self._execute_gps_navigate()
        elif self.state == NavigationState.APPROACH_TARGET:
            self._execute_approach_target()
        elif self.state == NavigationState.EXPLORE_AREA:
            self._execute_explore_area()
        elif self.state == NavigationState.SIGNAL_LOST:
            self._execute_signal_lost()
        elif self.state == NavigationState.EMERGENCY_STOP:
            self._execute_emergency_stop()
        
        self._publish_status()
    
    def on_mission_target(self, msg):
        """Receive GPS waypoint mission"""
        self.gps_target = (msg.pose.position.x, msg.pose.position.y)
        self.state = NavigationState.GPS_NAVIGATE
        self.get_logger().info(f'GPS target received: {self.gps_target}')
    
    def on_aruco_detection(self, msg):
        """Receive ArUco detection"""
        if self.state == NavigationState.GPS_NAVIGATE:
            # Switch to approach if tag close and confident
            if msg.confidence > 0.8 and msg.distance < 5.0:
                self.aruco_target = msg
                self.state = NavigationState.APPROACH_TARGET
                self.get_logger().info(f'ArUco detected: ID {msg.tag_id}, distance {msg.distance:.2f}m')
        
        elif self.state == NavigationState.EXPLORE_AREA:
            # Found target while exploring
            if msg.confidence > 0.8:
                self.aruco_target = msg
                self.state = NavigationState.APPROACH_TARGET
                self.get_logger().info(f'ArUco found during exploration!')
    
    def on_heartbeat(self, msg):
        """Receive heartbeat from remote control"""
        self.last_heartbeat = time.time()
    
    def on_slam_pose(self, msg):
        """Receive current pose from SLAM"""
        self.current_pose = (msg.pose.pose.position.x, 
                            msg.pose.pose.position.y,
                            self._get_yaw_from_quaternion(msg.pose.pose.orientation))
    
    def heartbeat_watchdog(self):
        """Monitor heartbeat timeout"""
        if time.time() - self.last_heartbeat > self.heartbeat_timeout:
            if self.state not in [NavigationState.IDLE, NavigationState.EMERGENCY_STOP]:
                self.get_logger().warn('Heartbeat timeout! Entering SIGNAL_LOST')
                self.state = NavigationState.SIGNAL_LOST
    
    def _execute_idle(self):
        """Stop all motion"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def _execute_gps_navigate(self):
        """Navigate to GPS target with obstacle avoidance"""
        if not hasattr(self, 'current_pose'):
            return
        
        # Plan motion
        vx, omega_z = self.planner.plan_reverse_motion(
            self.current_pose, self.gps_target, self._get_obstacles())
        
        # Check if reached target
        dist_to_target = np.sqrt((self.current_pose[0] - self.gps_target[0])**2 +
                                 (self.current_pose[1] - self.gps_target[1])**2)
        if dist_to_target < 1.0:
            self.state = NavigationState.IDLE
            self.get_logger().info('GPS target reached!')
            return
        
        # Send velocity command
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = omega_z
        self.cmd_vel_pub.publish(twist)
    
    def _execute_approach_target(self):
        """Approach ArUco tag with vision guidance"""
        if self.aruco_target is None:
            self.state = NavigationState.IDLE
            return
        
        # Extract ArUco pose
        distance = self.aruco_target.distance
        bearing = self.aruco_target.bearing
        
        if distance < 0.3:
            # Target reached!
            self.state = NavigationState.IDLE
            twist = Twist()
            self.cmd_vel_pub.publish(twist)
            self.get_logger().info('Target approached successfully!')
            return
        
        # Plan approach motion
        vx = -0.2 if distance > 0.1 else 0.0  # Slow backing
        omega_z = bearing * 0.5  # Proportional steering
        
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = omega_z
        self.cmd_vel_pub.publish(twist)
    
    def _execute_explore_area(self):
        """Explore area with spiral search"""
        if not hasattr(self, 'current_pose'):
            return
        
        # If no waypoints generated yet
        if not self.explore_waypoints:
            self.explore_origin = self.current_pose[:2]
            self.explore_waypoints = self._generate_spiral_waypoints(
                self.explore_origin, self.explore_radius)
            self.current_waypoint_idx = 0
        
        # Navigate to current waypoint
        current_wp = self.explore_waypoints[self.current_waypoint_idx]
        vx, omega_z = self.planner.plan_reverse_motion(
            self.current_pose, current_wp, self._get_obstacles())
        
        # Check if reached waypoint
        dist_to_wp = np.sqrt((self.current_pose[0] - current_wp[0])**2 +
                            (self.current_pose[1] - current_wp[1])**2)
        if dist_to_wp < 0.5:
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.explore_waypoints):
                self.state = NavigationState.IDLE
                self.get_logger().info('Exploration complete!')
                return
        
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = omega_z
        self.cmd_vel_pub.publish(twist)
    
    def _execute_signal_lost(self):
        """Retreat to safe zone on signal loss"""
        if not hasattr(self, 'current_pose'):
            return
        
        # Simple retreat toward origin
        if not hasattr(self, 'retreat_origin'):
            self.retreat_origin = self.explore_origin if self.explore_origin else self.current_pose[:2]
        
        distance_to_origin = np.sqrt((self.current_pose[0] - self.retreat_origin[0])**2 +
                                    (self.current_pose[1] - self.retreat_origin[1])**2)
        
        if distance_to_origin < 2.0:
            # Reached safe zone
            self.state = NavigationState.IDLE
            self.get_logger().info('Safely retreated!')
            return
        
        # Back up slowly toward origin
        vx, omega_z = self.planner.plan_reverse_motion(
            self.current_pose, self.retreat_origin, self._get_obstacles())
        
        # Limit speed for safety
        vx = np.clip(vx, -0.5, 0.5)
        
        twist = Twist()
        twist.linear.x = vx
        twist.angular.z = omega_z
        self.cmd_vel_pub.publish(twist)
    
    def _execute_emergency_stop(self):
        """Emergency stop - send zero velocities"""
        twist = Twist()
        self.cmd_vel_pub.publish(twist)
    
    def _check_state_transitions(self):
        """Check for forced state transitions"""
        # Critical safety always takes priority
        if self._check_critical_safety():
            self.state = NavigationState.EMERGENCY_STOP
    
    def _check_critical_safety(self):
        """Check for critical safety conditions"""
        # Could check: imminent collision, thermal shutdown, etc.
        return False
    
    def _generate_spiral_waypoints(self, origin, max_radius, ring_spacing=5.0):
        """Generate spiral search waypoints"""
        waypoints = []
        ring = 0
        while ring * ring_spacing <= max_radius:
            radius = ring * ring_spacing
            if radius == 0:
                waypoints.append(origin)
            else:
                num_points = 8
                for i in range(num_points):
                    angle = (i / num_points) * 2 * np.pi
                    x = origin[0] + radius * np.cos(angle)
                    y = origin[1] + radius * np.sin(angle)
                    waypoints.append((x, y))
            ring += 1
        return waypoints
    
    def _get_obstacles(self):
        """Get obstacle positions from SLAM depth"""
        # Would subscribe to /obstacles topic
        return []
    
    def _get_yaw_from_quaternion(self, quat):
        """Extract yaw from quaternion"""
        return np.arctan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
    
    def _publish_status(self):
        """Publish navigation status"""
        status = String()
        status.data = f'State: {self.state.name}'
        self.nav_status_pub.publish(status)

def main(args=None):
    rclpy.init(args=args)
    node = AutonomousNavigationController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 7. Integration Points

### With SLAM System
```
/slam/pose/fused         → Navigation: Current position
/slam/rgb/image          → ArUco Detection: Tag detection
/slam/depth/processed    → Obstacle Detection: Avoidance
/slam/system/health      → Navigation: Monitor SLAM status
```

### With Control System
```
/cmd_vel                 → Motor Controller: Velocity commands
                            vx = backward speed (-1.0 to 1.0 m/s)
                            ωz = yaw rate (-0.8 to 0.8 rad/s)
```

### With Mission Control
```
/nav/mission_target      ← Remote: GPS waypoint
/heartbeat              ← Remote: Alive signal
/nav/status             → Remote: Current navigation state
```

---

## 8. Launch File

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # ArUco Detection
        Node(
            package='autonomy_navigation',
            executable='aruco_detector',
            name='aruco_detector',
            output='screen',
            parameters=[{
                'marker_size': 0.05,
                'confidence_threshold': 0.8,
            }]
        ),
        
        # Obstacle Detection
        Node(
            package='autonomy_navigation',
            executable='obstacle_detector',
            name='obstacle_detector',
            output='screen',
            parameters=[{
                'min_obstacle_height': 0.05,
                'max_range': 5.0,
            }]
        ),
        
        # Navigation Controller
        Node(
            package='autonomy_navigation',
            executable='navigation_controller',
            name='navigation_controller',
            output='screen',
            parameters=[{
                'heartbeat_timeout': 2.0,
                'explore_radius': 50.0,
                'max_reverse_speed': 1.0,
                'max_angular_speed': 0.8,
            }]
        ),
    ])
```

---

## 9. Message Definitions

### Autonomy Interfaces Extensions

```yaml
# autonomy_interfaces/msg/ArUcoDetection.msg
int32 tag_id
float32 distance                  # meters
float32 bearing                   # radians
float32 confidence               # [0-1]
geometry_msgs/PoseStamped pose
---

# autonomy_interfaces/srv/NavigationCommand.srv
geometry_msgs/PoseStamped target
string command                   # "gps_nav", "explore", "approach_target"
---
int32 result                     # 0=pending, 1=success, 2=failure
```

---

## 10. Reverse Navigation Considerations

### Why Rear-Facing SLAM Works
1. **SLAM is agnostic to direction** - Only tracks motion and mapping
2. **Camera looks backward** - Sees obstacles behind rover
3. **Navigation adds direction** - Controller interprets "backing up" as negative vx
4. **Transforms handle orientation** - TF2 tracks camera → body → map

### Challenges & Solutions

| Challenge | Solution |
|-----------|----------|
| Rear camera can't see ahead | Use SLAM map for lookahead; slower speeds |
| Turning while backing | Simultaneous vx + ωz commands work in reverse |
| Feature-sparse behind rover | Spiral search covers area; SLAM + GPS fusion |
| Operator confusion | Clear state machine; telemetry feedback |

---

## 11. Safety Features

### Hard Stops
- Emergency stop on critical sensor failure
- Collision imminent detection
- Thermal/power monitoring

### Graceful Degradation
- No GPS → use SLAM-only + explore
- No ArUco → continue GPS nav or retreat
- SLAM loss → switch to GPS, then retreat

### Operator Override
- Heartbeat timeout → automatic retreat
- Manual signal loss recovery (5-10 sec window)
- Full telemetry feedback

---

## 12. Testing Strategy

### Unit Tests
- [x] Spiral waypoint generation
- [x] Angle normalization
- [x] Distance calculations
- [ ] Velocity command generation

### Integration Tests
- [ ] SLAM pose → navigation commands
- [ ] ArUco detection → state transition
- [ ] Obstacle avoidance response
- [ ] Signal loss retreat

### Field Testing
- [ ] GPS navigation in arena
- [ ] ArUco approach precision
- [ ] Exploration coverage (50m radius)
- [ ] Signal loss recovery
- [ ] Full mission scenarios

---

**Status**: Design Complete - Ready for Implementation

**Next Steps**: 
1. Create ROS 2 package structure
2. Implement navigation nodes
3. Integrate with SLAM system
4. Field testing & parameter tuning

