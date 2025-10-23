# Code Templates and Style Guide

This directory contains templates and examples for writing clean, maintainable ROS 2 code for the URC 2026 autonomy system.

## 📋 Style Guide

### Python Code Standards

#### Naming Conventions
```python
# Classes: PascalCase
class NavigationNode(Node):
    pass

class TerrainClassifier:
    pass

# Functions and methods: snake_case
def calculate_trajectory(self, start_pose, goal_pose):
    pass

def _validate_inputs(self, data):
    pass  # Private methods start with _

# Variables: snake_case
current_velocity = 0.5
sensor_readings = {}
is_obstacle_detected = False

# Constants: UPPER_SNAKE_CASE
MAX_VELOCITY = 2.0
SENSOR_TIMEOUT = 1.0
DEFAULT_FRAME_ID = "base_link"
```

#### Code Structure
```python
# File header
"""
URC 2026 Autonomy System - Navigation Node

Provides waypoint navigation and obstacle avoidance capabilities.

Author: [Your Name]
Date: [Date]
"""

# Imports (grouped and sorted)
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import numpy as np

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry, Path
from sensor_msgs.msg import LaserScan

# Constants
DEFAULT_UPDATE_RATE = 10.0  # Hz
MAX_LINEAR_VELOCITY = 1.0   # m/s
MAX_ANGULAR_VELOCITY = 1.0  # rad/s

class NavigationNode(Node):
    """ROS 2 node for autonomous navigation.

    This node implements waypoint navigation with obstacle avoidance
    using sensor fusion from LIDAR, IMU, and GPS data.
    """

    def __init__(self):
        """Initialize the navigation node."""
        super().__init__('navigation_node')

        # Declare parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', DEFAULT_UPDATE_RATE),
                ('max_linear_velocity', MAX_LINEAR_VELOCITY),
                ('max_angular_velocity', MAX_ANGULAR_VELOCITY),
                ('goal_tolerance', 0.1),
                ('obstacle_margin', 0.3),
            ]
        )

        # Get parameters
        self.update_rate = self.get_parameter('update_rate').value
        self.max_linear_vel = self.get_parameter('max_linear_velocity').value
        self.max_angular_vel = self.get_parameter('max_angular_velocity').value

        # Setup QoS profiles
        self._setup_qos_profiles()

        # Create publishers
        self._setup_publishers()

        # Create subscribers
        self._setup_subscribers()

        # Create services
        self._setup_services()

        # Create timers
        self._setup_timers()

        # Initialize state
        self._initialize_state()

        self.get_logger().info('Navigation node initialized')

    def _setup_qos_profiles(self):
        """Setup QoS profiles for different message types."""
        self.qos_sensor = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        self.qos_control = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            depth=5
        )

    def _setup_publishers(self):
        """Setup ROS 2 publishers."""
        self.cmd_vel_pub = self.create_publisher(
            Twist, 'cmd_vel', self.qos_control
        )

        self.path_pub = self.create_publisher(
            Path, 'planned_path', self.qos_control
        )

    def _setup_subscribers(self):
        """Setup ROS 2 subscribers."""
        self.odom_sub = self.create_subscription(
            Odometry, 'odom', self.odom_callback,
            self.qos_sensor
        )

        self.scan_sub = self.create_subscription(
            LaserScan, 'scan', self.scan_callback,
            self.qos_sensor
        )

        self.goal_sub = self.create_subscription(
            PoseStamped, 'goal_pose', self.goal_callback,
            self.qos_control
        )

    def _setup_services(self):
        """Setup ROS 2 services."""
        self.stop_service = self.create_service(
            Trigger, 'stop_navigation',
            self.stop_navigation_callback
        )

    def _setup_timers(self):
        """Setup ROS 2 timers."""
        self.control_timer = self.create_timer(
            1.0 / self.update_rate, self.control_loop
        )

    def _initialize_state(self):
        """Initialize node state variables."""
        self.current_pose = None
        self.goal_pose = None
        self.current_velocity = Twist()
        self.is_navigating = False
        self.obstacle_detected = False

    # Callback methods
    def odom_callback(self, msg):
        """Handle odometry updates."""
        self.current_pose = msg.pose.pose
        # Process odometry data

    def scan_callback(self, msg):
        """Handle LIDAR scan data."""
        # Process scan data for obstacle detection
        self.obstacle_detected = self._check_obstacles(msg)

    def goal_callback(self, msg):
        """Handle navigation goal updates."""
        self.goal_pose = msg.pose
        self.is_navigating = True
        self.get_logger().info(f'New goal received: {self.goal_pose}')

    def stop_navigation_callback(self, request, response):
        """Handle stop navigation service calls."""
        self.is_navigating = False
        self.current_velocity = Twist()
        self.cmd_vel_pub.publish(self.current_velocity)

        response.success = True
        response.message = "Navigation stopped"
        return response

    def control_loop(self):
        """Main control loop executed at update rate."""
        if not self.is_navigating or self.current_pose is None:
            return

        if self.obstacle_detected:
            self._handle_obstacle()
        else:
            self._navigate_to_goal()

    def _check_obstacles(self, scan_msg):
        """Check for obstacles in the scan data."""
        # Implementation for obstacle detection
        return False

    def _handle_obstacle(self):
        """Handle obstacle avoidance behavior."""
        # Implementation for obstacle avoidance
        pass

    def _navigate_to_goal(self):
        """Navigate towards the current goal."""
        # Implementation for goal navigation
        pass

def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    try:
        node = NavigationNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        node.get_logger().error(f'Node failed: {e}')
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### Error Handling Patterns

#### Exception Handling
```python
def safe_sensor_callback(self, msg):
    """Safely handle sensor callbacks with error recovery."""
    try:
        self.process_sensor_data(msg)
    except ValueError as e:
        self.get_logger().warn(f'Invalid sensor data: {e}')
    except Exception as e:
        self.get_logger().error(f'Sensor callback failed: {e}')
        self._handle_sensor_error()

def _handle_sensor_error(self):
    """Handle sensor errors gracefully."""
    # Implement error recovery logic
    # Switch to backup sensors, reduce functionality, etc.
    pass
```

#### Validation Functions
```python
def validate_pose(self, pose):
    """Validate pose message integrity."""
    if pose is None:
        raise ValueError("Pose cannot be None")

    if not hasattr(pose, 'position') or not hasattr(pose, 'orientation'):
        raise ValueError("Invalid pose structure")

    # Check for NaN/inf values
    if not self._is_finite_pose(pose):
        raise ValueError("Pose contains non-finite values")

    return True

def _is_finite_pose(self, pose):
    """Check if pose contains only finite values."""
    pos = pose.position
    ori = pose.orientation

    values = [pos.x, pos.y, pos.z, ori.x, ori.y, ori.z, ori.w]
    return all(np.isfinite(v) for v in values)
```

### Logging Standards

#### Log Levels and Usage
```python
# DEBUG: Detailed diagnostic information
self.get_logger().debug(f'Control loop iteration: dt={dt:.3f}')

# INFO: General information about node operation
self.get_logger().info('Navigation goal reached successfully')

# WARN: Potentially harmful situations
self.get_logger().warn(f'Sensor timeout: {sensor_name}')

# ERROR: Error conditions that don't stop operation
self.get_logger().error(f'Failed to process {message_type}: {e}')

# FATAL: Severe errors that stop operation
self.get_logger().fatal('Critical sensor failure, shutting down')
```

#### Structured Logging
```python
def log_navigation_status(self):
    """Log current navigation status with context."""
    self.get_logger().info(
        'Navigation status update',
        extra={
            'is_navigating': self.is_navigating,
            'distance_to_goal': self._calculate_distance_to_goal(),
            'current_velocity': self.current_velocity.linear.x,
            'obstacle_detected': self.obstacle_detected
        }
    )
```

## 📋 What Else Should Be Included

### Testing Framework
```python
# tests/test_navigation_node.py
import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from autonomy.navigation.navigation_node import NavigationNode

class TestNavigationNode:
    """Test cases for NavigationNode."""

    @pytest.fixture
    def node(self):
        """Fixture to create a navigation node."""
        rclpy.init()
        node = NavigationNode()
        yield node
        node.destroy_node()
        rclpy.shutdown()

    def test_initialization(self, node):
        """Test node initializes correctly."""
        assert node.update_rate == DEFAULT_UPDATE_RATE
        assert node.is_navigating == False
        assert node.goal_pose is None

    def test_goal_callback(self, node):
        """Test goal pose callback."""
        goal_msg = PoseStamped()
        goal_msg.pose.position.x = 1.0
        goal_msg.pose.position.y = 2.0

        node.goal_callback(goal_msg)

        assert node.is_navigating == True
        assert node.goal_pose == goal_msg.pose

    def test_validate_pose_valid(self, node):
        """Test pose validation with valid pose."""
        pose = PoseStamped().pose
        pose.position.x = 1.0
        pose.orientation.w = 1.0

        assert node.validate_pose(pose) == True

    def test_validate_pose_invalid(self, node):
        """Test pose validation with invalid pose."""
        with pytest.raises(ValueError):
            node.validate_pose(None)

        pose = PoseStamped().pose
        pose.position.x = float('nan')

        with pytest.raises(ValueError):
            node.validate_pose(pose)
```

### Documentation Standards
```python
class NavigationNode(Node):
    """ROS 2 node for autonomous navigation.

    This node provides waypoint navigation capabilities with obstacle avoidance
    using sensor fusion from multiple sources.

    Attributes:
        update_rate (float): Control loop update rate in Hz
        max_linear_vel (float): Maximum linear velocity in m/s
        max_angular_vel (float): Maximum angular velocity in rad/s

    Publishers:
        cmd_vel (geometry_msgs/Twist): Velocity commands for base controller
        planned_path (nav_msgs/Path): Planned navigation path

    Subscribers:
        odom (nav_msgs/Odometry): Odometry feedback from base controller
        scan (sensor_msgs/LaserScan): LIDAR scan data for obstacle detection
        goal_pose (geometry_msgs/PoseStamped): Navigation goal positions

    Services:
        stop_navigation (std_srvs/Trigger): Emergency stop service

    Parameters:
        update_rate (float): Control loop update rate (default: 10.0)
        max_linear_velocity (float): Maximum linear velocity (default: 1.0)
        max_angular_velocity (float): Maximum angular velocity (default: 1.0)
        goal_tolerance (float): Distance tolerance for goal reaching (default: 0.1)
        obstacle_margin (float): Safety margin around obstacles (default: 0.3)
    """
```

### Configuration Management
```python
# config/navigation_config.yaml
navigation:
  update_rate: 10.0
  max_linear_velocity: 1.0
  max_angular_velocity: 1.0
  goal_tolerance: 0.1
  obstacle_margin: 0.3

  # Path planning parameters
  path_planning:
    algorithm: "a_star"
    grid_resolution: 0.05
    max_planning_time: 1.0

  # Obstacle avoidance parameters
  obstacle_avoidance:
    enabled: true
    safety_distance: 0.3
    avoidance_gain: 0.5
    max_avoidance_angle: 1.57  # 90 degrees
```

### Performance Monitoring
```python
class PerformanceMonitor:
    """Monitor node performance metrics."""

    def __init__(self, node):
        self.node = node
        self.metrics = {
            'control_loop_time': [],
            'callback_count': 0,
            'error_count': 0
        }
        self.monitor_timer = node.create_timer(1.0, self._log_metrics)

    def measure_execution_time(self, func):
        """Decorator to measure function execution time."""
        def wrapper(*args, **kwargs):
            start_time = time.time()
            result = func(*args, **kwargs)
            execution_time = time.time() - start_time

            metric_name = f"{func.__name__}_time"
            if metric_name not in self.metrics:
                self.metrics[metric_name] = []

            self.metrics[metric_name].append(execution_time)

            # Keep only last 100 measurements
            if len(self.metrics[metric_name]) > 100:
                self.metrics[metric_name].pop(0)

            return result
        return wrapper

    def _log_metrics(self):
        """Log performance metrics."""
        for metric_name, values in self.metrics.items():
            if values and metric_name.endswith('_time'):
                avg_time = sum(values) / len(values)
                max_time = max(values)
                self.node.get_logger().info(
                    f'Performance: {metric_name} - avg: {avg_time:.3f}s, max: {max_time:.3f}s'
                )
```

### Health Monitoring
```python
class HealthMonitor(Node):
    """Monitor system health and publish diagnostics."""

    def __init__(self):
        super().__init__('health_monitor')

        # Create diagnostic publisher
        self.diag_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', 10
        )

        # Monitor timers for different components
        self.create_timer(1.0, self._check_navigation_health)
        self.create_timer(5.0, self._check_sensor_health)
        self.create_timer(30.0, self._check_system_health)

    def _check_navigation_health(self):
        """Check navigation system health."""
        diag = DiagnosticStatus()
        diag.name = "Navigation System"
        diag.level = DiagnosticStatus.OK

        # Check if navigation node is responsive
        # Check if path planning is working
        # Check control loop timing

        diag.message = "Navigation system healthy"
        self._publish_diagnostic(diag)

    def _check_sensor_health(self):
        """Check sensor health."""
        diag = DiagnosticStatus()
        diag.name = "Sensor Suite"
        diag.level = DiagnosticStatus.OK

        # Check sensor timestamps
        # Check data validity
        # Check communication status

        diag.message = "All sensors operational"
        self._publish_diagnostic(diag)

    def _publish_diagnostic(self, status):
        """Publish diagnostic status."""
        array = DiagnosticArray()
        array.header.stamp = self.get_clock().now().to_msg()
        array.status = [status]
        self.diag_pub.publish(array)
```

## 🧪 Testing Standards

### Unit Testing
```python
# tests/test_terrain_classifier.py
import pytest
import numpy as np
from autonomy.navigation.terrain_classifier import TerrainClassifier

class TestTerrainClassifier:
    @pytest.fixture
    def classifier(self):
        return TerrainClassifier()

    def test_classify_flat_terrain(self, classifier):
        # Test flat terrain classification
        elevation_map = np.zeros((10, 10))
        roughness = classifier.calculate_roughness(elevation_map)
        traversability = classifier.classify_traversability(elevation_map)

        assert roughness < 0.1
        assert traversability == TerrainType.FLAT

    def test_classify_rough_terrain(self, classifier):
        # Test rough terrain classification
        elevation_map = np.random.normal(0, 0.5, (10, 10))
        roughness = classifier.calculate_roughness(elevation_map)
        traversability = classifier.classify_traversability(elevation_map)

        assert roughness > 0.2
        assert traversability == TerrainType.ROUGH

    @pytest.mark.parametrize("terrain_type,expected_traversability", [
        ("flat", TerrainType.FLAT),
        ("slope", TerrainType.SLOPING),
        ("rough", TerrainType.ROUGH),
        ("obstacle", TerrainType.OBSTACLE),
    ])
    def test_terrain_type_classification(self, classifier, terrain_type, expected_traversability):
        # Parametrized test for different terrain types
        elevation_map = self._generate_terrain_map(terrain_type)
        traversability = classifier.classify_traversability(elevation_map)
        assert traversability == expected_traversability
```

### Integration Testing
```python
# tests/integration/test_navigation_stack.py
import pytest
import rclpy
import time
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

class TestNavigationStack:
    @pytest.fixture(scope="class")
    def ros_context(self):
        rclpy.init()
        yield
        rclpy.shutdown()

    def test_complete_navigation_workflow(self, ros_context):
        """Test complete navigation from goal to completion."""

        # Launch navigation stack
        # This would typically use launch_testing

        # Create test node
        test_node = rclpy.create_node('test_node')

        # Send navigation goal
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = "map"
        goal_msg.pose.position.x = 5.0
        goal_msg.pose.position.y = 0.0
        goal_msg.pose.orientation.w = 1.0

        goal_publisher = test_node.create_publisher(
            PoseStamped, '/goal_pose', 10
        )
        goal_publisher.publish(goal_msg)

        # Wait for navigation to start
        time.sleep(1.0)

        # Monitor progress
        odom_sub = test_node.create_subscription(
            Odometry, '/odom', self.odom_callback, 10
        )

        # Wait for goal completion or timeout
        timeout = 30.0  # seconds
        start_time = time.time()

        while time.time() - start_time < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)

            if self.goal_reached:
                break

        assert self.goal_reached, "Navigation did not reach goal within timeout"
```

## 📊 Code Quality Standards

### Code Metrics
```bash
# Run code quality checks
flake8 autonomy/ --max-line-length=88 --extend-ignore=E203,W503
black --check --diff autonomy/
isort --check-only --diff autonomy/
mypy autonomy/ --ignore-missing-imports
```

### Performance Benchmarks
```python
# benchmarks/benchmark_navigation.py
import time
import numpy as np
from autonomy.navigation.path_planner import PathPlanner

def benchmark_path_planning():
    """Benchmark path planning performance."""

    planner = PathPlanner()
    num_runs = 100

    # Generate test scenarios
    scenarios = [
        {"start": (0, 0), "goal": (10, 0), "obstacles": []},
        {"start": (0, 0), "goal": (10, 10), "obstacles": [(5, 5)]},
        {"start": (0, 0), "goal": (20, 20), "obstacles": [(5, 5), (10, 10), (15, 15)]},
    ]

    results = []

    for scenario in scenarios:
        times = []

        for _ in range(num_runs):
            start_time = time.time()
            path = planner.plan_path(scenario["start"], scenario["goal"], scenario["obstacles"])
            end_time = time.time()

            times.append(end_time - start_time)

        avg_time = np.mean(times)
        std_time = np.std(times)
        max_time = np.max(times)

        results.append({
            'scenario': scenario,
            'avg_time': avg_time,
            'std_time': std_time,
            'max_time': max_time,
            'path_length': len(path) if path else 0
        })

    return results

if __name__ == "__main__":
    results = benchmark_path_planning()
    for result in results:
        print(f"Scenario: {result['scenario']}")
        print(".3f")
        print(".3f")
        print(".3f")
        print(f"Path length: {result['path_length']}")
        print()
```

### Continuous Integration
```yaml
# .github/workflows/ci.yml
name: CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-latest

    steps:
    - uses: actions/checkout@v2

    - name: Set up Python
      uses: actions/setup-python@v2
      with:
        python-version: '3.10'

    - name: Install dependencies
      run: |
        pip install -r requirements.txt
        pip install pytest pytest-cov flake8 black isort mypy

    - name: Lint with flake8
      run: flake8 autonomy/ --max-line-length=88

    - name: Check formatting with black
      run: black --check autonomy/

    - name: Check imports with isort
      run: isort --check-only autonomy/

    - name: Type check with mypy
      run: mypy autonomy/ --ignore-missing-imports

    - name: Run tests
      run: pytest --cov=autonomy --cov-report=xml

    - name: Upload coverage
      uses: codecov/codecov-action@v2
      with:
        file: ./coverage.xml
```

This comprehensive template and style guide provides the foundation for writing clean, maintainable, and testable ROS 2 code for the URC 2026 autonomy system.
