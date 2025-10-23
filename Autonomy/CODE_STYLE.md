# 🧹 Code Style & Consistency Guide

**"Clean code that works together"** - Our standard for the URC 2026 autonomy system.

## 🎯 Quick Rules

### ✅ **DO**
```python
# Good: Clear names, simple logic
def calculate_distance(point_a, point_b):
    """Calculate Euclidean distance between two points."""
    return math.sqrt((point_a.x - point_b.x)**2 +
                    (point_a.y - point_b.y)**2)

# Good: Small functions, single responsibility
def validate_sensor_data(data):
    if data is None:
        raise ValueError("Sensor data cannot be None")
    return True
```

### ❌ **DON'T**
```python
# Bad: Unclear abbreviations, complex logic
def calc_dist(pa, pb):
    return ((pa.x-pb.x)**2 + (pa.y-pb.y)**2)**0.5

# Bad: Large function, multiple responsibilities
def process_data(data):
    if data is None: raise ValueError("Bad data")
    # 50 more lines doing different things...
```

## 📏 Standards Overview

### **Python Style**
- **PEP 8** compliant (88 char line length)
- **Black** formatted (auto-formatter)
- **isort** for import sorting
- **MyPy** type hints required

### **ROS 2 Conventions**
- **Snake_case** for node names: `navigation_node`
- **Consistent topics**: `/namespace/subsystem/data`
- **QoS profiles**: Sensor, control, state
- **Error handling**: Try/catch with logging

### **Project Structure**
```
code/[subsystem]/
├── src/           # Implementation
├── test/          # Unit tests
├── config/        # Parameters
└── launch/        # ROS launch files
```

## 🛠️ Code Quality Tools

### **Automatic Checks**
```bash
# Format code (run before commit)
black autonomy/
isort autonomy/

# Check style
flake8 autonomy/ --max-line-length=88

# Type check
mypy autonomy/ --ignore-missing-imports

# Run tests
pytest autonomy/ -v --cov=autonomy
```

### **Pre-commit Hooks**
```bash
# Install hooks (runs automatically on git commit)
pip install pre-commit
pre-commit install

# Manual check
pre-commit run --all-files
```

## 📝 Writing Clean Code

### **1. Function Design**
```python
# ✅ Good: Single responsibility, clear name
def publish_velocity_command(linear, angular):
    """Publish velocity command to base controller."""
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    self.cmd_vel_pub.publish(msg)

# ❌ Bad: Multiple responsibilities, unclear
def pub_vel(lin, ang):
    msg = Twist()
    msg.linear.x = lin
    msg.angular.z = ang
    self.pub.publish(msg)
    # Also does logging and error handling...
```

### **2. Class Design**
```python
# ✅ Good: Clear responsibilities, proper encapsulation
class NavigationNode(Node):
    """ROS 2 node for autonomous navigation."""

    def __init__(self):
        super().__init__('navigation_node')
        self._setup_publishers()  # Private setup methods
        self._current_goal = None  # Private attributes

    def navigate_to_goal(self, goal):
        """Navigate to specified goal pose."""
        self._validate_goal(goal)
        self._current_goal = goal
        # Implementation...

# ❌ Bad: God class, public everything
class NavNode(Node):
    def __init__(self):
        self.goal = None  # Public attribute
        self.setup()      # Public setup

    def go_to_goal(self, g):  # Unclear name
        self.goal = g
        # 100 lines of logic...
```

### **3. Error Handling**
```python
# ✅ Good: Specific exceptions, meaningful messages
def process_lidar_scan(scan):
    """Process LIDAR scan data."""
    try:
        validated_scan = self._validate_scan(scan)
        processed_data = self._filter_scan(validated_scan)
        return processed_data
    except ValueError as e:
        self.get_logger().error(f"Invalid scan data: {e}")
        return None
    except Exception as e:
        self.get_logger().fatal(f"Unexpected error: {e}")
        raise

# ❌ Bad: Bare except, generic handling
def process_scan(scan):
    try:
        # Process...
        return data
    except:  # Catches everything!
        self.get_logger().error("Error!")
        return None
```

### **4. ROS 2 Patterns**
```python
# ✅ Good: Consistent QoS, proper cleanup
class SensorNode(Node):
    def __init__(self):
        super().__init__('sensor_node')

        # Standard QoS profiles
        self.qos_sensor = QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT, depth=10)
        self.qos_control = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=5)

        # Proper setup pattern
        self._setup_publishers()
        self._setup_subscribers()
        self._cleanup_timer = self.create_timer(1.0, self._cleanup_old_data)

    def __del__(self):
        # Proper cleanup
        if hasattr(self, '_cleanup_timer'):
            self._cleanup_timer.cancel()

# ❌ Bad: Inconsistent QoS, no cleanup
class BadSensorNode(Node):
    def __init__(self):
        self.pub = self.create_publisher(LaserScan, 'scan', 10)  # No QoS specified
        self.sub = self.create_subscription(Odometry, 'odom', self.callback, 10)
        # No cleanup!
```

## 🧪 Testing Standards

### **Unit Tests**
```python
# ✅ Good: Isolated, clear assertions
import pytest
from autonomy.navigation.path_planner import PathPlanner

class TestPathPlanner:
    def test_straight_path(self):
        planner = PathPlanner()
        start = (0, 0)
        goal = (10, 0)

        path = planner.plan_path(start, goal)

        assert len(path) > 0
        assert path[0] == start
        assert path[-1] == goal

    def test_obstacle_avoidance(self):
        planner = PathPlanner()
        obstacles = [(5, 0)]

        path = planner.plan_path((0, 0), (10, 0), obstacles)

        # Assert path avoids obstacles
        for point in path:
            assert not any(abs(point[0] - obs[0]) < 1.0 and
                          abs(point[1] - obs[1]) < 1.0 for obs in obstacles)
```

### **ROS 2 Integration Tests**
```python
# ✅ Good: Proper test fixtures, cleanup
import pytest
import rclpy
from autonomy.navigation.navigation_node import NavigationNode

class TestNavigationNode:
    @pytest.fixture
    def ros_context(self):
        rclpy.init()
        yield
        rclpy.shutdown()

    @pytest.fixture
    def node(self, ros_context):
        node = NavigationNode()
        yield node
        node.destroy_node()

    def test_initialization(self, node):
        assert node.update_rate == 10.0
        assert node.is_navigating == False
```

## 📊 Performance Standards

### **Timing Requirements**
```python
# ✅ Good: Meet real-time deadlines
def control_loop(self):
    start_time = time.time()

    # Control logic here
    self._compute_control_commands()
    self._publish_commands()

    # Check timing
    elapsed = time.time() - start_time
    if elapsed > 0.1:  # 10Hz requirement
        self.get_logger().warn(f"Control loop slow: {elapsed:.3f}s")
```

### **Resource Usage**
```python
# ✅ Good: Monitor and limit resource usage
class ResourceMonitor:
    def __init__(self, node):
        self.node = node
        self.memory_usage = []
        self.cpu_usage = []

    def check_resources(self):
        # Monitor memory/CPU usage
        # Warn if exceeding limits
        pass
```

## 🔗 Interoperability Standards

### **Message Types**
```python
# ✅ Good: Use standard ROS messages
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import LaserScan, Image
from nav_msgs.msg import Odometry, Path

# Custom messages only when necessary
from autonomy_msgs.msg import NavigationStatus
```

### **Topic Naming**
```python
# ✅ Good: Consistent, hierarchical naming
TOPIC_CMD_VEL = '/cmd_vel'
TOPIC_ODOM = '/odom'
TOPIC_SCAN = '/scan'
TOPIC_GOAL = '/goal_pose'

# With namespaces
NAMESPACE_NAV = '/navigation'
TOPIC_NAV_STATUS = f'{NAMESPACE_NAV}/status'
```

### **Parameter Names**
```python
# ✅ Good: Consistent parameter naming
PARAM_UPDATE_RATE = 'update_rate'
PARAM_MAX_VELOCITY = 'max_linear_velocity'
PARAM_GOAL_TOLERANCE = 'goal_tolerance'
PARAM_OBSTACLE_MARGIN = 'obstacle_margin'
```

## 🐛 Debugging Standards

### **Logging Levels**
```python
# Use appropriate log levels
self.get_logger().debug("Detailed diagnostic info")
self.get_logger().info("Normal operation messages")
self.get_logger().warn("Potential issues")
self.get_logger().error("Error conditions")
self.get_logger().fatal("Severe errors")
```

### **Debugging Tools**
```python
# ✅ Good: Structured debugging
def debug_navigation_state(self):
    """Log current navigation state for debugging."""
    self.get_logger().info(
        "Navigation Debug",
        extra={
            'is_navigating': self.is_navigating,
            'current_pose': self.current_pose,
            'goal_pose': self.goal_pose,
            'distance_to_goal': self._calculate_distance_to_goal(),
            'obstacle_detected': self.obstacle_detected,
            'current_velocity': self.current_velocity.linear.x
        }
    )
```

## 📋 Code Review Checklist

**Before submitting code:**

### **Functionality**
- [ ] Code compiles and runs without errors
- [ ] Meets requirements specified in TODO/issue
- [ ] Handles edge cases and error conditions
- [ ] Includes appropriate unit tests

### **Style & Quality**
- [ ] Follows PEP 8 and project style guide
- [ ] Passes all automated checks (black, flake8, mypy)
- [ ] Functions are small and focused
- [ ] Variables and functions have clear names
- [ ] Code is well-documented

### **ROS 2 Standards**
- [ ] Proper QoS profiles used
- [ ] Consistent topic and parameter naming
- [ ] Appropriate error handling and logging
- [ ] Resources properly cleaned up

### **Integration**
- [ ] Compatible with existing codebase
- [ ] No breaking changes to APIs
- [ ] Proper dependencies declared
- [ ] Tested with other subsystems

## 🎯 Summary

**Clean code principles:**
1. **Simple** - Easy to understand and modify
2. **Consistent** - Follows established patterns
3. **Tested** - Includes comprehensive tests
4. **Documented** - Clear comments and docstrings
5. **Compatible** - Works with existing systems

**Tools for consistency:**
- Black (formatting)
- isort (imports)
- flake8 (linting)
- mypy (types)
- pytest (testing)

**Remember:** Code is read more than written. Make it easy for your teammates!
