# Autonomous Typing Subsystem

Complete ROS 2 system for autonomous robotic keyboard typing. Detects keyboard position via ArUco markers and autonomously presses keys using 6-DOF arm control.

## System Overview

```
Camera (Image Stream)
        |
        v
Keyboard Localization (ArUco Detection)
        |
        v
Transform Manager (camera → keyboard frame)
        |
        v
Keyboard Dictionary (key positions)
        |
        v
Arm Controller (IK + trajectory planning)
        |
        v
Typing Executor (sequence orchestration)
        |
        v
Autonomous Typing Node (main orchestrator)
```

## Architecture

### Core Components

#### 1. Keyboard Localization (`keyboard_localization.py`)
- **Purpose**: Detect ArUco markers on keyboard corners
- **Inputs**: Camera image stream, camera intrinsics
- **Outputs**: 
  - `/keyboard_pose` - PoseStamped with keyboard position
  - TF transform: `camera_optical_frame` → `keyboard_frame`
- **Key Features**:
  - Real-time marker detection
  - PnP-based pose estimation
  - Robust marker tracking with fallback

#### 2. Keyboard Dictionary (`keyboard_dictionary.py`)
- **Purpose**: Manage keyboard layout and key positions
- **Classes**:
  - `KeyboardDictionary`: QWERTY layout with 3D positions
  - `KeyboardTransformManager`: Convert keyboard-frame to world-frame coordinates
- **Key Methods**:
  - `get_key_position(char)` - Get key position relative to keyboard
  - `keyboard_to_world()` - Transform positions with rotation + translation
  - `get_key_press_trajectory()` - Generate approach/press/retract trajectory

#### 3. Arm Controller (`arm_controller.py`)
- **Purpose**: Control robotic arm for key pressing
- **Components**:
  - `SimpleInverseKinematics`: Numerical IK solver
  - `ArmController`: High-level arm control with MoveIt interface
- **Key Methods**:
  - `move_to_position(xyz)` - Move arm to world position
  - `press_key(key_pos)` - Execute full key press motion
  - `_plan_trajectory()` - Generate joint trajectories

#### 4. Typing Executor (`typing_executor.py`)
- **Purpose**: Orchestrate typing sequences with error handling
- **Features**:
  - Character-by-character execution
  - Automatic retry on failures
  - Progress feedback via callbacks
  - Statistics tracking
- **Key Methods**:
  - `execute_sequence(string)` - Type full sequence
  - `_press_character(char)` - Press single key with retries
  - `validate_key_press()` - Verify press success

#### 5. Main Node (`autonomous_typing_node.py`)
- **Purpose**: Integrate all subsystems
- **Inputs**:
  - `/keyboard_pose` - Keyboard location from localization
  - `/arm/joint_states` - Current arm joint positions
- **Outputs**:
  - `/autonomous_typing/status` - System status
  - ROS 2 Action: `perform_typing` - Execute typing sequences

## Installation & Setup

### Prerequisites
```bash
# Install ROS 2 Humble
sudo apt update && sudo apt install ros-humble-desktop

# Install dependencies
sudo apt install python3-opencv-python
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-tf2-ros
sudo apt install ros-humble-moveit-ros-planning-interface
```

### Build Package
```bash
cd ~/robotics2025/Autonomy/code
colcon build --packages-select autonomy_autonomous_typing
source install/setup.bash
```

## Usage

### 1. Basic System Startup

```bash
# Terminal 1: Launch keyboard localization
ros2 launch autonomy_autonomous_typing typing_system.launch.py

# Terminal 2: Verify keyboard detection
ros2 topic echo /keyboard_pose
```

### 2. Execute Typing Sequence

#### Via ROS 2 Action:
```python
#!/usr/bin/env python3
import rclpy
from autonomy_interfaces.action import PerformTyping

def main():
    rclpy.init()
    node = rclpy.create_node('typing_client')
    
    client = node.create_client(PerformTyping, 'perform_typing')
    
    goal = PerformTyping.Goal()
    goal.sequence = "HELLO"
    
    future = client.call_async(goal)
    rclpy.spin_until_future_complete(node, future)
    
    result = future.result()
    print(f"Typing success: {result.success}")
    print(f"Characters typed: {result.characters_typed}")
    print(f"Message: {result.message}")

if __name__ == '__main__':
    main()
```

#### Via Command Line:
```bash
ros2 action send_goal perform_typing autonomy_interfaces/action/PerformTyping \
  "{sequence: 'HELLO'}"
```

### 3. Monitor System Status

```bash
# Watch typing status
ros2 topic echo /autonomous_typing/status

# Monitor keyboard detection
ros2 topic hz /keyboard_pose
tf2_echo world keyboard_frame
```

## Configuration

### Keyboard Config (`config/keyboard.yaml`)

```yaml
markers:
  0: [0.0, 0.0, 0.0]      # Marker positions in keyboard frame
  1: [0.3, 0.0, 0.0]
  2: [0.0, 0.15, 0.0]
  3: [0.3, 0.15, 0.0]

pressing:
  approach_height: 0.02   # 2cm above keyboard
  press_depth: 0.015      # 1.5cm into key
  dwell_time: 0.1         # 100ms hold at bottom

retry:
  max_retries_per_key: 2
  retry_delay: 0.5
```

## ROS 2 Topics

### Published
- `/keyboard_pose` (PoseStamped) - Keyboard position and orientation
- `/keyboard_detection_image` (Image) - Debug visualization
- `/autonomous_typing/status` (String) - System status
- `/typing_result` (String) - Execution results

### Subscribed
- `/camera/color/image_raw` (Image) - Camera stream
- `/camera/color/camera_info` (CameraInfo) - Camera intrinsics
- `/arm/joint_states` (JointState) - Arm feedback

### TF Frames
- `world` - Global reference frame
- `camera_optical_frame` - Camera sensor frame
- `keyboard_frame` - Keyboard-centered frame
- `arm_base` - Robotic arm base

## ROS 2 Actions

### PerformTyping
```
Goal:
  sequence: string          # Text to type (e.g., "HELLO")

Feedback:
  characters_typed: int     # Progress
  current_character: string # Current key

Result:
  success: bool             # Overall success
  characters_typed: int     # Total successful presses
  message: string           # Status message
```

## Supported Keys

**Letters**: a-z
**Numbers**: 0-9
**Special**: space, backspace, tab, enter, shift_l, shift_r, caps
**Symbols**: `, -, =, [, ], \, ;, ', ,, ., /

## Performance Expectations

| Metric | Target | Notes |
|--------|--------|-------|
| Marker Detection | >95% | Real-time at ~30fps |
| Pose Accuracy | <5mm | ArUco-based PnP |
| Key Position Error | <2mm | Relative to marker |
| Key Press Time | 1.5-2s | Approach + press + retract |
| Typing Speed | 2-3 chars/sec | With validation |
| Sequence Success | >90% | For 6+ character sequences |

## Debugging

### Check Marker Detection
```bash
# View annotated image with detected markers
ros2 topic echo /keyboard_detection_image
```

### Verify Transforms
```bash
# List TF tree
ros2 run tf2_tools view_frames.py

# Check transform
tf2_echo camera_optical_frame keyboard_frame
```

### Test Arm Controller
```python
from autonomy_autonomous_typing.arm_controller import ArmController

ik = ArmController(node)
target = np.array([0.5, 0.2, 0.3])
success = ik.move_to_position(target)
```

### Monitor Typing Progress
```bash
# See real-time character typing
ros2 topic echo --once /autonomous_typing/status
```

## Common Issues & Solutions

### Issue: Markers Not Detected
**Cause**: Poor lighting, camera out of focus, incorrect marker size
**Solution**:
1. Check lighting conditions
2. Verify marker size matches configuration (3cm)
3. Confirm marker IDs match (0-3)
4. Check camera focus and calibration

### Issue: Large Pose Error
**Cause**: Camera calibration drift, marker placement inaccuracy
**Solution**:
1. Re-run camera calibration
2. Verify marker positions are accurate
3. Check that all 4 markers are visible

### Issue: Arm Reaches Wrong Position
**Cause**: IK solver converging to local minimum
**Solution**:
1. Check arm joint limits configuration
2. Provide better initial guess to IK
3. Use analytical IK if available

### Issue: Keys Pressed at Wrong Positions
**Cause**: Keyboard frame not properly aligned
**Solution**:
1. Re-place ArUco markers at exact corners
2. Verify marker orientation
3. Re-run calibration sequence

## Testing

### Unit Tests
```bash
# Test keyboard dictionary
python3 autonomy_autonomous_typing/keyboard_dictionary.py

# Test IK solver
python3 autonomy_autonomous_typing/arm_controller.py

# Test typing executor
python3 autonomy_autonomous_typing/typing_executor.py
```

### Integration Test
```bash
# Run with simulated arm and keyboard
ros2 launch autonomy_autonomous_typing typing_system.launch.py use_sim:=true
```

## Future Improvements

- [ ] Implement MoveIt! for motion planning (currently simplified IK)
- [ ] Add force/torque sensor feedback for validation
- [ ] Support multiple keyboard layouts
- [ ] Add visual feedback to operator
- [ ] Implement online calibration
- [ ] Support rapid typing (parallel key detection/pressing)

## References

- **ArUco Markers**: OpenCV documentation
- **ROS 2**: https://docs.ros.org/en/humble/
- **TF2**: https://wiki.ros.org/tf2
- **IK Solvers**: https://moveit.picknik.ai/

## License

MIT - See LICENSE file

