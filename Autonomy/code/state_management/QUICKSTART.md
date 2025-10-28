# State Machine Quick Start Guide

Get the state machine up and running in 5 minutes!

## Prerequisites

```bash
# Install Python dependencies
pip install structlog

# Make sure you're in your ROS2 workspace
cd ~/ros2_ws  # or wherever your workspace is
```

## Build

```bash
# Build the interfaces first (if not already built)
colcon build --packages-select autonomy_interfaces

# Build the state machine
colcon build --packages-select autonomy_state_machine

# Source the workspace
source install/setup.bash
```

## Run

### Terminal 1: Launch State Machine

```bash
ros2 launch autonomy_state_machine state_machine.launch.py
```

You should see:
```
[INFO] [state_machine_director]: State Machine Director initialized successfully
[INFO] [state_machine_director]: Boot sequence complete
[INFO] [state_machine_director]: Transitioned to IDLE
```

### Terminal 2: Monitor State Updates

```bash
ros2 topic echo /state_machine/current_state
```

### Terminal 3: Control the State Machine

```bash
# Transition to TELEOPERATION mode
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState \
    "{desired_state: 'TELEOPERATION', reason: 'Manual control', operator_id: 'user'}"

# Transition to AUTONOMOUS mode
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState \
    "{desired_state: 'AUTONOMOUS', desired_substate: 'AUTONOMOUS_NAVIGATION', \
      reason: 'Start mission', operator_id: 'user'}"

# Query current state
ros2 service call /state_machine/get_system_state autonomy_interfaces/srv/GetSystemState \
    "{include_history: true, include_subsystems: true}"
```

## Python Integration Example

Create a file `test_frontend.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from autonomy_state_machine.frontend_interface import FrontendInterface

class TestFrontend(Node):
    def __init__(self):
        super().__init__('test_frontend')
        self.interface = FrontendInterface(self)
        self.interface.register_state_callback(self.on_state_update)
        self.get_logger().info("Frontend ready!")
    
    def on_state_update(self, msg):
        self.get_logger().info(
            f"State: {msg.current_state} | "
            f"Substate: {msg.substate} | "
            f"Time: {msg.time_in_state:.1f}s"
        )

def main():
    rclpy.init()
    node = TestFrontend()
    
    try:
        # Let it run for a few seconds to see state updates
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

Run it:
```bash
chmod +x test_frontend.py
python3 test_frontend.py
```

## Common State Transitions

### Start Autonomous Navigation Mission

```bash
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState \
    "{desired_state: 'AUTONOMOUS', desired_substate: 'AUTONOMOUS_NAVIGATION', \
      reason: 'URC autonomous navigation mission', operator_id: 'operator1'}"
```

### Return to Teleoperation

```bash
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState \
    "{desired_state: 'TELEOPERATION', reason: 'Manual control needed', operator_id: 'operator1'}"
```

### Handle Emergency (Safety State)

```bash
# Transition to safety
ros2 service call /state_machine/change_state autonomy_interfaces/srv/ChangeState \
    "{desired_state: 'SAFETY', reason: 'Emergency stop', operator_id: 'safety_system'}"

# Recover from safety
ros2 service call /state_machine/recover_from_safety autonomy_interfaces/srv/RecoverFromSafety \
    "{recovery_method: 'AUTO', operator_id: 'operator1', acknowledge_risks: true, \
      notes: 'Issue resolved'}"
```

## Verify Everything Works

Run the tests:

```bash
# Unit tests
colcon test --packages-select autonomy_state_machine

# View test results
colcon test-result --verbose
```

## Troubleshooting

### "Package not found"
```bash
# Make sure you built and sourced
colcon build --packages-select autonomy_state_machine
source install/setup.bash
```

### "Service not available"
```bash
# Check if node is running
ros2 node list | grep state_machine

# If not, launch it
ros2 launch autonomy_state_machine state_machine.launch.py
```

### "Module 'structlog' not found"
```bash
pip install structlog
```

## Next Steps

1. Read the full [README.md](README.md) for detailed documentation
2. Integrate with your frontend application
3. Connect to subsystem nodes (navigation, vision, etc.)
4. Test with URC competition scenarios

## Quick Reference

**Services:**
- `/state_machine/change_state` - Request state transition
- `/state_machine/get_system_state` - Query current state
- `/state_machine/recover_from_safety` - Recover from safety state

**Topics:**
- `/state_machine/current_state` - State updates (10Hz)
- `/state_machine/transitions` - Transition events
- `/state_machine/safety_status` - Safety status
- `/state_machine/led_info` - LED information

**States:**
- BOOT, CALIBRATION, IDLE, TELEOPERATION, AUTONOMOUS, SAFETY, SHUTDOWN

**Autonomous Missions:**
- SCIENCE, DELIVERY, EQUIPMENT_SERVICING, AUTONOMOUS_NAVIGATION

## Support

For issues or questions, refer to:
- [README.md](README.md) - Full documentation
- [statemachine_todo.md](statemachine_todo.md) - Implementation status
- URC 2026 rules (in `/rules.md`)

---

**Happy State Machining! 🤖**

