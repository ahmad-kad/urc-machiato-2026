# Contributing to URC 2026 Autonomy System

Thank you for your interest in contributing to the URC 2026 Autonomy System! This document provides guidelines and information for contributors.

## 🚀 Quick Start

1. **Fork** the repository on GitHub
2. **Clone** your fork locally
3. **Create** a feature branch (`git checkout -b feature/amazing-feature`)
4. **Make** your changes following our coding standards
5. **Test** your changes thoroughly
6. **Commit** with descriptive messages (`git commit -m "Add amazing feature"`)
7. **Push** to your fork (`git push origin feature/amazing-feature`)
8. **Create** a Pull Request

## 📋 Development Workflow

### Two-Workspace Architecture

This project uses a **two-workspace development system**:
- **`Autonomy/code/`** = **Development workspace** (where you write code)
- **`Autonomy/ros2_ws/`** = **ROS2 workspace** (where ROS2 builds and runs code)

**Always edit in `code/`, build in `ros2_ws/`!**

### Setting Up Development Environment

```bash
# 1. Clone the repository
git clone https://github.com/your-org/robotics2025.git
cd robotics2025

# 2. Set up ROS2 workspace
cd Autonomy
mkdir -p ros2_ws/src
ln -s ../code/* ros2_ws/src/

# 3. Install dependencies
cd ros2_ws
rosdep update
rosdep install --from-paths src --ignore-src -r -y

# 4. Build packages
source /opt/ros/humble/setup.bash
colcon build

# 5. Test your setup
source install/setup.bash
ros2 node list
```

## 🛠️ Code Standards

### Python Code Style

We follow **PEP 8** with some modifications:
- **Line length**: 120 characters (not 79)
- **Naming**: snake_case for variables/functions, PascalCase for classes
- **Imports**: Group standard library, third-party, local imports
- **Docstrings**: Use Google-style docstrings

```python
# Good example
def calculate_trajectory(current_pose: Pose, target_pose: Pose) -> Trajectory:
    """Calculate optimal trajectory between two poses.

    Args:
        current_pose: Current robot position and orientation
        target_pose: Desired robot position and orientation

    Returns:
        Trajectory object with waypoints and timing
    """
    # Implementation here
    pass
```

### ROS2 Best Practices

- **Node naming**: Use descriptive, subsystem-specific names
- **Topic naming**: Follow ROS2 conventions (`/namespace/subsystem/status`)
- **QoS settings**: Configure appropriately for real-time requirements
- **Error handling**: Always handle exceptions gracefully

### Commit Messages

Follow conventional commit format:
```
type(scope): description

[optional body]

[optional footer]
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

Examples:
- `feat(navigation): add obstacle avoidance algorithm`
- `fix(slam): resolve memory leak in point cloud processing`
- `docs: update installation instructions`

## 🧪 Testing

### Unit Tests
```bash
# Run Python unit tests
cd Autonomy/ros2_ws
source install/setup.bash
colcon test --packages-select autonomy_navigation
colcon test-result --verbose
```

### Integration Tests
```bash
# Run integration tests
cd Autonomy
python tests/integration_test.py
```

### Manual Testing
- Test in simulation before hardware deployment
- Verify ROS2 topic/service communication
- Check system performance under various conditions

## 📝 Pull Request Process

1. **Update documentation** for any new features
2. **Add tests** for new functionality
3. **Ensure CI passes** (GitHub Actions)
4. **Get review** from at least one team member
5. **Merge** using squash commits for clean history

### PR Template

All PRs must include:
- [ ] Description of changes
- [ ] Testing performed
- [ ] Documentation updated
- [ ] Breaking changes noted

## 🐛 Issue Reporting

When reporting bugs, please include:
- **Environment**: ROS2 version, OS, hardware
- **Steps to reproduce**: Detailed reproduction steps
- **Expected vs actual behavior**
- **Logs**: Error messages, stack traces
- **Screenshots/videos**: If applicable

## 📚 Documentation

- **Code comments**: Explain complex algorithms and business logic
- **README files**: Update for any new setup requirements
- **API documentation**: Document public interfaces
- **Architecture decisions**: Document in ADRs (Architecture Decision Records)

## 🤝 Code of Conduct

This project follows our [Code of Conduct](CODE_OF_CONDUCT.md). By participating, you agree to uphold these standards.

## 📞 Getting Help

- **Slack**: Join our robotics team channel
- **Issues**: Use GitHub issues for bugs and feature requests
- **Discussions**: Use GitHub discussions for questions

## 🎯 Areas for Contribution

### High Priority
- Navigation system improvements
- Computer vision enhancements
- SLAM algorithm optimization
- System integration testing

### Medium Priority
- Documentation improvements
- Additional sensor integrations
- Performance optimizations
- Docker environment enhancements

### Future Areas
- Machine learning model development
- Simulation environment expansion
- Telemetry and monitoring systems

## 🙏 Recognition

Contributors are recognized in our project documentation and release notes. Major contributors may be invited to join the core development team.

---

Thank you for contributing to the URC 2026 Autonomy System! 🚀🤖
