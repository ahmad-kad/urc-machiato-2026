# Docker Environment Quick Start Guide

## 🚀 Getting Started with Docker Development

This guide will get you up and running with the autonomy development environment in under 15 minutes.

## Prerequisites

### System Requirements
- **Docker Engine**: Version 20.10 or later
- **Docker Compose**: Version 2.0 or later
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space
- **GPU**: NVIDIA GPU (optional, for accelerated computing)

### Installation Check
```bash
# Check Docker installation
docker --version
docker compose version

# Check NVIDIA support (optional)
nvidia-docker --version  # If you have NVIDIA GPU
```

## 1. Quick Setup (5 minutes)

### Clone and Navigate
```bash
cd ~/robotics2025/Autonomy/docker
```

### Environment Configuration
```bash
# Copy environment template
cp .env .env.local

# Edit if needed (usually not required)
nano .env.local  # Or use your preferred editor
```

### Build and Start
```bash
# Build Docker images (first time only, ~5-10 minutes)
./scripts/docker-dev.sh build

# Start services
./scripts/docker-dev.sh start

# Check status
./scripts/docker-dev.sh status
```

## 2. Enter Development Environment

### Development Container
```bash
# Enter the development container
./scripts/docker-dev.sh enter

# You should now be inside the container as user 'ros'
whoami  # Should show 'ros'
pwd     # Should show '/workspace'
```

### Verify ROS 2 Installation
```bash
# Check ROS 2 installation
source /opt/ros/humble/setup.bash
ros2 --version

# Check workspace
ls -la /workspace/
```

## 3. Basic ROS 2 Testing

### Test ROS 2 Communication
```bash
# In development container terminal 1
ros2 run demo_nodes_cpp talker

# In development container terminal 2 (open new terminal)
./scripts/docker-dev.sh enter  # New terminal session
ros2 run demo_nodes_cpp listener
```

### Test Gazebo Simulation
```bash
# Launch Gazebo (may take time first run)
ros2 launch gazebo_ros gazebo.launch.py

# In another terminal, spawn a robot
ros2 run gazebo_ros spawn_entity.py -topic /robot_description -entity rover
```

## 4. Development Workflow

### Workspace Setup
```bash
# Create your ROS 2 package
cd /workspace/ros2_ws/src
ros2 pkg create --build-type ament_python my_autonomy_pkg

# Build workspace
cd /workspace/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Code Development
```bash
# Your code is automatically synced between host and container
# Edit files on your host machine with your preferred IDE
# Changes appear immediately in the container

# Example: Create a simple ROS 2 node
mkdir -p /workspace/ros2_ws/src/my_autonomy_pkg/my_autonomy_pkg
cat > /workspace/ros2_ws/src/my_autonomy_pkg/my_autonomy_pkg/hello_world.py << 'EOF'
import rclpy
from rclpy.node import Node

class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world')
        self.get_logger().info('Hello from Docker ROS 2!')

def main(args=None):
    rclpy.init(args=args)
    node = HelloWorldNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
EOF
```

### Running Your Code
```bash
# Build and run
cd /workspace/ros2_ws
colcon build --symlink-install --packages-select my_autonomy_pkg
source install/setup.bash
ros2 run my_autonomy_pkg hello_world
```

## 5. Data Management

### Accessing Shared Data
```bash
# Data is shared between host and container
ls /workspace/data/          # Shared data directory
ls /workspace/models/        # Shared models directory
ls /workspace/calibration/   # Shared calibration directory
```

### Working with Datasets
```bash
# Example: Add training data
mkdir -p /workspace/data/vision/datasets/test
# Add your images/videos here on the host machine
# They will be accessible in the container
```

## 6. GPU Support (Optional)

### Enable GPU Acceleration
If you have an NVIDIA GPU, the containers are already configured for GPU support. Test it:

```bash
# In container
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
nvidia-smi  # Should show GPU information
```

## 7. Troubleshooting

### Common Issues

#### Container Won't Start
```bash
# Check Docker is running
docker info

# Check logs
./scripts/docker-dev.sh logs

# Clean and rebuild
./scripts/docker-dev.sh clean
./scripts/docker-dev.sh build
./scripts/docker-dev.sh start
```

#### X11/GUI Issues
```bash
# Allow Docker to access X server
xhost +local:docker

# Or set DISPLAY variable correctly
export DISPLAY=:0
```

#### Permission Issues
```bash
# Fix workspace permissions
sudo chown -R $USER:$USER /path/to/Autonomy
```

#### ROS 2 Communication Issues
```bash
# Check ROS_DOMAIN_ID
echo $ROS_DOMAIN_ID

# Test basic communication
ros2 topic list
```

## 8. Daily Workflow

### Starting Work
```bash
cd ~/robotics2025/Autonomy/docker
./scripts/docker-dev.sh start
./scripts/docker-dev.sh enter
# You're now in the development environment
```

### During Development
```bash
# Build your code
cd /workspace/ros2_ws
colcon build --symlink-install

# Run tests
colcon test
colcon test-result --verbose

# Launch simulation
ros2 launch your_package simulation.launch.py
```

### Ending Work
```bash
# Exit container (Ctrl+D or exit)
# Stop services (optional, keeps containers running)
./scripts/docker-dev.sh stop
```

## 9. Advanced Usage

### Multiple Services
```bash
# Start only specific services
docker compose up autonomy-dev autonomy-sim

# Start with GPU acceleration
docker compose --profile gpu up
```

### Custom Configuration
```bash
# Use custom environment file
docker compose --env-file .env.custom up

# Override specific settings
ROS_DOMAIN_ID=100 docker compose up autonomy-dev
```

### Development with IDE

#### VS Code Integration
1. Install "Remote-Containers" extension in VS Code
2. Open folder in container: `Ctrl+Shift+P` → "Remote-Containers: Attach to Running Container"
3. Select the `autonomy_development` container
4. Develop directly in the container with full ROS 2 support

## 10. Backup and Recovery

### Data Backup
```bash
# Backup important data
tar -czf autonomy_backup_$(date +%Y%m%d).tar.gz \
  /workspace/data \
  /workspace/models \
  /workspace/calibration \
  /workspace/ros2_ws/src
```

### Environment Reset
```bash
# Clean rebuild (WARNING: deletes all containers/volumes)
./scripts/docker-dev.sh clean
./scripts/docker-dev.sh build
./scripts/docker-dev.sh start
```

## 📞 Support

If you encounter issues:

1. Check the troubleshooting section above
2. Review Docker and ROS 2 logs: `./scripts/docker-dev.sh logs`
3. Verify system requirements
4. Check the main documentation in `../README.md`

## 🎯 Next Steps

Once your Docker environment is working:

1. **Read the subsystem guides** in the respective folders
2. **Start with simulation** using the provided Gazebo setups
3. **Implement basic nodes** for your assigned subsystem
4. **Integrate with the team** using shared data directories

Happy developing! 🚀
