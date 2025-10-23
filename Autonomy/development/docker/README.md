# Docker Development Environment for URC 2026 Autonomy

This directory contains the complete Docker setup for cross-collaboration on the URC 2026 autonomy system.

## 📁 Directory Structure

```
docker/
├── docker-compose.yml          # Multi-service container orchestration
├── Dockerfile.ros2-core        # ROS 2 base image
├── Dockerfile.dev              # Development environment
├── Dockerfile.sim              # Simulation environment (Gazebo)
├── Dockerfile.vision           # Computer vision service
├── .env                        # Environment configuration template
├── .dockerignore              # Docker build exclusions
├── .gitignore                 # Git exclusions for docker directory
├── QUICKSTART.md              # Quick start guide (start here!)
├── README.md                  # This file
├── scripts/
│   └── docker-dev.sh          # Development utility script
├── config/
│   ├── autonomy_config.yaml   # Main system configuration
│   └── geofence.yaml          # Safety boundary configuration
└── data/                      # Shared data directories
    ├── vision/                # Computer vision datasets
    ├── slam/                  # SLAM maps and data
    ├── navigation/            # Waypoints and paths
    └── calibration/           # Calibration data
```

## 🚀 Quick Start

1. **Read the Quick Start Guide**: `QUICKSTART.md`
2. **Configure Environment**: Copy and edit `.env`
3. **Build and Start**: `./scripts/docker-dev.sh build && ./scripts/docker-dev.sh start`
4. **Enter Development**: `./scripts/docker-dev.sh enter`

## 🐳 Container Services

### Core Services
- **ros2-core**: ROS 2 Humble base with DDS communication
- **autonomy-dev**: Full development environment with tools
- **autonomy-sim**: Gazebo simulation with Mars terrain
- **vision-service**: Dedicated computer vision processing (optional)

### Optional Services
- **autonomy-db**: PostgreSQL database for data persistence

## 🔧 Key Features

### Cross-Collaboration
- **Shared Volumes**: Data, models, and calibration shared across team
- **Version Control**: Git integration with LFS for large files
- **Real-time Sync**: Changes reflect immediately across containers

### Development Environment
- **ROS 2 Humble**: Full robotics framework
- **Python/ML Stack**: PyTorch, OpenCV, NumPy, etc.
- **Development Tools**: VS Code compatible, testing frameworks
- **GPU Support**: NVIDIA GPU acceleration when available

### Simulation Environment
- **Gazebo Integration**: Physics-based simulation
- **Mars Terrain**: Custom desert environment models
- **Competition Scenarios**: Pre-built mission configurations
- **Sensor Simulation**: Realistic IMU, GPS, camera simulation

## 📊 Shared Data Structure

### Vision Data (`data/vision/`)
- **Datasets**: Training data for object detection
- **Models**: Trained neural networks
- **Calibration**: Camera calibration data

### SLAM Data (`data/slam/`)
- **Maps**: Generated environment maps
- **Bagfiles**: ROS data recordings
- **Trajectories**: Ground truth paths

### Navigation Data (`data/navigation/`)
- **Waypoints**: GNSS coordinates for targets
- **Paths**: Planned routes and trajectories
- **Costmaps**: Navigation obstacle maps

### Calibration Data (`data/calibration/`)
- **Cameras**: Intrinsic/extrinsic parameters
- **IMU**: Sensor calibration data
- **Arm**: Robotic manipulator calibration

## ⚙️ Configuration

### Environment Variables (`.env`)
```bash
# ROS Configuration
ROS_DOMAIN_ID=42
ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST

# Display for GUI applications
DISPLAY=:0

# GPU Configuration
NVIDIA_VISIBLE_DEVICES=all
NVIDIA_DRIVER_CAPABILITIES=all

# Team Configuration
TEAM_NAME=UCR_Robotics
COMPETITION_YEAR=2026
```

### System Configuration (`config/autonomy_config.yaml`)
- ROS 2 settings and QoS profiles
- Subsystem-specific parameters
- Safety and performance limits
- Environmental adaptation settings

## 🛠️ Usage Examples

### Development Workflow
```bash
# Start environment
./scripts/docker-dev.sh start

# Enter development container
./scripts/docker-dev.sh enter

# Build ROS workspace
cd /workspace/ros2_ws
colcon build --symlink-install

# Run your code
ros2 run your_package your_node
```

### Simulation Testing
```bash
# Launch Mars simulation
ros2 launch mars_simulation simulation.launch.xml

# Run autonomy stack
ros2 launch autonomy_system autonomy.launch.xml

# Monitor in RViz
ros2 run rviz2 rviz2 -d /workspace/config/navigation.rviz
```

### Data Management
```bash
# Access shared data
ls /workspace/data/vision/datasets/

# Add new training data (on host machine)
# Data appears automatically in containers
cp new_images/* Autonomy/docker/data/vision/datasets/objects/
```

## 🔐 Security & Best Practices

### Container Security
- Non-root user for development
- Minimal base images
- No privileged containers by default
- Regular security updates

### Data Management
- Sensitive data excluded from containers
- Git LFS for large binary files
- Regular backups of critical data
- Access controls for shared resources

### Development Practices
- Code formatting (black, flake8)
- Unit testing (pytest)
- Documentation requirements
- Code review processes

## 🚨 Troubleshooting

### Common Issues

#### Build Failures
```bash
# Clean and rebuild
./scripts/docker-dev.sh clean
./scripts/docker-dev.sh build
```

#### GPU Issues
```bash
# Check NVIDIA Docker installation
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi
```

#### Network Issues
```bash
# Check ROS 2 communication
ros2 topic list
ros2 node list
```

#### Permission Issues
```bash
# Fix workspace permissions
sudo chown -R $USER:$USER Autonomy/
```

### Logs and Debugging
```bash
# View container logs
./scripts/docker-dev.sh logs

# Enter container for debugging
./scripts/docker-dev.sh enter

# Check system resources
docker stats
```

## 📈 Performance Optimization

### Container Optimization
- Multi-stage builds for smaller images
- Layer caching for faster rebuilds
- Volume mounts for persistent data
- Resource limits and reservations

### Development Optimization
- Hot reloading for Python code
- Incremental builds with colcon
- Parallel test execution
- GPU acceleration for ML workloads

## 🔄 Backup & Recovery

### Data Backup
```bash
# Backup shared data
tar -czf autonomy_data_backup_$(date +%Y%m%d).tar.gz \
  docker/data \
  docker/models \
  docker/calibration
```

### Environment Recovery
```bash
# Full environment reset
./scripts/docker-dev.sh clean
./scripts/docker-dev.sh build
./scripts/docker-dev.sh start
```

## 📚 Additional Resources

- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Docker Best Practices**: https://docs.docker.com/develop/dev-best-practices/
- **Gazebo Tutorials**: https://gazebosim.org/tutorials
- **PyTorch for Robotics**: https://pytorch.org/tutorials/

## 📊 Docker Suitability Analysis

See `../DockerSuitabilityAnalysis.md` for detailed analysis of Docker suitability for each autonomy subsystem:

- **🟢 Fully Suitable**: Computer Vision (95%), State Management (100%), SLAM (85%)
- **🟡 Hybrid Approach**: Navigation (70%), LED Status (60%)
- **🔴 Limited Suitability**: Autonomous Typing (40%)

### Hardware Access Configuration
The Docker environment is configured for hardware access:
- **Privileged mode** for GPIO and device control
- **Device passthrough** for USB, I2C, SPI devices
- **GPU acceleration** for computer vision and SLAM
- **Network access** for ROS 2 multi-machine communication

### Development Recommendations
1. **Start in Docker**: Develop algorithms and logic in containers
2. **Use hybrid approach**: Combine Docker services with native hardware interfaces
3. **Test incrementally**: Validate in simulation, then with real hardware
4. **Maintain consistency**: Keep development workflows similar across environments

## 🤝 Team Collaboration

### Shared Development
- All team members use identical environments
- Code changes sync automatically
- Data updates visible to entire team
- Consistent toolchains and dependencies

### Version Control
- Git for source code
- Git LFS for large data files
- Branching strategy for feature development
- Code review requirements

### Communication
- Shared documentation in repository
- Regular integration meetings
- Issue tracking for bugs and features
- Progress tracking dashboards

This Docker environment provides a robust, collaborative platform for developing the URC 2026 autonomy system, ensuring consistency and efficiency across the entire team.
