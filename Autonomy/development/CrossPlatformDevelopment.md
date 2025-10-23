# Cross-Platform Development Guide

## Overview
This guide covers developing the URC 2026 autonomy system on Mac and Windows computers using Docker, enabling team members to contribute regardless of their operating system.

## 🖥️ **Mac Development Setup**

### Prerequisites
- **macOS**: 11.0 (Big Sur) or later
- **Docker Desktop for Mac**: Download from https://www.docker.com/products/docker-desktop
- **Git**: `brew install git` or Xcode Command Line Tools
- **VS Code**: With Remote-Containers extension

### Quick Start
See `../docker/QUICKSTART.md` for detailed Docker setup instructions.

### Installation Steps

#### 1. Install Docker Desktop
```bash
# Download and install Docker Desktop for Mac
# https://www.docker.com/products/docker-desktop

# Verify installation
docker --version
docker compose version
```

#### 2. Clone Repository
```bash
cd ~/Documents  # Or your preferred workspace location
git clone https://github.com/your-org/urc-2026-autonomy.git
cd urc-2026-autonomy/Autonomy
```

#### 3. Configure Environment
```bash
# Copy environment template
cp docker/.env docker/.env.local

# Edit for Mac-specific settings (usually defaults work)
code docker/.env.local
```

#### 4. Build and Start
```bash
# Make scripts executable
chmod +x docker/scripts/docker-dev.sh

# Build Docker images (first time, ~10-15 minutes)
./docker/scripts/docker-dev.sh build

# Start services
./docker/scripts/docker-dev.sh start
```

### Mac-Specific Configuration

#### X11 Forwarding for GUI Applications
```bash
# Install XQuartz (X11 server for Mac)
brew install --cask xquartz

# Start XQuartz and allow connections
# System Settings → Security & Privacy → Allow XQuartz

# In terminal:
xhost +localhost

# Docker will use X11 for GUI apps like RViz
```

#### GPU Support (Optional)
```bash
# For Mac with Apple Silicon, Docker uses Rosetta
# GPU acceleration available through Rosetta translation
# Performance may be limited compared to native Linux
```

## 🪟 **Windows Development Setup**

### Prerequisites
- **Windows**: 10/11 Pro, Enterprise, or Education (for Hyper-V)
- **Docker Desktop for Windows**: Download from https://www.docker.com/products/docker-desktop
- **WSL 2**: Required backend for Docker
- **Git for Windows**: https://gitforwindows.org/
- **VS Code**: With Remote-Containers extension

### Installation Steps

#### 1. Enable WSL 2
```powershell
# Open PowerShell as Administrator
wsl --install

# Set WSL 2 as default
wsl --set-default-version 2

# Install Ubuntu distribution
wsl --install -d Ubuntu-22.04
```

#### 2. Install Docker Desktop
```powershell
# Download and install Docker Desktop for Windows
# https://www.docker.com/products/docker-desktop

# Enable WSL 2 backend in Docker Desktop settings
# Settings → General → Use the WSL 2 based engine
```

#### 3. Configure WSL Integration
```powershell
# In Docker Desktop: Settings → Resources → WSL Integration
# Enable integration with your Ubuntu WSL distribution

# Restart Docker Desktop
```

#### 4. Clone Repository (in WSL)
```bash
# Open WSL terminal (Ubuntu)
cd ~
git clone https://github.com/your-org/urc-2026-autonomy.git
cd urc-2026-autonomy/Autonomy
```

#### 5. Build and Start
```bash
# Make scripts executable
chmod +x docker/scripts/docker-dev.sh

# Build and start (same as Mac/Linux)
./docker/scripts/docker-dev.sh build
./docker/scripts/docker-dev.sh start
```

### Windows-Specific Configuration

#### X11 Forwarding for GUI Applications
```bash
# Install VcXsrv Windows X Server
# Download from: https://sourceforge.net/projects/vcxsrv/

# Configure VcXsrv:
# - Multiple windows
# - Start no client
# - Disable access control

# In WSL terminal:
export DISPLAY=$(awk '/nameserver / {print $2; exit}' /etc/resolv.conf):0.0
```

#### File Permissions
```bash
# Windows filesystem permissions can be tricky
# Work in WSL filesystem (/home/username) for better performance
# Or configure Docker to work with Windows paths
```

## 🐧 **Linux Development Setup** (Reference)

### Ubuntu 22.04 (Recommended for Performance)
```bash
# Install Docker
sudo apt update
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker $USER
# Logout and login again

# Continue with repository setup...
```

## 🔧 **Development Workflow**

### Starting Development Environment
```bash
# Works identically on Mac, Windows (WSL), and Linux
cd Autonomy/docker
./scripts/docker-dev.sh start
./scripts/docker-dev.sh enter

# You're now in the standardized Linux development environment
# regardless of your host OS
```

### VS Code Integration

#### Method 1: Remote-Containers Extension
1. Install "Remote-Containers" extension in VS Code
2. Open command palette: `Ctrl+Shift+P`
3. Select "Remote-Containers: Attach to Running Container"
4. Choose `autonomy_development` container
5. Develop directly in container with full ROS 2 support

#### Method 2: Volume Mounts
1. Edit files on host with your preferred editor
2. Changes sync automatically to running container
3. Use container for running/testing code

### Code Development
```bash
# In development container
cd /workspace

# Edit code (changes sync to host)
code code/navigation/src/navigation_node.py  # Opens in VS Code

# Run tests
cd code/navigation
python3 -m pytest test/ -v

# Build ROS 2 packages
cd ros2_ws
colcon build --symlink-install
```

## 🧪 **Testing on Different Platforms**

### Simulation Testing
```bash
# Launch Gazebo (works on all platforms via Docker)
ros2 launch mars_simulation simulation.launch.xml

# Run autonomy algorithms
ros2 launch autonomy_system autonomy.launch.xml

# Visualize in RViz
ros2 run rviz2 rviz2 -d config/navigation.rviz
```

### Unit Testing
```bash
# Run test suites (platform-independent)
cd /workspace
python3 -m pytest code/*/test/ -v --tb=short

# Generate coverage reports
python3 -m pytest --cov=code --cov-report=html
```

### Integration Testing
```bash
# Test ROS 2 communication between containers
ros2 topic list
ros2 node list

# Test specific subsystems
ros2 launch computer_vision test.launch.xml
ros2 launch slam test.launch.xml
```

## 🔄 **Cross-Platform Data Management**

### Shared Data Volumes
```bash
# Data is shared across all platforms via Docker volumes
# Works identically on Mac, Windows, and Linux

ls /workspace/data/vision/datasets/
ls /workspace/models/
ls /workspace/calibration/
```

### Git LFS for Large Files
```bash
# Large datasets/models are handled by Git LFS
# Works the same on all platforms
git lfs install
git lfs track "*.bag" "*.pcd" "*.h5" "*.pt"
```

## 🚀 **Performance Considerations**

### Mac Performance
- **Apple Silicon**: Good performance with Rosetta 2
- **Intel Macs**: Native x86_64 performance
- **GPU**: Limited GPU acceleration (use CPU-only for now)
- **File I/O**: Generally good performance

### Windows Performance
- **WSL 2**: Excellent performance for Linux workloads
- **File I/O**: Fast when working in WSL filesystem
- **GPU**: Limited GPU acceleration in WSL
- **Memory**: Ensure adequate RAM for Docker + WSL

### Linux Performance
- **Native**: Best performance for all workloads
- **GPU**: Full NVIDIA GPU acceleration available
- **Real-time**: Can achieve best real-time performance

## 🐛 **Platform-Specific Troubleshooting**

### Mac Issues
```bash
# X11 forwarding issues
brew install --cask xquartz
xhost +localhost

# Docker performance
# Ensure Docker Desktop has adequate resources allocated
# Settings → Resources → Advanced
```

### Windows Issues
```bash
# WSL integration issues
wsl --list --verbose
wsl --shutdown
wsl --start Ubuntu-22.04

# File permission issues
# Work in WSL filesystem (/home/username) instead of Windows mounts
```

### General Issues
```bash
# Docker build issues
docker system prune -a  # Clean old images
./scripts/docker-dev.sh clean

# Port conflicts
docker ps  # Check running containers
docker stop $(docker ps -aq)  # Stop all containers
```

## 📊 **Platform Comparison**

| Feature | Mac | Windows (WSL2) | Linux |
|---------|-----|----------------|-------|
| Setup Complexity | Low | Medium | Low |
| Performance | Good | Excellent | Best |
| GPU Support | Limited | Limited | Full |
| GUI Apps | Good (XQuartz) | Good (VcXsrv) | Excellent |
| File I/O | Good | Excellent (WSL) | Best |
| Real-time | Good | Good | Best |

## 🎯 **Best Practices**

### Development Environment
- **Use Docker**: Ensures identical environments across platforms
- **VS Code**: Use Remote-Containers for best development experience
- **Git**: Use Git LFS for large files, regular commits
- **Documentation**: Keep setup instructions updated

### Performance Optimization
- **Resource Allocation**: Give Docker adequate CPU/memory
- **Volume Mounts**: Use cached mounts for better performance
- **Background Services**: Keep only necessary services running
- **Regular Cleanup**: `docker system prune` periodically

### Team Collaboration
- **Shared Workflows**: All team members use same Docker commands
- **Documentation**: Update platform-specific issues in repo
- **Code Reviews**: Ensure cross-platform compatibility
- **CI/CD**: Use GitHub Actions for automated testing

## 🚀 **Getting Started**

### Quick Start for Any Platform
```bash
# 1. Install Docker Desktop for your platform
# 2. Clone repository
git clone https://github.com/your-org/urc-2026-autonomy.git

# 3. Navigate to autonomy directory
cd urc-2026-autonomy/Autonomy

# 4. Start development environment
cd docker
./scripts/docker-dev.sh build    # First time only
./scripts/docker-dev.sh start    # Daily startup
./scripts/docker-dev.sh enter    # Enter development container

# 5. Start developing!
cd /workspace
# Your autonomy code awaits...
```

This cross-platform setup ensures your entire team can contribute effectively, regardless of their preferred development platform! 🎯
