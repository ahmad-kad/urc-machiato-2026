# Docker Development Environment - University Rover Challenge 2026

## Overview
This guide covers setting up a containerized development environment using Docker for the autonomy system. Docker provides consistent, reproducible development environments across different machines and team members.

## Prerequisites

### System Requirements
- **Docker Engine**: Version 20.10 or later
- **Docker Compose**: Version 2.0 or later
- **RAM**: 8GB minimum, 16GB recommended
- **Storage**: 20GB free space for containers and images

### Installation
```bash
# Install Docker (Ubuntu/Debian)
sudo apt update
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release

# Add Docker's official GPG key
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg

# Set up stable repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] https://download.docker.com/linux/ubuntu $(lsb_release -cs) stable" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null

# Install Docker Engine
sudo apt update
sudo apt install docker-ce docker-ce-cli containerd.io docker-compose-plugin

# Start and enable Docker
sudo systemctl start docker
sudo systemctl enable docker

# Add user to docker group (logout/login required)
sudo usermod -aG docker $USER

# Verify installation
docker --version
docker compose version
```

## 1. Base ROS 2 Docker Image

### Create Dockerfile for ROS 2 Humble
```dockerfile
# Dockerfile.ros2_humble
FROM ubuntu:22.04

# Avoid interactive prompts during build
ENV DEBIAN_FRONTEND=noninteractive

# Install system dependencies
RUN apt-get update && apt-get install -y \
    curl \
    gnupg2 \
    lsb-release \
    sudo \
    && rm -rf /var/lib/apt/lists/*

# Add ROS 2 repository
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    python3-argcomplete \
    && rm -rf /var/lib/apt/lists/*

# Install development tools
RUN apt-get update && apt-get install -y \
    python3-pip \
    git \
    vim \
    wget \
    build-essential \
    cmake \
    && rm -rf /var/lib/apt/lists/*

# Install Python dependencies
RUN pip3 install \
    numpy \
    scipy \
    matplotlib \
    opencv-contrib-python \
    torch \
    torchvision \
    torchaudio \
    --extra-index-url https://download.pytorch.org/whl/cpu

# Create workspace directory
RUN mkdir -p /ros2_ws/src
WORKDIR /ros2_ws

# Setup ROS 2 environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc

# Create non-root user
RUN useradd -m -s /bin/bash ros && \
    usermod -aG sudo ros && \
    echo 'ros ALL=(ALL) NOPASSWD:ALL' >> /etc/sudoers

USER ros
WORKDIR /home/ros

# Default command
CMD ["bash"]
```

### Build Base Image
```bash
# Build the ROS 2 base image
docker build -f Dockerfile.ros2_humble -t autonomy:ros2-humble .

# Verify image
docker images
```

## 2. Autonomy Development Environment

### Development Dockerfile
```dockerfile
# Dockerfile.development
FROM autonomy:ros2_humble

# Switch to root for installation
USER root

# Install Gazebo
RUN wget https://packages.osrfoundation.org/gazebo.gpg -O /usr/share/keyrings/pkgs-osrf-archive-keyring.gpg
RUN echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/pkgs-osrf-archive-keyring.gpg] http://packages.osrfoundation.org/gazebo/ubuntu-stable jammy main" | tee /etc/apt/sources.list.d/gazebo-stable.list > /dev/null
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    && rm -rf /var/lib/apt/lists/*

# Install computer vision dependencies
RUN apt-get update && apt-get install -y \
    libopencv-dev \
    python3-opencv \
    && rm -rf /var/lib/apt/lists/*

# Install CUDA (if GPU available)
RUN apt-get update && apt-get install -y \
    nvidia-cuda-toolkit \
    && rm -rf /var/lib/apt/lists/*

# Install additional development tools
RUN pip3 install \
    pytest \
    pytest-cov \
    black \
    flake8 \
    mypy \
    jupyter \
    notebook

# Install SLAM and navigation packages
RUN apt-get update && apt-get install -y \
    ros-humble-slam-toolbox \
    ros-humble-navigation2 \
    ros-humble-nav2-bringup \
    ros-humble-tf-transformations \
    && rm -rf /var/lib/apt/lists/*

# Create development directories
RUN mkdir -p /workspace /shared

# Set permissions
RUN chown -R ros:ros /workspace /shared /ros2_ws

# Switch back to ros user
USER ros
WORKDIR /workspace

# Setup environment
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
RUN echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
RUN echo "export GAZEBO_MODEL_PATH=/workspace/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc

# Default command
CMD ["bash"]
```

### Docker Compose for Development
```yaml
# docker-compose.yml
version: '3.8'

services:
  autonomy-dev:
    build:
      context: .
      dockerfile: Dockerfile.development
    container_name: autonomy_development
    volumes:
      - ./:/workspace:cached
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      - ~/.Xauthority:/root/.Xauthority:rw
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    devices:
      - /dev/dri:/dev/dri
    network_mode: host
    privileged: true
    stdin_open: true
    tty: true
    command: bash

  autonomy-sim:
    build:
      context: .
      dockerfile: Dockerfile.simulation
    container_name: autonomy_simulation
    volumes:
      - ./:/workspace:cached
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
    environment:
      - DISPLAY=${DISPLAY}
      - ROS_DOMAIN_ID=42
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
    devices:
      - /dev/dri:/dev/dri
    network_mode: host
    privileged: true
    depends_on:
      - autonomy-dev
    command: ros2 launch mars_simulation simulation.launch.xml
```

### Simulation Dockerfile
```dockerfile
# Dockerfile.simulation
FROM autonomy:ros2_humble

USER root

# Install simulation dependencies
RUN apt-get update && apt-get install -y \
    gazebo \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    xvfb \
    && rm -rf /var/lib/apt/lists/*

# Install additional simulation tools
RUN pip3 install \
    gym \
    stable-baselines3 \
    pybullet

USER ros
WORKDIR /workspace

# Setup environment for headless simulation
RUN echo "export DISPLAY=:99" >> ~/.bashrc

CMD ["bash"]
```

## 3. Development Workflow

### Start Development Environment
```bash
# Build and start development container
docker compose build autonomy-dev
docker compose up -d autonomy-dev

# Enter the container
docker exec -it autonomy_development bash

# Inside container, setup workspace
cd /workspace
source /opt/ros/humble/setup.bash
```

### Workspace Setup in Container
```bash
# Create ROS 2 workspace
mkdir -p /workspace/ros2_ws/src
cd /workspace/ros2_ws/src

# Clone your packages
git clone https://github.com/your-org/rover_autonomy.git
git clone https://github.com/your-org/rover_description.git
git clone https://github.com/your-org/mars_simulation.git

# Install dependencies
cd /workspace/ros2_ws
rosdep install --from-paths src --ignore-src -r -y

# Build packages
colcon build --symlink-install
source install/setup.bash
```

### Running Simulations
```bash
# Start simulation in separate container
docker compose up -d autonomy-sim

# Or run simulation in development container
cd /workspace/ros2_ws
source install/setup.bash
ros2 launch mars_simulation simulation.launch.xml
```

## 4. GPU Support

### NVIDIA GPU Integration
```yaml
# Add to docker-compose.yml
services:
  autonomy-dev:
    # ... existing config ...
    deploy:
      resources:
        reservations:
          devices:
            - driver: nvidia
              count: 1
              capabilities: [gpu]
    environment:
      - NVIDIA_VISIBLE_DEVICES=all
      - NVIDIA_DRIVER_CAPABILITIES=all
```

### CUDA Development Setup
```dockerfile
# Add to Dockerfile.development
# Install CUDA toolkit
RUN apt-get update && apt-get install -y \
    nvidia-cuda-toolkit \
    nvidia-cuda-dev \
    && rm -rf /var/lib/apt/lists/*

# Install cuDNN (if needed)
# Download and install cuDNN from NVIDIA
```

## 5. Volume Management

### Development Volumes
```yaml
# docker-compose.yml volumes
volumes:
  - ./ros2_ws:/workspace/ros2_ws:cached
  - ./models:/workspace/models:cached
  - ./data:/workspace/data:cached
  - ./scripts:/workspace/scripts:cached
  - ~/.ssh:/home/ros/.ssh:ro
  - ~/.gitconfig:/home/ros/.gitconfig:ro
```

### Data Persistence
```bash
# Create named volumes for data persistence
docker volume create autonomy_data
docker volume create autonomy_models

# Use in compose
volumes:
  - autonomy_data:/workspace/data
  - autonomy_models:/workspace/models
```

## 6. Multi-Container Setup

### Service Architecture
```yaml
# docker-compose.multi.yml
version: '3.8'

services:
  # ROS 2 Core
  ros2-core:
    image: autonomy:ros2-humble
    command: roscore
    networks:
      - ros

  # Computer Vision Service
  vision-service:
    build:
      context: .
      dockerfile: Dockerfile.vision
    depends_on:
      - ros2-core
    networks:
      - ros
    environment:
      - ROS_MASTER_URI=http://ros2-core:11311

  # SLAM Service
  slam-service:
    build:
      context: .
      dockerfile: Dockerfile.slam
    depends_on:
      - ros2-core
    networks:
      - ros
    environment:
      - ROS_MASTER_URI=http://ros2-core:11311

  # Development Environment
  dev-env:
    build:
      context: .
      dockerfile: Dockerfile.development
    depends_on:
      - ros2-core
    networks:
      - ros
    volumes:
      - ./:/workspace
    environment:
      - ROS_MASTER_URI=http://ros2-core:11311

networks:
  ros:
    driver: bridge
```

## 7. CI/CD with Docker

### GitHub Actions Workflow
```yaml
# .github/workflows/docker-ci.yml
name: Docker CI

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
    - uses: actions/checkout@v2

    - name: Build Docker image
      run: docker build -f Dockerfile.development -t autonomy:test .

    - name: Run tests in container
      run: |
        docker run --rm autonomy:test /bin/bash -c "
          source /opt/ros/humble/setup.bash &&
          cd /workspace &&
          colcon build --symlink-install &&
          colcon test
        "

    - name: Upload test results
      uses: actions/upload-artifact@v2
      if: always()
      with:
        name: test-results
        path: /tmp/test_results/
```

## 8. Debugging and Monitoring

### Container Debugging
```bash
# View container logs
docker compose logs autonomy-dev

# Enter running container
docker exec -it autonomy_development bash

# Monitor resource usage
docker stats

# View container environment
docker exec autonomy_development env
```

### ROS 2 Debugging in Docker
```bash
# Check ROS nodes
docker exec autonomy_development ros2 node list

# Monitor topics
docker exec autonomy_development ros2 topic list

# Debug with gdb
docker exec -it autonomy_development gdb --args your_node_executable
```

## 9. Best Practices

### Container Optimization
```dockerfile
# Use multi-stage builds
FROM autonomy:ros2-humble AS base

# Development stage
FROM base AS development
# Add development tools

# Production stage
FROM base AS production
# Strip development dependencies
```

### Security Considerations
```dockerfile
# Don't run as root
RUN useradd -m -s /bin/bash ros
USER ros

# Minimize attack surface
RUN apt-get remove -y curl wget  # Remove after installation

# Use specific versions
FROM ubuntu:22.04  # Specific version tags
```

### Performance Tuning
```yaml
# Resource limits
services:
  autonomy-dev:
    deploy:
      resources:
        limits:
          cpus: '4.0'
          memory: 8G
        reservations:
          cpus: '2.0'
          memory: 4G
```

## 10. Common Issues and Solutions

### X11 Display Issues
```bash
# Allow Docker to connect to X server
xhost +local:docker

# Or use X11 forwarding
docker run -e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix autonomy:dev
```

### Permission Issues
```bash
# Fix workspace permissions
docker exec autonomy_development sudo chown -R ros:ros /workspace

# Mount with correct permissions
volumes:
  - ./:/workspace:cached
  - /etc/passwd:/etc/passwd:ro
  - /etc/group:/etc/group:ro
```

### Network Issues
```bash
# Check container networking
docker network ls
docker network inspect bridge

# Use host networking for ROS 2
network_mode: host
```

### GPU Issues
```bash
# Check NVIDIA container toolkit
docker run --rm --gpus all nvidia/cuda:11.0-base nvidia-smi

# Install NVIDIA container toolkit
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -s -L https://nvidia.github.io/nvidia-docker/gpgkey | sudo apt-key add -
curl -s -L https://nvidia.github.io/nvidia-docker/$distribution/nvidia-docker.list | sudo tee /etc/apt/sources.list.d/nvidia-docker.list
```

## 11. Backup and Recovery

### Container Backups
```bash
# Save container state
docker commit autonomy_development autonomy:backup-$(date +%Y%m%d)

# Export container
docker save autonomy:latest > autonomy_latest.tar

# Import container
docker load < autonomy_latest.tar
```

### Data Backup
```bash
# Backup volumes
docker run --rm -v autonomy_data:/data -v $(pwd):/backup alpine tar czf /backup/data_backup.tar.gz -C /data .

# Restore volumes
docker run --rm -v autonomy_data:/data -v $(pwd):/backup alpine tar xzf /backup/data_backup.tar.gz -C /data
```

This Docker setup provides a consistent, reproducible development environment for the autonomy team, enabling efficient collaboration and deployment across different machines and operating systems.
