# Raspberry Pi Deployment Guide

## Overview
This guide covers deploying the autonomy system to Raspberry Pi devices for onboard processing, sensor integration, and distributed computing in the rover.

## 🖥️ **Supported Raspberry Pi Models**

| Model | CPU | RAM | Recommended Use | Performance Rating |
|-------|-----|-----|-----------------|-------------------|
| Pi 5 | Quad-core 64-bit | 8GB | Full autonomy node | ⭐⭐⭐⭐⭐ |
| Pi 4 | Quad-core 64-bit | 8GB | Computer vision/SLAM | ⭐⭐⭐⭐ |
| Pi 4 | Quad-core 64-bit | 4GB | State management/Navigation | ⭐⭐⭐⭐ |
| Pi 3B+ | Quad-core 64-bit | 1GB | Basic sensor processing | ⭐⭐⭐ |
| Pi Zero 2 W | Quad-core 64-bit | 512MB | Distributed sensors | ⭐⭐ |

## 📋 **Prerequisites**

### Hardware Requirements
- **Raspberry Pi**: Model 4 or 5 recommended
- **Power Supply**: Official 27W USB-C (Pi 5) or 15W USB-C (Pi 4)
- **MicroSD Card**: 32GB+ Class 10 (A1 rating preferred)
- **Cooling**: Active cooling (fan + heatsink) for sustained workloads
- **Storage**: External SSD recommended for data logging

### Software Requirements
- **Raspberry Pi Imager**: https://www.raspberrypi.com/software/
- **Ubuntu Server 22.04.3 LTS** (64-bit) for Raspberry Pi
- **SSH access** for headless deployment

## 🚀 **Initial Setup**

### 1. Flash Ubuntu Server
```bash
# Download Raspberry Pi Imager
# Select Ubuntu Server 22.04.3 LTS (64-bit)
# Choose your microSD card
# Configure WiFi and SSH during imaging
# Flash the image
```

### 2. Initial Boot and Configuration
```bash
# Insert microSD and boot Raspberry Pi
# SSH into the device (default user: ubuntu)
ssh ubuntu@raspberrypi.local

# Update system
sudo apt update && sudo apt upgrade -y

# Configure hostname (optional)
sudo hostnamectl set-hostname autonomy-pi
```

### 3. Install Docker
```bash
# Install Docker
sudo apt install apt-transport-https ca-certificates curl gnupg lsb-release
curl -fsSL https://download.docker.com/linux/ubuntu/gpg | sudo gpg --dearmor -o /usr/share/keyrings/docker-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/docker-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt update
sudo apt install docker.io docker-compose-plugin

# Add user to docker group
sudo usermod -aG docker ubuntu

# Enable Docker service
sudo systemctl enable docker
sudo systemctl start docker

# Test Docker
docker run hello-world
```

## 🔧 **ROS 2 Installation**

### Method 1: Native ROS 2 Installation (Recommended for Performance)
```bash
# Add ROS 2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu jammy main" | tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble
sudo apt update
sudo apt install ros-humble-ros-base python3-argcomplete -y

# Setup environment
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc

# Test ROS 2
ros2 run demo_nodes_cpp talker
```

### Method 2: Docker ROS 2 (For Development/Testing)
```bash
# Pull ROS 2 Docker image
docker pull osrf/ros:humble-desktop

# Run ROS 2 container
docker run -it --rm osrf/ros:humble-desktop bash
```

## 📦 **Autonomy System Deployment**

### 1. Clone Repository
```bash
# On Raspberry Pi
cd ~
git clone https://github.com/your-org/urc-2026-autonomy.git
cd urc-2026-autonomy/Autonomy
```

### 2. Docker Deployment
```bash
# Navigate to docker directory
cd docker

# Configure environment for Raspberry Pi
cp .env .env.pi
# Edit .env.pi for Raspberry Pi specific settings
nano .env.pi

# Build images optimized for ARM64
docker compose --env-file .env.pi build

# Start services
docker compose --env-file .env.pi up -d
```

### 3. Native Deployment (Recommended for Performance)
```bash
# Install Python dependencies
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cpu
pip3 install opencv-contrib-python numpy scipy matplotlib

# Build autonomy packages
cd ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select autonomy_*

# Create systemd services for auto-start
sudo cp scripts/autonomy.service /etc/systemd/system/
sudo systemctl enable autonomy
sudo systemctl start autonomy
```

## 🔌 **Hardware Integration**

### GPIO Setup (LED Status, Sensors)
```bash
# Install GPIO libraries
sudo apt install python3-gpiozero python3-rpi.gpio

# Test GPIO access
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); print('GPIO working')"

# Configure pin permissions
sudo usermod -a -G gpio ubuntu
```

### Camera Integration
```bash
# Enable camera interface
sudo raspi-config  # Interfacing Options → Camera → Enable

# Test camera
sudo apt install python3-picamera2
python3 -c "from picamera2 import Picamera2; print('Camera working')"
```

### Serial Device Setup (GPS, IMU)
```bash
# Configure serial ports
sudo raspi-config  # Interfacing Options → Serial → Enable

# Add user to dialout group for serial access
sudo usermod -a -G dialout ubuntu

# Test serial devices
ls /dev/tty*  # Should show ttyUSB0, ttyACM0, etc.
```

### I2C/SPI Setup (Sensors)
```bash
# Enable I2C/SPI interfaces
sudo raspi-config  # Interfacing Options → I2C/SPI → Enable

# Install I2C tools
sudo apt install i2c-tools python3-smbus

# Test I2C
sudo i2cdetect -y 1
```

## ⚙️ **Performance Optimization**

### CPU/GPU Optimization
```bash
# Enable performance governor
echo performance | sudo tee /sys/devices/system/cpu/cpu*/cpufreq/scaling_governor

# Disable unnecessary services
sudo systemctl disable bluetooth
sudo systemctl disable hciuart

# Optimize for real-time (if needed)
sudo apt install rt-tests
```

### Memory Management
```bash
# Create swap file for memory-intensive tasks
sudo fallocate -l 2G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
echo '/swapfile none swap sw 0 0' | sudo tee -a /etc/fstab

# Monitor memory usage
free -h
vmstat 1
```

### Thermal Management
```bash
# Monitor temperature
vcgencmd measure_temp
cat /sys/class/thermal/thermal_zone0/temp

# Install fan control (if using active cooling)
sudo apt install python3-gpiozero
# Create fan control script
```

## 🌐 **Networking Configuration**

### Static IP Configuration
```bash
# Configure static IP for reliable ROS 2 communication
sudo nano /etc/netplan/50-cloud-init.yaml

# Add static IP configuration:
network:
  version: 2
  ethernets:
    eth0:
      dhcp4: false
      addresses:
        - 192.168.1.100/24
      gateway4: 192.168.1.1
      nameservers:
        addresses: [8.8.8.8, 1.1.1.1]

sudo netplan apply
```

### ROS 2 Multi-Machine Setup
```bash
# Configure ROS 2 for multi-machine communication
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0

# Allow connections from development machine
export ROS_IP=$(hostname -I | awk '{print $1}')

# Or use ROS_DISCOVERY_SERVER for larger networks
export ROS_DISCOVERY_SERVER=192.168.1.10:11811
```

### Firewall Configuration
```bash
# Configure UFW for ROS 2 ports
sudo ufw allow 11811  # ROS discovery server
sudo ufw allow 7400:7410/udp  # ROS 2 DDS ports
sudo ufw allow ssh
sudo ufw enable
```

## 🚀 **Deployment Strategies**

### Strategy 1: Full Autonomy Node (Recommended for Pi 5)
```
Raspberry Pi 5 (8GB RAM)
├── Computer Vision (GPU acceleration)
├── SLAM Processing (real-time)
├── Navigation Algorithms
├── State Management
└── Sensor Integration (GPS, IMU, Cameras)
```

### Strategy 2: Distributed Vision Node (Pi 4)
```
Raspberry Pi 4 (8GB RAM)
├── Primary Computer Vision
├── Neural Network Inference
├── Camera Processing
└── Image Streaming to Central Computer
```

### Strategy 3: Sensor Hub (Pi 4)
```
Raspberry Pi 4 (4GB RAM)
├── GPS Processing
├── IMU Data Fusion
├── Sensor Data Aggregation
└── Health Monitoring
```

### Strategy 4: Distributed Sensors (Pi Zero 2 W)
```
Raspberry Pi Zero 2 W
├── Local Sensor Processing
├── Data Pre-filtering
├── Wireless Communication
└── Power Management
```

## 📊 **Monitoring & Diagnostics**

### System Monitoring
```bash
# Install monitoring tools
sudo apt install htop iotop sysstat

# Monitor system resources
htop
iostat -x 1
free -h

# ROS 2 monitoring
ros2 topic hz /camera/image_raw
ros2 topic delay /imu/data
```

### Logging Configuration
```bash
# Configure ROS 2 logging
export RCUTILS_LOGGING_SEVERITY=INFO
export RCUTILS_LOGGING_USE_STDOUT=1

# Log rotation
sudo apt install logrotate
sudo nano /etc/logrotate.d/autonomy
```

### Remote Monitoring
```bash
# Install monitoring server (optional)
sudo apt install prometheus-node-exporter
sudo systemctl enable prometheus-node-exporter

# Access via web interface
# http://raspberrypi.local:9100
```

## 🔄 **Update & Maintenance**

### Over-the-Air Updates
```bash
# Create update script
cat > update_autonomy.sh << 'EOF'
#!/bin/bash
cd ~/urc-2026-autonomy/Autonomy

# Pull latest changes
git pull origin main

# Update Docker images
cd docker
docker compose pull
docker compose build --no-cache
docker compose up -d

# Restart services
sudo systemctl restart autonomy
EOF

chmod +x update_autonomy.sh
```

### Backup Strategy
```bash
# Backup configuration and data
tar -czf autonomy_backup_$(date +%Y%m%d).tar.gz \
  ~/urc-2026-autonomy \
  /etc/systemd/system/autonomy.service \
  /etc/netplan/50-cloud-init.yaml

# Automated backup (add to cron)
crontab -e
# Add: 0 2 * * * ~/backup_autonomy.sh
```

## 🐛 **Troubleshooting**

### Common Issues

#### Docker Performance Issues
```bash
# Check Docker resource usage
docker stats

# Limit container resources
docker compose --env-file .env.pi up -d --scale vision-service=0

# Clean up Docker
docker system prune -f
```

#### ROS 2 Communication Issues
```bash
# Check ROS 2 discovery
ros2 node list
ros2 topic list

# Verify network configuration
ip addr show
ping development-machine-ip

# Check firewall
sudo ufw status
```

#### Hardware Interface Issues
```bash
# Test GPIO
python3 -c "import RPi.GPIO as GPIO; GPIO.setmode(GPIO.BCM); GPIO.setup(17, GPIO.OUT); GPIO.output(17, 1); print('GPIO working')"

# Test camera
python3 -c "from picamera2 import Picamera2; picam2 = Picamera2(); print('Camera working')"

# Test serial
ls /dev/tty*
stty -F /dev/ttyUSB0
```

#### Thermal Issues
```bash
# Monitor temperature
vcgencmd measure_temp
cat /sys/class/thermal/thermal_zone0/temp

# Check throttling
vcgencmd get_throttled

# Improve cooling
sudo apt install python3-gpiozero
# Implement fan control based on temperature
```

## 📈 **Performance Benchmarks**

### Raspberry Pi 5 Performance
- **Computer Vision**: 15-20 FPS YOLOv5 inference
- **SLAM**: Real-time processing with ORB-SLAM3
- **Navigation**: Complex path planning algorithms
- **Multi-threading**: Full utilization of 4 cores

### Raspberry Pi 4 Performance
- **Computer Vision**: 8-12 FPS neural network inference
- **SLAM**: Real-time with optimized algorithms
- **Navigation**: Standard path planning algorithms
- **Memory**: 4GB limit for complex models

### Optimization Tips
- Use TensorFlow Lite for smaller models
- Implement model quantization
- Use ROS 2 QoS for efficient communication
- Optimize camera resolution vs. performance

## 🎯 **Deployment Checklist**

### Pre-Deployment
- [ ] Ubuntu Server flashed and configured
- [ ] Docker installed and tested
- [ ] ROS 2 installed and verified
- [ ] Network configured with static IP
- [ ] Hardware interfaces tested (GPIO, I2C, SPI, Serial)

### Deployment
- [ ] Repository cloned and built
- [ ] Services configured and started
- [ ] ROS 2 communication verified
- [ ] Hardware integration tested
- [ ] Performance benchmarks completed

### Post-Deployment
- [ ] Monitoring tools configured
- [ ] Backup strategy implemented
- [ ] Update mechanism tested
- [ ] Documentation updated

This deployment guide ensures your Raspberry Pi becomes a reliable, high-performance component of your autonomy system! 🚀
