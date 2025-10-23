# Client-Server Architecture Guide

## Overview
This guide covers setting up a distributed autonomy system with Raspberry Pi clients and a development machine server, enabling scalable and modular autonomy processing across multiple devices.

## 🏗️ **Architecture Overview**

```
┌─────────────────────────────────────────────────┐
│                Development Server               │
│  (High-performance workstation/laptop)          │
├─────────────────────────────────────────────────┤
│ • Central State Management                      │
│ • Complex Algorithm Processing                  │
│ • Mission Planning & Coordination               │
│ • Data Aggregation & Analysis                   │
│ • User Interface & Visualization                │
└─────────────────┬───────────────────────────────┘
                  │ ROS 2 DDS Communication
                  │ (Real-time, reliable)
┌─────────────────┼───────────────────────────────┐
│                 │                               │
├─────────────────┼───────────────────────────────┼─────────────────┐
│   Raspberry Pi  │     Raspberry Pi    │   Raspberry Pi   │
│   Client #1     │     Client #2       │   Client #3      │
│                 │                     │                 │
│ • Computer      │ • SLAM Processing   │ • Navigation     │
│   Vision        │ • Sensor Fusion     │ • Motor Control  │
│ • Object        │ • GPS/IMU           │ • Terrain        │
│   Detection     │ • Real-time         │ • Analysis       │
│ • Image         │ • Mapping           │ • Odometry       │
│   Processing    │                     │                 │
└─────────────────┼─────────────────────┼─────────────────┘
                  │
          ┌───────┼───────┐
          │       │       │
    ┌─────▼────┐ ┌▼──────▼┐
    │Raspberry │ │Raspberry│
    │Pi Zero   │ │Pi Zero  │
    │Client #4 │ │Client #5│
    │          │ │         │
    │• Sensor  │ │• Sensor │
    │• Data    │ │• Data   │
    │• Pre-    │ │• Pre-   │
    │• processing│• processing│
    └───────────┘ └─────────┘
```

## 🎯 **Node Roles & Responsibilities**

### Server (Development Machine)
- **State Management**: Central mission coordination
- **Complex Algorithms**: Path planning, optimization
- **Data Processing**: Log analysis, performance metrics
- **User Interface**: RViz visualization, control interfaces
- **Mission Planning**: High-level decision making

### Client Nodes (Raspberry Pi)

#### Vision Client (Pi 4/5)
- **Camera Processing**: Real-time image capture and preprocessing
- **Object Detection**: Neural network inference for objects/markers
- **Image Streaming**: Compressed video feed to server
- **Local Processing**: Basic filtering and enhancement

#### SLAM Client (Pi 4/5)
- **Sensor Fusion**: IMU, GPS, wheel odometry integration
- **Local Mapping**: Real-time occupancy grid generation
- **Pose Estimation**: Local position tracking
- **Feature Extraction**: Visual/LiDAR feature detection

#### Navigation Client (Pi 4)
- **Motor Control**: Low-level wheel velocity commands
- **Odometry**: Wheel encoder processing
- **Terrain Sensing**: Local terrain classification
- **Obstacle Detection**: Proximity sensor processing

#### Sensor Clients (Pi Zero 2 W)
- **Data Acquisition**: Specialized sensor reading
- **Preprocessing**: Basic filtering and validation
- **Wireless Communication**: Reliable data transmission
- **Power Management**: Efficient operation on battery

## 🔧 **Setup Instructions**

### 1. Server Setup (Development Machine)

#### Install ROS 2 and Dependencies
```bash
# Ubuntu/Debian
sudo apt update
sudo apt install ros-humble-desktop python3-colcon-common-extensions

# Source ROS 2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc

# Install additional tools
sudo apt install ros-humble-rviz2 ros-humble-rqt ros-humble-plotjuggler
```

#### Configure as ROS 2 Server
```bash
# Set ROS 2 configuration
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # Server IP

# Create discovery server
ros2 run ros_discovery_server ros_discovery_server
```

#### Launch Server Components
```bash
# Launch central coordination
ros2 launch autonomy_server server.launch.xml

# Parameters:
# - discovery_server_ip: 192.168.1.100
# - domain_id: 42
# - qos_profile: reliable
```

### 2. Client Setup (Raspberry Pi)

#### Basic ROS 2 Installation
```bash
# On each Raspberry Pi
sudo apt update
sudo apt install ros-humble-ros-base python3-argcomplete

# Configure as client
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER=192.168.1.100:11811  # Server IP
```

#### Client-Specific Configuration
```bash
# Vision Client (Pi with camera)
export ROS_NAMESPACE=vision_client_1
ros2 launch autonomy_vision vision_client.launch.xml

# SLAM Client
export ROS_NAMESPACE=slam_client_1
ros2 launch autonomy_slam slam_client.launch.xml

# Navigation Client
export ROS_NAMESPACE=navigation_client_1
ros2 launch autonomy_navigation navigation_client.launch.xml
```

## 🌐 **Network Configuration**

### ROS 2 Discovery Server Setup
```bash
# On server machine
ros2 run ros_discovery_server ros_discovery_server \
  --address 192.168.1.100 \
  --port 11811

# On all clients
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
```

### Quality of Service (QoS) Configuration
```yaml
# qos_profiles.yaml
sensor_data:
  reliability: BEST_EFFORT
  durability: VOLATILE
  history: KEEP_LAST
  depth: 10

control_commands:
  reliability: RELIABLE
  durability: VOLATILE
  history: KEEP_LAST
  depth: 5

state_information:
  reliability: RELIABLE
  durability: TRANSIENT_LOCAL
  history: KEEP_ALL
  depth: 100
```

### Firewall Configuration
```bash
# On server (allow ROS 2 ports)
sudo ufw allow 11811/tcp  # Discovery server
sudo ufw allow 7400:7500/udp  # DDS participant ports
sudo ufw allow ssh

# On clients (minimal restrictions)
sudo ufw allow 22/tcp  # SSH
sudo ufw allow 11811/tcp  # Discovery
```

## 📡 **Communication Patterns**

### 1. Publisher-Subscriber (Data Streaming)
```python
# Sensor data publishing (clients → server)
class SensorPublisher(Node):
    def __init__(self):
        super().__init__('sensor_publisher')
        self.publisher = self.create_publisher(
            Imu, '/sensors/imu', qos_profile_sensor)

    def publish_data(self, imu_data):
        msg = Imu()
        # Fill message
        self.publisher.publish(msg)
```

```python
# Data aggregation (server)
class DataAggregator(Node):
    def __init__(self):
        super().__init__('data_aggregator')
        self.subscriptions = []

        # Subscribe to all client sensor topics
        for client_id in range(1, 6):
            topic = f'/client_{client_id}/sensors/imu'
            self.subscriptions.append(
                self.create_subscription(Imu, topic, self.imu_callback, qos_profile_sensor))
```

### 2. Service-Client (Request-Response)
```python
# Command service (server → clients)
class CommandService(Node):
    def __init__(self):
        super().__init__('command_service')
        self.service = self.create_service(
            SetMode, '/command/set_mode', self.set_mode_callback)

    def send_command_to_client(self, client_id, command):
        # Direct communication to specific client
        client = self.get_client_for_id(client_id)
        response = client.call(command)
        return response
```

### 3. Action Server-Client (Long-Running Tasks)
```python
# Navigation action (server → navigation client)
class NavigationCoordinator(Node):
    def __init__(self):
        super().__init__('navigation_coordinator')
        self.action_clients = {}

        # Create action clients for each navigation client
        for client_id in [3]:  # Navigation clients
            self.action_clients[client_id] = ActionClient(
                self, NavigateToPose, f'/client_{client_id}/navigate_to_pose')

    async def send_navigation_goal(self, client_id, goal):
        client = self.action_clients[client_id]
        await client.send_goal(goal)
```

## 🔄 **Data Flow Architecture**

### Sensor Data Pipeline
```
Raspberry Pi Sensors → Local Preprocessing → ROS 2 Topic → Server Aggregation → State Estimation → Control Commands
```

### Control Command Pipeline
```
Server Planning → ROS 2 Service/Action → Client Execution → Hardware Control → Feedback Loop
```

### Visualization Pipeline
```
Client Cameras → Image Streaming → Server Processing → RViz Display → Operator Feedback
```

## ⚙️ **Configuration Management**

### Centralized Configuration
```yaml
# server_config.yaml
server:
  ip: 192.168.1.100
  discovery_port: 11811
  domain_id: 42

clients:
  vision_1:
    ip: 192.168.1.101
    role: computer_vision
    capabilities: [object_detection, aruco_tracking]

  slam_1:
    ip: 192.168.1.102
    role: slam_processing
    capabilities: [pose_estimation, mapping]

  navigation_1:
    ip: 192.168.1.103
    role: motor_control
    capabilities: [wheel_control, odometry]
```

### Dynamic Client Discovery
```python
class ClientManager(Node):
    def __init__(self):
        super().__init__('client_manager')
        self.clients = {}
        self.discovery_timer = self.create_timer(5.0, self.discovery_callback)

    def discovery_callback(self):
        # Query ROS 2 graph for active clients
        node_names = self.get_node_names()
        for node in node_names:
            if 'client_' in node:
                client_id = self.extract_client_id(node)
                self.register_client(client_id, node)
```

## 🔍 **Monitoring & Diagnostics**

### System Health Monitoring
```bash
# On server - monitor all clients
ros2 run autonomy_monitor health_monitor

# Check client connectivity
ros2 node list | grep client

# Monitor topic statistics
ros2 topic hz /client_1/sensors/imu
ros2 topic delay /client_1/camera/image_raw
```

### Performance Profiling
```python
# Network latency monitoring
class LatencyMonitor(Node):
    def __init__(self):
        super().__init__('latency_monitor')
        self.latency_stats = {}

    def measure_latency(self, client_id, topic):
        # Send timestamped message
        # Measure round-trip time
        # Store statistics
        pass
```

### Diagnostic Tools
```bash
# ROS 2 diagnostic aggregator
ros2 run diagnostic_aggregator aggregator

# Custom diagnostics
ros2 run autonomy_diagnostics client_diagnostics --client-id 1
ros2 run autonomy_diagnostics network_diagnostics
```

## 🛠️ **Deployment Scripts**

### Automated Client Setup
```bash
# client_setup.sh
#!/bin/bash

# Configure network
sudo nmcli con mod "Wired connection 1" ipv4.addresses "192.168.1.10$1/24"
sudo nmcli con mod "Wired connection 1" ipv4.gateway "192.168.1.1"
sudo nmcli con down "Wired connection 1" && sudo nmcli con up "Wired connection 1"

# Set ROS configuration
export ROS_DOMAIN_ID=42
export ROS_DISCOVERY_SERVER=192.168.1.100:11811
export ROS_NAMESPACE=client_$1

# Launch client services based on ID
case $1 in
    1) ros2 launch autonomy_vision vision_client.launch.xml ;;
    2) ros2 launch autonomy_slam slam_client.launch.xml ;;
    3) ros2 launch autonomy_navigation navigation_client.launch.xml ;;
    *) echo "Unknown client ID" ;;
esac
```

### Server Deployment
```bash
# server_setup.sh
#!/bin/bash

# Start discovery server
ros2 run ros_discovery_server ros_discovery_server \
  --address 192.168.1.100 \
  --port 11811 &

# Launch server components
ros2 launch autonomy_server server.launch.xml &

# Start monitoring
ros2 run autonomy_monitor health_monitor &
```

## 🚨 **Fault Tolerance & Recovery**

### Client Failure Handling
```python
class FaultToleranceManager(Node):
    def __init__(self):
        super().__init__('fault_tolerance')
        self.client_health = {}
        self.health_timer = self.create_timer(1.0, self.health_check)

    def health_check(self):
        # Check client responsiveness
        for client_id, health in self.client_health.items():
            if not health['responsive']:
                self.handle_client_failure(client_id)

    def handle_client_failure(self, client_id):
        # Implement failover strategies
        # - Switch to backup client
        # - Reduce functionality
        # - Alert operator
        pass
```

### Network Failure Recovery
```python
class NetworkRecovery(Node):
    def __init__(self):
        super().__init__('network_recovery')
        self.network_timer = self.create_timer(5.0, self.network_check)

    def network_check(self):
        # Test connectivity to all clients
        # Implement reconnection logic
        # Handle partial network failures
        pass
```

## 📊 **Performance Optimization**

### Load Balancing
```python
class LoadBalancer(Node):
    def __init__(self):
        super().__init__('load_balancer')
        self.client_load = {}

    def distribute_task(self, task_type, task_data):
        # Find least loaded client for task type
        suitable_clients = self.get_clients_for_task(task_type)
        best_client = min(suitable_clients, key=lambda c: self.client_load[c])
        return best_client
```

### Bandwidth Optimization
```python
# Compressed image streaming
class CompressedImagePublisher(Node):
    def __init__(self):
        super().__init__('compressed_publisher')
        self.publisher = self.create_publisher(CompressedImage, '/camera/compressed', 10)

    def publish_compressed(self, image):
        # Compress image before publishing
        # Reduce bandwidth usage
        pass
```

## 🧪 **Testing Distributed System**

### Unit Testing (Individual Components)
```bash
# Test individual clients
ros2 test autonomy_vision --gtest_filter=ArucoDetector.*
ros2 test autonomy_slam --gtest_filter=PoseEstimator.*

# Test server components
ros2 test autonomy_server --gtest_filter=StateManager.*
```

### Integration Testing (Multi-Client)
```bash
# Launch all clients and server
./scripts/deploy_all.sh

# Run integration tests
ros2 launch autonomy_tests integration_tests.launch.xml

# Test communication reliability
ros2 run autonomy_tests communication_test --clients 5 --duration 300
```

### Performance Testing
```bash
# Load testing
ros2 run autonomy_tests load_test --clients 5 --stress-level high

# Latency testing
ros2 run autonomy_tests latency_test --round-trips 1000

# Bandwidth testing
ros2 run autonomy_tests bandwidth_test --duration 60
```

## 🎯 **Scaling Considerations**

### Adding More Clients
1. **IP Address Assignment**: Configure static IPs for new clients
2. **ROS Configuration**: Add to discovery server configuration
3. **Load Balancing**: Update task distribution algorithms
4. **Monitoring**: Add to health monitoring system

### Performance Scaling
- **Network Infrastructure**: Consider dedicated ROS 2 network
- **Discovery Server**: Use multiple discovery servers for large deployments
- **QoS Profiles**: Optimize for different data types and requirements
- **Resource Management**: Monitor and limit resource usage per client

## 📈 **Success Metrics**

### Performance Metrics
- **Latency**: <50ms end-to-end for control commands
- **Throughput**: >10MB/s aggregate data rate
- **Reliability**: >99.9% message delivery
- **Scalability**: Linear performance scaling with clients

### System Metrics
- **Client Health**: >95% uptime per client
- **Network Utilization**: <70% bandwidth usage
- **CPU Usage**: <80% per client, <60% server
- **Memory Usage**: <75% per device

This client-server architecture provides a scalable, fault-tolerant foundation for distributed autonomy processing across multiple Raspberry Pi devices! 🚀
