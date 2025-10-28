# Gazebo Mission Test Scenarios

This directory contains comprehensive test scenarios for validating autonomous navigation and SLAM performance in Gazebo simulation.

## Overview

The test scenarios are designed to assess:
- SLAM performance and accuracy
- Sensor data quality and fidelity
- Navigation capability in various environments
- Simulation adequacy for real hardware development

## Test Scenarios

### 1. Autonomous Waypoint Navigation (`autonomous_waypoint_navigation.py`)

**Purpose**: Validate SLAM performance during autonomous navigation through a 5-waypoint course.

**Test Profile**:
- 5-waypoint course over 500m in desert terrain
- Mix of GPS-available and GPS-denied sections
- Obstacles requiring path planning
- Loop closure opportunity (return near start)

**Validation Metrics**:
- SLAM pose drift vs ground truth (< 1m over 500m)
- Loop closure detection success (> 95%)
- Waypoint arrival accuracy (< 2m)
- Feature tracking stability (> 100 features/frame)
- Map consistency (< 10cm error after loop closure)
- Real-time performance (> 5Hz pose updates)

**Usage**:
```bash
# Launch Gazebo with desert terrain
ros2 launch autonomy_simulation rover_gazebo.launch.py world:=urc_desert_terrain

# Run the test
python3 autonomous_waypoint_navigation.py
```

### 2. GPS-Denied SLAM (`gps_denied_slam.py`)

**Purpose**: Validate visual-inertial SLAM performance during GPS-denied operation.

**Test Profile**:
- Start with GPS, transition to GPS-denied warehouse
- Navigate 200m indoor course
- Multiple turns and feature-rich environment
- Exit back to GPS-available area

**Validation Metrics**:
- SLAM-only drift during GPS-denied period (< 2% distance)
- GPS reacquisition and fusion time (< 30s)
- Visual feature count (> 200 features in indoor environment)
- Depth data quality (< 5cm noise std dev)
- Navigation continuity across GPS transitions

**Usage**:
```bash
# Launch Gazebo with GPS-denied area
ros2 launch autonomy_simulation rover_gazebo.launch.py world:=gps_denied_area

# Run the test
python3 gps_denied_slam.py
```

### 3. Dynamic Obstacle Avoidance (`dynamic_obstacle_avoidance.py`)

**Purpose**: Validate real-time mapping and obstacle avoidance capabilities.

**Test Profile**:
- Navigate cluttered obstacle field
- Real-time map updates from LiDAR
- Dynamic path replanning
- Narrow passage navigation

**Validation Metrics**:
- Obstacle detection range (> 3m for large objects)
- Map update latency (< 100ms)
- Path replan success rate (> 90%)
- Collision avoidance distance (> 0.5m clearance)
- LiDAR scan quality (< 3cm accuracy)

**Usage**:
```bash
# Launch Gazebo with obstacles course
ros2 launch autonomy_simulation rover_gazebo.launch.py world:=urc_obstacles

# Run the test
python3 dynamic_obstacle_avoidance.py
```

### 4. Endurance SLAM Test (`endurance_slam_test.py`)

**Purpose**: Validate long-duration operation stability and performance.

**Test Profile**:
- 30-minute continuous operation
- Multiple loops over same terrain
- Mix of GPS and SLAM operation
- Gradual environmental changes (simulated lighting)

**Validation Metrics**:
- Memory usage growth (< 500MB total, < 50MB/10min growth)
- CPU utilization stability (< 70% sustained)
- SLAM confidence over time (> 0.7 maintained)
- Loop closure accumulation (> 10 successful closures)
- Pose estimate stability (no catastrophic failures)

**Usage**:
```bash
# Launch Gazebo with any world
ros2 launch autonomy_simulation rover_gazebo.launch.py

# Run the test (will run for 30 minutes)
python3 endurance_slam_test.py
```

### 5. Sensor Failure Recovery (`sensor_failure_recovery.py`)

**Purpose**: Validate graceful degradation and recovery from sensor failures.

**Test Profile**:
- Normal operation with sequential sensor failures
- GPS dropout simulation
- Camera occlusion (simulated dust on lens)
- LiDAR partial blockage
- IMU noise injection

**Validation Metrics**:
- Sensor failure detection time (< 5s)
- Graceful degradation response (no crashes)
- Recovery time after sensor restoration (< 30s)
- Position error under degraded mode (< 5m)
- Navigation continuation capability

**Usage**:
```bash
# Launch Gazebo with any world
ros2 launch autonomy_simulation rover_gazebo.launch.py

# Run the test
python3 sensor_failure_recovery.py
```

## Running Tests

### Prerequisites

1. **ROS2 Environment**: Ensure ROS2 is properly installed and sourced
2. **Gazebo**: Install Gazebo with ROS2 integration
3. **Dependencies**: Install required Python packages:
   ```bash
   pip3 install numpy matplotlib scipy
   ```

### Quick Start

1. **Build the simulation package**:
   ```bash
   cd /path/to/urc-machiato-2026/Autonomy/simulation
   colcon build --packages-select autonomy_simulation
   source install/setup.bash
   ```

2. **Launch Gazebo with rover**:
   ```bash
   ros2 launch autonomy_simulation rover_gazebo.launch.py world:=urc_desert_terrain
   ```

3. **Run a test scenario**:
   ```bash
   cd test_scenarios
   python3 autonomous_waypoint_navigation.py
   ```

### Test Configuration

Tests can be configured using launch parameters:

```bash
# Launch with specific sensor noise level
ros2 launch autonomy_simulation rover_gazebo.launch.py \
    world:=urc_desert_terrain \
    sensor_noise_level:=high

# Launch with GPS disabled
ros2 launch autonomy_simulation rover_gazebo.launch.py \
    world:=gps_denied_area \
    enable_gps:=false

# Launch with custom starting position
ros2 launch autonomy_simulation rover_gazebo.launch.py \
    world:=urc_obstacles \
    x_pose:=5.0 \
    y_pose:=5.0 \
    yaw_pose:=1.57
```

## Interpreting Results

### Test Output

Each test generates:
- **Console output**: Real-time status and metrics
- **JSON results file**: Detailed test data saved to `/tmp/`
- **Log files**: ROS2 logs for debugging

### Key Metrics

#### SLAM Performance
- **Pose Drift**: Distance error between SLAM estimate and ground truth
- **Loop Closure Success**: Percentage of successful loop closures
- **Waypoint Accuracy**: Average distance error when reaching waypoints
- **Feature Count**: Number of visual features detected per frame
- **Map Consistency**: Error in map after loop closure

#### Navigation Capability
- **Waypoint Success Rate**: Percentage of waypoints successfully reached
- **Obstacle Avoidance Success**: Percentage of obstacles successfully avoided
- **Path Planning Quality**: Smoothness and efficiency of planned paths
- **Real-time Performance**: Whether system maintains real-time operation
- **Collision Count**: Number of collisions during test

#### Sensor Quality
- **Update Rate**: Frequency of sensor data updates (Hz)
- **Noise Level**: Standard deviation of sensor noise
- **Data Completeness**: Percentage of expected data received
- **Accuracy**: Measurement accuracy compared to ground truth

### Pass/Fail Criteria

#### High Confidence (Proceed with Development)
- Overall fidelity > 80%
- SLAM pose drift < 1m over 500m
- Waypoint success rate > 90%
- All sensors have fidelity > 70%

#### Medium Confidence (Validate Critical Paths)
- Overall fidelity 60-80%
- SLAM pose drift 1-2m over 500m
- Waypoint success rate 80-90%
- Some sensors have fidelity 60-70%

#### Low Confidence (Hardware Required)
- Overall fidelity < 60%
- SLAM pose drift > 2m over 500m
- Waypoint success rate < 80%
- Multiple sensors have fidelity < 60%

## Troubleshooting

### Common Issues

1. **Test fails to start**:
   - Check that Gazebo is running
   - Verify rover is spawned correctly
   - Check ROS2 topic connections

2. **No sensor data received**:
   - Verify sensor plugins are loaded
   - Check sensor topic names
   - Ensure proper QoS settings

3. **Poor SLAM performance**:
   - Check sensor noise parameters
   - Verify camera and LiDAR calibration
   - Ensure sufficient visual features

4. **Navigation failures**:
   - Check path planning parameters
   - Verify obstacle detection
   - Ensure proper coordinate frames

### Debug Mode

Enable debug output:
```bash
# Set ROS2 log level
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=DEBUG

# Run test with verbose output
python3 autonomous_waypoint_navigation.py --verbose
```

### Performance Monitoring

Monitor system resources during tests:
```bash
# Monitor CPU and memory
htop

# Monitor ROS2 topics
ros2 topic list
ros2 topic echo /rover/odom

# Monitor Gazebo performance
gz stats
```

## Expected Results

### Baseline Performance

For a well-configured simulation, expect:

- **SLAM Performance**:
  - Pose drift: 0.5-1.0m over 500m
  - Loop closure success: 95-98%
  - Waypoint accuracy: 1-2m
  - Feature count: 100-200 per frame

- **Navigation Capability**:
  - Waypoint success rate: 90-95%
  - Obstacle avoidance: 95-98%
  - Real-time performance: Yes
  - Collision count: 0

- **Sensor Quality**:
  - IMU fidelity: 0.8-0.9
  - GPS fidelity: 0.7-0.8
  - LiDAR fidelity: 0.8-0.9
  - Camera fidelity: 0.8-0.9

### Performance Variations

Performance may vary based on:
- **Hardware**: CPU, GPU, RAM available
- **Configuration**: Sensor noise levels, update rates
- **Environment**: World complexity, lighting conditions
- **Parameters**: SLAM algorithm settings

## Contributing

When adding new test scenarios:

1. Follow the existing naming convention
2. Include comprehensive documentation
3. Add appropriate validation metrics
4. Include usage examples
5. Update this README

## Support

For issues or questions:
1. Check the troubleshooting section
2. Review test logs and error messages
3. Verify system requirements
4. Contact the development team
