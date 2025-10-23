# RGB-D SLAM System for URC 2026 - Desert Optimization

Complete SLAM (Simultaneous Localization and Mapping) subsystem optimized for low-feature desert environments using RGB-D cameras and GPS integration.

## System Overview

```
RGB-D Camera Input
    ↓
Depth Processing (Desert Noise Filtering)
    ↓
    ├─ Bilateral Filtering (edge-preserving)
    ├─ Dust Detection & Removal
    ├─ Temporal Smoothing
    └─ Range Filtering
    ↓
RTAB-Map SLAM (RGB-D Mapping & Localization)
    ↓
    ├─ Feature Extraction (ORB)
    ├─ Loop Closure Detection
    ├─ Place Recognition
    └─ Graph Optimization
    ↓
GPS Fusion Layer (Extended Kalman Filter)
    ↓
    ├─ Local Accuracy (SLAM)
    ├─ Global Reference (GPS)
    └─ Graceful Degradation
    ↓
Navigation System Output
```

## Key Features

### 1. Desert-Optimized Depth Processing

- **Bilateral Filtering**: Preserves object edges while reducing sand noise
- **Dust Particle Detection**: Identifies and removes isolated dust particles
- **Temporal Smoothing**: Reduces temporal noise using median filtering
- **Range Filtering**: Removes out-of-range and invalid depth measurements

**Why This Matters**: Desert sand creates significant depth noise that breaks standard SLAM algorithms. Our preprocessing addresses this directly.

### 2. RTAB-Map RGB-D SLAM

RTAB-Map (Real-Time Appearance-Based Mapping) is specifically designed for RGB-D sensors and excels in:

- **Loop Closure Detection**: Recognizes when the robot returns to a previously mapped area
- **Place Recognition**: Uses visual appearance to identify loop closures even in low-texture environments
- **Memory-Efficient Mapping**: Can prune old data to stay within Raspberry Pi 5 memory constraints
- **Multi-Modal Fusion**: Combines RGB features with depth information

**Configuration Tuned For Desert**:
- Increased feature detection (400 features vs 300 default)
- More aggressive loop closure detection
- Memory-aware operation (<500MB sustained)
- Relaxed feature quality thresholds for sparse environments

### 3. GPS Integration via Extended Kalman Filter

- **Sensor Fusion**: Combines SLAM (local accuracy) with GPS (global reference)
- **Confidence-Based Weighting**: Automatically adjusts SLAM/GPS trust based on covariance
- **Graceful Degradation**: Falls back to GPS-only if SLAM confidence drops
- **Fallback Chain**: GPS → Dead Reckoning → Last Known Position

**Fusion Modes**:
- `gps_slam_fusion`: Combines both (best performance)
- `slam_only`: SLAM only (GPS unavailable)
- `gps_only`: GPS fallback
- `dead_reckoning`: Dead reckoning only

### 4. System Health Monitoring

- **Real-Time Diagnostics**: Tracks feature count, confidence, loop closures
- **Status Reporting**: Publishes system state and mode
- **Performance Metrics**: Monitors CPU, memory, update rates
- **Automatic Degradation**: Reduces processing load under system stress

## Dependencies

### Hardware
- RGB-D Camera (Oak-D RGB-D recommended)
- GPS receiver (RTK preferred for accuracy)
- Raspberry Pi 5 (8GB RAM recommended)
- IMU (optional, for enhanced fusion)

### Software
```bash
sudo apt install ros-humble-rtabmap-ros ros-humble-rtabmap
pip install numpy opencv-python scipy
```

### ROS 2 Packages
- `rtabmap_ros` - RTAB-Map ROS 2 wrapper
- `tf2_ros` - Transform framework
- `image_transport` - Efficient image transport
- `message_filters` - Message synchronization

## Installation & Setup

### 1. Build the Package
```bash
cd ~/robotics2025/Autonomy/ros2_ws
colcon build --packages-select autonomy_slam
source install/setup.bash
```

### 2. Configure Camera Topics
Ensure your RGB-D camera publishes to:
- `/camera/rgb/image_raw` - RGB images
- `/camera/depth/image_raw` - Depth images
- `/camera/depth/camera_info` - Camera calibration

### 3. Configure GPS Topic
GPS data should be published to:
- `/gps/fix` - NavSatFix messages (latitude, longitude, altitude)

### 4. Camera Calibration
Camera intrinsic and extrinsic calibration is critical for SLAM accuracy:
```bash
# See calibration subsystem for detailed procedure
rosrun camera_calibration cameracalibrator.py --size 8x6 --square 0.108 image:=/camera/rgb/image_raw camera:=/camera/rgb
```

## Running the System

### Full SLAM System (Recommended)
```bash
ros2 launch autonomy_slam slam_rtabmap.launch.py
```

### Individual Components

**Depth Processor Only**:
```bash
ros2 run autonomy_slam depth_processor
```

**RTAB-Map SLAM**:
```bash
ros2 run rtabmap_ros rtabmap
```

**GPS Fusion**:
```bash
ros2 run autonomy_slam gps_fusion_node
```

**Orchestrator (Health Monitoring)**:
```bash
ros2 run autonomy_slam slam_node
```

## Configuration & Tuning

### Desert-Specific Parameters

**For More Aggressive Loop Closure** (sparse environments):
```yaml
# In rtabmap_desert.yaml
Mem/RehearsalSimilarity: 0.40      # Lower = more aggressive
Loop/MinVisualInliers: 10          # Reduced threshold
Reg/Histogram: 0.20                # More permissive
```

**For Tighter Memory (< 300MB)**:
```yaml
Mem/MaxMemoryMB: 300
Mem/BadSignaturesIgnored: true
Kp/MaxFeatures: 300                # Reduce from 400
```

**For Higher Accuracy (if CPU allows)**:
```yaml
Rtabmap/GraphOptimization: toro    # More accurate than g2o
RGBD/OptimizeMaxError: 0.01        # Lower error threshold
Mem/RecentWmSize: 400              # Larger working memory
```

### Depth Processing Tuning

**Noisier Sand Environment**:
```yaml
depth_sigma_color: 100.0           # Increase smoothing
depth_sigma_space: 100.0           # Larger neighborhood
temporal_smoothing_window: 5       # More history
dust_threshold: 30                 # More aggressive dust removal
```

**Cleaner Environment**:
```yaml
depth_sigma_color: 50.0            # Less smoothing
depth_sigma_space: 50.0            # Tighter neighborhood
temporal_smoothing_window: 2       # Less history
enable_dust_detection: false       # Skip dust filtering
```

## Topics & Message Flows

### Input Topics
```
/camera/rgb/image_raw (sensor_msgs/Image)
    → RGB images at 30Hz

/camera/depth/image_raw (sensor_msgs/Image)
    → Depth images (uint16) at 30Hz

/camera/depth/camera_info (sensor_msgs/CameraInfo)
    → Camera intrinsic calibration

/gps/fix (sensor_msgs/NavSatFix)
    → GPS position (lat/lon/alt) at 1-10Hz
```

### Output Topics
```
slam/rgb/image (sensor_msgs/Image)
    → Synchronized RGB for SLAM

slam/depth/processed (sensor_msgs/Image)
    → Filtered depth for SLAM

slam/pose (geometry_msgs/PoseWithCovarianceStamped)
    → SLAM pose estimate

slam/pose/fused (geometry_msgs/PoseWithCovarianceStamped)
    → GPS-fused pose (recommended for navigation)

slam/system/status (std_msgs/String)
    → System operational status

slam/system/health (std_msgs/String)
    → Health metrics (features, confidence, loops)

slam/system/diagnostics (diagnostic_msgs/DiagnosticArray)
    → Detailed diagnostic information

rtabmap/stat (std_msgs/String)
    → RTAB-Map statistics (features, nodes, etc.)

slam/fusion/status (std_msgs/String)
    → GPS fusion mode and confidence
```

### TF Transforms
```
map → base_link_fused  (SLAM pose, updated 10Hz)
map → base_link        (SLAM raw pose from RTAB-Map)
```

## Performance Targets

### Raspberry Pi 5
- **CPU Usage**: 40-60% multicore
- **Memory**: <500MB sustained, <800MB peak
- **Pose Update Rate**: 10 Hz
- **Feature Detection**: 50-400 features/frame (tunable)
- **Loop Closure Rate**: 0.1-1 Hz (depends on environment)

### Accuracy
- **SLAM Drift**: <1% distance traveled (GPS-denied)
- **GPS Integration**: <0.5m absolute position error
- **Loop Closure Accuracy**: <5cm for revisited areas
- **Latency**: <100ms end-to-end (sensor → navigation)

## Troubleshooting

### Issue: "Features Below Minimum Threshold"
**Cause**: Not enough distinctive features in environment
**Solution**:
1. Increase `Kp/MaxFeatures` to 500+
2. Lower `Loop/MinVisualInliers` to 10
3. Ensure good lighting conditions
4. Add visual markers (ArUco tags) to environment

### Issue: "SLAM Confidence Very Low"
**Cause**: Poor initial calibration or extreme dust
**Solution**:
1. Recalibrate camera (see Calibration subsystem)
2. Increase temporal smoothing
3. Reduce `dust_threshold` for more aggressive dust removal
4. Verify camera is mounted rigidly (minimal vibration)

### Issue: "Memory Usage Exceeds Limit"
**Cause**: Map growing too large or images accumulating
**Solution**:
1. Reduce `Mem/MaxMemoryMB` (forces more aggressive pruning)
2. Enable `Mem/BinDataKept: false` (don't store binary data)
3. Reduce `Mem/RecentWmSize` (smaller working memory)
4. Increase loop closure detection (force relocation)

### Issue: "GPS and SLAM Diverging"
**Cause**: GPS covariance poorly estimated or SLAM drifting
**Solution**:
1. Increase `gps_std_dev` if GPS is less accurate
2. Reduce `slam_confidence_threshold` to trust GPS more
3. Check GPS is getting good fix (status > 0)
4. Verify GPS and SLAM coordinate frames aligned

### Issue: "High Latency / Jerky Motion"
**Cause**: Processing bottleneck or sync issues
**Solution**:
1. Reduce `Kp/MaxFeatures` to 300
2. Reduce `temporal_smoothing_window` to 2
3. Disable `Mem/CompressionParallelized`
4. Profile with `ros2 run` `--profile` to identify bottleneck

## Integration with Navigation

The fused pose from `/slam/pose/fused` should be used by the navigation system as the primary localization input. Fallback chain:

```
Primary:   /slam/pose/fused     (SLAM + GPS fusion)
Secondary: /slam/pose           (SLAM only)
Tertiary:  /gps/fix             (GPS only)
```

The SLAM system automatically selects the best available mode and publishes via `/slam/fusion/status`.

## Testing & Validation

### Unit Tests
```bash
# Test depth processing
colcon test --packages-select autonomy_slam --ctest-args tests/test_depth_processor

# Test GPS fusion
colcon test --packages-select autonomy_slam --ctest-args tests/test_gps_fusion
```

### Integration Tests
```bash
# Full system integration
colcon test --packages-select autonomy_slam --ctest-args tests/test_integration
```

### Field Testing Checklist
- [ ] Test GPS-denied operation (SLAM only) - 30 min mission
- [ ] Test GPS recovery (simulate outage, verify recovery)
- [ ] Test loop closure detection with known path
- [ ] Test long-duration mission (1+ hour) for memory leaks
- [ ] Test in dusty conditions (simulate with plastic particles)
- [ ] Test parameter tuning for local environment
- [ ] Validate with ground truth trajectory

## Competition Preparation

### Pre-Competition Checklist
- [ ] Run full system for 1+ hour, verify no crashes
- [ ] Calibrate camera fresh before each competition
- [ ] Save baseline calibration data as backup
- [ ] Test with actual competition arena if possible
- [ ] Verify all nodes start cleanly with `ros2 launch`
- [ ] Monitor memory usage during mission (should stay <500MB)
- [ ] Test GPS initialization in competition location
- [ ] Validate loop closure works in expected terrain
- [ ] Create emergency shutdown procedure (systemctl stop)

### Recovery Procedures
If SLAM fails during mission:
1. SLAM system automatically switches to GPS-only mode
2. Navigation continues with lower accuracy but maintained reliability
3. Monitor `/slam/system/status` and `/slam/fusion/status` for updates
4. Can manually recalibrate if needed: `ros2 srv call /rtabmap/reset_odom`

## References

- **RTAB-Map**: https://github.com/introlab/rtabmap_ros
- **ROS 2 Documentation**: https://docs.ros.org/en/humble/
- **Extended Kalman Filter**: Standard EKF algorithm implementation
- **Desert SLAM Challenges**: RGB-D depth noise, low texture, dust

## Contributing

When modifying this SLAM system:
1. Update both the code and configuration documentation
2. Test extensively in varied conditions
3. Measure performance impact (CPU, memory, accuracy)
4. Document any parameter changes with rationale
5. Create unit tests for new filtering algorithms
6. Validate with ground truth data when possible

## License

MIT License - See LICENSE file

