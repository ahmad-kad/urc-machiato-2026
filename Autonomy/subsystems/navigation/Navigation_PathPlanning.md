# Navigation & Path Planning Track - University Rover Challenge 2026

## Introduction
The Navigation & Path Planning subsystem is responsible for autonomous rover movement to reach specified targets while avoiding obstacles and managing terrain traversal. This is the core mobility system that enables the rover to execute the Autonomous Navigation Mission, covering 7 targets across up to 2km of varied desert terrain within 30 minutes.

## Domain Information
**Mission Context**: The navigation system must handle three types of targets with different precision requirements:
- GNSS-only locations (3m accuracy requirement)
- AR-tagged posts (2m accuracy requirement)
- Ground objects in boulder fields (variable accuracy with obstacle avoidance)

**Environmental Challenges**:
- Desert terrain: sand, gravel, rocks, boulders, slopes
- GPS signal variations and potential interference
- Real-time performance requirements (30-minute mission limit)
- Integration with computer vision for final approach verification

**Key Performance Metrics**:
- Position accuracy: 2-3 meter precision depending on target type
- Time efficiency: Optimize routes across 2km total distance
- Reliability: Handle terrain variations and GPS uncertainties
- Safety: Maintain stable operation on varied terrain

## Pre-Requirements
Before starting development, ensure the following are available:

### Hardware Prerequisites
- GNSS receiver system (differential GNSS capable)
- IMU (Inertial Measurement Unit) for motion sensing
- Wheel encoders or odometry sensors
- Drive motor controllers with velocity feedback
- Computing platform capable of real-time processing

### Software Prerequisites
- ROS (Robot Operating System) or similar robotics framework
- Basic drive system integration and control
- Sensor data acquisition and timestamping
- Communication interfaces with other subsystems

### Team Prerequisites
- Understanding of mobile robot kinematics
- Experience with GPS navigation systems
- Knowledge of path planning algorithms
- Familiarity with control systems and PID controllers

## Technical Approach Possibilities

### A. Global Path Planning Approaches
```
Global Path Planning
├── Search-based Methods
│   ├── A* Algorithm
│   │   ├── Grid-based discretization
│   │   ├── Cost functions for terrain types
│   │   └── Heuristic optimization for desert terrain
│   ├── Dijkstra's Algorithm
│   │   ├── Guaranteed optimal paths
│   │   └── Higher computational cost
│   └── D* Lite (Dynamic A*)
│       ├── Real-time replanning capability
│       └── Incremental path updates
├── Sampling-based Methods
│   ├── Rapidly-exploring Random Trees (RRT)
│   │   ├── Probabilistic exploration
│   │   └── Efficient in high-dimensional spaces
│   ├── RRT*
│   │   ├── Asymptotically optimal
│   │   └── Better path quality than RRT
│   └── Probabilistic Roadmaps (PRM)
│       ├── Pre-computed roadmaps
│       └── Fast online planning
└── Optimization-based Methods
    ├── Gradient descent approaches
    ├── Nonlinear optimization
    └── Model Predictive Control (MPC)
        ├── Predictive horizon planning
        └── Constraint handling for terrain limits
```

### B. Local Path Planning & Obstacle Avoidance
```
Local Planning & Obstacle Avoidance
├── Reactive Methods
│   ├── Potential Fields
│   │   ├── Attractive forces to goals
│   │   ├── Repulsive forces from obstacles
│   │   └── Local minima avoidance techniques
│   ├── Vector Field Histogram (VFH)
│   │   ├── Histogram grid of obstacle directions
│   │   └── Steering angle selection
│   └── Dynamic Window Approach (DWA)
│       ├── Velocity space sampling
│       ├── Collision prediction
│       └── Terrain adaptability
├── Behavior-based Systems
│   ├── Motor schemas
│   ├── Subsumption architecture
│   └── Hierarchical behavior coordination
└── Sensor Integration
    ├── Multi-sensor fusion
    │   ├── Lidar point cloud processing
    │   ├── Stereo vision depth maps
    │   └── Radar obstacle detection
    └── Terrain classification
        ├── Sand vs rock differentiation
        ├── Slope analysis
        └── Traversability assessment
```

### C. Terrain-Adaptive Navigation
```
Terrain-Adaptive Navigation
├── Terrain Classification
│   ├── Visual terrain assessment
│   │   ├── Color-based classification
│   │   ├── Texture analysis
│   │   └── Supervised learning approaches
│   ├── Geometric terrain analysis
│   │   ├── Slope computation
│   │   ├── Roughness metrics
│   │   └── Obstacle size estimation
│   └── Multi-modal fusion
│       ├── Vision + Lidar integration
│       └── Sensor redundancy for reliability
├── Adaptive Control Strategies
│   ├── Terrain-dependent speed control
│   ├── Wheel torque distribution
│   └── Suspension adjustment (if available)
└── Learning-based Approaches
    ├── Reinforcement learning for terrain traversal
    ├── Imitation learning from human drivers
    └── Adaptive parameter tuning
```

### D. GNSS Integration and Precision Navigation
```
GNSS Integration
├── Standard GPS/GNSS
│   ├── Single-point positioning
│   ├── Real-time kinematic (RTK) GPS
│   └── Post-processing kinematic (PPK)
├── Differential GNSS
│   ├── Base station setup
│   ├── Real-time corrections
│   └── Accuracy validation
├── Sensor Fusion Approaches
│   ├── Extended Kalman Filter (EKF)
│   ├── Unscented Kalman Filter (UKF)
│   ├── Particle filters for robust estimation
│   └── GPS-denied navigation backup
└── Precision Enhancement
    ├── Multi-constellation GNSS
    ├── Inertial navigation integration
    └── Map-based corrections
```

### E. Mission-Specific Navigation Strategies
```
Mission-Specific Strategies
├── Target Approach Sequences
│   ├── GNSS-only targeting
│   │   ├── Dead reckoning backup
│   │   ├── Accuracy monitoring
│   │   └── Position uncertainty handling
│   ├── AR tag assisted navigation
│   │   ├── GNSS to visual handoff
│   │   ├── Marker pose estimation
│   │   └── Precision landing
│   └── Object search patterns
│       ├── Spiral search algorithms
│       ├── Grid-based systematic search
│       └── Information-driven exploration
├── Abort and Recovery
│   ├── Autonomous return to previous waypoints
│   ├── Safe stopping procedures
│   └── Mission continuation strategies
└── Performance Optimization
    ├── Time-to-target minimization
    ├── Energy-efficient path planning
    └── Reliability-focused routing
```

## Recommended Development Approach
1. **Phase 1**: Implement basic GNSS waypoint navigation with simple obstacle avoidance
2. **Phase 2**: Add terrain classification and adaptive speed control
3. **Phase 3**: Integrate differential GNSS and precision approaches
4. **Phase 4**: Implement full mission autonomy with abort/recovery capabilities

## Integration Points
- **SLAM**: Provides map information for path planning
- **Computer Vision**: Assists with precision final approaches
- **State Management**: Handles mode switching and mission progress
- **Drive Systems**: Executes planned trajectories

## Testing Strategy
- **Simulation**: Gazebo or similar with terrain models
- **Hardware-in-the-loop**: Test with actual rover hardware
- **Field testing**: Progressive complexity from flat terrain to full desert conditions
- **Integration testing**: Full autonomous missions with all subsystems
