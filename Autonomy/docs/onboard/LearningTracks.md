# 📚 Learning Tracks - Your Path to Autonomy Mastery

**Not sure where to start?** This guide provides structured learning paths tailored to your background and goals. Each track builds progressively, with clear milestones and recommended reading order.

## 🎯 Quick Track Selector

| **Your Background** | **Your Goal** | **Recommended Track** |
|---|---|---|
| **New to Robotics** | Learn autonomy basics | 🚀 **Beginner Track** |
| **Some Programming** | Build one subsystem | 🔧 **Subsystem Specialist** |
| **ROS Experience** | Lead team development | 👥 **Team Lead Track** |
| **Advanced Robotics** | Architect full systems | 🏗️ **System Architect** |
| **Competition Focus** | Win URC 2026 | 🏆 **Competition Champion** |

---

## 🚀 BEGINNER TRACK: "From Zero to Rover"

**Goal:** Build confidence and basic autonomy knowledge in 4 weeks
**Prerequisites:** Basic Python programming
**Outcome:** Contribute to one subsystem with supervision

### **Week 1: Foundation & Setup**
```
🎯 Milestones: Environment working, basic ROS 2 understanding
📖 Reading Order:
├── QUICKSTART.md (30 min) - Get started immediately
├── ONBOARDING.md (45 min) - Structured setup checklist
├── docs/README.md (20 min) - Overview of entire system
├── development/docker/QUICKSTART.md (20 min) - Docker environment
└── ./scripts/navigate.sh setup - Interactive setup help

💻 Hands-on Tasks:
├── Install Docker Desktop
├── Clone repository: git clone [repo-url]
├── Run: ./scripts/navigate.sh quickstart
└── Verify: docker ps shows running containers
```

### **Week 2: ROS 2 & Basic Programming**
```
🎯 Milestones: Publish/subscribe to ROS 2 topics, basic node structure
📖 Reading Order:
├── code/templates/README.md (60 min) - Code style & templates
├── docs/reference/LibrariesGuide.md (45 min) - ROS 2 basics
├── docs/reference/SensorGuide.md (30 min) - What sensors do
├── Choose subsystem docs (30 min each):
│   ├── subsystems/navigation/Navigation_PathPlanning.md
│   ├── subsystems/slam/SLAM.md
│   ├── subsystems/computer_vision/ComputerVision_ObjectClassification.md
│   ├── subsystems/autonomous_typing/AutonomousTyping.md
│   ├── subsystems/state_management/StateManagement_ModeControl.md
│   └── subsystems/led_status/LED_StatusSignaling.md

💻 Hands-on Tasks:
├── Copy template: cp code/templates/ros2_node_template.py code/[subsystem]/src/my_node.py
├── Edit template to add basic logging
├── Run in Docker: python3 code/[subsystem]/src/my_node.py
└── Verify ROS topics: ros2 topic list
```

### **Week 3: Your First Real Contribution**
```
🎯 Milestones: Working code that interacts with ROS 2, basic testing
📖 Reading Order:
├── CODE_STYLE.md (45 min) - Clean code principles
├── docs/guides/DevelopmentPipeline.md (30 min) - Development workflow
├── docs/reference/CalibrationGuide.md (20 min) - Sensor setup
└── Subsystem implementation guide (45 min)

💻 Hands-on Tasks:
├── Implement one TODO item from code/[subsystem]/[subsystem]_TODO.md
├── Add proper error handling and logging
├── Test with: python3 -m pytest code/[subsystem]/test/ -v
├── Run code quality check: ./scripts/check_code.sh
└── Create simple ROS 2 launch file
```

### **Week 4: Integration & Simulation**
```
🎯 Milestones: Code works with other subsystems, basic simulation
📖 Reading Order:
├── development/SimulationSetup.md (30 min) - Gazebo basics
├── docs/reference/HybridDevelopmentGuide.md (30 min) - Docker + hardware
├── docs/reference/DockerSuitabilityAnalysis.md (20 min) - When Docker works
└── docs/reference/EnvironmentalChallenges.md (25 min) - Real-world considerations

💻 Hands-on Tasks:
├── Launch simulation: ros2 launch mars_simulation simulation.launch.xml
├── Test integration with other team members' code
├── Add basic unit tests for your functions
├── Document your code with docstrings
└── Present your work to the team
```

**🎉 Week 4 Celebration:** You're now a contributing autonomy developer!

---

## 🔧 SUBSYSTEM SPECIALIST TRACKS

**Goal:** Become an expert in one specific area
**Prerequisites:** Completed Beginner Track or equivalent ROS 2 experience
**Duration:** 6-8 weeks per track

### **🧭 Navigation Specialist Path**
```
🎯 Focus: Path planning, obstacle avoidance, motion control

📖 Reading Sequence:
├── subsystems/navigation/Navigation_PathPlanning.md (deep read)
├── subsystems/navigation/NavigationGuide.md (implementation details)
├── docs/reference/LibrariesGuide.md (control algorithms section)
├── docs/reference/SensorGuide.md (GPS, IMU, encoders sections)
├── docs/reference/CalibrationGuide.md (wheel odometry calibration)

🔧 Advanced Topics:
├── A* vs RRT* algorithms
├── Kalman filtering for localization
├── PID control tuning
├── Terrain classification
└── Multi-robot coordination

💻 Projects:
├── Implement A* path planner
├── Add terrain analysis
├── Integrate GPS waypoints
├── Real-time obstacle avoidance
└── Performance optimization
```

### **🗺️ SLAM Specialist Path**
```
🎯 Focus: Mapping, localization, sensor fusion

📖 Reading Sequence:
├── subsystems/slam/SLAM.md (deep read)
├── subsystems/slam/SLAMGuide.md (implementation details)
├── docs/reference/LibrariesGuide.md (PCL, optimization sections)
├── docs/reference/SensorGuide.md (LIDAR, IMU, camera sections)
├── docs/reference/CalibrationGuide.md (camera-IMU calibration)

🔧 Advanced Topics:
├── Visual inertial odometry
├── Graph-based SLAM
├── Multi-sensor fusion
├── Loop closure detection
├── Map compression techniques

💻 Projects:
├── Implement feature extraction
├── Build occupancy grid mapping
├── Add loop closure detection
├── Sensor fusion pipeline
└── Real-time performance optimization
```

### **👁️ Computer Vision Specialist Path**
```
🎯 Focus: Object detection, tracking, visual servoing

📖 Reading Sequence:
├── subsystems/computer_vision/ComputerVision_ObjectClassification.md
├── subsystems/computer_vision/ComputerVisionGuide.md
├── docs/reference/LibrariesGuide.md (OpenCV, PyTorch sections)
├── docs/reference/SensorGuide.md (RGB-D cameras section)
├── docs/reference/CalibrationGuide.md (camera calibration)

🔧 Advanced Topics:
├── Deep learning model optimization
├── Real-time object tracking
├── Visual servoing algorithms
├── Multi-camera calibration
├── Edge computing for vision

💻 Projects:
├── Train object detection model
├── Implement ArUco marker tracking
├── Add visual servoing
├── Multi-camera fusion
└── Performance optimization for embedded hardware
```

### **⌨️ Autonomous Typing Specialist Path**
```
🎯 Focus: Robotic manipulation, precision control

📖 Reading Sequence:
├── subsystems/autonomous_typing/AutonomousTyping.md
├── subsystems/autonomous_typing/AutonomousTypingGuide.md
├── docs/reference/LibrariesGuide.md (control algorithms)
├── docs/reference/SensorGuide.md (force/torque sensors)
├── docs/reference/CalibrationGuide.md (hand-eye calibration)

🔧 Advanced Topics:
├── Inverse kinematics
├── Force control algorithms
├── Computer vision for manipulation
├── Error detection and recovery
├── Human-robot interaction safety

💻 Projects:
├── Implement keyboard detection
├── Add trajectory planning
├── Force control for typing
├── Error recovery algorithms
└── Safety interlocks
```

### **🎛️ State Management Specialist Path**
```
🎯 Focus: System coordination, fault tolerance, mission planning

📖 Reading Sequence:
├── subsystems/state_management/StateManagement_ModeControl.md
├── subsystems/state_management/StateManagementGuide.md
├── docs/reference/DistributedSystemsGuide.md (core concepts)
├── docs/reference/LibrariesGuide.md (communication patterns)
├── docs/reference/EnvironmentalChallenges.md (robustness)

🔧 Advanced Topics:
├── Finite state machines
├── Behavior trees
├── Fault detection and recovery
├── Multi-agent coordination
├── Real-time system design

💻 Projects:
├── Implement mode switching logic
├── Add fault detection
├── Mission state management
├── Inter-subsystem communication
└── Emergency stop coordination
```

### **🔴 LED Status Specialist Path**
```
🎯 Focus: Visual communication, embedded systems

📖 Reading Sequence:
├── subsystems/led_status/LED_StatusSignaling.md
├── subsystems/led_status/LED_StatusGuide.md
├── docs/reference/LibrariesGuide.md (GPIO, PWM sections)
├── docs/reference/SensorGuide.md (GPIO interfaces)
├── docs/reference/DistributedSystemsGuide.md (distributed coordination)

🔧 Advanced Topics:
├── PWM control algorithms
├── Color theory for signaling
├── Low-power embedded design
├── Real-time performance
├── Multi-device synchronization

💻 Projects:
├── Implement status LED patterns
├── Add PWM brightness control
├── Color-coded status system
├── Multi-LED array coordination
└── Power-efficient operation
```

---

## 👥 TEAM LEAD TRACK: "Leading Autonomy Development"

**Goal:** Lead subsystem teams and coordinate system integration
**Prerequisites:** Completed at least one Subsystem Specialist track
**Duration:** 8-12 weeks

### **Phase 1: Deep System Understanding**
```
📖 Reading Focus:
├── docs/guides/ProjectStatus.md (complete project overview)
├── docs/guides/DevelopmentPipeline.md (full development process)
├── docs/reference/DistributedSystemsGuide.md (architecture patterns)
├── All subsystem technical requirements
└── docs/reference/EnvironmentalChallenges.md (competition realities)

💻 Leadership Tasks:
├── Understand all 6 subsystems' requirements
├── Review existing code across subsystems
├── Identify integration points and dependencies
└── Create team communication channels
```

### **Phase 2: Integration & Testing**
```
📖 Reading Focus:
├── docs/reference/HybridDevelopmentGuide.md (Docker + hardware)
├── docs/reference/DockerSuitabilityAnalysis.md (system architecture)
├── docs/reference/CalibrationGuide.md (full system calibration)
└── docs/reference/LibrariesGuide.md (integration patterns)

💻 Leadership Tasks:
├── Design system integration architecture
├── Create integration testing framework
├── Coordinate multi-subsystem testing
└── Manage version control and releases
```

### **Phase 3: Production & Deployment**
```
📖 Reading Focus:
├── deployment/RaspberryPiDeployment.md (hardware deployment)
├── deployment/ClientServerArchitecture.md (distributed systems)
├── docs/reference/SensorGuide.md (full sensor integration)
└── docs/reference/DistributedSystemsGuide.md (fault tolerance, monitoring)

💻 Leadership Tasks:
├── Plan hardware procurement and setup
├── Design deployment and update procedures
├── Implement monitoring and diagnostics
└── Prepare competition deployment strategy
```

### **Phase 4: Team Management & Scaling**
```
📖 Reading Focus:
├── docs/reference/DistributedSystemsGuide.md (advanced patterns)
├── docs/reference/LibrariesGuide.md (performance optimization)
├── docs/reference/EnvironmentalChallenges.md (robustness testing)
└── All subsystem implementation guides

💻 Leadership Tasks:
├── Mentor new team members
├── Optimize system performance
├── Plan for team growth and new subsystems
└── Document lessons learned and best practices
```

---

## 🏗️ SYSTEM ARCHITECT TRACK: "Designing Autonomous Systems"

**Goal:** Design and architect complete autonomous robot systems
**Prerequisites:** Multiple subsystem specialties + team leadership experience
**Duration:** Ongoing advanced learning

### **Core Architecture Skills**
```
📖 Essential Reading:
├── docs/reference/DistributedSystemsGuide.md (complete architecture patterns)
├── docs/reference/LibrariesGuide.md (advanced algorithms and tools)
├── docs/reference/SensorGuide.md (sensor fusion and selection)
├── docs/reference/DockerSuitabilityAnalysis.md (system design trade-offs)
└── docs/reference/EnvironmentalChallenges.md (real-world constraints)

🔧 Advanced Topics:
├── System-level optimization
├── Real-time performance analysis
├── Safety-critical system design
├── Multi-robot coordination
└── Long-term system evolution
```

### **Emerging Technologies**
```
📖 Cutting-Edge Topics:
├── Edge AI and embedded ML
├── 5G-enabled robotics
├── Cloud robotics integration
├── Augmented reality interfaces
└── Human-robot collaboration
```

### **Research & Innovation**
```
📖 Research Areas:
├── Novel sensor fusion techniques
├── Learning-based control systems
├── Adaptive autonomy algorithms
├── Energy-aware robotics
└── Robustness in extreme environments
```

---

## 🏆 COMPETITION CHAMPION TRACK: "Winning URC 2026"

**Goal:** Optimize for competition success and real-world performance
**Prerequisites:** System integration experience
**Duration:** 12-16 weeks to competition

### **Phase 1: Competition Analysis**
```
📖 Reading Focus:
├── docs/guides/ProjectStatus.md (competition requirements)
├── UniversityRoverChallenge2026.md (official rules)
├── docs/reference/EnvironmentalChallenges.md (Mojave Desert conditions)
├── docs/reference/SensorGuide.md (competition-legal sensors)
└── docs/reference/CalibrationGuide.md (field calibration procedures)

💻 Competition Tasks:
├── Detailed rule analysis and compliance checking
├── Risk assessment for competition scenarios
├── Required vs. optional mission components
└── Performance benchmarking against requirements
```

### **Phase 2: System Optimization**
```
📖 Reading Focus:
├── docs/reference/LibrariesGuide.md (performance optimization)
├── docs/reference/DistributedSystemsGuide.md (fault tolerance)
├── deployment/RaspberryPiDeployment.md (embedded optimization)
├── docs/reference/HybridDevelopmentGuide.md (development workflow)
└── docs/guides/DevelopmentPipeline.md (production deployment)

💻 Competition Tasks:
├── Performance profiling and bottleneck identification
├── Power consumption optimization
├── Reliability testing under competition conditions
└── Backup system design and implementation
```

### **Phase 3: Integration & Validation**
```
📖 Reading Focus:
├── development/SimulationSetup.md (competition course simulation)
├── docs/reference/DistributedSystemsGuide.md (distributed testing)
├── docs/reference/EnvironmentalChallenges.md (environmental testing)
└── deployment/ClientServerArchitecture.md (field deployment)

💻 Competition Tasks:
├── Full system integration testing
├── Competition course simulation and validation
├── Environmental stress testing (heat, dust, vibration)
└── Complete mission scenario walkthroughs
```

### **Phase 4: Competition Preparation**
```
📖 Reading Focus:
├── docs/reference/EnvironmentalChallenges.md (field operations)
├── deployment/RaspberryPiDeployment.md (field deployment)
├── docs/reference/DistributedSystemsGuide.md (remote monitoring)
└── docs/reference/CalibrationGuide.md (field calibration)

💻 Competition Tasks:
├── Field deployment procedures and checklists
├── Remote monitoring and troubleshooting setup
├── Team communication protocols for competition
└── Contingency plans for various failure scenarios
```

---

## 🛣️ TRACK NAVIGATION TOOLS

### **Interactive Track Selection**
```bash
# Get personalized recommendations
./scripts/navigate.sh pick

# Start with a specific subsystem
./scripts/navigate.sh start navigation

# Check your progress
./scripts/navigate.sh status
```

### **Track Progress Tracking**
```bash
# Create personal progress tracker
touch ~/autonomy_progress.md

# Template for tracking:
# ## My Learning Journey
# ### Week 1: ✅ Setup complete
# ### Week 2: 🔄 Working on ROS 2 basics
# ### Week 3: 📋 TODO: Implement path planner
```

### **Recommended Reading Order for Each Track**
```bash
# Beginner Track - Week by week
./scripts/navigate.sh quickstart    # Week 1
./scripts/navigate.sh style         # Week 2
./scripts/navigate.sh docs          # Week 3
./scripts/navigate.sh deploy        # Week 4

# Specialist Tracks - Deep dive
cat subsystems/[your-track]/*Guide.md  # Implementation details
cat docs/reference/[relevant-guide].md # Technical background
cat code/[your-track]/[your-track]_TODO.md          # What to build
```

### **Track Switching Guidelines**
```
From Beginner → Specialist:
├── Choose 1-2 subsystems you're passionate about
├── Complete their TODO lists
├── Join subsystem-specific discussions
└── Present your work to the team

From Specialist → Team Lead:
├── Understand all subsystems at high level
├── Lead integration of 2+ subsystems
├── Mentor junior team members
└── Coordinate team meetings and standups

From Team Lead → Architect:
├── Design complete system architectures
├── Evaluate new technologies and approaches
├── Plan for future competitions
└── Contribute to robotics research
```

---

## 📈 PROGRESS MEASUREMENT

### **Beginner Track Milestones**
- [ ] Week 1: Docker environment running
- [ ] Week 2: ROS 2 node publishing/subscribing
- [ ] Week 3: Code passes style checks and tests
- [ ] Week 4: Integration with simulation environment

### **Subsystem Specialist Milestones**
- [ ] Basic functionality implemented
- [ ] Unit tests written and passing
- [ ] Integration with other subsystems
- [ ] Performance meets requirements
- [ ] Documentation updated

### **Team Lead Milestones**
- [ ] All subsystems integrated
- [ ] System-level testing complete
- [ ] Deployment procedures documented
- [ ] Team productivity metrics improving

### **Competition Champion Milestones**
- [ ] All mission requirements met
- [ ] System tested in competition-like conditions
- [ ] Backup systems operational
- [ ] Performance exceeds baseline requirements

---

## 🎓 LEARNING RESOURCES BY TRACK

### **Beginner Resources**
- **ROS 2 Tutorials**: https://docs.ros.org/en/humble/Tutorials.html
- **Python for Robotics**: Automate the Boring Stuff + ROS 2
- **Git Fundamentals**: https://learngitbranching.js.org/
- **Docker Basics**: https://docker-curriculum.com/

### **Specialist Resources**
- **Navigation**: https://navigation.ros.org/
- **SLAM**: https://openslam.org/
- **Computer Vision**: https://opencv.org/ + https://pytorch.org/tutorials/
- **Control Systems**: https://www.coursera.org/learn/robotics-motion-planning

### **Advanced Resources**
- **Distributed Systems**: https://www.distributed-systems.net/
- **Real-time Systems**: https://www.realtime-systems.net/
- **Robotics Research**: IEEE Robotics & Automation Society
- **System Architecture**: "Designing Data-Intensive Applications"

**Ready to start your journey? Choose your track and `./scripts/navigate.sh [track-name]` to begin!** 🚀
