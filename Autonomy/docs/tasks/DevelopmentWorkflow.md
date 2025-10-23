# 🔄 Development Workflow Guide

## Overview

This guide explains the **two-workspace development system** used in the URC 2026 Autonomy project. Understanding this workflow is crucial for efficient development.

---

## 🏗️ **Two-Workspace Architecture**

### **Why Two Workspaces?**

The project uses **separation of concerns**:
- **`Autonomy/code/`** = **Development workspace** (where you write code)
- **`Autonomy/ros2_ws/`** = **ROS2 workspace** (where ROS2 builds and runs code)

This follows **professional ROS2 development practices** used by companies like Boston Dynamics and Open Robotics.

### **Visual Architecture**

```
┌─────────────────────────────────────┐
│         Your Computer                │
├─────────────────────────────────────┤
│  📁 Autonomy/                       │
│  ├── 📁 code/          ← EDIT HERE   │ ← Your IDE workspace
│  │   ├── navigation/                │
│  │   ├── slam/                      │
│  │   └── computer_vision/           │
│  │                                  │
│  ├── 🔗 ros2_ws/       ← BUILD HERE  │ ← ROS2 build system
│  │   ├── src/ (symlinks to code/)   │
│  │   ├── build/                     │
│  │   ├── install/                   │
│  │   └── log/                       │
│  │                                  │
│  └── 📁 docs/                       │
└─────────────────────────────────────┘
```

---

## 🚀 **Daily Development Workflow**

### **Step 1: Edit Code (Development Workspace)**

```bash
# Always start here - your familiar workspace
cd Autonomy/code/navigation

# Open TODO to see what to implement
cat TODO.md

# Edit code in your IDE
code autonomy_navigation/navigation_node.py

# Make changes, test logic, etc.
# This is your creative workspace!
```

### **Step 2: Build Code (ROS2 Workspace)**

```bash
# Switch to ROS2 workspace for building
cd Autonomy/ros2_ws

# Build your package (or all packages)
colcon build --packages-select autonomy_navigation
# OR: colcon build  # Build everything

# Source the environment
source install/setup.bash
```

### **Step 3: Run & Test (ROS2 Workspace)**

```bash
# Still in ros2_ws/ directory
source install/setup.bash

# Run your node
ros2 run autonomy_navigation navigation_node

# Check ROS2 system
ros2 node list
ros2 topic list
ros2 service list
```

### **Step 4: Iterate (Back to Development)**

```bash
# Go back to edit more code
cd ../code/navigation
code autonomy_navigation/navigation_node.py

# Repeat the cycle: Edit → Build → Run → Test
```

---

## 📋 **Complete Workflow Examples**

### **Example 1: Navigation Development**

```bash
# 1. Edit navigation code
cd Autonomy/code/navigation
code autonomy_navigation/navigation_node.py

# 2. Build navigation package
cd ../../ros2_ws
colcon build --packages-select autonomy_navigation

# 3. Run navigation node
source install/setup.bash
ros2 run autonomy_navigation navigation_node

# 4. Monitor navigation topics
ros2 topic echo /navigation/status
```

### **Example 2: Multi-Package Development**

```bash
# Edit multiple subsystems
cd Autonomy/code/state_management
code autonomy_state_management/state_management_node.py

cd ../navigation
code autonomy_navigation/navigation_node.py

# Build all changed packages
cd ../../ros2_ws
colcon build

# Run integrated system
source install/setup.bash
ros2 launch autonomy_system system_integration.launch.py
```

### **Example 3: Interface Development**

```bash
# Edit custom message types
cd Autonomy/ros2_ws/src/autonomy_interfaces/msg
code NavigationStatus.msg

# Rebuild interfaces (required when changing .msg/.srv/.action)
cd ../../../../  # Back to ros2_ws
colcon build --packages-select autonomy_interfaces

# Then rebuild dependent packages
colcon build --packages-select autonomy_navigation
```

---

## 🔧 **Common Commands Reference**

### **Building**
```bash
# Build specific package
colcon build --packages-select autonomy_navigation

# Build all packages
colcon build

# Clean build (rebuild from scratch)
rm -rf build/ install/ log/
colcon build
```

### **Running**
```bash
# Source environment (always do this first)
source install/setup.bash

# Run a node
ros2 run package_name executable_name

# Launch multiple nodes
ros2 launch package_name launch_file.py

# Run with parameters
ros2 run autonomy_navigation navigation_node --ros-args -p update_rate:=5.0
```

### **Debugging**
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list
ros2 topic info /topic_name
ros2 topic echo /topic_name

# Check services
ros2 service list
ros2 service call /service_name package/srv/Type "{}"

# Check parameters
ros2 param list /node_name
```

---

## 📁 **File Organization Deep Dive**

### **Development Workspace (`Autonomy/code/`)**

```
code/
├── navigation/                    # Subsystem directory
│   ├── autonomy_navigation/       # Python package
│   │   ├── navigation_node.py     # Main node
│   │   ├── gnss_processor.py      # GNSS handling
│   │   ├── path_planner.py        # Path planning
│   │   └── motion_controller.py   # Motion control
│   ├── package.xml               # ROS2 package metadata
│   ├── setup.py                  # Python package setup
│   ├── setup.cfg                 # Build configuration
│   ├── CMakeLists.txt            # Build instructions
│   └── TODO.md                   # Development tasks
├── slam/                         # Another subsystem
└── computer_vision/              # Another subsystem
```

**What goes here:**
- ✅ Source code (`.py` files)
- ✅ Package configuration (`package.xml`, `setup.py`)
- ✅ Documentation (`TODO.md`, `README.md`)
- ✅ Tests (`test/` directory)
- ❌ Build artifacts (generated by ROS2)
- ❌ Dependencies (managed by ROS2)

### **ROS2 Workspace (`Autonomy/ros2_ws/`)**

```
ros2_ws/
├── src/                          # Source packages (symlinks)
│   ├── autonomy_navigation -> ../../code/navigation
│   ├── autonomy_interfaces/      # Custom messages
│   └── [other symlinks]
├── build/                        # Build artifacts (generated)
│   ├── autonomy_navigation/
│   └── autonomy_interfaces/
├── install/                      # Installed packages (generated)
│   ├── lib/python3.10/site-packages/
│   └── share/
└── log/                         # Build logs (generated)
```

**What goes here:**
- ✅ Symlinks to your development code
- ✅ Custom interfaces (`.msg`, `.srv`, `.action`)
- ✅ Build artifacts (auto-generated)
- ✅ Installed packages (auto-generated)
- ❌ Your source code (lives in `code/`)

---

## 🐛 **Troubleshooting Common Issues**

### **"Package not found" Error**
```bash
# Problem: ROS2 can't find your package
ros2 run autonomy_navigation navigation_node
# Error: package 'autonomy_navigation' not found

# Solution: Build the package first
cd Autonomy/ros2_ws
colcon build --packages-select autonomy_navigation
source install/setup.bash
```

### **"Module not found" Error**
```bash
# Problem: Python can't import your modules
from autonomy_navigation.gnss_processor import GNSSProcessor
# Error: ModuleNotFoundError

# Solution: Build and source environment
cd Autonomy/ros2_ws
colcon build
source install/setup.bash
```

### **Changes Not Taking Effect**
```bash
# Problem: You edited code but changes don't appear

# Solution: Rebuild after editing
cd Autonomy/ros2_ws
colcon build --packages-select your_package
source install/setup.bash
```

---

## 🎯 **Best Practices**

### **1. Always Source Environment**
```bash
# Do this every time you open a new terminal
cd Autonomy/ros2_ws
source install/setup.bash
```

### **2. Build Incrementally**
```bash
# Build only what changed (faster)
colcon build --packages-select autonomy_navigation

# Build everything when needed
colcon build
```

### **3. Use Symlinks Effectively**
```bash
# Edit in code/, build in ros2_ws/
cd Autonomy/code/navigation          # Edit here
cd ../../ros2_ws                     # Build here
```

### **4. Version Control Strategy**
```bash
# Track your source code
git add Autonomy/code/
git add Autonomy/docs/
git add Autonomy/ros2_ws/src/autonomy_interfaces/

# Don't track build artifacts
echo "Autonomy/ros2_ws/build/" >> .gitignore
echo "Autonomy/ros2_ws/install/" >> .gitignore
echo "Autonomy/ros2_ws/log/" >> .gitignore
```

---

## 🚀 **Advanced Workflows**

### **Working with Multiple Developers**
```bash
# Each developer works in their own ros2_ws/
# But shares the same code/ via symlinks
cd Autonomy/code/navigation
git pull  # Get latest changes
cd ../../ros2_ws
colcon build
```

### **CI/CD Integration**
```bash
# GitHub Actions builds from ros2_ws/
- name: Build ROS2 packages
  run: |
    cd Autonomy/ros2_ws
    colcon build
```

### **Testing Workflow**
```bash
# Run tests from ros2_ws/
cd Autonomy/ros2_ws
source install/setup.bash
colcon test --packages-select autonomy_navigation
colcon test-result --verbose
```

---

## 📚 **Additional Resources**

- **ROS2 Documentation**: https://docs.ros.org/en/humble/
- **Colcon Tutorial**: https://colcon.readthedocs.io/
- **Package Creation**: https://docs.ros.org/en/humble/Tutorials/Beginner-Client-Libraries/Creating-Your-First-ROS2-Package.html

**Remember: Edit in `code/`, build in `ros2_ws/`! This workflow becomes second nature after a few days.** 🎯
