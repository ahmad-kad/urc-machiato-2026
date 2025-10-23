# 🎯 URC 2026 Autonomy - Onboarding Checklist

**New to the team?** Follow this 15-minute checklist to get started.

## ✅ Phase 1: Environment Setup (5 min)

- [ ] **Install Docker Desktop**
  - Mac: Download from docker.com
  - Windows: Enable WSL2, install Docker Desktop
  - Linux: `sudo apt install docker.io`

- [ ] **Clone Repository**
  ```bash
  git clone [repo-url]
  cd Autonomy
  ```

- [ ] **Start Docker Environment**
  ```bash
  ./development/docker/scripts/docker-dev.sh build
  ./development/docker/scripts/docker-dev.sh start
  ./development/docker/scripts/docker-dev.sh enter
  ```

- [ ] **Verify Setup**
  ```bash
  ros2 --version  # Should show ROS 2 Humble
  python3 --version  # Should be 3.10+
  ```

## 🎯 Phase 2: Choose Your Role (2 min)

**Pick based on your interests:**

| **I Like...** | **Join This Team** | **You'll Build** |
|---|---|---|
| 🤖 **Driving robots** | Navigation | Path planning, obstacle avoidance |
| 🗺️ **Maps & GPS** | SLAM | 3D mapping, localization |
| 📷 **Computer vision** | Computer Vision | Object detection, ArUco markers |
| ⌨️ **Hardware control** | Autonomous Typing | Robotic manipulation |
| 🎛️ **System design** | State Management | Mission coordination |
| 🔴 **LEDs & status** | LED Status | Visual indicators |

**Not sure?** Start with **Navigation** - it's the most beginner-friendly!

## 💻 Phase 3: Your First Code (8 min)

### Step 1: Explore Your Subsystem
```bash
# Go to your subsystem
cd code/[your-subsystem]

# Read what to build
cat TODO.md

# See existing code
ls src/
```

### Step 2: Copy a Template
```bash
# Copy the ROS 2 node template
cp ../templates/ros2_node_template.py src/my_first_node.py

# Edit it
code src/my_first_node.py
```

### Step 3: Make It Yours
**Change these in the template:**
- `NODE_NAME = "your_node_name"`
- `class YourNodeName(Node):`
- Add your logic in `control_loop()`

**Example for Navigation:**
```python
def control_loop(self):
    """Drive forward, avoid obstacles."""
    if self.obstacle_detected:
        self.cmd_vel_pub.publish(Twist())  # Stop
    else:
        msg = Twist()
        msg.linear.x = 0.5  # Drive forward
        self.cmd_vel_pub.publish(msg)
```

### Step 4: Test Your Code
```bash
# Run it
python3 src/my_first_node.py

# In another terminal, check ROS topics
ros2 topic list
ros2 topic echo /cmd_vel
```

## 🧪 Phase 4: Integration & Testing

### Test with Simulation
```bash
# Launch Gazebo (in Docker)
ros2 launch mars_simulation simulation.launch.xml

# Your node should now control the robot!
```

### Test with Real Hardware
```bash
# Deploy to Raspberry Pi
# Follow deployment guides
```

## 📞 Get Help Fast

### **"My code doesn't work!"**
```bash
# Check for syntax errors
python3 -m py_compile src/your_file.py

# Run with debug logging
python3 src/your_file.py  # Look for error messages
```

### **"Docker isn't working!"**
```bash
# Check Docker status
docker ps

# Restart everything
./development/docker/scripts/docker-dev.sh restart
```

### **"I need help!"**
1. **Read the docs**: `./scripts/navigate.sh docs`
2. **Check examples**: Look at `code/templates/`
3. **Ask the team**: Slack or team meeting
4. **File an issue**: GitHub repository

## 🎯 Success Checklist

**By end of Week 1:**
- [ ] Environment setup complete
- [ ] Basic ROS 2 node running
- [ ] Publishing/subscribing to topics
- [ ] Tested in simulation

**By end of Week 2:**
- [ ] Integration with other subsystems
- [ ] Real sensor data processing
- [ ] Basic autonomous behavior

**By end of Week 4:**
- [ ] Full subsystem implementation
- [ ] Hardware testing complete
- [ ] Ready for competition

## 🚀 You're Officially Onboarded!

**Welcome to the URC 2026 Autonomy Team!** 🤖

**Next steps:**
1. Join the team Slack channel
2. Attend the next team meeting
3. Start contributing to your subsystem
4. Have fun building autonomous robots!

---

**Questions?** This guide is living documentation. Suggest improvements!
