# System Charts & Diagrams Overview

This document provides a visual overview of the entire robot system architecture, data flows, and integration processes.

---

## 🗂️ Chart Index

| Chart Type | Location | Description |
|------------|----------|-------------|
| **System Architecture** | [SystemArchitecture.md](SystemArchitecture.md) | Complete robot component overview |
| **Team Integration** | [../onboard/TeamIntegration.md](../onboard/TeamIntegration.md) | How teams work together |
| **Development Workflow** | [../onboard/TeamHandbook.md](../onboard/TeamHandbook.md) | Your coding workflow |
| **Interface Contracts** | [InterfaceContract.md](InterfaceContract.md) | ROS2 communication specifications |

---

## 🎯 Key System Diagrams

### **Complete Robot Architecture**
```mermaid
flowchart TD
    subgraph "👤 User Layer"
        UI[🎮 Control Pi<br/>User Interface]
    end

    subgraph "🧠 Coordination Layer"
        COORD[🎮 Control Pi<br/>System Coordination]
    end

    subgraph "🤖 Autonomy Layer"
        SLAM[🤖 Autonomy Pi<br/>SLAM & Mapping]
        VISION[🤖 Autonomy Pi<br/>Computer Vision]
        NAV[🤖 Autonomy Pi<br/>Path Planning]
        GPS_PROC[🤖 Autonomy Pi<br/>GPS Processing]
    end

    subgraph "⚙️ Hardware Layer"
        WHEELS[🔧 Wheel Controller<br/>Motor Control]
        ARM[🔧 Arm Controller<br/>Joint Control]
        CAMERA[🔧 Camera Controller<br/>Pan-Tilt]
        SENSORS[🔧 Sensor Hub<br/>Data Collection]
    end

    UI --> COORD
    COORD --> SLAM
    COORD --> VISION
    COORD --> NAV
    COORD --> GPS_PROC

    SLAM --> NAV
    VISION --> NAV
    GPS_PROC --> SLAM

    NAV --> WHEELS
    SLAM --> ARM
    VISION --> CAMERA
    SENSORS --> GPS_PROC

    classDef userInterface fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px
    classDef autonomy fill:#e3f2fd,stroke:#0277bd,stroke-width:2px
    classDef hardware fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px

    class UI,COORD userInterface
    class SLAM,VISION,NAV,GPS_PROC autonomy
    class WHEELS,ARM,CAMERA,SENSORS hardware
```

### **Data Flow Overview**
```mermaid
flowchart LR
    subgraph "Input"
        USER[👤 User Commands]
        SENSORS_RAW[📡 Raw Sensors<br/>GPS, IMU, Cameras]
    end

    subgraph "Processing"
        CONTROL[🎮 Control Pi<br/>Coordination]
        AUTONOMY[🤖 Autonomy Pi<br/>Algorithms]
        MICRO[🔧 Microcontrollers<br/>Hardware Control]
    end

    subgraph "Output"
        MOTORS[🏃 Motor Actions]
        DISPLAY[📺 User Display]
        LOGGING[📊 Data Logging]
    end

    USER --> CONTROL
    SENSORS_RAW --> AUTONOMY

    CONTROL --> AUTONOMY
    AUTONOMY --> MICRO
    MICRO --> AUTONOMY

    MICRO --> MOTORS
    AUTONOMY --> CONTROL
    CONTROL --> DISPLAY
    CONTROL --> LOGGING

    classDef input fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px
    classDef output fill:#ffebee,stroke:#c62828,stroke-width:2px

    class USER,SENSORS_RAW input
    class MOTORS,DISPLAY,LOGGING output
```

---

## 🔄 Operational Flow Diagrams

### **Mission Execution Timeline**
```mermaid
gantt
    title Competition Mission Timeline
    dateFormat  HH:mm:ss
    axisFormat %H:%M

    section Mission Setup
    System Boot           :done, boot, 08:00:00, 10m
    GPS Initialization    :done, gps, after boot, 5m
    System Calibration    :done, cal, after gps, 10m
    Pre-mission Checks    :done, check, after cal, 5m

    section Autonomous Navigation
    Waypoint 1 Navigation :done, wp1, after check, 15m
    Waypoint 2 Navigation :done, wp2, after wp1, 12m
    Waypoint 3 Navigation :done, wp3, after wp2, 18m
    Waypoint 4 Navigation :done, wp4, after wp3, 10m

    section Equipment Servicing
    Approach Equipment    :done, approach, after wp4, 8m
    Robotic Arm Setup     :done, arm_setup, after approach, 5m
    Autonomous Typing     :done, typing, after arm_setup, 15m
    Verification          :done, verify, after typing, 3m

    section Return Navigation
    Return Path Planning  :done, return_plan, after verify, 2m
    Return Navigation     :done, return_nav, after return_plan, 20m
    Mission Complete      :done, complete, after return_nav, 1m
```

### **System State Machine**
```mermaid
stateDiagram-v2
    [*] --> SystemOff

    SystemOff --> Booting: Power On
    Booting --> Initializing: Boot Complete
    Initializing --> Ready: Initialization Complete

    Ready --> PreMission: Mission Configured
    PreMission --> Autonomous: Start Mission

    Autonomous --> Navigating: Path Planning Active
    Autonomous --> Servicing: Equipment Reached
    Autonomous --> EmergencyStop: Critical Failure

    Navigating --> Autonomous: Waypoint Reached
    Servicing --> Autonomous: Task Complete

    Autonomous --> ManualOverride: User Intervention
    Autonomous --> Completed: Mission Success

    ManualOverride --> Autonomous: Resume Autonomy
    ManualOverride --> EmergencyStop: Safety Trigger

    EmergencyStop --> SafeState: Emergency Handled
    SafeState --> Ready: System Reset
    SafeState --> SystemOff: Power Cycle Required

    Completed --> Ready: New Mission
    Completed --> SystemOff: Competition End

    note right of Autonomous : Primary autonomous operation\nwith parallel subsystems
    note right of EmergencyStop : All systems stop immediately\nsafety first
```

---

## 👥 Team Integration Diagrams

### **Team Communication Flow**
```mermaid
flowchart TD
    A[🤖 Autonomy Team<br/>ROS2 Interfaces] --> C[🔗 Integration<br/>Testing]
    B[🎮 Control Team<br/>User Interface] --> C
    D[🔧 Hardware Team<br/>Motor Control] --> C

    C --> E[🚨 Issues Found?]
    E -->|Yes| F[🤝 Team Meeting<br/>Problem Solving]
    E -->|No| G[✅ Integration<br/>Complete]

    F --> H[📋 Action Items<br/>Assigned]
    H --> I[👥 Individual Teams<br/>Fix Issues]
    I --> C

    classDef autonomy fill:#e1f5fe,stroke:#0277bd,stroke-width:2px
    classDef control fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef hardware fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px
    classDef success fill:#c8e6c9,stroke:#2e7d32,stroke-width:2px
    classDef meeting fill:#fff3e0,stroke:#f57c00,stroke-width:2px

    class A autonomy
    class B control
    class D hardware
    class G success
    class F meeting
```

### **Development Dependencies**
```mermaid
flowchart TD
    subgraph "🎯 Competition Ready"
        COMPETE[Full System<br/>Integration]
    end

    subgraph "🔗 Integration Phase"
        REAL_HW[Real Hardware<br/>Testing]
        PERF_TEST[Performance<br/>Validation]
        STRESS_TEST[Stress<br/>Testing]
    end

    subgraph "🧪 Testing Phase"
        MOCK_TEST[Mock Component<br/>Testing]
        UNIT_TEST[Unit Tests<br/>Per Team]
        INTERFACE_TEST[Interface<br/>Validation]
    end

    subgraph "🏗️ Development Phase"
        AUTO_DEV[Autonomy<br/>Algorithms]
        UI_DEV[User Interface<br/>Development]
        HW_DEV[Hardware<br/>Control]
    end

    AUTO_DEV --> UNIT_TEST
    UI_DEV --> UNIT_TEST
    HW_DEV --> UNIT_TEST

    UNIT_TEST --> MOCK_TEST
    MOCK_TEST --> INTERFACE_TEST

    INTERFACE_TEST --> REAL_HW
    REAL_HW --> PERF_TEST
    PERF_TEST --> STRESS_TEST
    STRESS_TEST --> COMPETE

    classDef compete fill:#c8e6c9,stroke:#2e7d32,stroke-width:2px
    classDef autonomy fill:#e1f5fe,stroke:#0277bd,stroke-width:2px
    classDef control fill:#f3e5f5,stroke:#7b1fa2,stroke-width:2px
    classDef hardware fill:#e8f5e8,stroke:#2e7d32,stroke-width:2px

    class COMPETE compete
    class AUTO_DEV autonomy
    class UI_DEV control
    class HW_DEV hardware
```

---

## 🔧 Technical Flow Diagrams

### **ROS2 Communication Architecture**
```mermaid
flowchart TD
    subgraph "ROS2 Topics (Async)"
        T1[/mission_status<br/>String/]
        T2[/navigation_status<br/>NavigationStatus/]
        T3[/cmd_vel<br/>Twist/]
        T4[/emergency_stop<br/>Bool/]
    end

    subgraph "ROS2 Services (Sync)"
        S1[configure_mission<br/>ConfigureMission]
        S2[get_autonomy_status<br/>GetSubsystemStatus]
        S3[reset_autonomy<br/>Trigger]
    end

    subgraph "ROS2 Actions (Complex)"
        AC1[navigate_to_pose<br/>NavigateToPose]
        AC2[perform_typing<br/>PerformTyping]
    end

    subgraph "Publishers"
        P1[🤖 Autonomy Pi]
        P2[🎮 Control Pi]
    end

    subgraph "Subscribers"
        SUB1[🤖 Autonomy Pi]
        SUB2[🎮 Control Pi]
        SUB3[🔧 Microcontrollers]
    end

    P1 --> T1
    P1 --> T2
    P2 --> T3
    P2 --> T4

    P2 --> S1
    P2 --> S2
    P2 --> S3

    P1 --> AC1
    P1 --> AC2

    T1 --> SUB2
    T2 --> SUB2
    T3 --> SUB3
    T4 --> SUB1
    T4 --> SUB2
    T4 --> SUB3

    S1 --> SUB1
    S2 --> SUB2
    S3 --> SUB1

    AC1 --> SUB2
    AC2 --> SUB2
```

### **Error Recovery Decision Tree**
```mermaid
flowchart TD
    START([Error Detected]) --> TYPE{Error Type?}

    TYPE -->|GPS Lost| GPS_CHECK{GPS Available<br/>in 30s?}
    TYPE -->|Vision Failed| VISION_CHECK{Vision Recoverable<br/>in 60s?}
    TYPE -->|Navigation Stuck| NAV_CHECK{Can Replan<br/>Path?}
    TYPE -->|Hardware Fault| HW_CHECK{Redundant<br/>System Available?}
    TYPE -->|Critical System| EMERGENCY[Emergency Stop<br/>All Systems]

    GPS_CHECK -->|Yes| GPS_RESUME[Resume with<br/>GPS]
    GPS_CHECK -->|No| DEAD_RECKON[Switch to<br/>Dead Reckoning]

    VISION_CHECK -->|Yes| VISION_RESUME[Resume with<br/>Vision]
    VISION_CHECK -->|No| REDUCED_VISION[Switch to<br/>Reduced Vision Mode]

    NAV_CHECK -->|Yes| NAV_RESUME[Replan and<br/>Resume Navigation]
    NAV_CHECK -->|No| MANUAL_REQUEST[Request Manual<br/>Override]

    HW_CHECK -->|Yes| HW_FAILOVER[Switch to<br/>Redundant System]
    HW_CHECK -->|No| SAFE_SHUTDOWN[Safe Shutdown<br/>Affected Systems]

    GPS_RESUME --> END([Continue Mission])
    DEAD_RECKON --> END
    VISION_RESUME --> END
    REDUCED_VISION --> END
    NAV_RESUME --> END
    MANUAL_REQUEST --> END
    HW_FAILOVER --> END
    SAFE_SHUTDOWN --> END
    EMERGENCY --> END

    classDef error fill:#ffebee,stroke:#c62828,stroke-width:2px
    classDef success fill:#c8e6c9,stroke:#2e7d32,stroke-width:2px

    class START,EMERGENCY error
    class END success
```

---

## 📊 Performance & Metrics

### **System Latency Breakdown**
```mermaid
pie title Total Control Loop Latency: 100ms
    "Sensor Acquisition" : 15
    "Data Preprocessing" : 10
    "Algorithm Execution" : 35
    "Command Generation" : 10
    "Network Communication" : 15
    "Hardware Response" : 10
    "Feedback Processing" : 5
```

### **Team Progress Tracking**
```mermaid
gantt
    title Team Development Timeline
    dateFormat YYYY-MM-DD
    axisFormat %m-%d

    section Autonomy Team
    SLAM Implementation    :done, slam, 2024-09-01, 30d
    Navigation System      :active, nav, after slam, 25d
    Computer Vision        :done, vision, 2024-09-15, 20d
    GPS Integration        :done, gps, 2024-10-01, 15d
    System Integration     :planned, integration, after nav, 20d

    section Control Team
    User Interface         :active, ui, 2024-09-10, 35d
    System Coordination    :planned, coord, after ui, 20d
    Monitoring Dashboard   :planned, monitor, after coord, 15d

    section Hardware Team
    Motor Controllers      :done, motors, 2024-09-05, 25d
    Arm Control            :active, arm, after motors, 20d
    Camera Control         :planned, camera, after arm, 15d
    Sensor Integration     :planned, sensors, after camera, 10d

    section Integration
    Mock Testing          :planned, mock, 2024-10-15, 10d
    Hardware Testing      :planned, hw_test, after mock, 15d
    Competition Prep      :planned, compete, after hw_test, 10d
```

---

## 🎯 Quick Reference Guide

### **Where to Find Specific Diagrams**

| Need | Document | Chart Name |
|------|----------|------------|
| **System overview** | [SystemArchitecture.md](SystemArchitecture.md) | Complete Robot Operation Flow |
| **Team coordination** | [../onboard/TeamIntegration.md](../onboard/TeamIntegration.md) | Team Integration Communication |
| **Development workflow** | [../onboard/TeamHandbook.md](../onboard/TeamHandbook.md) | Daily Development Cycle |
| **Interface details** | [InterfaceContract.md](InterfaceContract.md) | ROS2 Topic Interfaces |
| **Error handling** | [SystemArchitecture.md](SystemArchitecture.md) | Error Handling & Recovery Flow |
| **Testing process** | [SystemArchitecture.md](SystemArchitecture.md) | Integration Testing Workflow |
| **Mission sequence** | [SystemArchitecture.md](SystemArchitecture.md) | Competition Mission Sequence |

### **Color Coding Legend**
- 🔵 **Blue tones**: Autonomy Team (Your team) - SLAM, Navigation, Computer Vision
- 🟣 **Purple tones**: Control Team (User interface) - Mission planning, displays
- 🟢 **Green tones**: Hardware Team (Microcontrollers) - Motor control, sensors
- 🔴 **Red tones**: Error/Safety conditions - Emergency stops, critical failures
- 🟡 **Yellow/Orange tones**: Warning/Attention needed - Meetings, issues to resolve
- 🟢 **Light green tones**: Success/Completion - Goals achieved, positive outcomes

*Colors are designed to work in both light and dark themes*

### **🎨 Guidelines for Theme-Compatible Diagrams**

**When creating new Mermaid diagrams, follow these guidelines:**

#### **✅ Recommended Approach: Use classDef**
```mermaid
classDef teamName fill:#e1f5fe,stroke:#0277bd,stroke-width:2px
class NODE1,NODE2 teamName
```

#### **✅ Color Palette for Consistency**
- **🔵 Blue tones** (`#e1f5fe`, `#e3f2fd`): Autonomy Team, development, coordination
- **🟣 Purple tones** (`#f3e5f5`): Control Team, user interfaces
- **🟢 Green tones** (`#e8f5e8`, `#c8e6c9`): Hardware Team, success states
- **🟡 Yellow/Orange tones** (`#fff3e0`): Warning, processing, meetings
- **🔴 Red tones** (`#ffebee`): Errors, emergencies, critical issues

#### **❌ Avoid These Patterns**
```mermaid
%% DON'T do this - hardcoded colors don't work in dark mode
style NODE fill:#ffffff,color:#000000
```

#### **🎯 Best Practices**
1. **Use classDef** instead of individual style statements
2. **Choose light colors** that work on both light/dark backgrounds
3. **Include stroke colors** for better visibility
4. **Test diagrams** in both light and dark modes
5. **Use semantic class names** (autonomy, control, hardware, error, success)

---



## 🚀 Next Steps

1. **Review the diagrams** that apply to your current development phase
2. **Use the charts** to understand how your code fits into the larger system
3. **Reference the interface contracts** when implementing ROS2 communication
4. **Follow the integration workflow** when working with other teams

**These visual guides transform complex system interactions into clear, actionable diagrams!** 📊✨
