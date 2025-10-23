# State Management & Mode Control Track - University Rover Challenge 2026

## Introduction
The State Management & Mode Control subsystem serves as the central coordinator for the autonomy system, managing transitions between autonomous and teleoperation modes, tracking mission progress, and orchestrating all subsystems. This is the "brain" that ensures coherent operation across the entire autonomous mission lifecycle.

## Domain Information
**Mission Context**: Critical coordination system that must manage:
- Seamless mode switching between autonomous and teleoperation
- Mission state tracking across 7 navigation targets
- Abort and return-to-previous functionality
- LED status signaling for judges
- Integration with all autonomy subsystems

**Key Challenges**:
- Real-time coordination of multiple complex subsystems
- Reliable mode transitions without system instability
- Mission state persistence across interruptions
- Error handling and recovery orchestration
- Judge-visible status indication requirements

**Performance Requirements**:
- Response time: Near-instantaneous mode switching (<100ms)
- Reliability: Zero system crashes during mode transitions
- Accuracy: Correct mission state tracking and target completion
- Visibility: Clear status indication in bright daylight

## Pre-Requirements
Before starting state management development, ensure the following are available:

### Hardware Prerequisites
- Reliable communication infrastructure between all subsystems
- Real-time computing platform with deterministic timing
- Emergency stop system integration
- LED status indicator hardware
- Synchronization mechanisms (hardware interrupts, shared memory)

### Software Prerequisites
- Real-time operating system or framework
- State machine libraries or frameworks
- Inter-process communication (IPC) mechanisms
- Logging and monitoring systems
- Watchdog timer implementations

### Team Prerequisites
- Understanding of state machines and control systems
- Experience with real-time system design
- Knowledge of concurrent programming
- Familiarity with system integration patterns

## Technical Approach Possibilities

### A. State Machine Architecture
```
State Machine Architecture
├── Hierarchical State Machines
│   ├── Mission-level States
│   │   ├── Pre-mission preparation
│   │   ├── Active mission execution
│   │   ├── Post-mission cleanup
│   │   └── Emergency states
│   ├── Subsystem-level States
│   │   ├── Navigation states
│   │   ├── Vision processing states
│   │   ├── Manipulation states
│   │   └── Communication states
│   └── Mode-level States
│       ├── Autonomous operation
│       ├── Teleoperation mode
│       ├── Mixed initiative
│       └── Safe stop states
├── Finite State Machines (FSM)
│   ├── Moore Machine Implementation
│   │   ├── State-based outputs
│   │   ├── Predictable behavior
│   │   └── Easy verification
│   ├── Mealy Machine Implementation
│   │   ├── Input-dependent transitions
│   │   ├── More compact representation
│   │   └── Flexible responses
│   └── Statecharts
│       ├── Hierarchical decomposition
│       ├── Concurrent states
│       └── Event broadcasting
└── Behavior Trees
    ├── Modular Behavior Composition
    │   ├── Composite nodes (sequences, selectors)
    │   ├── Decorator nodes (inverters, timeouts)
    │   ├── Leaf nodes (actions, conditions)
    │   └── Dynamic behavior modification
    ├── Execution Control
    │   ├── Priority-based execution
    │   ├── Interrupt handling
    │   └── Resource management
    └── Runtime Adaptation
        ├── Dynamic tree modification
        ├── Parameter adjustment
        └── Learning-based adaptation
```

### B. Mode Transition Management
```
Mode Transition Management
├── Autonomous to Teleoperation Transitions
│   ├── Operator-Initiated Switching
│   │   ├── Command validation
│   │   ├── State preservation
│   │   ├── Smooth handoff procedures
│   │   └── Feedback confirmation
│   ├── Emergency Transitions
│   │   ├── Fault detection triggers
│   │   ├── Immediate safety actions
│   │   ├── Diagnostic data preservation
│   │   └── Recovery preparation
│   └── Conditional Transitions
│       ├── Terrain-based switching
│       ├── Confidence threshold triggers
│       ├── Progress-based decisions
│       │   └── Timeout handling
├── Teleoperation to Autonomous Transitions
│   ├── Operator Command Validation
│   │   ├── Command authentication
│   │   ├── State consistency checks
│   │   ├── Subsystem readiness verification
│   │   └── Safety condition assessment
│   ├── Automatic Transitions
│   │   ├── Mission phase completion
│   │   ├── Target achievement confirmation
│   │   ├── System recovery success
│   │   └── Operator inactivity detection
│   └── Gradual Transitions
│       ├── Progressive autonomy increase
│       ├── Operator supervision periods
│       ├── Confidence building phases
│       └── Fallback mechanisms
└── Transition Safety Mechanisms
    ├── State Consistency Verification
    │   ├── Data integrity checks
    │   ├── Subsystem synchronization
    │   ├── Command queue management
    │   └── Rollback capabilities
    ├── Safety Interlocks
    │   ├── Motion stop confirmation
    │   ├── Sensor agreement validation
    │   ├── Emergency stop overrides
    │   └── Watchdog timer resets
    └── Transition Logging
        ├── Event timestamping
        ├── Context preservation
        ├── Failure analysis data
        └── Performance metrics
```

### C. Mission State Tracking and Coordination
```
Mission State Tracking and Coordination
├── Target Sequence Management
│   ├── GNSS-Only Target Handling
│   │   ├── Coordinate validation
│   │   ├── Arrival detection (3m threshold)
│   │   ├── Progress confirmation
│   │   └── Sequence advancement
│   ├── AR Tag Target Processing
│   │   ├── GNSS approach coordination
│   │   ├── Vision-based final alignment
│   │   ├── Arrival verification (2m threshold)
│   │   └── Status signaling
│   └── Object Detection Targets
│       ├── Search pattern coordination
│       ├── Detection confirmation
│       ├── Display highlighting
│       └── Completion validation
├── Progress Monitoring
│   ├── Time Tracking
│   │   ├── Mission elapsed time
│   │   ├── Target approach times
│   │   ├── Timeout monitoring
│   │   └── Performance metrics
│   ├── Success Metrics
│   │   ├── Target completion rates
│   │   ├── Accuracy measurements
│   │   ├── Error counts and types
│   │   └── Recovery effectiveness
│   └── Quality Assessment
│       ├── Subsystem health monitoring
│       ├── Data quality validation
│       ├── Confidence level tracking
│       └── Performance degradation detection
└── Abort and Recovery Coordination
    ├── Autonomous Return Implementation
    │   ├── Previous waypoint identification
    │   ├── Path planning coordination
    │   ├── Safe return execution
    │   └── Arrival confirmation (5m threshold)
    ├── Teleoperation Return Handling
    │   ├── Operator guidance coordination
    │   ├── Direct path enforcement
    │   ├── Time penalty application
    │   └── Progress preservation
    └── Mission Continuation Logic
        ├── Failure assessment
        ├── Recovery strategy selection
        ├── Alternative approach activation
        └── Mission completion evaluation
```

### D. Subsystem Coordination and Communication
```
Subsystem Coordination and Communication
├── Inter-Subsystem Communication
│   ├── Publish-Subscribe Patterns
│   │   ├── ROS topic-based communication
│   │   ├── Event-driven messaging
│   │   ├── Data streaming protocols
│   │   └── Quality of service levels
│   ├── Request-Response Protocols
│   │   ├── Synchronous command execution
│   │   ├── Timeout handling
│   │   ├── Error propagation
│   │   └── Acknowledgment mechanisms
│   └── Shared Memory Interfaces
│       ├── Real-time data sharing
│       ├── Lock-free implementations
│       ├── Memory-mapped files
│       └── Synchronization primitives
├── Synchronization Mechanisms
│   ├── Clock Synchronization
│   │   ├── NTP-based synchronization
│   │   ├── PTP (Precision Time Protocol)
│   │   ├── Hardware timestamping
│   │   └── Time offset compensation
│   ├── State Synchronization
│   │   ├── Consensus algorithms
│   │   ├── State machine alignment
│   │   ├── Data consistency checks
│   │   └── Conflict resolution
│   └── Event Ordering
│       ├── Lamport timestamps
│       ├── Vector clocks
│       ├── Causality tracking
│       └── Event replay capabilities
└── Resource Management
    ├── Computational Resource Allocation
    │   ├── CPU core assignment
    │   ├── Memory management
    │   ├── GPU resource sharing
    │   └── Priority scheduling
    ├── Power Management
    │   ├── Subsystem power cycling
    │   ├── Energy budget tracking
    │   ├── Performance scaling
    │   └── Thermal monitoring
    └── Data Flow Management
        ├── Bandwidth allocation
        ├── Message prioritization
        ├── Compression strategies
        └── Flow control mechanisms
```

### E. Error Handling and Recovery
```
Error Handling and Recovery
├── Fault Detection and Classification
│   ├── Subsystem Health Monitoring
│   │   ├── Heartbeat monitoring
│   │   ├── Performance metric tracking
│   │   ├── Error rate analysis
│   │   └── Anomaly detection
│   ├── Sensor Validation
│   │   ├── Data consistency checks
│   │   ├── Redundancy verification
│   │   ├── Outlier detection
│   │   └── Confidence assessment
│   └── Communication Monitoring
│       ├── Link quality assessment
│       ├── Message integrity validation
│       ├── Latency monitoring
│       └── Connection recovery
├── Recovery Strategy Selection
│   ├── Error Severity Assessment
│   │   ├── Impact analysis
│   │   ├── Recovery time estimation
│   │   ├── Resource requirement evaluation
│   │   └── Safety risk assessment
│   ├── Recovery Action Planning
│   │   ├── Pre-defined recovery procedures
│   │   ├── Adaptive strategy selection
│   │   ├── Multi-step recovery sequences
│   │   └── Fallback mechanism activation
│   └── Operator Notification
│       ├── Alert prioritization
│       ├── Status display updates
│       ├── Recovery option presentation
│       └── Intervention guidance
└── System Resilience Features
    ├── Graceful Degradation
    │   ├── Functionality reduction
    │   ├── Performance scaling
    │   ├── Partial subsystem operation
    │   └── Mission adaptation
    ├── Redundant Architectures
    │   ├── Backup system activation
    │   ├── Failover mechanisms
    │   ├── Hot standby systems
    │   └── Data replication
    └── Learning and Adaptation
        ├── Error pattern recognition
        ├── Preventive measure activation
        ├── Parameter adjustment
        └── System reconfiguration
```

## Recommended Development Approach
1. **Phase 1**: Design and implement basic state machine architecture
2. **Phase 2**: Develop mode transition mechanisms with safety interlocks
3. **Phase 3**: Implement mission state tracking and target coordination
4. **Phase 4**: Add comprehensive error handling and recovery
5. **Phase 5**: Integrate LED status signaling and judge visibility features
6. **Phase 6**: Full system integration testing and robustness validation

## Integration Points
- **All Subsystems**: Coordinates operation of navigation, SLAM, vision, and typing
- **LED Control**: Manages status indicator state changes
- **C2 Station**: Provides mission status and control interfaces
- **Emergency Systems**: Integrates with rover safety mechanisms

## Testing Strategy
- **Unit testing**: Individual state machine components and transitions
- **Integration testing**: Multi-subsystem coordination scenarios
- **Mode transition testing**: Extensive autonomous/teleop switching validation
- **Error injection testing**: Fault simulation and recovery verification
- **Mission simulation**: Complete autonomous mission execution with all failure modes
- **Operator-in-the-loop**: Human-supervised testing with intervention scenarios
