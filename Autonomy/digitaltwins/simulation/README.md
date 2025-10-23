# Digital Twins for URC 2026 Rover Autonomy

This directory contains the digital twin implementation for real-time synchronization between the physical URC 2026 rover and its virtual counterpart.

## ⚠️ PLACEHOLDER VALUES WARNING

**ALL VALUES AND IMPLEMENTATIONS IN THIS DIRECTORY ARE PLACEHOLDERS**

The digital twin system is currently implemented with:
- Example configuration values
- Simplified synchronization logic
- Mock prediction algorithms
- Placeholder model parameters

**Real implementation requires:**
- Actual hardware calibration
- Physics-based simulation models
- Machine learning prediction models
- Real sensor data integration

## 📁 Directory Structure

```
digitaltwins/simulation/
├── config/
│   └── digital_twin_config.yaml    # PLACEHOLDER: Configuration with example values
├── models/
│   └── digital_twin_models.py      # PLACEHOLDER: Data models and utilities
├── scripts/
│   └── digital_twin_manager.py     # PLACEHOLDER: Main digital twin node
├── launch/
│   └── digital_twin.launch.py      # PLACEHOLDER: Launch configuration
└── data/                          # PLACEHOLDER: Would contain twin data/models
```

## 🚀 Quick Start

### Basic Digital Twin Operation

```bash
# 1. Ensure simulation is running
ros2 launch autonomy_simulation basic_simulation.launch.py

# 2. Launch digital twin in another terminal
ros2 launch autonomy_simulation digitaltwins/launch/digital_twin.launch.py

# 3. Monitor digital twin status
ros2 topic echo /digital_twin/sync_status
ros2 topic echo /digital_twin/health
```

### Topics Published by Digital Twin

```
/digital_twin/state          # Current twin state (JSON)
/digital_twin/prediction      # Motion predictions
/digital_twin/sync_status     # Synchronization status
/digital_twin/health          # System health
```

## ⚙️ PLACEHOLDER Configuration

### Key PLACEHOLDER Values (Require Calibration)

```yaml
# From digital_twin_config.yaml
dynamics:
  mass: 50.0                    # PLACEHOLDER: Real rover mass TBD
  inertia_matrix:               # PLACEHOLDER: Real inertia tensor TBD
    ixx: 2.0, ixy: 0.0, ...     # PLACEHOLDER values

sensors:
  gps:
    accuracy: 3.0               # PLACEHOLDER: Real GPS accuracy TBD
  imu:
    gyro_drift: 0.01            # PLACEHOLDER: Real drift rate TBD

actuators:
  wheels:
    max_torque: 50.0            # PLACEHOLDER: Real motor specs TBD
```

## 🔄 Synchronization Logic

### PLACEHOLDER Implementation

The current implementation uses simplified synchronization:

1. **Real-to-Virtual Sync**: Copies physical sensor data to virtual model
2. **Prediction**: Basic kinematic extrapolation (PLACEHOLDER)
3. **Health Monitoring**: Basic checks with PLACEHOLDER thresholds
4. **Model Updates**: Simple parameter updates (PLACEHOLDER)

### Real Implementation Requirements

- Advanced Kalman filtering for state estimation
- Physics engines (PyBullet, Gazebo) for accurate simulation
- Machine learning models for behavior prediction
- Real-time data fusion algorithms

## 📊 Digital Twin Models

### PLACEHOLDER Model Structure

```python
# From digital_twin_models.py
@dataclass
class TwinModel:
    dynamics: Dict[str, Any]     # PLACEHOLDER: Mass, inertia, etc.
    sensors: Dict[str, Any]      # PLACEHOLDER: Accuracy, noise models
    actuators: Dict[str, Any]    # PLACEHOLDER: Motor specs, limits
    environment: Dict[str, Any]  # PLACEHOLDER: Mars conditions
```

### Required Real Models

- **Dynamics Model**: Multi-body physics simulation
- **Sensor Models**: Noise, bias, and failure modeling
- **Actuator Models**: Motor dynamics and control limits
- **Environment Models**: Terrain, weather, and Mars-specific conditions

## 🎯 Use Cases

### PLACEHOLDER Applications (Current)

- Basic sensor data mirroring
- Simple motion prediction
- Health status monitoring
- Configuration validation

### Planned Real Applications

- **Predictive Maintenance**: Anticipate hardware failures
- **Virtual Testing**: Test autonomy algorithms virtually
- **Mission Planning**: Optimize routes using twin predictions
- **Real-time Optimization**: Adjust behavior based on twin insights

## 🔧 Development Roadmap

### Phase 1: PLACEHOLDER (Current)
- [x] Basic twin structure with PLACEHOLDER values
- [x] Simple synchronization logic
- [x] Mock sensor integration

### Phase 2: Basic Implementation
- [ ] Real sensor calibration
- [ ] Physics-based dynamics model
- [ ] Basic prediction algorithms

### Phase 3: Advanced Features
- [ ] Machine learning predictions
- [ ] Real-time optimization
- [ ] Hardware failure prediction

### Phase 4: Production Ready
- [ ] Full system integration
- [ ] Validation and testing
- [ ] Performance optimization

## 🧪 Testing and Validation

### PLACEHOLDER Testing

```bash
# Run digital twin validation
python3 -c "
from models.digital_twin_models import TwinModel, validate_twin_health
model = TwinModel()
issues = model.validate()
print(f'Model validation: {len(issues)} issues found')
"
```

### Real Validation Requirements

- Hardware-in-the-loop testing
- Model accuracy validation
- Prediction performance metrics
- Synchronization latency testing

## 📝 API Reference

### PLACEHOLDER Classes

- `PhysicalState`: Real rover state representation
- `VirtualState`: Simulated rover state representation
- `TwinModel`: Digital twin model parameters
- `SyncStatus`: Synchronization status tracking

### PLACEHOLDER Functions

- `calculate_sync_fidelity()`: Compare real vs virtual states
- `predict_rover_motion()`: Generate motion predictions
- `validate_twin_health()`: Check system health

## 🚨 Important Notes

1. **DO NOT use PLACEHOLDER values in production**
2. **All model parameters require hardware calibration**
3. **Current implementation is for development/testing only**
4. **Real digital twin requires significant additional development**

## 🤝 Contributing

When developing the real digital twin:

1. Start with hardware characterization
2. Implement proper physics models
3. Add comprehensive validation
4. Document all calibration procedures
5. Test extensively with real hardware

---

**Status**: PLACEHOLDER Implementation - Not Production Ready
**Next Step**: Hardware calibration and physics model development
