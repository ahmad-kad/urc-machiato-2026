# Autonomy Interfaces TODO

## Interface Definition Tasks

### High Priority
- [ ] **Add MissionState.msg** - Define mission lifecycle states ("pre_mission", "executing", "completed", "aborted")
- [ ] **Add SystemHealth.msg** - Overall system health status with subsystem statuses
- [ ] **Add EmergencyStop.msg** - Emergency stop state and recovery information
- [ ] **Add CalibrationStatus.msg** - Calibration state and quality metrics for all calibrated systems

### Medium Priority
- [ ] **Enhance VisionDetection.msg** - Add bounding box coordinates, tracking IDs, and classification confidence per detection
- [ ] **Add SensorHealth.msg** - Individual sensor health status and diagnostic information
- [ ] **Add BatteryStatus.msg** - Battery state, voltage, current, and estimated runtime
- [ ] **Add GpsStatus.msg** - GPS fix quality, satellite count, and accuracy metrics

### Service Interface Gaps
- [ ] **Add EmergencyStop.srv** - Service to trigger emergency stop with reason codes
- [ ] **Add ResetSubsystem.srv** - Service to reset individual subsystems to known state
- [ ] **Add GetSystemDiagnostics.srv** - Comprehensive system diagnostic information
- [ ] **Add ValidateSystem.srv** - Pre-mission system validation and readiness check

### Action Interface Enhancements
- [ ] **Add PerformCalibration.action** - Long-running calibration sequences with progress feedback
- [ ] **Add ExecuteMission.action** - Complete mission execution with waypoint progress
- [ ] **Add SystemRecovery.action** - Automated recovery from error states

## Validation & Testing

### Interface Validation
- [ ] **Create interface validation scripts** - Verify all messages/services/actions compile correctly
- [ ] **Add schema validation** - JSON schema validation for complex message structures
- [ ] **Create interface usage examples** - Sample code showing how to use each interface
- [ ] **Add interface dependency analysis** - Track which subsystems use which interfaces

### Integration Testing
- [ ] **Create interface mock implementations** - For testing subsystems in isolation
- [ ] **Add interface compatibility tests** - Ensure interface changes don't break existing code
- [ ] **Create end-to-end interface tests** - Full system integration testing through interfaces

## Documentation & Maintenance

### Documentation Updates
- [ ] **Update interface README** - Add specific examples from implemented interfaces
- [ ] **Create interface usage guide** - How to extend and modify interfaces safely
- [ ] **Add interface versioning strategy** - How to handle breaking changes
- [ ] **Document interface ownership** - Which team/subsystem owns each interface

### Code Quality
- [ ] **Add interface linters** - Custom linting rules for ROS2 interface files
- [ ] **Create interface templates** - Standardized templates for new interfaces
- [ ] **Add interface deprecation warnings** - Graceful handling of deprecated interfaces
- [ ] **Implement interface monitoring** - Runtime validation of interface usage

## Implementation Status

### Currently Implemented
- [x] NavigationStatus.msg - Basic navigation state reporting
- [x] VisionDetection.msg - Computer vision detections
- [x] LedCommand.msg - LED status control
- [x] TypingGoal.msg - Autonomous typing goals
- [x] SlamStatus.msg - SLAM system status
- [x] CameraCommand.msg - Camera control commands
- [x] NavigateToPose.action - Navigation to pose with feedback
- [x] PerformTyping.action - Typing execution with progress
- [x] SwitchMode.srv - System mode switching
- [x] CalibrateCamera.srv - Camera calibration
- [x] GetSubsystemStatus.srv - Individual subsystem status
- [x] ConfigureMission.srv - Mission configuration
- [x] LoadCalibrationParameters.srv - Calibration parameter loading
- [x] ValidateCalibration.srv - Calibration validation

### Interface Quality Metrics
- [ ] **Interface coverage analysis** - What percentage of subsystem communication is through defined interfaces
- [ ] **Interface stability tracking** - How often interfaces change vs subsystem code
- [ ] **Interface usage metrics** - Which interfaces are most/least used in the system

## Future Considerations

### Advanced Features
- [ ] **Add interface introspection** - Runtime discovery of available interfaces
- [ ] **Implement interface negotiation** - Dynamic interface compatibility negotiation
- [ ] **Add interface security** - Authentication and authorization for critical interfaces
- [ ] **Create interface federation** - Cross-system interface coordination

### Performance & Scalability
- [ ] **Interface performance profiling** - Message throughput and latency analysis
- [ ] **Add interface compression** - Efficient serialization for bandwidth-constrained scenarios
- [ ] **Implement interface batching** - Group related messages for efficiency
- [ ] **Add interface prioritization** - Quality-of-service levels for different message types

---

## Interface Design Checklist

When adding new interfaces, ensure they follow these principles:

- [ ] **Single Responsibility** - Each interface has one clear purpose
- [ ] **Minimal Data** - Only essential fields, no optional bloat
- [ ] **Clear Semantics** - Field names and values have obvious meanings
- [ ] **Error Information** - Include success/failure status and error details
- [ ] **Version Compatibility** - New versions don't break existing code
- [ ] **Documentation** - Comprehensive comments explaining purpose and usage
- [ ] **Testing** - Interface contracts are testable and validated
