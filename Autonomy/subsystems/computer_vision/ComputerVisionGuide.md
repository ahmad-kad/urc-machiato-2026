# Computer Vision & Object Classification Implementation Guide

## Current BOM Components Available
- **RGB-D camera (OakD or Intel 435i)** - Depth sensing and RGB imaging
- **Raspberry Pi AI camera (Mast camera with 360° gimbal)** - Panoramic vision capability
- **Raspberry Pi 5** - Main processing unit for vision algorithms
- **5 Raspberry Pi Zero** - Potential for distributed vision processing

## Implementation Roadmap

### Phase 1: Camera Setup & Basic Detection (2-3 weeks)
**Goal**: Establish working camera systems and basic ArUco detection

#### Step-by-Step Implementation:
1. **Camera Integration Setup**
   ```bash
   # Install OpenCV and camera drivers on Raspberry Pi 5
   sudo apt-get install python3-opencv libopencv-dev
   # Install depthai for Oak-D or librealsense for Intel camera
   pip install depthai-sdk  # or librealsense python wrapper
   ```

2. **Camera Calibration**
   - Follow detailed procedures in `../../docs/reference/CalibrationGuide.md`
   - Calibrate RGB-D camera intrinsics and extrinsics
   - Set up stereo calibration for depth accuracy
   - Test depth data quality and range

3. **ArUco Tag Detection**
   ```python
   # Basic ArUco detection setup
   import cv2
   aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
   aruco_params = cv2.aruco.DetectorParameters()
   detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
   ```

#### Component Improvements Needed:
- **GPU acceleration**: NVIDIA Jetson Nano ($100-150) - Essential for real-time neural networks
- **Additional cameras**: 2-3 more RGB cameras ($50-100 each) - Multi-view coverage
- **Better RGB-D camera**: Intel RealSense D435i ($150-200) - Superior outdoor performance

### Phase 2: Object Detection Pipeline (4-5 weeks)
**Goal**: Implement neural network-based object detection for mission targets

#### Step-by-Step Implementation:
1. **Dataset Collection**
   - Photograph objects: orange mallet, rock pick hammer, water bottle
   - Capture varied lighting, angles, distances
   - Create training/validation/test splits (70/20/10)

2. **Model Training Setup**
   ```bash
   # Install PyTorch/TensorFlow on Jetson Nano
   pip install torch torchvision torchaudio
   # Or TensorFlow for Coral TPU compatibility
   pip install tensorflow
   ```

3. **YOLOv5 Implementation**
   - Fine-tune pre-trained YOLOv5 on custom dataset
   - Optimize for Raspberry Pi/Jetson performance
   - Implement real-time inference pipeline

#### Component Improvements Needed:
- **Edge TPU**: Google Coral ($75-150) - Hardware acceleration for ML models
- **High-quality lenses**: Better camera optics ($50-100) - Improved image quality
- **Pan-tilt mechanism**: For the mast camera ($100-200) - Better object tracking
- **Thermal camera**: FLIR Lepton ($200-300) - Alternative sensing for robustness

### Phase 3: Advanced Features & Robustness (3-4 weeks)
**Goal**: Add tracking, servoing, and desert environment adaptation

#### Step-by-Step Implementation:
1. **Object Tracking**
   - Implement DeepSORT or similar multi-object tracking
   - Add temporal consistency across frames
   - Handle occlusions and re-identification

2. **Visual Servoing**
   - Implement IBVS (Image-Based Visual Servoing) for precision approach
   - Add pose estimation for ArUco tags
   - Integrate with navigation system

3. **Desert Adaptation**
   - Implement dust detection and filtering
   - Add dynamic lighting compensation
   - Test wind-induced motion compensation

#### Component Improvements Needed:
- **Multiple camera array**: 4-6 synchronized cameras ($200-400) - Full 360° coverage
- **High-performance GPU**: NVIDIA Jetson Xavier NX ($400-600) - Advanced ML capabilities
- **Weatherproof enclosures**: IP67-rated camera housings ($100-200) - Desert protection
- **Optical filters**: Polarizing/UV filters ($50-100) - Improved outdoor performance

### Phase 4: Mission Integration & C2 Display (2-3 weeks)
**Goal**: Connect vision system with mission requirements and operator display

#### Step-by-Step Implementation:
1. **C2 Station Integration**
   - Implement object highlighting on operator display
   - Add real-time video streaming
   - Create intuitive visualization interface

2. **State Management Integration**
   - Connect detection results with mission state
   - Add confidence reporting and validation
   - Implement mission-specific detection modes

3. **Performance Optimization**
   - Optimize model inference speed
   - Add power management features
   - Implement automatic quality assessment

#### Component Improvements Needed:
- **Dedicated display computer**: Mini PC for C2 station ($300-500) - Better visualization
- **High-bandwidth communication**: 5GHz WiFi or Ethernet ($50-100) - Real-time video streaming
- **Backup cameras**: Redundant vision systems ($150-250) - Mission reliability

## Recommended Component Upgrades

### High Priority (Essential for Success):
1. **NVIDIA Jetson Nano**: $100-150 - GPU acceleration for neural networks
2. **Google Coral TPU**: $75-100 - Hardware ML acceleration
3. **Additional RGB cameras**: 2-3 cameras ($50-100 each) - Multi-view coverage
4. **Intel RealSense D435i**: $150-200 - Better RGB-D performance

### Medium Priority (Significant Improvement):
1. **Camera pan-tilt units**: $100-200 - Better object tracking and coverage
2. **High-quality lenses**: $50-100 - Improved image quality and zoom
3. **Environmental protection**: Weatherproof housings ($100-150) - Desert durability
4. **Optical filters**: $50-80 - Enhanced outdoor performance

### Low Priority (Nice-to-have):
1. **Thermal camera**: $200-300 - Alternative sensing modality
2. **Event cameras**: $150-250 - Motion detection in challenging conditions
3. **Hyperspectral camera**: $500+ - Advanced material classification

## Neural Network Architecture Recommendations

### For Object Detection:
```
YOLOv5s (Small model for edge deployment)
├── Input: 416x416 RGB images
├── Performance: 30-60 FPS on Jetson Nano
├── Accuracy: 85-90% mAP on custom dataset
└── Memory: ~100MB

YOLOv7-tiny (Latest efficient architecture)
├── Input: 640x640 RGB images
├── Performance: 20-40 FPS on Jetson Nano
├── Accuracy: 90-95% mAP with fine-tuning
└── Memory: ~50MB
```

### For ArUco Detection Enhancement:
```
Custom CNN + ArUco (Hybrid approach)
├── ArUco detection: Traditional computer vision
├── CNN verification: Neural network confidence
├── Pose refinement: Deep learning pose estimation
└── Robustness: Combined traditional + ML approaches
```

## Data Collection Strategy

### Training Dataset Requirements:
- **Per object class**: 1000-2000 images minimum
- **Variations**: Different lighting, angles, distances, backgrounds
- **Negative samples**: 500+ images without target objects
- **Augmentation**: Brightness, contrast, rotation, scale variations

### Data Sources:
1. **Laboratory collection**: Controlled lighting, known distances
2. **Outdoor testing**: Real desert conditions, varied weather
3. **Simulation**: Blender/Unity synthetic data generation
4. **Public datasets**: Similar object categories for pre-training

## Integration Points

### With Navigation System:
- Provide precise target poses for approach
- Receive navigation commands for visual servoing
- Coordinate search patterns for object detection

### With SLAM System:
- Share visual features for mapping
- Provide loop closure candidates
- Coordinate landmark detection

### With State Management:
- Report detection confidence and results
- Receive mission context and target priorities
- Handle different operational modes

## Testing Protocol

### Unit Testing:
- Individual camera calibration accuracy
- Detection accuracy on static images
- Model inference speed and resource usage

### Integration Testing:
- Multi-camera synchronization
- Real-time processing pipeline validation
- C2 station display integration

### Field Testing:
- Progressive difficulty: controlled → outdoor → desert conditions
- Environmental variations: lighting, dust, wind
- Mission simulation: full autonomous object detection scenarios

## Performance Metrics

### Detection Accuracy:
- **ArUco tags**: 99%+ detection rate at mission ranges
- **Objects**: 90%+ accuracy with 20-30% false positive rate
- **Pose estimation**: <5cm position, <2° orientation accuracy

### Real-time Performance:
- **Frame rate**: 15-30 FPS processing
- **Latency**: <100ms end-to-end detection
- **Resource usage**: <70% CPU/GPU utilization

### Robustness:
- **Lighting variation**: Maintain 80%+ accuracy in all conditions
- **Distance range**: Reliable detection from 1m to 20m+
- **Motion blur**: Handle rover speeds up to 2 m/s

## Budget Considerations

### Essential Vision Upgrades:
- **GPU acceleration**: $175-250 (Jetson Nano + Coral TPU)
- **Camera improvements**: $200-300 (RealSense + additional cameras)
- **Protection**: $100-150 (weatherproofing + filters)

### Advanced Vision System:
- **High-performance GPU**: $400-600 (Jetson Xavier NX)
- **Multi-camera array**: $300-500 (4-6 cameras + synchronization)
- **Professional optics**: $200-400 (lenses + pan-tilt mechanisms)

### Total Vision Budget: $800-2000+ for competition readiness

---

*Computer vision is computationally intensive - prioritize GPU acceleration and comprehensive training data for reliable desert operation.*
