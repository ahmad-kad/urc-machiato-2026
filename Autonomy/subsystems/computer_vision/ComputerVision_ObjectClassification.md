# Computer Vision & Object Classification Track - University Rover Challenge 2026

## Introduction
The Computer Vision & Object Classification subsystem detects, classifies, and tracks visual targets including ArUco markers and ground objects. This is critical for AR tag navigation posts, autonomous object detection (mallet, hammer, water bottle), and providing visual servoing for precise positioning in the Autonomous Navigation Mission.

## Domain Information
**Mission Context**: Vision system must operate in challenging desert conditions while detecting:
- ArUco tags on 3-sided posts (20x20cm faces, 0.5-1.5m height)
- Three specific ground objects with varying characteristics
- Real-time performance for 30-minute missions
- Mandatory object highlighting on C2 station display

**Key Challenges**:
- Outdoor illumination variations (bright sunlight, shadows, dust)
- Distance variations (close-up detection to long-range identification)
- Object pose and orientation changes
- Real-time processing requirements
- Desert environment degradation (dust on lenses, wind effects)

**Performance Requirements**:
- Detection range: Up to 10-20 meters for objects depending on size
- Accuracy: Reliable classification with low false positives
- Speed: Real-time processing (15-30 FPS)
- Robustness: Operation in varying lighting and weather conditions

## Pre-Requirements
Before starting computer vision development, ensure the following are available:

### Hardware Prerequisites
- RGB camera(s) with good low-light performance
- Stereo camera pair or depth sensor (optional but beneficial)
- Sufficient lighting for low-light operation
- Vibration-dampened camera mounts
- Weather protection (dust covers, sealed enclosures)
- GPU-capable computing platform for real-time processing

### Software Prerequisites
- OpenCV library with contrib modules
- Deep learning frameworks (TensorFlow, PyTorch, or similar)
- ROS computer vision packages
- Camera calibration tools and procedures
- Image processing libraries (scikit-image, PIL)

### Team Prerequisites
- Understanding of image processing fundamentals
- Experience with deep learning for computer vision
- Knowledge of camera calibration and projective geometry
- Familiarity with real-time image processing constraints

## Technical Approach Possibilities

### A. AR Tag Detection and Pose Estimation
```
AR Tag Detection and Pose Estimation
├── ArUco Marker Detection
│   ├── Standard ArUco Library Implementation
│   │   ├── 4x4_50 dictionary usage
│   │   ├── Corner detection and identification
│   │   └── Built-in pose estimation
│   ├── Custom Detection Pipeline
│   │   ├── Adaptive thresholding for lighting
│   │   ├── Contour analysis for marker extraction
│   │   └── Robustness to partial occlusion
│   └── Multi-marker Tracking
│       ├── Simultaneous detection of multiple tags
│       ├── Relative pose computation
│       └── Marker array handling
├── Pose Estimation Refinement
│   ├── Perspective-n-Point (PnP) Algorithms
│   │   ├── Efficient PnP (EPnP)
│   │   ├── Iterative closest point refinement
│   │   └── Bundle adjustment for accuracy
│   ├── Kalman Filtering for Pose Tracking
│   │   ├── Prediction and measurement updates
│   │   ├── Handling of temporary occlusions
│   │   └── Smoothing of pose estimates
│   └── Multi-camera Fusion
│       ├── Stereo pose triangulation
│       ├── Redundant pose validation
│       └── Improved accuracy through consensus
└── Robustness Enhancements
    ├── Lighting Invariant Detection
    │   ├── Histogram equalization
    │   ├── Shadow detection and compensation
    │   └── Multi-spectral approaches
    ├── Motion Blur Compensation
    │   ├── Deblurring algorithms
    │   ├── Motion prediction
    │   └── High frame rate processing
    └── Environmental Adaptation
        ├── Dust detection and cleaning
        ├── Wind-induced motion compensation
        └── Temperature-based calibration
```

### B. Object Detection and Classification
```
Object Detection and Classification
├── Traditional Computer Vision Methods
│   ├── Feature-based Detection
│   │   ├── SIFT/SURF feature extraction
│   │   ├── Bag-of-words classification
│   │   └── Template matching approaches
│   ├── Color-based Segmentation
│   │   ├── HSV color space processing
│   │   ├── Adaptive color thresholding
│   │   └── Color constancy algorithms
│   └── Shape-based Recognition
│       ├── Contour analysis and matching
│       ├── Fourier descriptors
│       └── Geometric primitive fitting
├── Deep Learning Approaches
│   ├── Single-Shot Detectors (SSD)
│   │   ├── MobileNet backbone for efficiency
│   │   ├── Real-time performance
│   │   └── Compact model size
│   ├── YOLO (You Only Look Once)
│   │   ├── YOLOv5/v7/v8 variants
│   │   ├── High speed and accuracy balance
│   │   └── Custom dataset training
│   └── Faster R-CNN
│       ├── Two-stage detection pipeline
│       ├── High accuracy for precise localization
│       └── Feature pyramid networks
├── Specialized Object Recognition
│   ├── Mallet Detection
│   │   ├── Orange color segmentation
│   │   ├── Handle and head feature detection
│   │   └── Size-based filtering
│   ├── Rock Pick Hammer Detection
│   │   ├── Metallic surface detection
│   │   ├── Handle and pick geometry
│   │   └── Texture analysis
│   └── Water Bottle Detection
│       ├── Plastic bottle shape recognition
│       ├── Label and cap detection
│       ├── Size and aspect ratio filtering
│       └── Color-agnostic approaches
└── Multi-modal Fusion
    ├── RGB + Depth Integration
    │   ├── 3D bounding box estimation
    │   ├── Size validation with depth
    │   └── Occlusion handling
    ├── Temporal Consistency
    │   ├── Object tracking across frames
    │   ├── Motion prediction
    │   └── Trajectory smoothing
    └── Confidence Fusion
        ├── Multi-detector consensus
        ├── Uncertainty quantification
        └── Decision confidence thresholding
```

### C. Visual Servoing and Precision Positioning
```
Visual Servoing and Precision Positioning
├── Position-Based Visual Servoing (PBVS)
│   ├── Feature Extraction
│   │   ├── Corner detection for alignment
│   │   ├── Edge detection for contours
│   │   └── Interest point selection
│   ├── Pose Estimation Pipeline
│   │   ├── 3D model registration
│   │   ├── Homography computation
│   │   └── Error minimization
│   └── Control Integration
│       ├── Jacobian matrix computation
│       ├── Velocity command generation
│       └── Stability analysis
├── Image-Based Visual Servoing (IBVS)
│   ├── Image Feature Tracking
│   │   ├── Lucas-Kanade optical flow
│   │   ├── KLT feature tracker
│   │   └── Robust feature matching
│   ├── Interaction Matrix
│   │   ├── Feature Jacobian estimation
│   │   ├── Depth information integration
│   │   └── Robust matrix inversion
│   └── Control Strategies
│       ├── Proportional control
│       ├── Adaptive gain scheduling
│       └── Redundant feature handling
└── Hybrid Approaches
    ├── 2.5D Visual Servoing
    │   ├── Combined position and image features
    │   ├── Depth-enhanced control
    │   └── Improved convergence
    ├── Switching Control
    │   ├── PBVS for far distances
    │   ├── IBVS for close approach
    │   └── Smooth transition logic
    └── Learned Visual Servoing
        ├── Deep learning for control policy
        ├── End-to-end visual control
        └── Data-driven approaches
```

### D. Real-time Processing and Optimization
```
Real-time Processing and Optimization
├── Hardware Acceleration
│   ├── GPU Processing
│   │   ├── CUDA/OpenCL implementations
│   │   ├── TensorRT for inference optimization
│   │   └── Parallel processing pipelines
│   ├── Edge Computing
│   │   ├── NVIDIA Jetson platforms
│   │   ├── Intel Movidius/Myriad
│   │   └── ARM-based accelerators
│   └── Distributed Processing
│       ├── Multi-camera processing
│       ├── Load balancing across devices
│       └── Networked computation
├── Algorithm Optimization
│   ├── Model Compression
│   │   ├── Quantization (INT8/FP16)
│   │   ├── Pruning for sparsity
│   │   └── Knowledge distillation
│   ├── Efficient Architectures
│   │   ├── MobileNet variants
│   │   ├── SqueezeNet architecture
│   │   └── EfficientNet scaling
│   └── Pipeline Optimization
│       ├── Asynchronous processing
│       ├── Frame skipping strategies
│       └── Priority-based computation
└── Performance Monitoring
    ├── Real-time Metrics
    │   ├── Processing latency tracking
    │   ├── Frame rate monitoring
    │   └── Memory usage profiling
    ├── Quality Assessment
    │   ├── Detection accuracy metrics
    │   ├── False positive/negative rates
    │   └── Robustness scoring
    └── Adaptive Parameters
        ├── Dynamic threshold adjustment
        ├── Resolution scaling
        └── Computational resource allocation
```

### E. Desert Environment Adaptation
```
Desert Environment Adaptation
├── Illumination Handling
│   ├── High Dynamic Range (HDR) Processing
│   │   ├── Multi-exposure fusion
│   │   ├── Tone mapping algorithms
│   │   └── Local contrast enhancement
│   ├── Shadow Detection and Compensation
│   │   ├── Shadow invariant features
│   │   ├── Illumination estimation
│   │   └── Adaptive normalization
│   └── Backlighting Mitigation
│       ├── Silhouette detection
│       ├── Local feature enhancement
│       └── Alternative sensing strategies
├── Dust and Particle Effects
│   ├── Lens Cleaning Detection
│   │   ├── Image quality assessment
│   │   ├── Blur detection algorithms
│   │   └── Automatic cleaning triggers
│   ├── Particle Filtering
│   │   ├── Morphological operations
│   │   ├── Temporal filtering
│   │   └── Motion-based discrimination
│   └── Multi-spectral Approaches
│       ├── Infrared supplementation
│       ├── Polarization filtering
│       └── Alternative sensing modalities
├── Thermal Effects
│   ├── Temperature-based Calibration
│   │   ├── Lens distortion correction
│   │   ├── Sensor response compensation
│   │   └── Automatic recalibration
│   ├── Heat Haze Mitigation
│   │   ├── Turbulence correction
│   │   ├── Image stabilization
│   │   └── Multi-frame averaging
│   └── Component Cooling
│       ├── Active cooling systems
│       ├── Thermal management
│       └── Performance monitoring
└── Wind and Vibration
    ├── Motion Compensation
    │   ├── Gyroscope-based stabilization
    │   ├── Optical flow correction
    │   └── Kalman filtering for motion
    ├── Vibration Dampening
    │   ├── Mechanical isolation
    │   ├── Digital stabilization
    │   └── Adaptive filtering
    └── Dynamic Object Discrimination
        ├── Wind-blown object filtering
        ├── Motion pattern analysis
        └── Environmental motion modeling
```

## Recommended Development Approach
1. **Phase 1**: Implement basic camera calibration and ArUco tag detection
2. **Phase 2**: Develop object detection pipeline with traditional methods
3. **Phase 3**: Train and integrate deep learning models for classification
4. **Phase 4**: Add visual servoing capabilities and precision positioning
5. **Phase 5**: Optimize for desert conditions and real-time performance

## Integration Points
- **SLAM**: Provides pose estimates and map context for vision
- **Navigation**: Uses vision for final approach and precision landing
- **State Management**: Coordinates vision operations with mission phases
- **C2 Station**: Mandatory object highlighting on operator display

## Testing Strategy
- **Lab testing**: Controlled lighting with printed markers and 3D-printed objects
- **Outdoor testing**: Progressive exposure to natural lighting and weather
- **Desert simulation**: Dust chambers and vibration testing
- **Integration testing**: Full pipeline testing with rover movement
- **Mission simulation**: Complete autonomous scenarios with timing constraints
