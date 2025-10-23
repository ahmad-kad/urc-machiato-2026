# 🎬 Multi-Camera Setup Guide - URC 2026

## Overview
This guide walks you through calibrating and using **5 cameras** sequentially for ArUco tag detection with distance measurement.

---

## 📋 Step 1: Calibrate All 5 Cameras

### Quick Start (Recommended)
```bash
cd /Users/ahmadkaddoura/robotics2025/Autonomy/calibration/camera

python calibrate_multiple_cameras.py \
  --num-cameras 5 \
  --cols 7 --rows 5 \
  --square-size 0.030 --marker-size 0.018 \
  --output-dir ./camera_calibrations \
  --duration 30
```

### What Happens
1. **Camera 0**: Calibration starts automatically
   - Window opens showing your camera feed
   - Move the ChArUco board around for 30 seconds
   - Collects 15+ frames with good marker visibility
   - Saves to `camera_calibrations/camera_0_calibration.json`
   - Window closes automatically

2. **Prompt**: "⏳ Prepare camera 1 and press ENTER to continue..."
   - Physically swap out camera 0 with camera 1
   - Press ENTER to start next calibration

3. **Repeat** for cameras 1-4

### Customization
```bash
# Faster calibration (20 seconds per camera)
python calibrate_multiple_cameras.py --num-cameras 5 --duration 20

# More thorough (60 seconds per camera)
python calibrate_multiple_cameras.py --num-cameras 5 --duration 60

# Different board size
python calibrate_multiple_cameras.py --num-cameras 5 --cols 8 --rows 6

# Start from camera 1 (skip camera 0)
python calibrate_multiple_cameras.py --num-cameras 5 --start-camera 1
```

### Output Structure
```
camera_calibrations/
├── camera_0_calibration.json
├── camera_1_calibration.json
├── camera_2_calibration.json
├── camera_3_calibration.json
└── camera_4_calibration.json
```

Each file contains:
```json
{
  "camera_index": 0,
  "camera_matrix": { ... },
  "distortion_coefficients": { ... },
  "image_width": 1280,
  "image_height": 720,
  "frames_used": 15,
  "board_size": "7x5"
}
```

---

## 🎯 Step 2: Detect ArUco Tags with All Cameras

### Process All Cameras in Sequence
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --sequential
```

### Process All Cameras in Parallel (Simultaneous)
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18
```

This opens 5 windows simultaneously, one for each camera.

### Process Single Camera Only
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --camera 0 \
  --tag-size 18
```

### Headless Mode (No Display)
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --no-display \
  --duration 60
```

Runs for 60 seconds without showing windows. Useful for automated testing.

---

## 📊 Implementation Workflow

### Setup Phase (One-Time)
```bash
# 1. Prepare ChArUco board (7×5, 30mm squares, 18mm markers)
# 2. Connect 5 cameras to computer

# 3. Run calibration
cd camera_calibrations_dir
python calibrate_multiple_cameras.py --num-cameras 5

# Save all calibration files to: ./camera_calibrations/
```

### Production Phase
```bash
# 4. Deploy detection code with calibrations
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --sequential  # or remove for parallel processing

# 5. Each camera independently:
#    - Detects ArUco tags
#    - Calculates distance using its own calibration
#    - Displays tag ID and distance
```

---

## 🔧 Architecture

### Multi-Camera Detector System

```
calibrate_multiple_cameras.py
├── Loops through cameras 0-4
├── For each camera:
│   ├── Opens VideoCapture(camera_index)
│   ├── Detects ChArUco markers
│   ├── Collects 15 frames minimum
│   ├── Calibrates using cv2.calibrateCamera()
│   └── Saves camera_N_calibration.json
└── Summary: X successful, Y failed

detect_with_multiple_calibrations.py
├── Loads all calibration files
├── For each camera (sequential or threaded):
│   ├── Opens VideoCapture(camera_index)
│   ├── Loads its specific calibration
│   ├── Detects markers in live feed
│   ├── Calculates distance using cv2.solvePnP()
│   └── Displays results
└── Press 'q' to quit any window
```

### Key Features
- **Sequential Calibration**: One camera at a time
- **Sequential Detection**: Process cameras one at a time (useful for debugging)
- **Parallel Detection**: Process all cameras simultaneously (production mode)
- **Individual Calibrations**: Each camera has its own K matrix and distortion coefficients
- **Per-Camera Distance**: Each camera measures distance to detected tags independently

---

## 📈 Workflow Example: 5-Camera Rover

### Hardware Setup
- 5 USB cameras connected to Raspberry Pi or PC
- Device indices: `/dev/video0`, `/dev/video1`, ... `/dev/video4`
- Single ChArUco board for calibration

### Initial Calibration (Once)
```bash
# Takes ~3 minutes total (30s × 5 cameras)
python calibrate_multiple_cameras.py --num-cameras 5 --output-dir /config/camera_cals

# Check output
ls /config/camera_cals/
# camera_0_calibration.json
# camera_1_calibration.json
# ... etc
```

### Deployment
```bash
# During rover operation, load and use all calibrations
python detect_with_multiple_calibrations.py \
  --calibration-dir /config/camera_cals \
  --tag-size 18 \
  --no-display  # No GUI on headless rover
```

### Real-Time Results
```
2025-10-21 12:34:56 - MultiCameraDetector - ✅ Loaded calibration for camera 0
2025-10-21 12:34:56 - MultiCameraDetector - ✅ Loaded calibration for camera 1
...
2025-10-21 12:34:57 - MultiCameraDetector - 🎬 Processing 5 cameras...
2025-10-21 12:34:57 - MultiCameraDetector - 📷 Starting detection on camera 0...
2025-10-21 12:34:58 - MultiCameraDetector - 📷 Starting detection on camera 1...
...
```

---

## ⚠️ Troubleshooting

### Issue: "Cannot open camera X"
**Cause**: Camera not connected or already in use by another program

**Solution**:
```bash
# Check connected cameras (Linux)
ls /dev/video*

# Test single camera first
python detect_with_multiple_calibrations.py --calibration-dir ./camera_calibrations --camera 0

# Kill any competing process
pkill -f "python.*video"
```

### Issue: Calibration file not found
**Cause**: Wrong path or calibration not run yet

**Solution**:
```bash
# Check files exist
ls -la camera_calibrations/

# Re-run calibration if missing
python calibrate_multiple_cameras.py --num-cameras 5 --output-dir ./camera_calibrations
```

### Issue: Distance values unrealistic
**Cause**: Wrong tag size or poor calibration

**Solution**:
```bash
# Verify tag size (must be in millimeters)
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18  # MUST be 18mm for your tags

# Re-calibrate with more frames
python calibrate_multiple_cameras.py --num-cameras 5 --duration 60
```

### Issue: Some cameras fail calibration
**Cause**: Insufficient marker visibility during calibration

**Solution**:
```bash
# Re-calibrate that camera only
python calibrate_multiple_cameras.py \
  --num-cameras 1 \
  --start-camera 2 \
  --duration 60  # Increase time

# Move board more during capture
# Vary angles and distances
```

---

## 📋 Calibration File Format

### Single Camera Calibration
```json
{
  "camera_index": 0,
  "camera_matrix": {
    "rows": 3,
    "cols": 3,
    "data": [
      669.13, 0.0, 644.66,
      0.0, 668.50, 357.35,
      0.0, 0.0, 1.0
    ]
  },
  "distortion_coefficients": {
    "rows": 1,
    "cols": 5,
    "data": [2.54, 6.38, -0.018, -0.226, -249.18]
  },
  "image_width": 1280,
  "image_height": 720,
  "frames_used": 15,
  "board_size": "7x5",
  "calibration_quality": "good"
}
```

### Using Calibrations in Code
```python
import json
import numpy as np

with open('camera_0_calibration.json') as f:
    calib = json.load(f)

camera_matrix = np.array(calib['camera_matrix']['data']).reshape(3, 3)
dist_coeffs = np.array(calib['distortion_coefficients']['data']).flatten()

# Use for solvePnP
success, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, camera_matrix, dist_coeffs)
```

---

## ✅ Checklist

- [ ] 5 USB cameras connected
- [ ] ChArUco board printed (7×5, 30mm×18mm)
- [ ] Board well-lit and visible
- [ ] Board mounted on stable surface
- [ ] Ran `calibrate_multiple_cameras.py` (5 successful)
- [ ] Calibration files in `./camera_calibrations/`
- [ ] Each camera can open independently (`--camera 0`, `--camera 1`, etc.)
- [ ] Distance values look realistic (50-200mm for typical setup)
- [ ] Parallel detection works (`python detect_with_multiple_calibrations.py`)
- [ ] Sequential detection works (`--sequential` flag)

---

## 🚀 Next Steps

1. **Integrate into ROS2**: Subscribe to distance data from all cameras
2. **Fusion**: Combine distance measurements from multiple views
3. **Tracking**: Correlate same tags across camera views
4. **Performance**: Monitor FPS and latency per camera
5. **Deployment**: Transfer calibrations to production hardware

---

**Status**: ✅ Ready for multi-camera deployment  
**Last Updated**: 2025-10-21  
**Board**: 7×5 ChArUco (30mm×18mm)  
**Cameras**: 5  
**Use Case**: URC 2026 Rover Multi-View ArUco Detection
