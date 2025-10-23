# Your ChArUco Calibration Setup

## 📐 Board Specifications

Your ChArUco board configuration:
- **Square Size**: 30mm (0.030m)
- **Marker Size**: 18mm (0.018m)
- **Pattern**: 8×6 (8 columns × 6 rows)

---

## 🚀 Calibration Command

Run this exact command to calibrate with your board: (parameterized by square and marker size in meters)

```bash
cd Autonomy/calibration/camera/

python quick_calibration.py \
  --square-size 0.030 \
  --marker-size 0.018 \
  --board-cols 8 \
  --board-rows 6 \
  --duration 30 \
  --output my_camera.json
```

### What to expect:
1. **30 seconds** of capturing frames
2. Move your ChArUco board around slowly
3. Keep it visible in most frames
4. Avoid sudden jerky movements
5. Press 'q' to stop early, or wait for auto-completion

### Output:
```
✅ Calibration successful!
�� Calibration Results:
   Image size: 1280x720
   Frames used: 15+
   Camera Matrix: [...]
   Distortion Coefficients: [...]
💾 Calibration saved to: my_camera.json
```

---

## 🎯 Test Your Calibration

After calibration, immediately test with ArUco detection:

```bash
cd Autonomy/calibration/aruco_tags/

# Test distance detection with calibrated camera
python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size 10.0
```

---

## 📋 Full Workflow with Your Board

```bash
# Step 1: Navigate to camera directory
cd Autonomy/calibration/camera/

# Step 2: Calibrate (30mm squares, 18mm markers)
python quick_calibration.py \
  --square-size 0.030 \
  --marker-size 0.018 \
  --board-cols 8 \
  --board-rows 6 \
  --duration 30 \
  --output my_camera.json

# Step 3: Check calibration was successful
# Look for "Calibration successful!" message

# Step 4: Test ArUco detection
cd ../aruco_tags/

python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size 10.0

# Step 5: Try detecting tags
# Hold printed ArUco tags in front of webcam
# Press 'q' to quit

# Step 6: For production/Raspberry Pi
python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size 10.0 \
  --no-display
```

---

## 📊 Quick Reference

| Parameter | Your Value | Description |
|-----------|-----------|-------------|
| Square Size | 30mm (0.030) | Size of each square in your board |
| Marker Size | 18mm (0.018) | Size of ArUco marker within each square |
| Board Pattern | 8×6 | 8 columns, 6 rows |
| Duration | 30s | How long to capture (increase to 60s for better results) |

---

## 🔧 Optional: Extended Calibration for Better Accuracy

For competition-grade calibration, use 60 seconds:

```bash
python quick_calibration.py \
  --square-size 0.030 \
  --marker-size 0.018 \
  --board-cols 8 \
  --board-rows 6 \
  --duration 60 \
  --output my_camera_calibrated.json
```

This captures more frames from different angles for higher accuracy.

---

## 📁 Output Files

After calibration, you'll have:

- `my_camera.json` - Your calibrated camera parameters
- `my_camera.yaml` - YAML format (compatibility)
- `calibration_workflow_summary.json` - Detailed calibration report

---

## ✅ Validation Checklist

- [ ] ChArUco board properly sized (30mm × 18mm)
- [ ] Board is flat and well-lit
- [ ] Calibration captures 15+ valid frames
- [ ] `my_camera.json` file created
- [ ] ArUco detection works with calibrated camera
- [ ] Distance measurements seem reasonable

---

## 🎓 Notes

- Your 30mm/18mm board is larger than standard (usually 20mm/15mm)
- This requires slightly different calibration parameters
- Distance measurements will be more accurate with proper calibration
- Print ArUco tags at exact sizes specified for distance to be accurate

---

**🎉 You're ready to calibrate and detect with your custom board!**
