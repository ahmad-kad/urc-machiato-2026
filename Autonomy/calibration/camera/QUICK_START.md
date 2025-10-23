# 📷 Camera Calibration - Quick Start

## Your Setup
- **Board**: 5×7 ChArUco (30mm squares, 18mm markers)
- **Time**: 30-60 seconds per camera
- **Output**: JSON calibration file

---

## ⚡ Quick Commands

### 1️⃣ Calibrate Single Camera
```bash
python calibrate_from_markers.py \
  --cols 7 --rows 5 \
  --square-size 0.030 --marker-size 0.018 \
  --output my_camera.json \
  --duration 30
```

**What to do:**
- Run command and wait for camera to open
- Move ChArUco board around for 30 seconds
- Board should fill 1/4 of the frame
- Different angles and distances are important
- Press `q` to stop early if you have enough frames

**Output:** `my_camera.json` with calibration parameters

---

### 2️⃣ Calibrate 5 Cameras (Sequential)
```bash
python calibrate_multiple_cameras.py \
  --num-cameras 5 \
  --cols 7 --rows 5 \
  --square-size 0.030 --marker-size 0.018 \
  --output-dir ./camera_calibrations \
  --duration 30
```

**What happens:**
1. Camera 0 calibrates (30 seconds)
2. Asks: "Prepare camera 1 and press ENTER"
3. Swap physical cameras and press ENTER
4. Repeat for cameras 1-4
5. All calibrations saved to `camera_calibrations/`

**Output:** 
```
camera_calibrations/
├── camera_0_calibration.json
├── camera_1_calibration.json
├── camera_2_calibration.json
├── camera_3_calibration.json
└── camera_4_calibration.json
```

---

### 3️⃣ Detect Tags & Measure Distance
```bash
python ../aruco_tags/aruco_validator.py \
  --calibration my_camera.json \
  --tag-size 18
```

**What it does:**
- Opens camera
- Detects ArUco tags in real-time
- Shows tag ID and distance in millimeters
- Displays FPS and statistics
- Press `q` to quit

**Example output on screen:**
```
ID:0 42.5mm ✓
ID:4 156.3mm ✓
```

---

## 📊 With Multiple Cameras

### Sequential Detection (One at a time)
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18 \
  --sequential
```

### Parallel Detection (All at once)
```bash
python detect_with_multiple_calibrations.py \
  --calibration-dir ./camera_calibrations \
  --tag-size 18
```
Opens 5 windows simultaneously, one per camera.

---

## ✅ Checklist

- [ ] Print 5×7 ChArUco board (30mm×18mm)
- [ ] Connect camera(s)
- [ ] Run calibration command
- [ ] Move board around for full 30 seconds
- [ ] Check output JSON file exists
- [ ] Test detection with aruco_validator.py
- [ ] Verify distance measurements look reasonable
- [ ] Save calibration files to safe location

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| "Cannot open camera" | Check USB connection, try `--camera 1` |
| "Need at least 5 frames" | Move board more, increase `--duration 60` |
| "Not detecting board" | Better lighting, focus camera, larger board |
| Distance values wrong | Check `--tag-size` is in millimeters |

---

## 📚 See Also
- Main guide: `../README.md`
- Multi-camera detailed: `../MULTI_CAMERA_GUIDE.md`
- ArUco reference: `../aruco_tags/QUICK_REFERENCE.md`
