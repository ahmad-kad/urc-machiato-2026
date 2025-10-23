# 🎯 ArUco Tags - Quick Reference

## Your Tags
- **Dictionary**: DICT_4X4_50 (4×4 grid, 50 unique markers)
- **Size**: 18mm (0.018m)
- **Format**: PNG + PDF printable sheets

---

## ⚡ Quick Commands

### 1️⃣ Generate Single Tag
```bash
python generate_aruco_tag.py --tag-id 0 --size 100 --margin 10 --output tag_0.png
```

**Arguments:**
- `--tag-id`: ID number (0-49 for DICT_4X4_50)
- `--size`: Size in pixels (100-500 recommended)
- `--margin`: White border in pixels
- `--output`: Output PNG filename

**Output:** `tag_0.png` (printable image)

---

### 2️⃣ Generate Printable Sheet
```bash
python aruco_sheets.py --output aruco_sheet.pdf
```

**What it does:**
- Generates PDF with all 50 tags
- 8.5"×11" page size
- Tags are 18mm at 300 DPI
- Ready to print and cut

**Output:** `aruco_sheet.pdf`

---

### 3️⃣ Detect Tags & Measure Distance

#### Single Camera
```bash
python aruco_validator.py \
  --calibration ../camera/my_camera.json \
  --tag-size 18
```

#### Multiple Cameras
```bash
python aruco_validator.py \
  --calibration ../camera/camera_0_calibration.json \
  --tag-size 18 \
  --test-mode 30
```

**On-screen output:**
```
Tag ID: 0 | Distance: 42.5mm | Confidence: 0.85
Tag ID: 4 | Distance: 156.3mm | Confidence: 0.92
```

**Arguments:**
- `--calibration`: Path to camera calibration JSON
- `--tag-size`: Tag size in millimeters
- `--test-mode SECONDS`: Run for N seconds then exit
- `--no-display`: Headless mode (no GUI)

---

## 📋 Print Instructions

### Option 1: Single Tags (Recommended for Testing)
1. Run: `python generate_aruco_tag.py --tag-id 0 --size 200`
2. Print: `tag_0.png` at 18mm physical size
3. Laminate if available (prevents curling)
4. Mount on board for calibration

### Option 2: Full Sheet
1. Run: `python aruco_sheets.py --output full_sheet.pdf`
2. Print on 8.5"×11" paper
3. Cut out individual tags
4. Mount and label each tag

### Print Quality Tips
- Use **200 DPI minimum** for sharp tags
- Print on **matte paper** (less glare for camera)
- Ensure **full black** and **white** (adjust contrast if needed)
- **Mount flat** on stiff cardboard (prevents warping)
- **Label each tag** with its ID number

---

## 🎯 Tag IDs for Your Setup

Recommended assignments:
- **Board Markers**: IDs 0-34 (for 5×7 ChArUco board)
- **Calibration Targets**: IDs 35-40
- **Rover Navigation**: IDs 41-45
- **Reserved**: IDs 46-49

---

## 📊 Distance Accuracy

| Distance Range | Accuracy | Notes |
|---|---|---|
| 50mm - 500mm | ±5mm | Good for close-range detection |
| 500mm - 2000mm | ±10mm | Typical rover navigation |
| 2000mm+ | ±50mm | Long-range detection |

**Factors affecting accuracy:**
- Lighting (good lighting → better accuracy)
- Tag size (larger tags → better accuracy)
- Camera focus (ensure sharp image)
- Calibration quality (better calibration → better accuracy)

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| Tags not detected | Check lighting, focus, tag visibility |
| Poor distance accuracy | Recalibrate camera, check tag size parameter |
| Can't print to 18mm | Adjust print settings in PDF viewer |
| Tags too blurry | Ensure camera is focused, use larger tags |

---

## 📚 See Also
- Camera calibration: `../camera/QUICK_START.md`
- Main guide: `../README.md`
- Multi-camera setup: `../camera/MULTI_CAMERA_GUIDE.md`

---

## 💾 Tag ID Tracking Template

```
Tag ID | Location | Purpose | Size | Notes
-------|----------|---------|------|-------
   0   | Board    | Calibration | 18mm | 
   1   | Board    | Calibration | 18mm |
  35   | Rover    | Front left  | 50mm |
  36   | Rover    | Front right | 50mm |
  ...
```

Print this table and mark which tags are assigned where!
