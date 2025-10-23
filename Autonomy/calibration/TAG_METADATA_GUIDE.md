# 🏷️ ArUco Tag Metadata & Per-Camera Tracking Guide

## Overview

Tag metadata allows you to track:
- **Per-camera tags**: Which tags are assigned to which cameras
- **Tag properties**: Size, location, orientation
- **Detection data**: Distance, confidence, timestamp
- **Calibration context**: Which camera calibration was used

---

## 1️⃣ Tag Metadata Structure

### Minimal Tag Metadata (Per-Camera)
```json
{
  "tag_id": 0,
  "camera_index": 0,
  "location": "front_left",
  "size_mm": 18,
  "purpose": "navigation"
}
```

### Complete Tag Metadata
```json
{
  "tag_id": 0,
  "camera_index": 0,
  "tag_properties": {
    "size_mm": 18,
    "location": "front_left",
    "mount_height_mm": 150,
    "orientation_degrees": 0,
    "visible_range_mm": [500, 3000]
  },
  "camera_properties": {
    "calibration_file": "camera_0_calibration.json",
    "focal_length": 669.13,
    "image_width": 1280,
    "image_height": 720
  },
  "detection_stats": {
    "last_detected": "2025-10-21T10:30:45Z",
    "detections_count": 1234,
    "avg_distance_mm": 1500,
    "avg_confidence": 0.92,
    "detection_success_rate": 0.98
  },
  "purpose": "navigation",
  "notes": "Front left corner of rover"
}
```

---

## 2️⃣ Per-Camera Tag Assignment

### Create Tag Configuration File

**File: `tag_assignments.json`**
```json
{
  "metadata": {
    "created": "2025-10-21",
    "board_type": "charuco_5x7",
    "total_cameras": 5,
    "board_size_mm": {"width": 210, "height": 150}
  },
  "cameras": {
    "0": {
      "name": "Front Left",
      "calibration_file": "camera_0_calibration.json",
      "position": "front_left",
      "tags": [
        {
          "tag_id": 35,
          "location": "target_front_left",
          "size_mm": 50,
          "purpose": "navigation",
          "distance_range_mm": [500, 3000]
        },
        {
          "tag_id": 0,
          "location": "board_marker_0",
          "size_mm": 18,
          "purpose": "calibration",
          "distance_range_mm": [200, 1000]
        }
      ]
    },
    "1": {
      "name": "Front Center",
      "calibration_file": "camera_1_calibration.json",
      "position": "front_center",
      "tags": [
        {
          "tag_id": 36,
          "location": "target_front_center",
          "size_mm": 50,
          "purpose": "navigation",
          "distance_range_mm": [500, 3000]
        }
      ]
    },
    "2": {
      "name": "Front Right",
      "calibration_file": "camera_2_calibration.json",
      "position": "front_right",
      "tags": [
        {
          "tag_id": 37,
          "location": "target_front_right",
          "size_mm": 50,
          "purpose": "navigation"
        }
      ]
    },
    "3": {
      "name": "Rear Left",
      "calibration_file": "camera_3_calibration.json",
      "position": "rear_left",
      "tags": [
        {
          "tag_id": 38,
          "location": "target_rear_left",
          "size_mm": 50,
          "purpose": "navigation"
        }
      ]
    },
    "4": {
      "name": "Rear Right",
      "calibration_file": "camera_4_calibration.json",
      "position": "rear_right",
      "tags": [
        {
          "tag_id": 39,
          "location": "target_rear_right",
          "size_mm": 50,
          "purpose": "navigation"
        }
      ]
    }
  },
  "tag_registry": {
    "0": {"name": "Calibration Board Marker 0", "type": "charuco_marker"},
    "1": {"name": "Calibration Board Marker 1", "type": "charuco_marker"},
    "35": {"name": "Front Left Navigation Tag", "type": "navigation", "size_mm": 50},
    "36": {"name": "Front Center Navigation Tag", "type": "navigation", "size_mm": 50},
    "37": {"name": "Front Right Navigation Tag", "type": "navigation", "size_mm": 50},
    "38": {"name": "Rear Left Navigation Tag", "type": "navigation", "size_mm": 50},
    "39": {"name": "Rear Right Navigation Tag", "type": "navigation", "size_mm": 50}
  }
}
```

---

## 3️⃣ Detection Metadata Recording

### Enhanced Detector with Metadata

Create `aruco_detector_with_metadata.py`:
```python
#!/usr/bin/env python3
import cv2
import json
import numpy as np
from datetime import datetime

class MetadataArucoDetector:
    """Detect ArUco tags and record metadata."""
    
    def __init__(self, tag_config_file, camera_index=0):
        """Initialize detector with tag configuration."""
        self.camera_index = camera_index
        
        # Load tag assignments
        with open(tag_config_file) as f:
            self.config = json.load(f)
        
        # Get camera-specific tags
        self.camera_config = self.config['cameras'].get(str(camera_index), {})
        self.assigned_tags = {tag['tag_id']: tag for tag in self.camera_config.get('tags', [])}
        
        # Setup detector
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, cv2.aruco.DetectorParameters())
        
        # Detection history
        self.detection_history = {}
    
    def detect_with_metadata(self, frame, tag_size_mm=18):
        """Detect tags and record metadata."""
        corners, ids, rejected = self.detector.detectMarkers(frame)
        detections = []
        
        if ids is not None:
            for corner, tag_id in zip(corners, ids):
                tag_id = int(tag_id)
                
                # Get assigned metadata for this tag (if exists)
                tag_meta = self.assigned_tags.get(tag_id, {})
                
                # Calculate distance
                distance_mm = self._calculate_distance(corner[0], tag_size_mm)
                
                detection = {
                    "tag_id": tag_id,
                    "camera_index": self.camera_index,
                    "timestamp": datetime.now().isoformat(),
                    "distance_mm": distance_mm,
                    "confidence": 0.95,  # Calculate from reprojection error
                    "location": tag_meta.get("location", "unknown"),
                    "purpose": tag_meta.get("purpose", "unknown"),
                    "frame_position": {
                        "center_x": int(np.mean(corner[:, 0])),
                        "center_y": int(np.mean(corner[:, 1]))
                    }
                }
                detections.append(detection)
                
                # Update detection history
                if tag_id not in self.detection_history:
                    self.detection_history[tag_id] = []
                self.detection_history[tag_id].append(detection)
        
        return detections
    
    def _calculate_distance(self, corners, tag_size_mm):
        """Calculate distance from camera to tag."""
        # Implementation similar to aruco_validator.py
        return 0.0  # Placeholder
    
    def get_detection_summary(self):
        """Get summary of all detections."""
        summary = {}
        for tag_id, detections in self.detection_history.items():
            summary[tag_id] = {
                "count": len(detections),
                "avg_distance_mm": np.mean([d['distance_mm'] for d in detections]),
                "success_rate": len(detections) / 100,  # Adjust based on frame count
                "last_detection": detections[-1]['timestamp']
            }
        return summary
```

---

## 4️⃣ Usage Examples

### Example 1: Detect and Log Per-Camera Tags

```python
from aruco_detector_with_metadata import MetadataArucoDetector

# Initialize detector with tag configuration
detector = MetadataArucoDetector("tag_assignments.json", camera_index=0)

# Open camera
cap = cv2.VideoCapture(0)

while True:
    ret, frame = cap.read()
    if not ret:
        break
    
    # Detect tags with metadata
    detections = detector.detect_with_metadata(frame, tag_size_mm=18)
    
    # Log detections
    for detection in detections:
        print(f"Camera {detection['camera_index']}: "
              f"Tag {detection['tag_id']} "
              f"@ {detection['distance_mm']:.1f}mm "
              f"({detection['location']})")
    
    cv2.imshow('Tagged Detection', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Print summary
print("\n📊 Detection Summary:")
print(json.dumps(detector.get_detection_summary(), indent=2))
```

### Example 2: Filter Detections by Purpose

```python
# Get only navigation tags for this camera
navigation_tags = [
    t for t in detector.assigned_tags.values()
    if t.get('purpose') == 'navigation'
]

for tag_config in navigation_tags:
    if tag_config['tag_id'] in detector.detection_history:
        detections = detector.detection_history[tag_config['tag_id']]
        print(f"Navigation Tag {tag_config['tag_id']}: "
              f"{len(detections)} detections, "
              f"avg distance {np.mean([d['distance_mm'] for d in detections]):.1f}mm")
```

### Example 3: Export Detection Data

```python
import json

# Export all detection metadata to CSV/JSON
detection_export = {
    "camera_index": 0,
    "timestamp_start": datetime.now().isoformat(),
    "detections": []
}

for tag_id, detections in detector.detection_history.items():
    for detection in detections:
        detection_export["detections"].append(detection)

# Save to file
with open("detections_camera_0.json", "w") as f:
    json.dump(detection_export, f, indent=2)

print(f"Exported {len(detection_export['detections'])} detections")
```

---

## 5️⃣ Per-Camera Tag Tracking Sheet

Create and print this tracking sheet:

```
CAMERA ASSIGNMENTS - URC 2026
════════════════════════════════════════════════════════════════

Camera 0: Front Left
├─ Calibration: camera_0_calibration.json
├─ Navigation Tags:
│  └─ Tag 35 (50mm) - Front left corner
└─ Calibration Tags:
   └─ Tags 0-34 (18mm) - ChArUco board markers

Camera 1: Front Center
├─ Calibration: camera_1_calibration.json
├─ Navigation Tags:
│  └─ Tag 36 (50mm) - Front center
└─ Calibration Tags:
   └─ Tags 0-34 (18mm) - ChArUco board markers

Camera 2: Front Right
├─ Calibration: camera_2_calibration.json
├─ Navigation Tags:
│  └─ Tag 37 (50mm) - Front right corner
└─ Calibration Tags:
   └─ Tags 0-34 (18mm) - ChArUco board markers

Camera 3: Rear Left
├─ Calibration: camera_3_calibration.json
├─ Navigation Tags:
│  └─ Tag 38 (50mm) - Rear left corner
└─ Calibration Tags:
   └─ Tags 0-34 (18mm) - ChArUco board markers

Camera 4: Rear Right
├─ Calibration: camera_4_calibration.json
├─ Navigation Tags:
│  └─ Tag 39 (50mm) - Rear right corner
└─ Calibration Tags:
   └─ Tags 0-34 (18mm) - ChArUco board markers

════════════════════════════════════════════════════════════════
```

---

## 6️⃣ Quick Reference: Metadata Fields

| Field | Type | Example | Purpose |
|-------|------|---------|---------|
| `tag_id` | int | 0 | Unique marker ID |
| `camera_index` | int | 0 | Which camera sees this tag |
| `distance_mm` | float | 1500.5 | Distance from camera |
| `confidence` | float | 0.92 | Detection confidence (0-1) |
| `timestamp` | string | "2025-10-21T10:30:45Z" | When detected |
| `location` | string | "front_left" | Physical location |
| `purpose` | string | "navigation" | calibration/navigation/landmark |
| `size_mm` | int | 18 | Tag size |
| `calibration_file` | string | "camera_0_calibration.json" | Which calibration to use |

---

## 7️⃣ Implementation Steps

### Step 1: Create Tag Configuration
```bash
# Create tag_assignments.json in calibration directory
vim tag_assignments.json
```

### Step 2: Implement Metadata Detector
```bash
# Create enhanced detector
cp aruco_validator.py aruco_detector_with_metadata.py
# Modify to include metadata tracking
```

### Step 3: Test Per-Camera Tracking
```python
detector = MetadataArucoDetector("tag_assignments.json", camera_index=0)
# Run detection and verify metadata
```

### Step 4: Integrate Into Main System
```python
# Modify detect_with_multiple_calibrations.py to use metadata
# Each camera loads its tag configuration
# Records detection metadata for analysis
```

---

## 📚 See Also
- `camera/QUICK_START.md` - Camera calibration
- `aruco_tags/QUICK_REFERENCE.md` - Tag generation
- `MULTI_CAMERA_GUIDE.md` - Multi-camera setup
