# ArUco Sheet Generator - Quick Reference

## 📋 One-Line Commands

### Small Format (Standard Letter - 11" × 8.5")
```bash
# Default with all small format tags
python aruco_sheets.py

# Custom output name
python aruco_sheets.py -o my_tags.pdf

# Explicit small format
python aruco_sheets.py --format small
```

### Large Format (Roll Paper - 36" × 44")
```bash
# Default large format
python aruco_sheets.py --format large

# Custom output name
python aruco_sheets.py --format large -o navigation_posts.pdf
```

---

## 📐 Format Specifications At A Glance

| | Small | Large |
|---|---|---|
| **Paper** | 11" × 8.5" (Letter) | 36" × 44" (Roll) |
| **Margins** | 0.75" | 0.5" |
| **Usable** | 9.5" × 7.0" | 35" × 43" |
| **Layout** | Space-filling (no gaps) | Row-based (fills width) |
| **Tags** | 1-15cm | 10-20cm |
| **Templates** | ✅ 42×2cm, 90×1cm | ✅ 8×20cm, 12×15cm |
| **Pages** | 6 | 2 |
| **Overlay** | ✅ template_silhouette.png | — |
| **Printer** | Standard | Wide Format |
| **Density** | ✅ Maximum | ✅ Optimal |

---

## 🎯 Use Case Decision Tree

```
Need ArUco tags?
│
├─ Dense/detailed markers?
│  ├─ Keyboard layout → Small Format
│  ├─ USB corners → Small Format
│  └─ Mixed sizes → Small Format
│
├─ Large navigation posts?
│  ├─ Field landmarks → Large Format
│  ├─ Wide spread → Large Format
│  └─ >15cm tags → Large Format
│
└─ Both types → Generate both!
   python aruco_sheets.py --format small
   python aruco_sheets.py --format large
```

---

## 📏 Tag Sizes Per Row/Page

### Small Format (Letter - 9.5" usable width)
- **1cm tags**: ~24 per row
- **2cm tags**: ~12 per row
- **5cm tags**: ~4 per row
- **10cm tags**: 2 per page (1 per row, 1 per page)
- **15cm tags**: 1 per page

### Large Format (Roll - 35" usable width)
- **10cm tags**: ~14 per row
- **15cm tags**: ~9 per row
- **20cm tags**: ~7 per row

---

## ✅ Print Checklist

### Small Format
- [ ] Use standard printer (inkjet/laser)
- [ ] Print at **100% scale** (NOT "Fit to Page")
- [ ] Use matte paper if possible
- [ ] Overlay with `template_silhouette.png` to verify margins
- [ ] Measure test tag (2cm should be 0.79")
- [ ] Cut along gray lines
- [ ] Test ArUco detection before deployment

### Large Format
- [ ] Use wide format printer (36"+ roll support)
- [ ] Print at **100% scale** (do NOT scale)
- [ ] Use matte finish paper
- [ ] Verify roll feeds smoothly
- [ ] Cut tags along heavy cutlines
- [ ] Store rolled to prevent damage
- [ ] Test ArUco detection before deployment

---

## 🔧 Customizing Tags

Edit `main()` function in `aruco_sheets.py`:

### Small Format Template
```python
if args.format == 'small':
    layouts = [
        ("Your layout name", [
            (size_cm, count),  # e.g., (2, 50) for 50×2cm tags
            (size_cm, count),  # Add more sizes as needed
        ], True),  # True = landscape, False = portrait
    ]
```

### Large Format Template
```python
else:  # format == 'large'
    layouts = [
        ("Your layout name", [
            (size_cm, count),  # e.g., (20, 10) for 10×20cm tags
            (size_cm, count),  # Add more sizes
        ]),
    ]
```

### Example: Custom Mixed Tags
```python
layouts = [
    ("Keyboard layout", [(2, 50), (5, 20)], True),
    ("Navigation posts", [(15, 3)], True),
]
```

---

## 🎲 Common Configurations

### Scenario 1: Keyboard Markers
```bash
python aruco_sheets.py --format small -o keyboard.pdf
# Creates: 42×2cm + 90×1cm tags on letter paper
```

### Scenario 2: Large Field Markers
```bash
python aruco_sheets.py --format large -o field_markers.pdf
# Creates: 8×20cm + 12×15cm tags on roll paper
```

### Scenario 3: Complete System
```bash
python aruco_sheets.py --format small -o system_details.pdf
python aruco_sheets.py --format large -o system_landmarks.pdf
# Print both for comprehensive marking system
```

---

## ⚠️ Troubleshooting Quick Fixes

| Problem | Solution |
|---------|----------|
| Tags too small | Increase tag size (cm) or reduce count |
| Tags too crowded | Use larger paper format (roll) or reduce count |
| Outside margins | Verify 0.75" (small) or 0.5" (large) margins |
| Page too tall | Reduce tags per page or use fewer tag sizes |
| Printer won't accept | Check printer manual for paper size support |

---

## 📚 Full Documentation

See `ARUCO_GENERATION_GUIDE.md` for comprehensive details:
- Detailed specifications
- Tag size recommendations
- Printing best practices
- Advanced customization
- Workflow examples

---

## 🔄 Version Info

- **Script**: `aruco_sheets.py` (517 lines)
- **Version**: v2.0 (Dual Format - Small & Large)
- **Dictionary**: DICT_4X4_50 (50 unique markers)
- **DPI**: 300 (consistent across both formats)
- **Last Updated**: 2025-10-18

---

## 📞 File Locations

```
Autonomy/calibration/aruco_tags/
├── aruco_sheets.py                    # Main generator script
├── ARUCO_GENERATION_GUIDE.md          # Full documentation
├── ARUCO_QUICK_REFERENCE.md           # This file
├── template_silhouette.png            # Small format margin guide
├── debug_tag.png                      # Example tag
├── aruco_sheets.pdf                   # Default small format output
└── aruco_sheets_large.pdf             # Default large format output
```

---

**Quick Start**: `python aruco_sheets.py` ✨
