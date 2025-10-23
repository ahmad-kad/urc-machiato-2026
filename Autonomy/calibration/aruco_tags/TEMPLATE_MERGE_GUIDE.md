# ArUco Template Merge Guide

## Overview

This guide explains how to merge ArUco tags with the `template_silhouette.png` template image to create registration marks, and how to export individual PNG tags for custom compositing.

## Three Generation Modes

### 1. Standard PDF Sheets (Default)
```bash
python aruco_sheets.py --format small          # 6-page letter format
python aruco_sheets.py --format large          # 2-page roll format
```
**Output**: Multi-page PDFs with tags arranged in grids/rows

---

### 2. Template Merge PDF (NEW!)
```bash
python aruco_sheets.py --template
python aruco_sheets.py --template -o custom_merged.pdf
```

**Output**: Single PDF with ArUco tags overlaid on `template_silhouette.png`

**Use Cases**:
- Create registration marks directly on the template
- Single combined document for printing
- Alignment verification with template outline
- Unified reference sheet

**Features**:
- ✅ ArUco tags as fiducial markers
- ✅ Template outline preserved underneath
- ✅ 0.75" safe margins respected
- ✅ ID labels on each tag
- ✅ Metadata footer with generation info

**Example Output**: `aruco_template_merged.pdf`

---

### 3. Individual PNG Export (NEW!)
```bash
python aruco_sheets.py --png
python aruco_sheets.py --png -o /path/to/output
```

**Output**: Individual PNG files for each tag in `aruco_tags_png/` directory

**File Structure**:
```
aruco_tags_png/
├── aruco_id_000_1cm.png      # ID 0, 1cm tag
├── aruco_id_000_2cm.png      # ID 0, 2cm tag
├── aruco_id_001_1cm.png      # ID 1, 1cm tag
├── aruco_id_001_2cm.png      # ID 1, 2cm tag
└── ... (132 total PNG files)
```

**Use Cases**:
- Custom image compositing
- Selective placement on documents
- Web/digital applications
- Advanced layout design
- Layering with other graphics

**Features**:
- ✅ Individual PNG files (transparent background option)
- ✅ White border for visibility
- ✅ ID and size labels
- ✅ 300 DPI resolution
- ✅ Ready for layering in design software

---

## Workflow Examples

### Workflow 1: Registration Marks on Template
```bash
# Generate merged template with ArUco registration marks
python aruco_sheets.py --template -o calibration_template.pdf

# Print the PDF
# Use with camera for automated alignment detection
# Overlay printed output on physical template
```

**Benefit**: Automatic fiducial detection for camera calibration

---

### Workflow 2: Custom Document Compositing
```bash
# Export individual PNG tags
python aruco_sheets.py --png

# In design software (GIMP, Photoshop, etc.):
# 1. Open template_silhouette.png
# 2. Add ArUco PNGs as layers
# 3. Position as needed
# 4. Export as PDF or image

# Or programmatically:
# from PIL import Image
# template = Image.open("template_silhouette.png")
# tag = Image.open("aruco_tags_png/aruco_id_000_2cm.png")
# template.paste(tag, (x, y))
# template.save("custom_merged.pdf")
```

**Benefit**: Maximum flexibility for custom layouts

---

### Workflow 3: Multi-Format Complete System
```bash
# Generate all formats
python aruco_sheets.py --format small -o details.pdf
python aruco_sheets.py --format large -o landmarks.pdf
python aruco_sheets.py --template -o calibration.pdf
python aruco_sheets.py --png

# Results:
# - details.pdf: Small tags for dense marking (6 pages)
# - landmarks.pdf: Large tags for navigation (2 pages)
# - calibration.pdf: Template with registration marks (1 page)
# - aruco_tags_png/: Individual tags for custom use (132 files)
```

**Benefit**: Complete toolkit for all use cases

---

## Specifications

### Template Merge PDF

| Property | Value |
|----------|-------|
| **Page Size** | 11" × 8.5" (US Letter) |
| **Resolution** | 300 DPI |
| **Base Image** | template_silhouette.png |
| **Overlaid Tags** | 42×2cm + 90×1cm (132 total) |
| **Margins** | 0.75" (respects template boundaries) |
| **Layout** | Space-filling (no centering gaps) |
| **File Size** | ~350 KB |
| **Pages** | 1 |

### Individual PNG Files

| Property | Value |
|----------|-------|
| **Count** | 132 files (42×2cm + 90×1cm) |
| **Filename** | `aruco_id_XXX_YYcm.png` |
| **Resolution** | 300 DPI |
| **Image Type** | RGB PNG with white background |
| **Border** | 10px white margin around tag |
| **Labels** | ID and size (bottom of image) |
| **Directory** | `aruco_tags_png/` |

---

## Technical Implementation

### Template Merge Process

```python
# 1. Load template image
template_img = Image.open("template_silhouette.png")
canvas = np.array(template_img)

# 2. Generate ArUco tags
aruco_dict = cv2.aruco.getPredefinedDictionary(DICT_4X4_50)
tag_img = cv2.aruco.generateImageMarker(aruco_dict, id, size_px)

# 3. Overlay on template
canvas[y:y+size, x:x+size] = tag_img

# 4. Save as PDF
Image.fromarray(canvas).save("output.pdf", format="PDF", dpi=(300, 300))
```

### PNG Export Process

```python
# 1. For each tag:
tag_img_gray = cv2.aruco.generateImageMarker(aruco_dict, id, size_px)
tag_img_rgb = cv2.cvtColor(tag_img_gray, cv2.COLOR_GRAY2RGB)

# 2. Add white border
canvas = Image.new("RGB", (size_px + 20, size_px + 20), "white")
canvas.paste(Image.fromarray(tag_img_rgb), (10, 10))

# 3. Add labels
draw.text((10, size_px + 15), f"ID:{id}\n{size_cm}cm")

# 4. Save PNG
canvas.save(f"aruco_id_{id:03d}_{size_cm}cm.png", dpi=(300, 300))
```

---

## Usage Tips

### Template Merge

1. **Printing**:
   - Print at 100% scale
   - Use matte paper for reduced glare
   - Verify margins match printer capabilities

2. **Alignment**:
   - Template outline serves as visual guide
   - ArUco tags act as registration marks
   - Use for camera calibration

3. **Overlay**:
   - Print on transparent film and overlay on physical template
   - Use for precise alignment verification

### PNG Export

1. **Selection**:
   - Browse `aruco_tags_png/` for specific tag IDs
   - Use naming convention to find desired sizes
   - Filename format: `aruco_id_XXX_YYcm.png`

2. **Design Software**:
   - Open PNGs in GIMP, Photoshop, etc.
   - Drag to template as new layer
   - Adjust opacity and position as needed

3. **Programmatic Use**:
   - Load PNG with PIL: `Image.open("aruco_id_000_2cm.png")`
   - Combine with template using numpy/PIL
   - Export to PDF or image format

---

## Command Reference

```bash
# PDF Sheets (6 pages, letter format)
python aruco_sheets.py
python aruco_sheets.py --format small

# PDF Sheets (2 pages, roll format)
python aruco_sheets.py --format large

# Template Merge (single PDF with overlaid tags)
python aruco_sheets.py --template
python aruco_sheets.py --template -o my_calibration.pdf

# PNG Export (individual tag files)
python aruco_sheets.py --png
python aruco_sheets.py --png -o /custom/path

# Custom combinations
python aruco_sheets.py --format small -o letter.pdf
python aruco_sheets.py --template -o template.pdf
python aruco_sheets.py --png
```

---

## Output Files

### Template Merge
- ✅ `aruco_template_merged.pdf` (default)
- ✅ Custom filename with `-o` flag

### PNG Export
- ✅ `aruco_tags_png/` directory (default)
- ✅ 132 individual PNG files
- ✅ Organized by ID and size

---

## FAQ

**Q: Can I use the template merge for camera calibration?**
A: Yes! The ArUco tags can serve as fiducial markers for automatic detection and alignment.

**Q: Can I select which tags to include?**
A: Currently, template merge includes all small tags. Edit the `tag_specs` in the `main()` function for custom selection.

**Q: Can I change the overlay position?**
A: The PNG export gives you full control. Compose in your design software or programmatically.

**Q: Will the PNGs work with image compositing software?**
A: Yes! They're standard RGB PNGs with white backgrounds. Compatible with GIMP, Photoshop, Inkscape, etc.

**Q: Can I add more tags or change sizes?**
A: Edit the `tag_specs` in the relevant generation function to customize.

---

## References

- **Script**: `aruco_sheets.py`
- **Template**: `template_silhouette.png`
- **Main Guide**: `ARUCO_GENERATION_GUIDE.md`
- **Quick Reference**: `ARUCO_QUICK_REFERENCE.md`

---

**Generated**: 2025-10-18  
**Version**: ArUco Sheets v2.2 (Template Merge + PNG Export)
