# ArUco Tag Sheet Generation Guide

## Overview

This guide explains how to generate ArUco tag sheets in two separate formats:
1. **Small Format**: Standard US Letter (11" × 8.5") for detailed/dense markers
2. **Large Format**: Roll paper (36" × 44") for large navigation posts

## Format Comparison

| Feature | Small Format | Large Format |
|---------|-------------|--------------|
| **Paper Size** | 11" × 8.5" (Letter) | 36" × 44" (Roll) |
| **Resolution** | 3301 × 2550 px @ 300 DPI | 10800 × 13200 px @ 300 DPI |
| **Safe Margins** | 0.75" all sides | 0.5" all sides |
| **Usable Area** | 9.5" × 7.0" | 35" × 43" |
| **Tag Layout** | Multi-row grid | Single row (fills width) |
| **Use Cases** | Keyboards, USB markers, details | Navigation posts, landmarks |
| **Tag Sizes** | 1-15cm | 10-20cm |
| **Overlay** | ✅ template_silhouette.png | N/A |
| **Printer** | Standard/inkjet | Wide format/roll printer |

---

## Small Format (Standard Letter - 11" × 8.5")

### Constraints & Specifications

Based on **template_silhouette.png**, the safe printing area is:

| Constraint | Value |
|-----------|-------|
| **Paper Size** | US Letter (11" × 8.5") |
| **Canvas Resolution** | 3301 × 2550 pixels @ 300 DPI |
| **Safe Margins** | 0.75" on all sides |
| **Safe Margins (px)** | 225px on all sides @ 300 DPI |
| **Usable Printable Area** | 9.5" × 7.0" (2851 × 2100px) |
| **Layout** | Space-filling (tags fill area efficiently, no centering gaps) |

The corner markers in **template_silhouette.png** indicate the safe boundaries.

### Layout Strategy

The small format now uses **space-filling layout**:
- Tags start from the top-left margin
- Multiple tags per row (fills width completely)
- No centering gaps - maximizes use of available area
- Minimal padding between tags (0.15")
- Sections stack vertically with small gaps

This means you get **maximum density** of tags while staying within safe margins!

### Tag Size Recommendations

#### Small Tags (1-2cm)
- **Use case**: Keyboard layout markers, USB port corners, dense marking
- **Quantity per page**: 42× (2cm) to 90× (1cm tags)
- **Layout**: Multiple rows and columns, well-fitted within printable area
- **Status**: ✅ Fully supported, no warnings

#### Medium Tags (5-10cm)
- **Use case**: Navigation landmark arrays, moderate environment coverage
- **Quantity per page**: 2-4 tags
- **Layout**: Centered with comfortable spacing
- **Status**: ✅ Fully supported, no warnings

#### Large Tags (15cm+)
- **Use case**: Primary navigation posts within letter paper
- **Quantity per page**: 1 tag per page (maximum)
- **Layout**: Single centered tag
- **Status**: ✅ Supported with 1 tag/page

### Default Small Format Layout

The default small format generates:

1. **Page 1**: 42× 2cm tags (keyboard USB corners + extras)
2. **Page 2**: 90× 1cm tags (fine detail markers)
3. **Pages 3-4**: 4× 10cm tags (2 per page, navigation markers)
4. **Pages 5-6**: 2× 15cm tags (1 per page, large landmarks)

### Generate Small Format

#### Default (all small format tags)
```bash
python aruco_sheets.py --format small
# Creates: aruco_sheets.pdf (6 pages)
```

#### Custom output filename
```bash
python aruco_sheets.py --format small -o my_small_tags.pdf
```

#### Or without specifying format (default is small)
```bash
python aruco_sheets.py
# Creates: aruco_sheets.pdf
```

---

## Large Format (Roll Paper - 36" × 44")

### Constraints & Specifications

Optimized for wide format printers using roll paper:

| Constraint | Value |
|-----------|-------|
| **Paper Size** | 36" × 44" (roll) |
| **Canvas Resolution** | 10800 × 13200 pixels @ 300 DPI |
| **Margins** | 0.5" on all sides |
| **Margins (px)** | 150px on all sides @ 300 DPI |
| **Usable Area** | 35" × 43" (10500 × 12900px) |
| **Layout Strategy** | Tags arranged in efficient rows (fills width) |

### Tag Size Recommendations

#### Medium-Large Tags (10cm)
- **Quantity per row**: ~14 tags (10cm + 0.3" spacing × 14 ≈ 35")
- **Use case**: Field markers, moderate visibility
- **Layout**: Multiple rows vertically
- **Status**: ✅ Optimal for wide roll

#### Large Tags (15cm)
- **Quantity per row**: ~9 tags (15cm + 0.3" spacing × 9 ≈ 35")
- **Use case**: Navigation landmarks, good visibility
- **Layout**: Multiple rows vertically
- **Status**: ✅ Fully supported

#### Extra Large Tags (20cm)
- **Quantity per row**: ~7 tags (20cm + 0.3" spacing × 7 ≈ 35")
- **Use case**: Primary navigation posts, maximum visibility
- **Layout**: Multiple rows vertically
- **Status**: ✅ Optimal for large format

### Default Large Format Layout

The default large format generates:

1. **Page 1**: 8× 20cm tags (large navigation posts, ~1-2 rows)
2. **Page 2**: 12× 15cm tags (medium-large navigation posts, ~2 rows)

### Generate Large Format

#### Default (all large format tags)
```bash
python aruco_sheets.py --format large
# Creates: aruco_sheets_large.pdf (2 pages)
```

#### Custom output filename
```bash
python aruco_sheets.py --format large -o navigation_markers.pdf
```

---

## How the Script Respects Constraints

### Small Format (Letter Paper)

```python
# 1. Enforces 0.75" margins on all sides
PRINT_MARGIN_INCHES = 0.75
margin = int(PRINT_MARGIN_INCHES * dpi)  # 225px @ 300 DPI

# 2. Calculates usable printable area
usable_width_px = page_width_px - 2 * margin   # 2851px
usable_height_px = page_height_px - 2 * margin # 2100px

# 3. Validates that tags fit within usable area
if tag_size_px > usable_width_px or tag_size_px > usable_height_px:
    print("⚠️ WARNING: Tag too large for printable area!")
    
# 4. Centers tag layout within printable area
start_x = margin + (usable_width_px - layout_width) // 2
start_y = margin + (usable_height_px - layout_height) // 2

# 5. Prevents tags from exceeding boundaries
if y + tag_size_px > page_height_px - margin:
    print("⚠️ WARNING: Exceeding vertical boundary!")
```

### Large Format (Roll Paper)

```python
# 1. Uses minimal margins for roll paper
PRINT_MARGIN_INCHES = 0.5
margin = int(PRINT_MARGIN_INCHES * dpi)  # 150px @ 300 DPI

# 2. Arranges tags in efficient rows
tags_per_row = max(1, usable_width_px // (tag_size_px + tag_spacing))

# 3. Centers rows horizontally
start_x = margin + (usable_width_px - layout_width) // 2

# 4. Stacks rows vertically with appropriate spacing
current_y += total_rows_needed * (tag_size_px + tag_spacing) + 0.5"
```

---

## Customizing Tag Specifications

### Small Format - Custom Tags

Edit the `layouts` list in `main()` function:

```python
if args.format == 'small':
    layouts = [
        ("Custom tags", [
            (2, 50),      # 50× 2cm tags
            (5, 20),      # 20× 5cm tags
            (15, 1),      # 1× 15cm tag
        ], True),
    ]
```

Format: `(size_cm, count)`
- `size_cm`: Tag size in centimeters
- `count`: Number of tags to generate

### Large Format - Custom Tags

```python
else:  # format == 'large'
    layouts = [
        ("Custom large tags", [
            (20, 10),     # 10× 20cm tags
            (15, 15),     # 15× 15cm tags
        ]),
    ]
```

Format: `(size_cm, count)`
- Tags automatically arranged in optimal rows for 36" width

---

## Printing Best Practices

### Small Format (Letter)

1. ✅ **Use standard printer**: Inkjet or laser printer
2. ✅ **Print settings**:
   - Scale: 100% (NOT "Fit to Page")
   - Color: Black & White (or Color if preferred)
   - Paper: Matte or glossy (avoid gloss to reduce glare)
3. ✅ **Overlay**: Place printed sheet under template_silhouette.png to verify alignment
4. ✅ **Cut along gray cutlines** for easy assembly
5. ✅ **Verify calibration**: Measure printed tag (e.g., 2cm tag should be 0.79")

### Large Format (Roll Paper)

1. ✅ **Use wide format printer**: Supports 36" roll
2. ✅ **Print settings**:
   - Scale: 100% (do NOT scale)
   - Color: Black & White recommended
   - Paper: Matte finish preferred
3. ✅ **Roll handling**: Ensure paper feeds smoothly
4. ✅ **Cut along heavy cutlines** for separated tags
5. ✅ **Storage**: Roll tightly to prevent damage

---

## Troubleshooting

### Small Format Issues

**Issue**: Tags appear outside the margins
- **Solution**: Verify template_silhouette.png constraints (0.75" margins)
- **Check**: Paper is US Letter (11" × 8.5"), not A4

**Issue**: PDF doesn't match template_silhouette.png
- **Solution**: Verify printer supports true 300 DPI
- **Action**: Print test page, overlay with template

**Issue**: Too many tags squeezed together
- **Solution**: Reduce count or increase tag size
- **Alternative**: Use smaller tags for density

### Large Format Issues

**Issue**: Tags don't fill row evenly
- **Solution**: Adjust count to match row capacity
- **Formula**: For 36" width with tag size X: count = (36" / X) - 1

**Issue**: Page height exceeded
- **Solution**: Reduce total tag count per page
- **Recommendation**: Use 1 layout per page for large tags (20cm)

**Issue**: Printer won't accept roll paper
- **Solution**: Verify printer manual for roll setup
- **Alternative**: Use 11"×36" paper instead of 36"×44"

---

## Comparing Formats

### When to Use Small Format

✅ Desktop environment markers
✅ Keyboard/USB layout mapping
✅ Precise/dense fiducial arrays
✅ Mixed tag sizes (1-15cm)
✅ Standard printer available
✅ Limited space/budget

### When to Use Large Format

✅ Large outdoor environments
✅ Wide area navigation landmarks
✅ Large tag sizes (10-20cm)
✅ Multiple replicas needed
✅ Roll paper printer available
✅ Uniform spacing important

---

## References

- **Small Format Template**: `template_silhouette.png` - Visual guide of safe printable area
- **ChArUco Boards**: `charuco_board/generate_chessboard.py` - For camera calibration
- **Debug Tag Example**: `debug_tag.png` - Reference for correct tag placement
- **ArUco Dictionary**: DICT_4X4_50 (50 unique 4×4 bit markers)

---

## Example Workflows

### Workflow 1: Keyboard Markers (Small Format)

```bash
# Generate small format with keyboard markers
python aruco_sheets.py --format small -o keyboard_markers.pdf

# Print on standard 8.5"×11" paper
# Overlay with template_silhouette.png to verify margins
# Cut along gray lines
# Place markers in keyboard layout
```

### Workflow 2: Navigation Posts (Large Format)

```bash
# Generate large format with 20cm tags
python aruco_sheets.py --format large -o navigation_posts.pdf

# Print on 36"×44" roll paper
# Cut individual tags
# Mount on stakes in field environment
# Verify ArUco detection with camera
```

### Workflow 3: Mixed Markers (Two Formats)

```bash
# Generate small format (details/dense markers)
python aruco_sheets.py --format small -o environment_details.pdf

# Generate large format (navigation landmarks)
python aruco_sheets.py --format large -o navigation_landmarks.pdf

# Print both sets
# Deploy in unified marker system
```

---

**Last Updated**: 2025-10-18  
**Script Version**: ArUco Sheets v2.0 (Dual Format - Small & Large)
