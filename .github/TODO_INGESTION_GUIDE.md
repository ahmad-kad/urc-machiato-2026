# TODO Ingestion Guide

This project has an automated system to populate your GitHub Project board from TODO files.

## How It Works

### Two Types of TODO Ingestion

#### 1. **Comment-Based TODOs** (Real-time)
Any `TODO:` or `FIXME:` comments in Python/markdown files under `Autonomy/` are automatically converted to issues.

```python
# Autonomy/code/navigation/path_planner.py
# TODO: Implement A* algorithm with grid optimization
def plan_path():
    pass
```

**Trigger:** Push to `Autonomy/**` → Issues created automatically
**Workflow:** `.github/workflows/todo-to-issues.yml`

#### 2. **Structured TODO Files** (Weekly)
All `*_TODO.md` files (like `navigation_TODO.md`, `slam_TODO.md`) are scanned for unchecked checkboxes `- [ ]`.

```markdown
# Navigation Subsystem TODO

## CRITICAL PATH ITEMS
- [ ] GNSS Processing (Day 8-10)
- [ ] AR Tag Navigation (Day 10-12)
- [x] Basic Movement Control (COMPLETED)
```

**Trigger:** Weekly (Monday 9 AM UTC) or manual
**Workflow:** `.github/workflows/populate-from-todo-files.yml`
**Script:** `scripts/extract-todos-to-issues.py`

---

## Quick Start

### Option 1: Populate Everything Now (Manual)

Go to **Actions** tab → **"Populate Project from TODO Files"** → **"Run workflow"**

This will:
1. Scan all 8 `*_TODO.md` files in `Autonomy/`
2. Find all unchecked items (`- [ ]`)
3. Create GitHub issues with labels:
   - `type:task` - marks as a task
   - `area:*` - based on subsystem (navigation, slam, cv, etc.)
   - `status:todo` - marks as to-do
4. Auto-add to your project board

### Option 2: Automatic Weekly Population

The workflow runs automatically every **Monday at 9 AM UTC**. No setup needed!

---

## Understanding Issue Creation

When an issue is created from a TODO file:

```
Title: [TODO] GNSS Processing (Day 8-10)
Labels: type:task, area:navigation, status:todo
Body: 
  From: `code/navigation/navigation_TODO.md`
  Section: CRITICAL PATH ITEMS → CRITICAL PATH ITEMS
```

Issues are:
- ✅ Automatically added to your project board
- ✅ Labeled with area (navigation, slam, cv, etc.)
- ✅ Marked as status:todo
- ✅ Linked back to the source TODO file

---

## Adding New TODOs

### In *_TODO.md Files (Structured)
```markdown
## My Section

- [ ] New feature to implement
- [ ] Another task
- [x] Completed task (won't create issue)
```

When you update a `*_TODO.md` file and run the workflow, new unchecked items will create issues.

### In Code (Real-time)
```python
# TODO: This creates an issue immediately on push
def my_function():
    pass
```

---

## FAQ

**Q: Why isn't my TODO creating an issue?**
A: 
- Comment TODOs: Must be in `Autonomy/` folder, format: `# TODO: description`
- Checkbox TODOs: Must be `- [ ] description` (note the space inside brackets)
- Already exists: Duplicate titles are skipped

**Q: How do I prevent an issue from being created?**
A: 
- For `*_TODO.md` files: Check the box `- [x]`
- For comments: Remove the TODO or move outside `Autonomy/`

**Q: Can I edit these issues?**
A: Yes! Issues are independent once created. Changes to the issue won't sync back to the TODO file.

**Q: How do I sync back?**
A: Manually close the issue when the TODO is completed, or check the box in the TODO file for next run.

---

## Workflow Details

### `populate-from-todo-files.yml`
- **When:** Every Monday at 9 AM UTC (or manual trigger)
- **What:** Runs `scripts/extract-todos-to-issues.py`
- **Output:** Issues created with auto-labels and project board assignment

### `extract-todos-to-issues.py`
- **Scans:** All `Autonomy/**/*_TODO.md` files
- **Extracts:** Unchecked checkbox items `- [ ]`
- **Creates:** GitHub issues via API
- **Prevents duplicates:** Checks if issue already exists before creating

---

## Project Board Integration

All TODO issues automatically:
1. ✅ Add to your project board
2. ✅ Get labeled with area (cv, navigation, slam, etc.)
3. ✅ Get labeled with type:task and status:todo
4. ✅ Link back to source TODO file

View your project: **https://github.com/users/ahmad-kad/projects/5**

---

## Customization

### Change Schedule
Edit `.github/workflows/populate-from-todo-files.yml`:
```yaml
schedule:
  - cron: '0 9 * * 1'  # Change this cron expression
```

### Add New TODO Files
Just create `*_TODO.md` in any subsystem folder. The script auto-detects:
- `Autonomy/code/navigation/navigation_TODO.md` → `area:navigation`
- `Autonomy/code/slam/slam_TODO.md` → `area:slam`
- etc.

### Customize Labels
Edit `scripts/extract-todos-to-issues.py`, function `extract_area_from_filename()`:
```python
area_map = {
    "navigation": "area:navigation",
    "slam": "area:slam",
    # Add your own mappings
}
```
