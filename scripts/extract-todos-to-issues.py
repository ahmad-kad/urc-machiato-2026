#!/usr/bin/env python3
"""
Extract TODO items from *_TODO.md files and create GitHub issues.

This script:
1. Finds all *_TODO.md files in Autonomy/
2. Parses checkbox items (- [ ] and - [x])
3. Creates GitHub issues for unchecked items
4. Labels them with area and type:task
"""

import os
import re
import sys
from pathlib import Path
from typing import List, Tuple, Optional
import json

try:
    import requests
except ImportError:
    print("❌ Error: 'requests' library not found. Install with: pip install requests")
    sys.exit(1)


class TodoExtractor:
    """Extract TODO items from markdown files."""

    def __init__(self, base_path: str = "Autonomy"):
        self.base_path = Path(base_path)
        self.todos: List[dict] = []

    def find_todo_files(self) -> List[Path]:
        """Find all *_TODO.md files."""
        return sorted(self.base_path.glob("**/*_TODO.md"))

    def extract_area_from_filename(self, filepath: Path) -> str:
        """Extract area label from filepath."""
        # Map subsystem folders to area labels
        area_map = {
            "calibration": "area:calibration",
            "navigation": "area:navigation",
            "slam": "area:slam",
            "autonomous_typing": "area:autonomous-typing",
            "computer_vision": "area:cv",
            "state_management": "area:state",
            "led_status": "area:led-status",
            "simulation": "area:simulation",
        }
        
        for key, label in area_map.items():
            if key in str(filepath):
                return label
        return "area:deployment"

    def parse_todo_file(self, filepath: Path) -> List[dict]:
        """Parse a TODO.md file and extract unchecked items."""
        with open(filepath, "r") as f:
            content = f.read()

        section = None
        todos = []
        area = self.extract_area_from_filename(filepath)

        # Extract the main heading to use as context
        match = re.search(r"^# (.+?)(?:\n|$)", content, re.MULTILINE)
        main_title = match.group(1) if match else "TODO"

        # Find all checkbox lines
        lines = content.split("\n")
        for i, line in enumerate(lines):
            # Track current section
            if line.startswith("## "):
                section = line.replace("## ", "").strip()
            elif line.startswith("### "):
                subsection = line.replace("### ", "").strip()
                section = f"{section} → {subsection}" if section else subsection

            # Extract unchecked items: - [ ]
            if re.match(r"^\s*-\s+\[\s*\]\s+", line):
                # This is an unchecked item
                title = re.sub(r"^\s*-\s+\[\s*\]\s+", "", line).strip()
                
                # Skip empty or metadata lines
                if title and not title.startswith("**") and len(title) > 3:
                    todo_item = {
                        "title": title,
                        "section": section or "General",
                        "file": str(filepath.relative_to(self.base_path)),
                        "area": area,
                        "parent_title": main_title,
                        "body": f"From: `{filepath.relative_to(self.base_path)}`\n\nSection: {section or 'General'}"
                    }
                    todos.append(todo_item)

        return todos

    def extract_all(self) -> List[dict]:
        """Extract all TODOs from all files."""
        all_todos = []
        files = self.find_todo_files()
        
        print(f"📋 Found {len(files)} TODO files")
        
        for filepath in files:
            print(f"   Parsing: {filepath.relative_to(self.base_path)}")
            todos = self.parse_todo_file(filepath)
            all_todos.extend(todos)
            print(f"      ✓ Found {len(todos)} unchecked items")

        return all_todos


class GitHubIssueCreator:
    """Create GitHub issues from TODO items."""

    def __init__(self, token: str, owner: str, repo: str):
        self.token = token
        self.owner = owner
        self.repo = repo
        self.base_url = f"https://api.github.com/repos/{owner}/{repo}"
        self.headers = {
            "Authorization": f"token {token}",
            "Accept": "application/vnd.github.v3+json"
        }
        self.created_count = 0
        self.skipped_count = 0

    def issue_exists(self, title: str) -> bool:
        """Check if an issue with this title already exists."""
        search_url = f"https://api.github.com/search/issues"
        search_headers = {**self.headers, "Accept": "application/vnd.github.v3+json"}
        params = {
            "q": f'repo:{self.owner}/{self.repo} is:issue "{title}"',
            "per_page": 1
        }
        
        try:
            resp = requests.get(search_url, headers=search_headers, params=params, timeout=5)
            return resp.json().get("total_count", 0) > 0
        except Exception as e:
            print(f"   ⚠️ Error checking existing issues: {e}")
            return False

    def create_issue(self, todo: dict) -> bool:
        """Create a GitHub issue for a TODO item."""
        # Check if issue already exists
        if self.issue_exists(todo["title"]):
            self.skipped_count += 1
            return False

        url = f"{self.base_url}/issues"
        
        payload = {
            "title": f"[TODO] {todo['title'][:100]}",
            "body": f"{todo['body']}\n\n**Status:** To Do\n**Auto-generated from:** {todo['file']}",
            "labels": ["type:task", todo["area"], "status:todo"]
        }

        try:
            resp = requests.post(url, headers=self.headers, json=payload, timeout=10)
            if resp.status_code == 201:
                issue = resp.json()
                print(f"   ✅ Created issue #{issue['number']}: {todo['title'][:60]}...")
                self.created_count += 1
                return True
            else:
                print(f"   ❌ Failed to create issue: {resp.status_code} - {resp.text}")
                return False
        except Exception as e:
            print(f"   ❌ Error creating issue: {e}")
            return False

    def create_all(self, todos: List[dict]) -> dict:
        """Create issues for all TODOs."""
        print(f"\n🚀 Creating issues (token auth: {'✅' if self.token else '❌'})")
        print(f"   Repository: {self.owner}/{self.repo}\n")

        for i, todo in enumerate(todos, 1):
            print(f"[{i}/{len(todos)}] {todo['section']} → {todo['area']}")
            self.create_issue(todo)

        return {
            "created": self.created_count,
            "skipped": self.skipped_count,
            "total": len(todos)
        }


def main():
    """Main entry point."""
    # Get environment variables
    token = os.getenv("GITHUB_TOKEN")
    owner = os.getenv("GITHUB_REPOSITORY_OWNER", "ahmad-kad")
    repo_name = os.getenv("GITHUB_REPOSITORY", "ahmad-kad/robotics2025").split("/")[1]

    if not token:
        print("❌ Error: GITHUB_TOKEN environment variable not set")
        sys.exit(1)

    # Extract TODOs
    extractor = TodoExtractor()
    todos = extractor.extract_all()

    if not todos:
        print("\n✅ No unchecked TODO items found!")
        return

    print(f"\n📊 Total unchecked items: {len(todos)}\n")

    # Create issues
    creator = GitHubIssueCreator(token, owner, repo_name)
    result = creator.create_all(todos)

    # Summary
    print(f"\n{'='*60}")
    print(f"📈 Summary:")
    print(f"   Created: {result['created']} new issues")
    print(f"   Skipped: {result['skipped']} (already exist)")
    print(f"   Total:   {result['total']} items processed")
    print(f"{'='*60}\n")

    if result['created'] > 0:
        print("✅ Issues created successfully!")
        print(f"📊 View your project: https://github.com/users/ahmad-kad/projects/5")


if __name__ == "__main__":
    main()
