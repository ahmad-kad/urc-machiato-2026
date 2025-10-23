#!/bin/bash

# Code Quality Checker for URC 2026 Autonomy
# Ensures all code follows consistent standards

echo "🧹 URC 2026 Code Quality Checker"
echo "================================="

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -f "CODE_STYLE.md" ]; then
    echo -e "${RED}❌ Error: Run this script from the Autonomy directory${NC}"
    exit 1
fi

echo "📁 Checking directory structure..."
# Check for required directories
required_dirs=("code" "docs" "development" "deployment" "subsystems")
for dir in "${required_dirs[@]}"; do
    if [ ! -d "$dir" ]; then
        echo -e "${RED}❌ Missing directory: $dir${NC}"
    else
        echo -e "${GREEN}✅ Found: $dir${NC}"
    fi
done

echo ""
echo "🐍 Checking Python code quality..."

# Find all Python files
python_files=$(find code/ -name "*.py" 2>/dev/null)

if [ -z "$python_files" ]; then
    echo -e "${YELLOW}⚠️  No Python files found in code/ directory${NC}"
    echo "   Start by copying a template:"
    echo "   cp code/templates/ros2_node_template.py code/[subsystem]/src/your_node.py"
else
    echo "📊 Found $(echo "$python_files" | wc -l) Python files"

    # Check each file
    for file in $python_files; do
        echo -n "  Checking $file... "

        # Check syntax
        if python3 -m py_compile "$file" 2>/dev/null; then
            echo -e "${GREEN}✅ Syntax OK${NC}"
        else
            echo -e "${RED}❌ Syntax Error${NC}"
        fi
    done
fi

echo ""
echo "🔍 Checking for code style issues..."

# Check if tools are available
if command -v black &> /dev/null; then
    echo -e "${GREEN}✅ Black (formatter) available${NC}"
else
    echo -e "${YELLOW}⚠️  Black not installed. Install with: pip install black${NC}"
fi

if command -v flake8 &> /dev/null; then
    echo -e "${GREEN}✅ Flake8 (linter) available${NC}"
else
    echo -e "${YELLOW}⚠️  Flake8 not installed. Install with: pip install flake8${NC}"
fi

if command -v mypy &> /dev/null; then
    echo -e "${GREEN}✅ MyPy (type checker) available${NC}"
else
    echo -e "${YELLOW}⚠️  MyPy not installed. Install with: pip install mypy${NC}"
fi

if command -v pytest &> /dev/null; then
    echo -e "${GREEN}✅ Pytest (testing) available${NC}"
else
    echo -e "${YELLOW}⚠️  Pytest not installed. Install with: pip install pytest${NC}"
fi

echo ""
echo "🧪 Checking for tests..."

# Check for test files
test_files=$(find . -name "test_*.py" -o -name "*_test.py" 2>/dev/null)
if [ -z "$test_files" ]; then
    echo -e "${YELLOW}⚠️  No test files found${NC}"
    echo "   Create tests in code/[subsystem]/test/"
else
    echo -e "${GREEN}✅ Found $(echo "$test_files" | wc -l) test files${NC}"
fi

echo ""
echo "📋 Checking for TODO files..."

# Check each subsystem has a TODO
subsystems=$(ls code/ 2>/dev/null)
for subsys in $subsystems; do
    if [ -f "code/$subsys/${subsys}_TODO.md" ]; then
        echo -e "${GREEN}✅ TODO found: $subsys${NC}"
    else
        echo -e "${RED}❌ Missing TODO: $subsys${NC}"
    fi
done

echo ""
echo "🚀 Quick Quality Check Commands:"
echo ""
echo "Format code:"
echo "  black code/"
echo "  isort code/"
echo ""
echo "Check style:"
echo "  flake8 code/ --max-line-length=88"
echo ""
echo "Type check:"
echo "  mypy code/ --ignore-missing-imports"
echo ""
echo "Run tests:"
echo "  python3 -m pytest code/*/test/ -v"
echo ""
echo "Fix common issues:"
echo "  ./check_code.sh"
echo ""
echo "📖 Read the full style guide:"
echo "  cat CODE_STYLE.md"
