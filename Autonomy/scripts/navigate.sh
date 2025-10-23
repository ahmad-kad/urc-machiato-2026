#!/bin/bash

# Interactive Navigation Helper for URC 2026 Autonomy
# Use: ./navigate.sh [command] or ./navigate.sh for interactive mode

# Check if running in interactive mode (no arguments)
if [ $# -eq 0 ]; then
    echo "🧭 Welcome to URC 2026 Autonomy Interactive Guide!"
    echo "=================================================="
    echo ""
    echo "I'll walk you through getting started with the project."
    echo "Let's begin with the basics..."
    echo ""

    # Interactive flow
    echo "🤔 What's your experience level?"
    echo "1) Complete beginner (never coded robotics before)"
    echo "2) Some programming experience"
    echo "3) Robotics experience (ROS/Arduino)"
    echo "4) Advanced (multiple robotics projects)"
    read -p "Enter your choice (1-4): " level

    case $level in
        1)
            echo ""
            echo "🚀 Perfect! Let's start with the absolute basics."
            echo "Run: ./navigate.sh quickstart"
            echo ""
            echo "This will give you a 5-minute setup guide to get coding immediately."
            echo ""
            echo "💡 Pro tip: The quickstart includes Docker setup and your first ROS 2 node!"
            ;;
        2)
            echo ""
            echo "📚 Great! You have some programming background."
            echo "Let's get you oriented with the project structure."
            echo ""
            echo "Run: ./navigate.sh onboarding"
            echo ""
            echo "This gives you a 15-minute checklist to set up your development environment."
            ;;
        3)
            echo ""
            echo "🤖 Excellent! You know robotics. Let's dive deeper."
            echo ""
            echo "Run: ./navigate.sh tracks"
            echo ""
            echo "Choose from 5 learning tracks: Beginner, Specialist, Team Lead, Architect, Competition."
            ;;
        4)
            echo ""
            echo "🏆 Advanced user! Let's get you contributing immediately."
            echo ""
            echo "Run: ./navigate.sh pick"
            echo ""
            echo "Pick a subsystem (navigation, SLAM, vision, etc.) and start coding!"
            ;;
        *)
            echo ""
            echo "❌ Invalid choice. Let's start with the basics..."
            echo "Run: ./navigate.sh quickstart"
            ;;
    esac

    echo ""
    echo "💭 What would you like to do next?"
    echo "• Run the suggested command above"
    echo "• Type './navigate.sh help' for all options"
    echo "• Ask questions in Slack/teams"
    echo ""
    echo "Happy coding! 🎉"
    exit 0
fi

case "$1" in
    "quickstart")
        echo "🚀 Quick Start Guide:"
        cat QUICKSTART.md | head -40
        echo ""
        echo "💡 Pro tip: ./navigate.sh setup"
        ;;
    "onboarding")
        echo "🎯 Onboarding Checklist:"
        cat ONBOARDING.md | head -30
        echo ""
        echo "📖 Full guide: cat ONBOARDING.md"
        ;;
    "style")
        echo "🧹 Code Style Guide:"
        cat CODE_STYLE.md | head -30
        echo ""
        echo "📖 Full guide: cat CODE_STYLE.md"
        ;;
    "tracks")
        echo "📚 Learning Tracks - Choose Your Path:"
        echo ""
        echo "🚀 BEGINNER TRACK (4 weeks):"
        echo "  - Start: ./navigate.sh quickstart"
        echo "  - Guide: cat docs/LearningTracks.md (Beginner section)"
        echo "  - Goal: First working ROS 2 node"
        echo ""
        echo "🔧 SPECIALIST TRACKS (6-8 weeks each):"
        echo "  - Navigation: ./navigate.sh start navigation"
        echo "  - SLAM: ./navigate.sh start slam"
        echo "  - Computer Vision: ./navigate.sh start computer_vision"
        echo "  - Autonomous Typing: ./navigate.sh start autonomous_typing"
        echo "  - State Management: ./navigate.sh start state_management"
        echo "  - LED Status: ./navigate.sh start led_status"
        echo ""
        echo "👥 TEAM LEAD TRACK (8-12 weeks):"
        echo "  - Guide: cat docs/LearningTracks.md (Team Lead section)"
        echo "  - Goal: Lead subsystem integration"
        echo ""
        echo "🏆 COMPETITION TRACK (12-16 weeks):"
        echo "  - Guide: cat docs/LearningTracks.md (Competition section)"
        echo "  - Goal: Win URC 2026!"
        ;;
    "docs")
        echo "📚 Documentation Overview:"
        cat docs/README.md | head -30
        ;;
    "overview")
        echo "🔬 Technical Overview (ELI5 → Technical):"
        echo ""
        echo "🤖 Simple explanations:"
        echo "  - What we're building and why it's hard"
        echo "  - Basic concepts explained simply"
        echo ""
        echo "🔧 Technical deep-dive:"
        echo "  - ROS 2 architecture details"
        echo "  - Algorithms and implementations"
        echo "  - Performance optimization"
        echo ""
        echo "📖 Full guide: cat docs/TechnicalOverview.md"
        ;;
    "api")
        echo "🔌 Subsystem API Guide:"
        echo ""
        echo "📤 What each subsystem PUBLISHES:"
        echo "  - Navigation: Odometry, path planning, status"
        echo "  - SLAM: Map data, pose estimates, localization"
        echo "  - Vision: Object detections, ArUco markers"
        echo "  - Typing: Arm feedback, keyboard detection"
        echo "  - State: System status, mission progress"
        echo "  - LED: Status indicators"
        echo ""
        echo "📥 What each subsystem ACCEPTS:"
        echo "  - Navigation commands, emergency stops"
        echo "  - SLAM reset, map saving"
        echo "  - Vision parameter tuning"
        echo "  - Typing sequences, arm control"
        echo "  - Mode changes, mission starts"
        echo ""
        echo "📖 Full guide: cat docs/reference/APIGuide.md"
        ;;
    "goals")
        echo "🎯 Project Goals & Success Metrics:"
        echo ""
        echo "🏆 Competition targets:"
        echo "  - Navigate 7 targets in 30 minutes"
        echo "  - Type 3-6 character codes autonomously"
        echo ""
        echo "📊 Subsystem requirements:"
        echo "  - Navigation: <1m accuracy, obstacle avoidance"
        echo "  - SLAM: <0.3m drift, real-time mapping"
        echo "  - Vision: >90% detection, ArUco tracking"
        echo "  - Typing: >95% accuracy, <2min per code"
        echo ""
        echo "📖 Full guide: cat docs/GoalsAndSuccessMetrics.md"
        ;;
    "setup")
        echo "🛠️  Development Setup:"
        cat development/docker/QUICKSTART.md | head -30
        ;;
    "subsystems")
        echo "🔧 Available subsystems:"
        echo ""
        echo "📚 Documentation:"
        ls -1 subsystems/
        echo ""
        echo "💻 Code:"
        ls -1 code/
        ;;
    "pick")
        echo "🎯 How to pick a subsystem:"
        echo ""
        echo "🔰 BEGINNER FRIENDLY:"
        echo "  - navigation (drive the robot)"
        echo "  - led_status (simple hardware)"
        echo ""
        echo "🔧 INTERMEDIATE:"
        echo "  - state_management (system coordination)"
        echo "  - computer_vision (AI/image processing)"
        echo ""
        echo "⚙️  ADVANCED:"
        echo "  - slam (3D mapping algorithms)"
        echo "  - autonomous_typing (hardware control)"
        echo ""
        echo "💡 Try: ./navigate.sh start [subsystem]"
        ;;
    "start")
        if [ -z "$2" ]; then
            echo "❌ Usage: ./navigate.sh start [subsystem]"
            echo "   Example: ./navigate.sh start navigation"
            exit 1
        fi
        SUBSYS=$2
        if [ -d "code/$SUBSYS" ]; then
            echo "🚀 Starting with $SUBSYS:"
            echo ""
            echo "📋 Your TODO list:"
            echo "  cat code/$SUBSYS/${SUBSYS}_TODO.md"
            echo ""
            echo "📖 Requirements:"
            echo "  cat subsystems/$SUBSYS/*$SUBSYS*.md"
            echo ""
            echo "🛠️  Templates:"
            echo "  cp code/templates/ros2_node_template.py code/$SUBSYS/src/your_node.py"
            echo ""
            echo "Happy coding! 🎉"
        else
            echo "❌ Subsystem '$SUBSYS' not found. Try:"
            ls -1 code/
        fi
        ;;
    "status")
        echo "📊 Project Status:"
        head -20 docs/guides/ProjectStatus.md
        ;;
    "deploy")
        echo "🚀 Deployment Options:"
        echo ""
        echo "💻 Development:"
        echo "  - Local: Already in Docker!"
        echo "  - Cross-platform: development/CrossPlatformDevelopment.md"
        echo ""
        echo "🔌 Hardware:"
        echo "  - Raspberry Pi: deployment/RaspberryPiDeployment.md"
        echo "  - Distributed: deployment/ClientServerArchitecture.md"
        ;;
    "help"|*)
        echo "🧭 URC 2026 Autonomy - Easy Navigation!"
        echo ""
        echo "🚀 GET STARTED:"
        echo "  ./navigate.sh quickstart   - 5-minute setup guide"
        echo "  ./navigate.sh onboarding  - Step-by-step checklist"
        echo "  ./navigate.sh tracks      - Choose your learning path"
        echo "  ./navigate.sh goals       - Project success metrics"
        echo "  ./navigate.sh overview    - ELI5 → technical concepts"
        echo "  ./navigate.sh setup       - Docker environment setup"
        echo ""
        echo "🎯 PICK YOUR TASK:"
        echo "  ./navigate.sh pick        - Help choosing subsystem"
        echo "  ./navigate.sh subsystems  - See all options"
        echo "  ./navigate.sh start [name]- Start coding guide"
        echo ""
        echo "📚 LEARN MORE:"
        echo "  ./navigate.sh docs        - Full documentation"
        echo "  ./navigate.sh api         - Subsystem communication interfaces"
        echo "  ./navigate.sh style       - Code style guide"
        echo "  ./navigate.sh status      - Project progress"
        echo "  ./navigate.sh deploy      - Hardware deployment"
        echo ""
        echo "💡 EXAMPLES:"
        echo "  ./navigate.sh tracks           # Choose learning path"
        echo "  ./navigate.sh goals            # See success metrics"
        echo "  ./navigate.sh api              # See communication interfaces"
        echo "  ./navigate.sh overview         # Learn concepts simply"
        echo "  ./navigate.sh quickstart      # Get started fast"
        echo "  ./navigate.sh pick            # Choose what to build"
        echo "  ./navigate.sh start navigation # Begin navigation coding"
        echo "  ./navigate.sh docs            # Read full docs"
        ;;
esac
