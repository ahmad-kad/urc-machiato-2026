#!/bin/bash
# Docker Development Environment Script
# Usage: ./docker-dev.sh [command]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

# Configuration
COMPOSE_FILE="docker-compose.yml"
PROJECT_NAME="autonomy"

# Function to print colored output
print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# Function to check if Docker is running
check_docker() {
    if ! docker info > /dev/null 2>&1; then
        print_error "Docker is not running. Please start Docker first."
        exit 1
    fi
}

# Function to check if NVIDIA Docker is available
check_nvidia() {
    if command -v nvidia-docker > /dev/null 2>&1; then
        print_info "NVIDIA Docker detected - GPU support enabled"
        export NVIDIA_DOCKER=1
    else
        print_warning "NVIDIA Docker not detected - running without GPU support"
    fi
}

# Function to build images
build() {
    print_info "Building Docker images..."
    docker compose -f $COMPOSE_FILE -p $PROJECT_NAME build "$@"
    print_success "Docker images built successfully"
}

# Function to start services
start() {
    check_docker
    check_nvidia

    print_info "Starting Docker services..."
    docker compose -f $COMPOSE_FILE -p $PROJECT_NAME up -d "$@"
    print_success "Docker services started"

    print_info "To enter the development container, run:"
    echo "  docker exec -it autonomy_development bash"
    print_info "To view logs:"
    echo "  docker compose -f docker/docker-compose.yml -p autonomy logs -f"
}

# Function to stop services
stop() {
    print_info "Stopping Docker services..."
    docker compose -f $COMPOSE_FILE -p $PROJECT_NAME down "$@"
    print_success "Docker services stopped"
}

# Function to restart services
restart() {
    stop
    sleep 2
    start
}

# Function to enter development container
enter() {
    print_info "Entering development container..."
    docker exec -it autonomy_development bash
}

# Function to view logs
logs() {
    docker compose -f $COMPOSE_FILE -p $PROJECT_NAME logs "$@"
}

# Function to clean up
clean() {
    print_warning "This will remove all containers, volumes, and images for the project"
    read -p "Are you sure? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "Cleaning up Docker resources..."
        docker compose -f $COMPOSE_FILE -p $PROJECT_NAME down -v --rmi all
        print_success "Cleanup completed"
    fi
}

# Function to show status
status() {
    print_info "Docker services status:"
    docker compose -f $COMPOSE_FILE -p $PROJECT_NAME ps

    print_info "Docker images:"
    docker images | grep autonomy

    print_info "Docker volumes:"
    docker volume ls | grep autonomy
}

# Function to show help
help() {
    echo "Docker Development Environment Script"
    echo ""
    echo "Usage: $0 [command] [options]"
    echo ""
    echo "Commands:"
    echo "  build     Build Docker images"
    echo "  start     Start Docker services"
    echo "  stop      Stop Docker services"
    echo "  restart   Restart Docker services"
    echo "  enter     Enter development container"
    echo "  logs      Show service logs"
    echo "  clean     Remove all containers, volumes, and images"
    echo "  status    Show status of services and resources"
    echo "  help      Show this help message"
    echo ""
    echo "Examples:"
    echo "  $0 build              # Build all images"
    echo "  $0 start              # Start all services"
    echo "  $0 enter              # Enter development container"
    echo "  $0 logs -f autonomy-dev  # Follow logs for development service"
    echo ""
    echo "Environment variables:"
    echo "  DISPLAY=:0            # X11 display for GUI applications"
    echo "  ROS_DOMAIN_ID=42      # ROS 2 domain ID"
}

# Main script logic
case "${1:-help}" in
    build)
        shift
        build "$@"
        ;;
    start)
        shift
        start "$@"
        ;;
    stop)
        shift
        stop "$@"
        ;;
    restart)
        shift
        restart "$@"
        ;;
    enter)
        enter
        ;;
    logs)
        shift
        logs "$@"
        ;;
    clean)
        clean
        ;;
    status)
        status
        ;;
    help|--help|-h)
        help
        ;;
    *)
        print_error "Unknown command: $1"
        echo ""
        help
        exit 1
        ;;
esac
