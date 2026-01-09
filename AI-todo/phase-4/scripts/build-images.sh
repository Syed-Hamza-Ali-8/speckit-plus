#!/bin/bash
# Build Docker images for TaskGPT
# Usage: ./phase-4/scripts/build-images.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHASE4_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$PHASE4_DIR")"

echo "=== Building TaskGPT Docker Images ==="
echo "Project root: $PROJECT_ROOT"
echo "Phase 4 dir: $PHASE4_DIR"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Check if we're using Minikube's Docker daemon
if [ -z "$MINIKUBE_ACTIVE_DOCKERD" ]; then
    echo -e "${YELLOW}WARNING: Not using Minikube's Docker daemon.${NC}"
    echo "Run 'eval \$(minikube docker-env)' first for local Minikube deployment."
    echo ""
fi

# Change to project root for build context
cd "$PROJECT_ROOT"

# Build backend image using Dockerfile from phase-4/docker
echo -e "\n${GREEN}Building backend image...${NC}"
docker build -f phase-4/docker/backend.Dockerfile -t taskgpt-backend:latest .
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Backend image built successfully${NC}"
else
    echo -e "${RED}✗ Backend image build failed${NC}"
    exit 1
fi

# Build frontend image using Dockerfile from phase-4/docker
echo -e "\n${GREEN}Building frontend image...${NC}"
docker build -f phase-4/docker/frontend.Dockerfile -t taskgpt-frontend:latest .
if [ $? -eq 0 ]; then
    echo -e "${GREEN}✓ Frontend image built successfully${NC}"
else
    echo -e "${RED}✗ Frontend image build failed${NC}"
    exit 1
fi

# Show built images
echo -e "\n${GREEN}=== Built Images ===${NC}"
docker images | grep -E "todo-(backend|frontend)" | head -4

echo -e "\n${GREEN}=== Build Complete ===${NC}"
echo "Images are ready for deployment."
echo "Run './phase-4/scripts/deploy-minikube.sh' to deploy to Minikube."
