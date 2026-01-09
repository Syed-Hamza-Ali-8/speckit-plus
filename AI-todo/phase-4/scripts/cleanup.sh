#!/bin/bash
# Cleanup TaskGPT from Minikube
# Usage: ./phase-4/scripts/cleanup.sh [--full]

set -e

NAMESPACE="taskgpt"
RELEASE_NAME="taskgpt"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== TaskGPT Cleanup ===${NC}"

# Parse arguments
FULL_CLEANUP=false
if [ "$1" == "--full" ]; then
    FULL_CLEANUP=true
fi

# Uninstall Helm release
if helm list -n $NAMESPACE 2>/dev/null | grep -q $RELEASE_NAME; then
    echo -e "\n${YELLOW}Uninstalling Helm release...${NC}"
    helm uninstall $RELEASE_NAME -n $NAMESPACE
    echo -e "${GREEN}✓ Helm release uninstalled${NC}"
else
    echo -e "${YELLOW}No Helm release found to uninstall${NC}"
fi

# Delete namespace
if kubectl get namespace $NAMESPACE &> /dev/null; then
    echo -e "\n${YELLOW}Deleting namespace...${NC}"
    kubectl delete namespace $NAMESPACE
    echo -e "${GREEN}✓ Namespace deleted${NC}"
else
    echo -e "${YELLOW}Namespace $NAMESPACE does not exist${NC}"
fi

# Full cleanup - delete Minikube cluster
if [ "$FULL_CLEANUP" = true ]; then
    echo -e "\n${YELLOW}Performing full cleanup...${NC}"

    # Remove Docker images
    echo "Removing Docker images..."
    docker rmi taskgpt-backend:latest 2>/dev/null || true
    docker rmi taskgpt-frontend:latest 2>/dev/null || true
    echo -e "${GREEN}✓ Docker images removed${NC}"

    # Delete Minikube cluster
    echo "Deleting Minikube cluster..."
    minikube delete
    echo -e "${GREEN}✓ Minikube cluster deleted${NC}"
fi

echo -e "\n${GREEN}=== Cleanup Complete ===${NC}"

if [ "$FULL_CLEANUP" = false ]; then
    echo ""
    echo "To perform a full cleanup (including Minikube cluster), run:"
    echo -e "  ${YELLOW}./phase-4/scripts/cleanup.sh --full${NC}"
fi
