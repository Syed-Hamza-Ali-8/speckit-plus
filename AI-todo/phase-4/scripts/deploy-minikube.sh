#!/bin/bash
# Deploy Todo App to Minikube
# Usage: ./phase-4/scripts/deploy-minikube.sh

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PHASE4_DIR="$(dirname "$SCRIPT_DIR")"
PROJECT_ROOT="$(dirname "$PHASE4_DIR")"
HELM_CHART="$PHASE4_DIR/helm/todo-app"
NAMESPACE="todo-app"
RELEASE_NAME="todo-app"

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}=== Todo App Minikube Deployment ===${NC}"
echo "Project root: $PROJECT_ROOT"
echo "Helm chart: $HELM_CHART"

# Check prerequisites
echo -e "\n${YELLOW}Checking prerequisites...${NC}"

if ! command -v minikube &> /dev/null; then
    echo -e "${RED}✗ Minikube not found. Please install Minikube first.${NC}"
    exit 1
fi

if ! command -v helm &> /dev/null; then
    echo -e "${RED}✗ Helm not found. Please install Helm first.${NC}"
    exit 1
fi

if ! command -v kubectl &> /dev/null; then
    echo -e "${RED}✗ kubectl not found. Please install kubectl first.${NC}"
    exit 1
fi

echo -e "${GREEN}✓ All prerequisites installed${NC}"

# Check if Minikube is running
if ! minikube status | grep -q "Running"; then
    echo -e "\n${YELLOW}Starting Minikube...${NC}"
    minikube start --driver=docker --memory=4096 --cpus=2
fi

echo -e "${GREEN}✓ Minikube is running${NC}"

# Configure Docker to use Minikube's daemon
echo -e "\n${YELLOW}Configuring Docker for Minikube...${NC}"
eval $(minikube docker-env)
echo -e "${GREEN}✓ Docker configured for Minikube${NC}"

# Build images
echo -e "\n${YELLOW}Building Docker images...${NC}"
"$SCRIPT_DIR/build-images.sh"

# Check if secrets are provided
if [ -z "$DATABASE_URL" ] || [ -z "$OPENAI_API_KEY" ] || [ -z "$BETTER_AUTH_SECRET" ]; then
    echo -e "\n${YELLOW}WARNING: Some secrets are not set as environment variables.${NC}"
    echo "Set the following environment variables before running:"
    echo "  export DATABASE_URL='your-neon-db-url'"
    echo "  export OPENAI_API_KEY='your-openai-key'"
    echo "  export BETTER_AUTH_SECRET='your-auth-secret'"
    echo ""
    echo "Or provide them directly to helm install with --set flags."
fi

# Deploy with Helm
echo -e "\n${YELLOW}Deploying with Helm...${NC}"

# Check if release already exists
if helm list -n $NAMESPACE 2>/dev/null | grep -q $RELEASE_NAME; then
    echo "Upgrading existing release..."
    helm upgrade $RELEASE_NAME "$HELM_CHART" \
        --namespace $NAMESPACE \
        --set secrets.databaseUrl="${DATABASE_URL:-}" \
        --set secrets.openaiApiKey="${OPENAI_API_KEY:-}" \
        --set secrets.betterAuthSecret="${BETTER_AUTH_SECRET:-}"
else
    echo "Installing new release..."
    helm install $RELEASE_NAME "$HELM_CHART" \
        --namespace $NAMESPACE \
        --create-namespace \
        --set secrets.databaseUrl="${DATABASE_URL:-}" \
        --set secrets.openaiApiKey="${OPENAI_API_KEY:-}" \
        --set secrets.betterAuthSecret="${BETTER_AUTH_SECRET:-}"
fi

echo -e "${GREEN}✓ Helm deployment complete${NC}"

# Wait for pods to be ready
echo -e "\n${YELLOW}Waiting for pods to be ready...${NC}"
kubectl wait --for=condition=ready pod -l app=todo-backend -n $NAMESPACE --timeout=120s || true
kubectl wait --for=condition=ready pod -l app=todo-frontend -n $NAMESPACE --timeout=120s || true

# Show deployment status
echo -e "\n${BLUE}=== Deployment Status ===${NC}"
kubectl get pods -n $NAMESPACE
echo ""
kubectl get services -n $NAMESPACE

# Get access URL
echo -e "\n${GREEN}=== Access Information ===${NC}"
echo "To access the application, run:"
echo -e "  ${YELLOW}minikube service todo-frontend -n $NAMESPACE${NC}"
echo ""
echo "Or use port-forward:"
echo -e "  ${YELLOW}kubectl port-forward svc/todo-frontend 3000:3000 -n $NAMESPACE${NC}"
echo "  Then open: http://localhost:3000"

echo -e "\n${GREEN}=== Deployment Complete ===${NC}"
