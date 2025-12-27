# Specification — Phase IV: Local Kubernetes Deployment

**Feature Branch**: `phase4-k8s-deployment`
**Created**: 2025-12-26
**Status**: Draft
**Input**: Deploy Phase III Todo AI Chatbot on local Kubernetes (Minikube) using Docker, Helm Charts, and AI-assisted tools

---

## Clarifications

### Session 2025-12-26

- Q: What restart policy should be used for pods when they fail? → A: `restartPolicy: Always` (K8s default for auto-restart on any failure)
- Q: What level of observability is needed for Phase IV? → A: Basic logging only - stdout/stderr via `kubectl logs`
- Q: Should containers run as non-root user? → A: Yes, enforce non-root via `USER` directive in Dockerfiles + `securityContext` in K8s

---

## Feature Overview

Containerize and deploy the Todo AI Chatbot application (from Phase III) onto a local Kubernetes cluster using Minikube. This phase introduces cloud-native practices including Docker containerization, Helm chart packaging, and AI-assisted Kubernetes operations.

**Surface**: Infrastructure/DevOps
**Duration**: 1-2 days
**Success Metric**: Full application stack (frontend + backend) running on Minikube, accessible via browser, with all Phase III features functional.

---

## System Architecture

### High-Level Deployment Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│                          MINIKUBE CLUSTER                                 │
│                                                                           │
│  ┌─────────────────────────────────────────────────────────────────────┐ │
│  │                     NAMESPACE: todo-app                              │ │
│  │                                                                      │ │
│  │  ┌──────────────────────┐      ┌──────────────────────┐            │ │
│  │  │   FRONTEND POD       │      │    BACKEND POD       │            │ │
│  │  │  ┌────────────────┐  │      │  ┌────────────────┐  │            │ │
│  │  │  │   Next.js App  │  │      │  │   FastAPI App  │  │            │ │
│  │  │  │   (Port 3000)  │  │      │  │   (Port 8000)  │  │            │ │
│  │  │  └────────────────┘  │      │  └────────────────┘  │            │ │
│  │  └──────────┬───────────┘      └──────────┬───────────┘            │ │
│  │             │                             │                         │ │
│  │  ┌──────────▼───────────┐      ┌──────────▼───────────┐            │ │
│  │  │  FRONTEND SERVICE    │      │   BACKEND SERVICE    │            │ │
│  │  │  (NodePort: 30000)   │      │   (ClusterIP: 8000)  │            │ │
│  │  └──────────────────────┘      └──────────────────────┘            │ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐   │ │
│  │  │                      CONFIGMAP                                │   │ │
│  │  │  - Backend URL, OpenAI settings, Database URL                 │   │ │
│  │  └──────────────────────────────────────────────────────────────┘   │ │
│  │                                                                      │ │
│  │  ┌──────────────────────────────────────────────────────────────┐   │ │
│  │  │                      SECRET                                   │   │ │
│  │  │  - API Keys, Database credentials, JWT secrets                │   │ │
│  │  └──────────────────────────────────────────────────────────────┘   │ │
│  └─────────────────────────────────────────────────────────────────────┘ │
│                                                                           │
└──────────────────────────────────────────────────────────────────────────┘
                │                                    │
                ▼                                    ▼
┌──────────────────────────┐          ┌──────────────────────────┐
│      USER BROWSER        │          │      EXTERNAL SERVICES   │
│   http://localhost:30000 │          │  - Neon DB (PostgreSQL)  │
└──────────────────────────┘          │  - OpenAI API            │
                                      └──────────────────────────┘
```

### Technology Stack

| Component | Technology | Version |
|-----------|------------|---------|
| Container Runtime | Docker Desktop | 4.53+ |
| Container AI | Docker Gordon (optional) | Latest |
| Orchestration | Kubernetes (Minikube) | 1.31+ |
| Package Manager | Helm | 3.x |
| AI DevOps | kubectl-ai, Kagent | Latest |
| Frontend | Next.js (from Phase III) | 15+ |
| Backend | FastAPI (from Phase III) | 0.100+ |
| Database | Neon PostgreSQL (external) | - |

---

## User Scenarios & Testing

### User Story 1 - Deploy Application to Minikube (Priority: P1)

As a developer, I want to deploy the entire Todo Chatbot stack to my local Minikube cluster so I can test cloud-native deployment patterns.

**Why this priority**: Core requirement - without this, Phase IV cannot be completed.

**Independent Test**: Run `helm install` and access the application via browser at `http://localhost:30000`

**Acceptance Scenarios**:

1. **Given** Minikube is running, **When** I run `helm install todo-app ./helm/todo-app`, **Then** all pods should reach Running state within 2 minutes
2. **Given** application is deployed, **When** I access `http://localhost:30000`, **Then** I see the Todo app frontend
3. **Given** frontend is accessible, **When** I interact with the chatbot, **Then** backend responds correctly via internal service communication

---

### User Story 2 - Containerize Applications (Priority: P1)

As a developer, I want to build Docker images for frontend and backend so they can run in any container environment.

**Why this priority**: Container images are prerequisite for Kubernetes deployment.

**Independent Test**: Run `docker build` for both services and verify images start correctly with `docker run`

**Acceptance Scenarios**:

1. **Given** backend source code, **When** I run `docker build -t todo-backend .`, **Then** image builds successfully under 5 minutes
2. **Given** frontend source code, **When** I run `docker build -t todo-frontend .`, **Then** image builds successfully with multi-stage build
3. **Given** built images, **When** I run containers locally, **Then** applications start and respond to health checks

---

### User Story 3 - Create Helm Charts (Priority: P1)

As a developer, I want reusable Helm charts so I can deploy with a single command and customize via values.

**Why this priority**: Helm charts enable reproducible, configurable deployments.

**Independent Test**: Run `helm template` to validate chart syntax and `helm install` to deploy

**Acceptance Scenarios**:

1. **Given** Helm chart files, **When** I run `helm lint ./helm/todo-app`, **Then** no errors are reported
2. **Given** valid chart, **When** I run `helm install todo-app ./helm/todo-app`, **Then** all resources are created
3. **Given** deployed release, **When** I change values.yaml and upgrade, **Then** changes are applied without downtime

---

### User Story 4 - Use AI-Assisted Kubernetes Tools (Priority: P2)

As a developer, I want to use kubectl-ai and Gordon for AI-assisted operations to simplify Kubernetes management.

**Why this priority**: Bonus feature for hackathon, improves developer experience.

**Independent Test**: Run natural language commands and verify correct operations executed

**Acceptance Scenarios**:

1. **Given** kubectl-ai is installed, **When** I run `kubectl-ai "show all pods in todo-app"`, **Then** correct kubectl command is generated and executed
2. **Given** Gordon is available, **When** I ask `docker ai "build my fastapi app"`, **Then** appropriate Dockerfile is suggested
3. **Given** running cluster, **When** I ask `kubectl-ai "scale backend to 2 replicas"`, **Then** deployment is scaled correctly

---

### Edge Cases

- What happens when Minikube runs out of resources?
  - Pods enter Pending state; user must increase Minikube resources
- What happens when external Neon DB is unreachable?
  - Backend pods fail health checks; restart with backoff
- What happens when Docker image pull fails?
  - Use `imagePullPolicy: Never` with locally built images
- What happens when secrets are not configured?
  - Pods fail to start; clear error in pod logs

---

## Requirements

### Functional Requirements

- **FR-001**: System MUST build Docker images for frontend (Next.js) and backend (FastAPI)
- **FR-002**: System MUST create Helm charts with templates for Deployment, Service, ConfigMap, and Secret
- **FR-003**: System MUST deploy to Minikube namespace `todo-app`
- **FR-004**: Frontend MUST be accessible via NodePort service on port 30000
- **FR-005**: Backend MUST be accessible internally via ClusterIP service on port 8000
- **FR-006**: System MUST use Kubernetes Secrets for sensitive data (API keys, DB credentials)
- **FR-007**: System MUST use ConfigMaps for non-sensitive configuration
- **FR-008**: System MUST include health check probes (liveness, readiness)
- **FR-009**: System SHOULD support AI-assisted operations via kubectl-ai (if available)
- **FR-010**: System SHOULD support Gordon AI for Docker operations (if available in region)

### Non-Functional Requirements

- **NFR-001**: Docker images MUST be optimized (multi-stage builds, <500MB for backend, <1GB for frontend)
- **NFR-002**: Pods MUST reach Running state within 120 seconds
- **NFR-003**: Application MUST function identically to Phase III (all chatbot features working)
- **NFR-004**: Helm charts MUST pass `helm lint` without errors
- **NFR-005**: Deployment MUST be reproducible (same result on clean Minikube)
- **NFR-006**: Observability via basic logging - stdout/stderr accessible via `kubectl logs` (advanced metrics deferred to Phase V)

### Key Entities

- **Docker Images**: `todo-frontend:latest`, `todo-backend:latest`
- **Kubernetes Resources**: Namespace, Deployment, Service, ConfigMap, Secret
- **Helm Release**: `todo-app` release in `todo-app` namespace

---

## Technical Specifications

### Directory Structure

```
AI-todo/
├── helm/
│   └── todo-app/
│       ├── Chart.yaml                 # Helm chart metadata
│       ├── values.yaml                # Default configuration values
│       └── templates/
│           ├── _helpers.tpl           # Template helpers
│           ├── namespace.yaml         # Namespace definition
│           ├── configmap.yaml         # Non-sensitive config
│           ├── secret.yaml            # Sensitive credentials
│           ├── backend-deployment.yaml
│           ├── backend-service.yaml
│           ├── frontend-deployment.yaml
│           └── frontend-service.yaml
│
├── phase2/
│   ├── backend/
│   │   ├── Dockerfile                 # Backend container definition
│   │   └── .dockerignore
│   │
│   └── frontend/
│       ├── Dockerfile                 # Frontend container definition
│       └── .dockerignore
│
└── scripts/
    ├── build-images.sh               # Build Docker images
    ├── deploy-minikube.sh            # Deploy to Minikube
    └── cleanup.sh                    # Remove deployment
```

### Dockerfile Specifications

#### Backend Dockerfile (phase2/backend/Dockerfile)

```dockerfile
# Multi-stage build for Python FastAPI
FROM python:3.13-slim AS builder
WORKDIR /app
COPY requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

FROM python:3.13-slim AS runtime
WORKDIR /app

# Create non-root user for security
RUN groupadd -r appgroup && useradd -r -g appgroup appuser
COPY --from=builder /usr/local/lib/python3.13/site-packages /usr/local/lib/python3.13/site-packages
COPY --chown=appuser:appgroup . .

USER appuser
EXPOSE 8000
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD curl -f http://localhost:8000/health || exit 1
CMD ["uvicorn", "app.main:app", "--host", "0.0.0.0", "--port", "8000"]
```

#### Frontend Dockerfile (phase2/frontend/Dockerfile)

```dockerfile
# Multi-stage build for Next.js
FROM node:20-alpine AS builder
WORKDIR /app
COPY package*.json ./
RUN npm ci
COPY . .
ENV NEXT_TELEMETRY_DISABLED=1
RUN npm run build

FROM node:20-alpine AS runner
WORKDIR /app
ENV NODE_ENV=production

# Create non-root user for security
RUN addgroup -S appgroup && adduser -S appuser -G appgroup
COPY --from=builder --chown=appuser:appgroup /app/.next/standalone ./
COPY --from=builder --chown=appuser:appgroup /app/.next/static ./.next/static
COPY --from=builder --chown=appuser:appgroup /app/public ./public

USER appuser
EXPOSE 3000
HEALTHCHECK --interval=30s --timeout=10s --retries=3 \
    CMD wget -qO- http://localhost:3000/api/health || exit 1
CMD ["node", "server.js"]
```

### Helm Chart Specifications

#### Chart.yaml

```yaml
apiVersion: v2
name: todo-app
description: Todo AI Chatbot Application - Phase IV Kubernetes Deployment
version: 1.0.0
appVersion: "1.0.0"
keywords:
  - todo
  - chatbot
  - fastapi
  - nextjs
maintainers:
  - name: Developer
```

#### values.yaml

```yaml
# Global settings
namespace: todo-app

# Backend configuration
backend:
  name: todo-backend
  image: todo-backend
  tag: latest
  replicas: 1
  port: 8000
  resources:
    requests:
      memory: "256Mi"
      cpu: "250m"
    limits:
      memory: "512Mi"
      cpu: "500m"

# Frontend configuration
frontend:
  name: todo-frontend
  image: todo-frontend
  tag: latest
  replicas: 1
  port: 3000
  nodePort: 30000
  resources:
    requests:
      memory: "256Mi"
      cpu: "250m"
    limits:
      memory: "512Mi"
      cpu: "500m"

# Environment configuration
config:
  backendUrl: "http://todo-backend:8000"
  nodeEnv: "production"

# Secrets (base64 encoded in actual deployment)
secrets:
  databaseUrl: ""        # Set via --set or external secret
  openaiApiKey: ""       # Set via --set or external secret
  betterAuthSecret: ""   # Set via --set or external secret
```

### Kubernetes Resource Specifications

#### Backend Deployment

- **Replicas**: 1 (configurable)
- **Image Pull Policy**: Never (for local Minikube images)
- **Restart Policy**: Always (auto-restart on any failure)
- **Security Context**: `runAsNonRoot: true`, `runAsUser: 1000`
- **Health Probes**:
  - Liveness: HTTP GET /health every 30s
  - Readiness: HTTP GET /health every 10s
- **Environment**: Injected from ConfigMap and Secret

#### Frontend Deployment

- **Replicas**: 1 (configurable)
- **Image Pull Policy**: Never
- **Restart Policy**: Always (auto-restart on any failure)
- **Security Context**: `runAsNonRoot: true`, `runAsUser: 1000`
- **Health Probes**:
  - Liveness: HTTP GET /api/health every 30s
  - Readiness: HTTP GET /api/health every 10s
- **Environment**: Backend URL from ConfigMap

#### Services

| Service | Type | Port | Target Port | NodePort |
|---------|------|------|-------------|----------|
| todo-backend | ClusterIP | 8000 | 8000 | - |
| todo-frontend | NodePort | 3000 | 3000 | 30000 |

---

## Deployment Commands

### Prerequisites

```bash
# Verify installations
docker --version          # 24.0+
minikube version          # 1.31+
kubectl version --client  # 1.28+
helm version              # 3.12+

# Start Minikube
minikube start --driver=docker --memory=4096 --cpus=2

# Point Docker to Minikube
eval $(minikube docker-env)    # Linux/Mac
minikube docker-env | Invoke-Expression  # Windows PowerShell
```

### Build Images

```bash
# Build backend image
docker build -t todo-backend:latest ./phase2/backend

# Build frontend image
docker build -t todo-frontend:latest ./phase2/frontend

# Verify images
docker images | grep todo
```

### Deploy with Helm

```bash
# Create namespace and install
helm install todo-app ./helm/todo-app \
  --namespace todo-app \
  --create-namespace \
  --set secrets.databaseUrl="your-neon-db-url" \
  --set secrets.openaiApiKey="your-openai-key" \
  --set secrets.betterAuthSecret="your-auth-secret"

# Verify deployment
kubectl get pods -n todo-app
kubectl get services -n todo-app

# Access application
minikube service todo-frontend -n todo-app
# Or manually: http://localhost:30000
```

### AI-Assisted Operations (Optional)

```bash
# Using kubectl-ai
kubectl-ai "list all pods in todo-app namespace"
kubectl-ai "describe the todo-backend deployment"
kubectl-ai "scale todo-frontend to 2 replicas"
kubectl-ai "check logs from todo-backend"

# Using Gordon (Docker AI)
docker ai "what's wrong with my container"
docker ai "optimize my Dockerfile for production"
```

---

## Success Criteria

### Measurable Outcomes

- **SC-001**: All pods in `todo-app` namespace are in Running state
- **SC-002**: Frontend accessible at `http://localhost:30000` via browser
- **SC-003**: AI Chatbot responds to messages (same as Phase III)
- **SC-004**: Task CRUD operations work through chatbot
- **SC-005**: `helm lint` passes with no errors
- **SC-006**: Deployment reproducible on clean Minikube (same steps = same result)
- **SC-007**: Docker images build successfully (<5 minutes each)

---

## Out of Scope

- Ingress controller configuration
- TLS/SSL termination
- Horizontal Pod Autoscaler
- Persistent Volume Claims (database is external)
- CI/CD pipeline (Phase V)
- Cloud deployment (Phase V)
- Dapr integration (Phase V)
- Kafka/event streaming (Phase V)

---

## Security Considerations

1. **Secrets Management**: Use Kubernetes Secrets, never hardcode in values.yaml
2. **Image Security**: Use slim base images, scan with `docker scout`
3. **Network Policy**: Services only exposed as needed (frontend NodePort, backend ClusterIP)
4. **Non-Root Containers**: Enforced via `USER` directive in Dockerfiles + `securityContext` in K8s deployments (`runAsNonRoot: true`, `runAsUser: 1000`)
5. **Resource Limits**: Prevent resource exhaustion attacks

---

## Troubleshooting Guide

| Issue | Cause | Solution |
|-------|-------|----------|
| Pod stuck in Pending | Insufficient resources | Increase Minikube memory/CPU |
| ImagePullBackOff | Image not in Minikube | Run `eval $(minikube docker-env)` before build |
| CrashLoopBackOff | App crash or bad config | Check logs: `kubectl logs <pod> -n todo-app` |
| Connection refused | Service not ready | Wait for readiness probe, check service endpoints |
| Frontend can't reach backend | Wrong service URL | Verify ConfigMap has correct backend URL |

---

## References

- [Minikube Documentation](https://minikube.sigs.k8s.io/docs/)
- [Helm Documentation](https://helm.sh/docs/)
- [kubectl-ai GitHub](https://github.com/GoogleCloudPlatform/kubectl-ai)
- [Docker Gordon](https://docs.docker.com/ai/gordon/)
- [Hackathon Phase IV Requirements](../Hackathon%20II%20-%20Todo%20Spec-Driven%20Development.md)
