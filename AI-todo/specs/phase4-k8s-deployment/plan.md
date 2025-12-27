# Implementation Plan: Phase IV - Local Kubernetes Deployment

**Branch**: `phase4-k8s-deployment` | **Date**: 2025-12-26 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/phase4-k8s-deployment/spec.md`

---

## Summary

Deploy the Phase III Todo AI Chatbot application onto a local Kubernetes cluster (Minikube) using Docker containerization, Helm charts for package management, and AI-assisted DevOps tools (kubectl-ai, Gordon). The deployment includes secure container configurations (non-root users), health probes, ConfigMaps, Secrets, and proper service exposure patterns.

---

## Technical Context

**Language/Version**: Python 3.13 (backend), Node.js 20 (frontend), YAML (K8s manifests)
**Primary Dependencies**: Docker Desktop 4.53+, Minikube 1.31+, Helm 3.x, kubectl 1.28+
**Storage**: External Neon PostgreSQL (no in-cluster database)
**Testing**: `helm lint`, `kubectl get pods`, manual verification
**Target Platform**: Local Minikube cluster on Windows/Linux/Mac
**Project Type**: Infrastructure/DevOps - containerization and orchestration
**Performance Goals**: Pods Running within 120 seconds, images <500MB (backend), <1GB (frontend)
**Constraints**: Local deployment only (no cloud), basic logging (kubectl logs), non-root containers
**Scale/Scope**: Single-node Minikube, 1 replica each (frontend/backend), NodePort access

---

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Rule | Status | Notes |
|------|--------|-------|
| Phase IV Technology Stack | ✅ PASS | Docker, Minikube, Helm, kubectl-ai, Kagent as specified |
| Phase IV Allowed Subagents | ✅ PASS | Docker Agent, Kubernetes Agent allowed |
| Deployment Standards | ✅ PASS | Helm charts, resource limits, health checks planned |
| Secrets Management | ✅ PASS | Kubernetes Secrets for sensitive data |
| kubectl-ai Usage | ✅ PASS | Context validation, dry-run, audit planned |
| Spec-Driven Development | ✅ PASS | Spec created and clarified before planning |
| Clean Architecture | ✅ PASS | Separation of concerns maintained |

**Gate Result**: ✅ PASS - All constitution rules satisfied for Phase IV

---

## Project Structure

### Documentation (this feature)

```text
specs/phase4-k8s-deployment/
├── spec.md              # Feature specification (completed)
├── plan.md              # This file
├── research.md          # Phase 0 output
├── data-model.md        # Phase 1 output (K8s resource definitions)
├── quickstart.md        # Phase 1 output (deployment guide)
├── contracts/           # Phase 1 output (Helm values schema)
└── tasks.md             # Phase 2 output (/sp.tasks command)
```

### Source Code (repository root)

```text
AI-todo/
├── helm/
│   └── todo-app/
│       ├── Chart.yaml                 # Helm chart metadata
│       ├── values.yaml                # Default configuration
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
│   │   ├── Dockerfile                 # Backend container (multi-stage, non-root)
│   │   └── .dockerignore
│   │
│   └── frontend/
│       ├── Dockerfile                 # Frontend container (multi-stage, non-root)
│       └── .dockerignore
│
└── scripts/
    ├── build-images.sh               # Build Docker images
    ├── deploy-minikube.sh            # Deploy to Minikube
    └── cleanup.sh                    # Remove deployment
```

**Structure Decision**: Infrastructure-as-Code structure with Helm charts at repository root, Dockerfiles within existing phase2 directories to containerize existing applications.

---

## Complexity Tracking

> No constitution violations requiring justification.

| Violation | Why Needed | Simpler Alternative Rejected Because |
|-----------|------------|-------------------------------------|
| N/A | N/A | N/A |

---

## Phase 0: Research

### Research Tasks

1. **Docker Multi-Stage Build Best Practices** for Python FastAPI and Next.js
2. **Helm Chart Structure** for multi-service applications
3. **Minikube Local Image Loading** - avoiding image registry for local development
4. **Non-Root Container Patterns** - USER directive and securityContext
5. **Health Probe Configuration** - liveness vs readiness patterns

### Research Findings

See [research.md](./research.md) for detailed findings.

---

## Phase 1: Design

### Component Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                         HELM CHART                                │
│                       (todo-app v1.0.0)                          │
├──────────────────────────────────────────────────────────────────┤
│                                                                   │
│  ┌─────────────────┐         ┌─────────────────┐                │
│  │   CONFIGMAP     │         │     SECRET      │                │
│  │ ─────────────── │         │ ─────────────── │                │
│  │ BACKEND_URL     │         │ DATABASE_URL    │                │
│  │ NODE_ENV        │         │ OPENAI_API_KEY  │                │
│  └────────┬────────┘         │ BETTER_AUTH_SEC │                │
│           │                  └────────┬────────┘                │
│           │                           │                          │
│  ┌────────▼───────────────────────────▼────────┐                │
│  │              BACKEND DEPLOYMENT              │                │
│  │ ─────────────────────────────────────────── │                │
│  │ Image: todo-backend:latest                   │                │
│  │ Replicas: 1                                  │                │
│  │ Port: 8000                                   │                │
│  │ securityContext: runAsNonRoot, runAsUser:1000│                │
│  │ Probes: /health (liveness/readiness)         │                │
│  └──────────────────────┬──────────────────────┘                │
│                         │                                        │
│  ┌──────────────────────▼──────────────────────┐                │
│  │           BACKEND SERVICE (ClusterIP)        │                │
│  │ Port: 8000 → targetPort: 8000                │                │
│  └──────────────────────────────────────────────┘                │
│                         ▲                                        │
│                         │ http://todo-backend:8000               │
│                         │                                        │
│  ┌──────────────────────┴──────────────────────┐                │
│  │             FRONTEND DEPLOYMENT              │                │
│  │ ─────────────────────────────────────────── │                │
│  │ Image: todo-frontend:latest                  │                │
│  │ Replicas: 1                                  │                │
│  │ Port: 3000                                   │                │
│  │ securityContext: runAsNonRoot, runAsUser:1000│                │
│  │ Probes: /api/health (liveness/readiness)     │                │
│  └──────────────────────┬──────────────────────┘                │
│                         │                                        │
│  ┌──────────────────────▼──────────────────────┐                │
│  │          FRONTEND SERVICE (NodePort)         │                │
│  │ Port: 3000 → targetPort: 3000 → nodePort:30000│               │
│  └──────────────────────────────────────────────┘                │
│                                                                   │
└──────────────────────────────────────────────────────────────────┘
                              │
                              ▼
                    Browser: http://localhost:30000
```

### Key Design Decisions

| Decision | Choice | Rationale |
|----------|--------|-----------|
| Image Registry | Local (imagePullPolicy: Never) | Simplest for Minikube, no registry setup |
| Service Type (Frontend) | NodePort | Direct browser access without Ingress complexity |
| Service Type (Backend) | ClusterIP | Internal-only, frontend proxies to backend |
| Secrets Handling | Helm --set flags | Secrets not committed to repo, passed at install time |
| Container User | Non-root (UID 1000) | Security best practice, prevents container escape |
| Resource Limits | 256Mi-512Mi RAM, 250m-500m CPU | Balanced for Minikube resource constraints |

### Artifacts to Generate

1. **research.md** - Docker and Helm best practices
2. **data-model.md** - Kubernetes resource definitions
3. **contracts/helm-values-schema.yaml** - Helm values schema
4. **quickstart.md** - Step-by-step deployment guide

---

## Risk Analysis

| Risk | Impact | Mitigation |
|------|--------|------------|
| Minikube resource exhaustion | Pod Pending state | Document minimum resource requirements (4GB RAM, 2 CPU) |
| Image not found in Minikube | ImagePullBackOff | Document `minikube docker-env` requirement |
| Secrets exposed in repo | Security breach | Use --set flags, .gitignore for any local secret files |
| Frontend can't reach backend | Application broken | Verify ConfigMap backend URL matches service name |
| Health check failures | CrashLoopBackOff | Ensure /health endpoints exist in Phase III code |

---

## Definition of Done

- [ ] Dockerfiles created for frontend and backend (multi-stage, non-root)
- [ ] .dockerignore files created
- [ ] Helm chart structure complete with all templates
- [ ] `helm lint` passes without errors
- [ ] Images build successfully in Minikube Docker environment
- [ ] `helm install` deploys all resources
- [ ] All pods reach Running state within 120 seconds
- [ ] Frontend accessible at http://localhost:30000
- [ ] Chatbot functionality works (same as Phase III)
- [ ] Documentation complete (quickstart.md, README updates)
