# Research: Phase IV - Local Kubernetes Deployment

**Date**: 2025-12-26
**Feature**: phase4-k8s-deployment

---

## Research Tasks

1. Docker Multi-Stage Build Best Practices
2. Helm Chart Structure for Multi-Service Apps
3. Minikube Local Image Loading
4. Non-Root Container Patterns
5. Health Probe Configuration

---

## 1. Docker Multi-Stage Build Best Practices

### Decision: Use multi-stage builds with separate builder and runtime stages

### Rationale
- Reduces final image size by 60-80% (excludes build tools, dev dependencies)
- Improves security by minimizing attack surface
- Faster deployment due to smaller images
- Clean separation between build and runtime environments

### Best Practices Applied

**Python FastAPI:**
```dockerfile
# Stage 1: Builder
FROM python:3.13-slim AS builder
# Install dependencies in builder

# Stage 2: Runtime
FROM python:3.13-slim AS runtime
# Copy only installed packages, not pip/build tools
COPY --from=builder /usr/local/lib/python3.13/site-packages ...
```

**Next.js:**
```dockerfile
# Stage 1: Builder (includes npm, devDependencies)
FROM node:20-alpine AS builder
RUN npm ci && npm run build

# Stage 2: Runtime (standalone output only)
FROM node:20-alpine AS runner
COPY --from=builder /app/.next/standalone ./
```

### Alternatives Considered
- Single-stage build: Rejected - larger images (2-3x size), includes build tools
- Distroless images: Considered but adds debugging complexity for hackathon scope

---

## 2. Helm Chart Structure for Multi-Service Apps

### Decision: Single chart with multiple templates, configurable via values.yaml

### Rationale
- Simpler than multiple charts for tightly-coupled services
- Single `helm install` deploys entire stack
- Easier to manage shared resources (ConfigMap, Secret)
- Values file provides clear configuration surface

### Structure Applied

```
helm/todo-app/
├── Chart.yaml           # Chart metadata (name, version, description)
├── values.yaml          # Default values (overridable)
└── templates/
    ├── _helpers.tpl     # Reusable template functions
    ├── namespace.yaml   # Namespace (optional, can use --create-namespace)
    ├── configmap.yaml   # Shared config (BACKEND_URL, NODE_ENV)
    ├── secret.yaml      # Credentials (DB, API keys)
    ├── backend-deployment.yaml
    ├── backend-service.yaml
    ├── frontend-deployment.yaml
    └── frontend-service.yaml
```

### Key Patterns
- Use `{{ .Values.x }}` for configurable values
- Use `{{ include "todo-app.fullname" . }}` for consistent naming
- Use `{{ .Release.Namespace }}` for namespace awareness

### Alternatives Considered
- Separate charts per service: Rejected - overhead for 2-service app
- Raw manifests: Rejected - no templating, harder to customize
- Kustomize: Considered but Helm is specified in hackathon requirements

---

## 3. Minikube Local Image Loading

### Decision: Build images inside Minikube's Docker daemon with `imagePullPolicy: Never`

### Rationale
- No image registry required (Docker Hub, ECR, etc.)
- Fastest workflow for local development
- Images immediately available to Minikube pods
- Avoids authentication and network issues

### Implementation

**Step 1: Point Docker CLI to Minikube's daemon**
```bash
# Linux/Mac
eval $(minikube docker-env)

# Windows PowerShell
minikube docker-env | Invoke-Expression
```

**Step 2: Build images (now builds inside Minikube)**
```bash
docker build -t todo-backend:latest ./phase2/backend
docker build -t todo-frontend:latest ./phase2/frontend
```

**Step 3: Set imagePullPolicy in deployments**
```yaml
spec:
  containers:
  - name: backend
    image: todo-backend:latest
    imagePullPolicy: Never  # Critical - don't try to pull from registry
```

### Alternatives Considered
- Push to Docker Hub: Rejected - requires account, network, slower
- `minikube image load`: Works but slower than docker-env approach
- Local registry (registry:2): Overkill for local development

---

## 4. Non-Root Container Patterns

### Decision: Create dedicated user in Dockerfile + enforce via K8s securityContext

### Rationale
- Security best practice (CIS Docker Benchmark)
- Prevents container escape vulnerabilities
- Limits file system damage if compromised
- Required by many production K8s policies (PodSecurityPolicies)

### Implementation

**Dockerfile (Python):**
```dockerfile
# Create non-root user
RUN groupadd -r appgroup && useradd -r -g appgroup appuser

# Change ownership of app files
COPY --chown=appuser:appgroup . .

# Switch to non-root user
USER appuser
```

**Dockerfile (Node.js Alpine):**
```dockerfile
# Alpine uses different commands
RUN addgroup -S appgroup && adduser -S appuser -G appgroup

COPY --chown=appuser:appgroup . .

USER appuser
```

**Kubernetes Deployment:**
```yaml
spec:
  securityContext:
    runAsNonRoot: true
    runAsUser: 1000
    fsGroup: 1000
  containers:
  - name: app
    securityContext:
      allowPrivilegeEscalation: false
      readOnlyRootFilesystem: true  # Optional, may need tmpdir
```

### Alternatives Considered
- Root user: Rejected - security vulnerability
- K8s-only enforcement: Works but better to also set in Dockerfile
- Rootless Docker: Out of scope, requires Docker daemon config

---

## 5. Health Probe Configuration

### Decision: HTTP probes with separate liveness/readiness configurations

### Rationale
- Liveness: Restarts container if app is deadlocked/hung
- Readiness: Removes from service until ready to accept traffic
- HTTP probes are simplest for web applications
- Different intervals appropriate for different probe types

### Implementation

**Backend (FastAPI):**
```yaml
livenessProbe:
  httpGet:
    path: /health
    port: 8000
  initialDelaySeconds: 10
  periodSeconds: 30
  timeoutSeconds: 5
  failureThreshold: 3

readinessProbe:
  httpGet:
    path: /health
    port: 8000
  initialDelaySeconds: 5
  periodSeconds: 10
  timeoutSeconds: 3
  failureThreshold: 3
```

**Frontend (Next.js):**
```yaml
livenessProbe:
  httpGet:
    path: /api/health  # Next.js API route
    port: 3000
  initialDelaySeconds: 15
  periodSeconds: 30

readinessProbe:
  httpGet:
    path: /api/health
    port: 3000
  initialDelaySeconds: 10
  periodSeconds: 10
```

### Key Parameters Explained
- `initialDelaySeconds`: Wait for app startup before probing
- `periodSeconds`: How often to probe
- `timeoutSeconds`: How long to wait for response
- `failureThreshold`: Consecutive failures before action

### Alternatives Considered
- TCP probes: Works but doesn't verify app logic
- Exec probes: More complex, useful for non-HTTP apps
- Startup probes: Useful for slow-starting apps, not needed here

---

## Summary

| Topic | Decision | Key Insight |
|-------|----------|-------------|
| Docker Builds | Multi-stage | 60-80% smaller images |
| Helm Structure | Single chart | Simpler for 2-service app |
| Image Loading | minikube docker-env | No registry needed |
| Container Security | Non-root + securityContext | Defense in depth |
| Health Probes | HTTP with separate liveness/readiness | Different intervals per type |

---

## References

- [Docker Multi-Stage Builds](https://docs.docker.com/build/building/multi-stage/)
- [Helm Chart Best Practices](https://helm.sh/docs/chart_best_practices/)
- [Minikube Handbook](https://minikube.sigs.k8s.io/docs/handbook/)
- [Kubernetes Security Context](https://kubernetes.io/docs/tasks/configure-pod-container/security-context/)
- [Configure Liveness, Readiness Probes](https://kubernetes.io/docs/tasks/configure-pod-container/configure-liveness-readiness-startup-probes/)
