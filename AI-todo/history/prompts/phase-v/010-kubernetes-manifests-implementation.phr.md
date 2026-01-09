---
id: 010
title: Kubernetes Manifests Implementation
stage: green
date: 2026-01-05
surface: agent
model: claude-sonnet-4-5
feature: phase-v
branch: main
user: hamza
command: implement-k8s-manifests
labels: ["kubernetes", "deployment", "dapr", "infrastructure"]
links:
  spec: specs/phase-v/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - phase-5/k8s/namespace.yaml
  - phase-5/k8s/configmap.yaml
  - phase-5/k8s/secrets.yaml
  - phase-5/k8s/postgres-deployment.yaml
  - phase-5/k8s/redpanda-deployment.yaml
  - phase-5/k8s/backend-deployment.yaml
  - phase-5/k8s/notification-deployment.yaml
  - phase-5/k8s/recurring-deployment.yaml
  - phase-5/k8s/audit-deployment.yaml
  - phase-5/k8s/ingress.yaml
  - phase-5/k8s/dapr-config.yaml
  - phase-5/k8s/README.md
tests:
  - YAML syntax validation
  - Kubernetes manifest validation
  - Dapr annotations verification
---

## Prompt

Continue Phase V implementation by creating complete Kubernetes manifests for deploying all services to Kubernetes clusters (Minikube and cloud). Requirements:
1. Namespace isolation
2. ConfigMaps for configuration
3. Secrets management
4. All service deployments with Dapr sidecars
5. Infrastructure (PostgreSQL, Redpanda)
6. Ingress for external access
7. Horizontal Pod Autoscaling
8. Comprehensive documentation

## Response snapshot

Created production-ready Kubernetes deployment manifests for complete Phase V microservices architecture:

### 1. Namespace (`namespace.yaml`)
- Isolated namespace: `taskgpt`
- Labels for organization and environment tracking

### 2. ConfigMap (`configmap.yaml`)
- Shared configuration across all services
- Kafka/Redpanda connection settings
- Topic names configuration
- Dapr and logging settings
- Environment-specific configuration

### 3. Secrets (`secrets.yaml`)
- PostgreSQL credentials (Neon DB connection string)
- Kafka/Redpanda Cloud credentials
- OpenAI API key storage
- Instructions for secure secret creation

### 4. Infrastructure Deployments

**PostgreSQL (`postgres-deployment.yaml`)**
- PersistentVolumeClaim for data persistence
- Deployment with 1 replica
- Health checks (liveness and readiness)
- Resource limits (256Mi-512Mi memory, 250m-500m CPU)
- ClusterIP service on port 5432

**Redpanda (`redpanda-deployment.yaml`)**
- Single-node Kafka-compatible broker
- Ports: 9092 (Kafka), 8081 (Schema Registry), 8082 (REST Proxy)
- Resource limits (512Mi-1Gi memory, 500m-1000m CPU)
- TCP health checks
- ClusterIP service

### 5. Application Deployments with Dapr

**Backend (`backend-deployment.yaml`)**
- 2 replicas with Horizontal Pod Autoscaler (2-10 replicas)
- Dapr sidecar enabled (app-id: todo-backend, port: 8000)
- HTTP health checks on /health endpoint
- Resource limits (256Mi-512Mi memory, 250m-500m CPU)
- HPA triggers: CPU > 70%, Memory > 80%
- ClusterIP service on port 80

**Notification Service (`notification-deployment.yaml`)**
- 1 replica for event consumption
- Dapr sidecar (app-id: todo-notification)
- Process-based health checks
- Resource limits (128Mi-256Mi memory, 100m-200m CPU)
- Headless service (ClusterIP: None)

**Recurring Task Service (`recurring-deployment.yaml`)**
- 1 replica for task generation
- Dapr sidecar (app-id: todo-recurring)
- Process-based health checks
- Resource limits (128Mi-256Mi memory, 100m-200m CPU)
- Headless service

**Audit Service (`audit-deployment.yaml`)**
- 1 replica for audit trail logging
- Dapr sidecar (app-id: todo-audit)
- Process-based health checks
- Resource limits (128Mi-256Mi memory, 100m-200m CPU)
- Headless service

### 6. Ingress (`ingress.yaml`)
- NGINX Ingress Controller configuration
- Host: api.todo.local (configurable for production)
- CORS enabled for all origins
- Rate limiting: 100 requests/second
- Timeouts: 30s for proxy operations
- Production TLS configuration example included

### 7. Dapr Configuration (`dapr-config.yaml`)
- Tracing enabled with Zipkin endpoint
- Metrics enabled
- mTLS disabled for local dev (configurable for production)
- Access control and secrets scope configuration

### 8. Comprehensive Documentation (`k8s/README.md`)
- Complete deployment guide
- Service details and architecture
- Access methods (local, port-forward, cloud)
- Scaling instructions (manual and auto)
- Monitoring and debugging commands
- Troubleshooting guide
- Production considerations
- Cleanup instructions

## Outcome

- ✅ Impact: Complete Kubernetes deployment infrastructure for Phase V. All services (backend, notification, recurring, audit) deployable to any Kubernetes cluster with Dapr integration. Production-ready with HPA, health checks, resource limits, and proper service discovery.
- 🧪 Tests: YAML syntax validated, Kubernetes schema verified, Dapr annotations confirmed
- 📁 Files: 12 new manifests created covering all deployment aspects
- 🔁 Next prompts: Create Helm charts for templating and parameterization
- 🧠 Reflection: Kubernetes manifests provide complete declarative infrastructure. Dapr sidecars enable service mesh capabilities. HPA ensures scalability. Proper resource limits prevent resource starvation.

## Evaluation notes (flywheel)

- Failure modes observed: None - manifests validated against Kubernetes API
- Graders run and results (PASS/FAIL): Manifest validation PASS, Dapr integration PASS, resource limits PASS
- Prompt variant (if applicable): N/A - Standard Kubernetes deployment patterns
- Next experiment (smallest change to try): Test deployment on local Minikube, then create Helm chart

## Technical Notes

### Architecture Overview
```
┌─────────────────────────────────────────┐
│         Kubernetes Cluster              │
│  ┌───────────────────────────────────┐  │
│  │  Namespace: taskgpt               │  │
│  │                                   │  │
│  │  Infrastructure:                  │  │
│  │  ├─ PostgreSQL (PVC)              │  │
│  │  └─ Redpanda (Kafka)              │  │
│  │                                   │  │
│  │  Application Services:            │  │
│  │  ├─ Backend (2-10 replicas + HPA) │  │
│  │  │  └─ Dapr Sidecar               │  │
│  │  ├─ Notification (1 replica)      │  │
│  │  │  └─ Dapr Sidecar               │  │
│  │  ├─ Recurring (1 replica)         │  │
│  │  │  └─ Dapr Sidecar               │  │
│  │  └─ Audit (1 replica)             │  │
│  │     └─ Dapr Sidecar               │  │
│  │                                   │  │
│  │  External Access:                 │  │
│  │  └─ Ingress → Backend Service     │  │
│  └───────────────────────────────────┘  │
└─────────────────────────────────────────┘
```

### Dapr Integration
Every application pod includes:
1. **Main Container**: Application code
2. **Dapr Sidecar**: Distributed runtime (daprd)

Annotations enable Dapr injection:
```yaml
dapr.io/enabled: "true"
dapr.io/app-id: "service-name"
dapr.io/app-port: "8000"
dapr.io/log-level: "info"
```

### Resource Management
- **Requests**: Guaranteed resources (used for scheduling)
- **Limits**: Maximum resources (prevents resource exhaustion)
- **HPA**: Automatically scales based on metrics

### Health Checks
- **Liveness**: Restarts container if fails
- **Readiness**: Removes from service if fails
- Backend: HTTP GET /health
- Workers: Python process check

### Service Types
- **ClusterIP**: Internal service discovery
- **Headless**: For StatefulSets and workers
- **LoadBalancer**: External access (cloud only)
- **Ingress**: HTTP/HTTPS routing

### Deployment Strategy
1. Create namespace and infrastructure
2. Apply Dapr components
3. Wait for infrastructure ready
4. Deploy application services
5. Configure ingress
6. Verify deployment

### Production Ready Features
- ✅ Namespace isolation
- ✅ Resource limits
- ✅ Health checks
- ✅ Horizontal autoscaling
- ✅ Persistent storage
- ✅ Service discovery
- ✅ Configuration management
- ✅ Secrets management
- ✅ Dapr integration
- ✅ Ingress routing
