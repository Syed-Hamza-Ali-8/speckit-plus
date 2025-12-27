# Data Model: Phase IV - Kubernetes Resource Definitions

**Date**: 2025-12-26
**Feature**: phase4-k8s-deployment

---

## Overview

This document defines the Kubernetes resources required for Phase IV deployment. Unlike traditional data models (database entities), this phase deals with infrastructure-as-code resources.

---

## Resource Hierarchy

```
Namespace: todo-app
├── ConfigMap: todo-app-config
├── Secret: todo-app-secrets
├── Deployment: todo-backend
│   └── Pod: todo-backend-*
├── Service: todo-backend (ClusterIP)
├── Deployment: todo-frontend
│   └── Pod: todo-frontend-*
└── Service: todo-frontend (NodePort)
```

---

## 1. Namespace

| Field | Value | Description |
|-------|-------|-------------|
| apiVersion | v1 | Kubernetes API version |
| kind | Namespace | Resource type |
| metadata.name | todo-app | Namespace name |
| metadata.labels.app | todo-app | Label for identification |

---

## 2. ConfigMap

**Name**: `todo-app-config`

| Key | Value | Description |
|-----|-------|-------------|
| BACKEND_URL | http://todo-backend:8000 | Internal service URL for frontend |
| NODE_ENV | production | Node.js environment |
| PYTHON_ENV | production | Python environment |

**Usage**: Mounted as environment variables in frontend and backend deployments.

---

## 3. Secret

**Name**: `todo-app-secrets`
**Type**: Opaque

| Key | Source | Description |
|-----|--------|-------------|
| DATABASE_URL | Helm --set | Neon PostgreSQL connection string |
| OPENAI_API_KEY | Helm --set | OpenAI API key for chatbot |
| BETTER_AUTH_SECRET | Helm --set | JWT signing secret |

**Security Notes**:
- Values provided via `helm install --set` flags
- Never committed to repository
- Base64 encoded in Kubernetes

---

## 4. Backend Deployment

**Name**: `todo-backend`

### Metadata

| Field | Value |
|-------|-------|
| namespace | todo-app |
| labels.app | todo-backend |
| labels.tier | backend |

### Spec

| Field | Value | Description |
|-------|-------|-------------|
| replicas | 1 | Single replica for local dev |
| selector.matchLabels.app | todo-backend | Pod selector |

### Pod Template

#### Container: backend

| Field | Value |
|-------|-------|
| name | backend |
| image | todo-backend:latest |
| imagePullPolicy | Never |
| ports[0].containerPort | 8000 |

#### Security Context (Pod Level)

| Field | Value | Description |
|-------|-------|-------------|
| runAsNonRoot | true | Enforce non-root |
| runAsUser | 1000 | UID for appuser |
| fsGroup | 1000 | File system group |

#### Security Context (Container Level)

| Field | Value |
|-------|-------|
| allowPrivilegeEscalation | false |

#### Resources

| Type | CPU | Memory |
|------|-----|--------|
| requests | 250m | 256Mi |
| limits | 500m | 512Mi |

#### Liveness Probe

| Field | Value |
|-------|-------|
| httpGet.path | /health |
| httpGet.port | 8000 |
| initialDelaySeconds | 10 |
| periodSeconds | 30 |
| timeoutSeconds | 5 |
| failureThreshold | 3 |

#### Readiness Probe

| Field | Value |
|-------|-------|
| httpGet.path | /health |
| httpGet.port | 8000 |
| initialDelaySeconds | 5 |
| periodSeconds | 10 |
| timeoutSeconds | 3 |
| failureThreshold | 3 |

#### Environment Variables

| Name | Source |
|------|--------|
| DATABASE_URL | Secret: todo-app-secrets |
| OPENAI_API_KEY | Secret: todo-app-secrets |
| BETTER_AUTH_SECRET | Secret: todo-app-secrets |
| PYTHON_ENV | ConfigMap: todo-app-config |

---

## 5. Backend Service

**Name**: `todo-backend`
**Type**: ClusterIP (internal only)

| Field | Value |
|-------|-------|
| selector.app | todo-backend |
| ports[0].port | 8000 |
| ports[0].targetPort | 8000 |
| ports[0].protocol | TCP |

**Access**: `http://todo-backend:8000` (cluster internal)

---

## 6. Frontend Deployment

**Name**: `todo-frontend`

### Metadata

| Field | Value |
|-------|-------|
| namespace | todo-app |
| labels.app | todo-frontend |
| labels.tier | frontend |

### Spec

| Field | Value |
|-------|-------|
| replicas | 1 |
| selector.matchLabels.app | todo-frontend |

### Pod Template

#### Container: frontend

| Field | Value |
|-------|-------|
| name | frontend |
| image | todo-frontend:latest |
| imagePullPolicy | Never |
| ports[0].containerPort | 3000 |

#### Security Context (Pod Level)

| Field | Value |
|-------|-------|
| runAsNonRoot | true |
| runAsUser | 1000 |
| fsGroup | 1000 |

#### Security Context (Container Level)

| Field | Value |
|-------|-------|
| allowPrivilegeEscalation | false |

#### Resources

| Type | CPU | Memory |
|------|-----|--------|
| requests | 250m | 256Mi |
| limits | 500m | 512Mi |

#### Liveness Probe

| Field | Value |
|-------|-------|
| httpGet.path | /api/health |
| httpGet.port | 3000 |
| initialDelaySeconds | 15 |
| periodSeconds | 30 |
| timeoutSeconds | 5 |
| failureThreshold | 3 |

#### Readiness Probe

| Field | Value |
|-------|-------|
| httpGet.path | /api/health |
| httpGet.port | 3000 |
| initialDelaySeconds | 10 |
| periodSeconds | 10 |
| timeoutSeconds | 3 |
| failureThreshold | 3 |

#### Environment Variables

| Name | Source |
|------|--------|
| NEXT_PUBLIC_API_URL | ConfigMap: todo-app-config (BACKEND_URL) |
| NODE_ENV | ConfigMap: todo-app-config |

---

## 7. Frontend Service

**Name**: `todo-frontend`
**Type**: NodePort (external access)

| Field | Value |
|-------|-------|
| selector.app | todo-frontend |
| ports[0].port | 3000 |
| ports[0].targetPort | 3000 |
| ports[0].nodePort | 30000 |
| ports[0].protocol | TCP |

**Access**: `http://localhost:30000` (via Minikube)

---

## Resource Dependencies

```
┌─────────────┐
│  Namespace  │
└──────┬──────┘
       │
       ▼
┌──────────────────────────────────────┐
│                                      │
│  ┌───────────┐    ┌───────────┐     │
│  │ ConfigMap │    │  Secret   │     │
│  └─────┬─────┘    └─────┬─────┘     │
│        │                │            │
│        └───────┬────────┘            │
│                │                     │
│    ┌───────────▼───────────┐        │
│    │                       │        │
│    ▼                       ▼        │
│ ┌──────────┐         ┌──────────┐   │
│ │ Backend  │         │ Frontend │   │
│ │Deployment│◄────────│Deployment│   │
│ └────┬─────┘         └────┬─────┘   │
│      │                    │         │
│      ▼                    ▼         │
│ ┌──────────┐         ┌──────────┐   │
│ │ Backend  │         │ Frontend │   │
│ │ Service  │         │ Service  │   │
│ │(ClusterIP)         │(NodePort)│   │
│ └──────────┘         └──────────┘   │
│                                      │
└──────────────────────────────────────┘
```

---

## Labels Strategy

| Label | Purpose | Example |
|-------|---------|---------|
| app | Resource identification | todo-backend, todo-frontend |
| tier | Architecture layer | backend, frontend |
| release | Helm release tracking | {{ .Release.Name }} |
| chart | Chart identification | todo-app-1.0.0 |

---

## Validation Rules

1. **Namespace**: Must exist before other resources
2. **ConfigMap/Secret**: Must exist before Deployments reference them
3. **Service**: Selector must match Deployment pod labels
4. **Probes**: Endpoint paths must exist in application
5. **Resources**: Limits must be >= requests
6. **Security**: runAsUser must match Dockerfile USER directive
