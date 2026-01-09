# TaskGPT Phase 5 Helm Chart

This Helm chart deploys the complete Phase V TaskGPT application with event-driven architecture, microservices, and Dapr integration.

## Prerequisites

- Kubernetes 1.20+
- Helm 3.8+
- Dapr installed on Kubernetes cluster
- kubectl configured to communicate with your cluster

## Installation

### Quick Start

```bash
# Install with default values (local development)
helm install taskgpt-phase5 ./helm/taskgpt-phase5 --namespace taskgpt --create-namespace

# Install with custom values
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  -f custom-values.yaml
```

### Install with Production Values

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  --set postgres.enabled=false \
  --set redpanda.enabled=false \
  --set secrets.postgres.connectionString="postgresql://..." \
  --set secrets.kafka.brokers="your-cluster.cloud.redpanda.com:9092" \
  --set secrets.kafka.username="your-username" \
  --set secrets.kafka.password="your-password" \
  --set ingress.hosts[0].host="api.yourdomain.com"
```

## Configuration

### Values Overview

| Parameter | Description | Default |
|-----------|-------------|---------|
| `namespace.name` | Kubernetes namespace | `taskgpt` |
| `backend.replicaCount` | Number of backend replicas | `2` |
| `backend.autoscaling.enabled` | Enable HPA | `true` |
| `backend.autoscaling.minReplicas` | Minimum replicas | `2` |
| `backend.autoscaling.maxReplicas` | Maximum replicas | `10` |
| `postgres.enabled` | Deploy PostgreSQL | `true` |
| `redpanda.enabled` | Deploy Redpanda | `true` |
| `ingress.enabled` | Enable ingress | `true` |
| `dapr.enabled` | Enable Dapr integration | `true` |

### Production Configuration

Create a `production-values.yaml`:

```yaml
app:
  environment: production

backend:
  replicaCount: 3
  image:
    repository: registry.digitalocean.com/your-registry/todo-backend
    tag: "v1.0.0"
  autoscaling:
    minReplicas: 3
    maxReplicas: 20

postgres:
  enabled: false  # Use managed database

redpanda:
  enabled: false  # Use Redpanda Cloud

secrets:
  postgres:
    connectionString: "postgresql://user:pass@neon-host:5432/db?sslmode=require"
  kafka:
    brokers: "your-cluster.cloud.redpanda.com:9092"
    username: "your-username"
    password: "your-password"

ingress:
  hosts:
    - host: api.yourdomain.com
      paths:
        - path: /
          pathType: Prefix
  tls:
    - secretName: todo-tls-secret
      hosts:
        - api.yourdomain.com

config:
  dapr:
    enableMtls: true
```

Install with production values:

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  -f production-values.yaml
```

## Upgrading

```bash
# Upgrade with new values
helm upgrade taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  -f custom-values.yaml

# Upgrade with CLI overrides
helm upgrade taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --set backend.replicaCount=5
```

## Uninstallation

```bash
# Uninstall release
helm uninstall taskgpt-phase5 --namespace taskgpt

# Delete namespace
kubectl delete namespace taskgpt
```

## Testing

```bash
# Test template rendering
helm template taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  -f custom-values.yaml

# Dry run
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  --dry-run --debug
```

## Monitoring

```bash
# Get release status
helm status taskgpt-phase5 -n taskgpt

# List all releases
helm list -n taskgpt

# Get release history
helm history taskgpt-phase5 -n taskgpt

# Rollback to previous version
helm rollback taskgpt-phase5 -n taskgpt
```

## Common Use Cases

### 1. Local Development (Minikube)

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace
```

### 2. Cloud Deployment (DOKS/GKE/AKS)

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  --set postgres.enabled=false \
  --set redpanda.enabled=false \
  -f production-values.yaml
```

### 3. Disable Specific Services

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  --set notification.enabled=false \
  --set audit.enabled=false
```

### 4. Custom Image Registry

```bash
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  --set backend.image.repository="registry.example.com/todo-backend" \
  --set backend.image.tag="1.2.3"
```

## Troubleshooting

### View Deployed Resources

```bash
kubectl get all -n taskgpt
```

### Check Pod Logs

```bash
kubectl logs -l app.kubernetes.io/instance=taskgpt-phase5 -n taskgpt
```

### Describe Failed Pods

```bash
kubectl describe pod <pod-name> -n taskgpt
```

### Verify Helm Values

```bash
helm get values taskgpt-phase5 -n taskgpt
```

## Chart Structure

```
taskgpt-phase5/
├── Chart.yaml                 # Chart metadata
├── values.yaml               # Default values
├── templates/
│   ├── _helpers.tpl          # Template helpers
│   ├── NOTES.txt            # Post-install notes
│   ├── namespace.yaml        # Namespace
│   ├── configmap.yaml        # ConfigMap
│   ├── secrets.yaml          # Secrets
│   ├── backend-deployment.yaml
│   ├── backend-service.yaml
│   ├── backend-hpa.yaml
│   ├── notification-deployment.yaml
│   ├── recurring-deployment.yaml
│   ├── audit-deployment.yaml
│   ├── postgres-deployment.yaml
│   ├── redpanda-deployment.yaml
│   ├── ingress.yaml
│   └── dapr-config.yaml
└── README.md                # This file
```

## Support

For issues and questions:
- GitHub Issues: https://github.com/panaversity/AI-todo/issues
- Documentation: https://github.com/panaversity/AI-todo/wiki
