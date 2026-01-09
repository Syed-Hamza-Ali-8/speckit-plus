# Phase 5 Kubernetes Manifests

Complete Kubernetes deployment configuration for Phase V Todo application with Dapr integration.

## Directory Structure

```
k8s/
├── namespace.yaml              # Application namespace
├── configmap.yaml             # Shared configuration
├── secrets.yaml               # Sensitive data (credentials)
├── postgres-deployment.yaml   # PostgreSQL database
├── redpanda-deployment.yaml   # Kafka/Redpanda message broker
├── backend-deployment.yaml    # Main API service
├── notification-deployment.yaml    # Notification consumer
├── recurring-deployment.yaml  # Recurring task consumer
├── audit-deployment.yaml      # Audit trail consumer
├── ingress.yaml              # External access
├── dapr-config.yaml          # Dapr configuration
└── README.md                 # This file
```

## Prerequisites

1. **Kubernetes Cluster**: Minikube, DOKS, GKE, or AKS
2. **kubectl**: Kubernetes CLI tool
3. **Dapr**: Distributed Application Runtime
4. **Docker Images**: Built and available

### Install Dapr on Kubernetes

```bash
# Install Dapr CLI
curl -fsSL https://raw.githubusercontent.com/dapr/cli/master/install/install.sh | bash

# Initialize Dapr on Kubernetes
dapr init -k

# Verify installation
dapr status -k
```

## Quick Start

### 1. Create Namespace

```bash
kubectl apply -f k8s/namespace.yaml
```

### 2. Update Secrets

**IMPORTANT**: Update `secrets.yaml` with your actual credentials:

```bash
# Edit secrets file
nano k8s/secrets.yaml

# Or create secrets from command line
kubectl create secret generic postgres-secrets \
  --from-literal=connectionString="postgresql://..." \
  --namespace=taskgpt

kubectl create secret generic kafka-secrets \
  --from-literal=username="..." \
  --from-literal=password="..." \
  --from-literal=brokers="..." \
  --namespace=taskgpt
```

### 3. Apply All Manifests

```bash
# Apply in order
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/secrets.yaml

# Apply Dapr components
kubectl apply -f ../dapr-components/

# Deploy infrastructure
kubectl apply -f k8s/postgres-deployment.yaml
kubectl apply -f k8s/redpanda-deployment.yaml

# Wait for infrastructure to be ready
kubectl wait --for=condition=ready pod -l app=postgres -n taskgpt --timeout=120s
kubectl wait --for=condition=ready pod -l app=redpanda -n taskgpt --timeout=120s

# Deploy application services
kubectl apply -f k8s/dapr-config.yaml
kubectl apply -f k8s/backend-deployment.yaml
kubectl apply -f k8s/notification-deployment.yaml
kubectl apply -f k8s/recurring-deployment.yaml
kubectl apply -f k8s/audit-deployment.yaml

# Deploy ingress
kubectl apply -f k8s/ingress.yaml
```

### 4. Verify Deployment

```bash
# Check all pods
kubectl get pods -n taskgpt

# Check services
kubectl get svc -n taskgpt

# Check Dapr components
kubectl get components -n taskgpt

# View logs
kubectl logs -l app=todo-backend -n taskgpt -f
```

## Service Details

### Backend Service
- **Replicas**: 2 (with HPA up to 10)
- **Port**: 8000
- **Dapr**: Enabled with app-id "todo-backend"
- **Health Check**: GET /health

### Notification Service
- **Replicas**: 1
- **Purpose**: Processes reminder events
- **Dapr**: Enabled for Kafka consumption

### Recurring Task Service
- **Replicas**: 1
- **Purpose**: Generates recurring task instances
- **Dapr**: Enabled for event processing

### Audit Service
- **Replicas**: 1
- **Purpose**: Logs all task operations
- **Dapr**: Enabled for audit trail

## Accessing Services

### Local (Minikube)

```bash
# Enable ingress addon
minikube addons enable ingress

# Get Minikube IP
minikube ip

# Add to /etc/hosts
echo "$(minikube ip) api.todo.local" | sudo tee -a /etc/hosts

# Access API
curl http://api.todo.local/health
```

### Port Forwarding

```bash
# Backend API
kubectl port-forward -n taskgpt svc/todo-backend 8000:80

# Access at http://localhost:8000

# PostgreSQL (for debugging)
kubectl port-forward -n taskgpt svc/postgres 5432:5432

# Redpanda (for debugging)
kubectl port-forward -n taskgpt svc/redpanda 9092:9092
```

### Cloud (LoadBalancer)

```bash
# Get external IP
kubectl get ingress -n taskgpt

# Wait for IP assignment
kubectl get ingress -n taskgpt --watch

# Access via external IP or domain
curl http://<EXTERNAL-IP>/health
```

## Scaling

### Manual Scaling

```bash
# Scale backend
kubectl scale deployment todo-backend -n taskgpt --replicas=5

# Scale notification service
kubectl scale deployment todo-notification -n taskgpt --replicas=3
```

### Auto-Scaling (HPA)

Backend includes Horizontal Pod Autoscaler:
- Min: 2 replicas
- Max: 10 replicas
- Triggers: CPU > 70%, Memory > 80%

```bash
# View HPA status
kubectl get hpa -n taskgpt

# Describe HPA
kubectl describe hpa todo-backend-hpa -n taskgpt
```

## Monitoring & Debugging

### View Logs

```bash
# All logs for a service
kubectl logs -l app=todo-backend -n taskgpt

# Follow logs
kubectl logs -l app=todo-backend -n taskgpt -f

# Logs from specific container (with Dapr sidecar)
kubectl logs <pod-name> -n taskgpt -c backend
kubectl logs <pod-name> -n taskgpt -c daprd
```

### Pod Status

```bash
# List all pods
kubectl get pods -n taskgpt

# Describe pod
kubectl describe pod <pod-name> -n taskgpt

# Get events
kubectl get events -n taskgpt --sort-by='.lastTimestamp'
```

### Exec into Pod

```bash
# Get a shell
kubectl exec -it <pod-name> -n taskgpt -c backend -- /bin/sh

# Run command
kubectl exec <pod-name> -n taskgpt -c backend -- env
```

### Check Dapr

```bash
# List Dapr components
kubectl get components -n taskgpt

# Check Dapr sidecar logs
kubectl logs <pod-name> -n taskgpt -c daprd

# Dapr dashboard
dapr dashboard -k -p 9999
```

## Troubleshooting

### Pod Not Starting

```bash
# Check pod status
kubectl describe pod <pod-name> -n taskgpt

# Check events
kubectl get events -n taskgpt | grep <pod-name>

# Check image pull
kubectl get pods -n taskgpt | grep ImagePull
```

### Database Connection Issues

```bash
# Test PostgreSQL connectivity
kubectl run -it --rm psql-test --image=postgres:16-alpine -n taskgpt -- \
  psql "postgresql://postgres:postgres@postgres:5432/todo" -c "\l"

# Check secret
kubectl get secret postgres-secrets -n taskgpt -o yaml
```

### Kafka Connection Issues

```bash
# Test Redpanda connectivity
kubectl run -it --rm kafka-test --image=redpandadata/redpanda:latest -n taskgpt -- \
  rpk cluster info --brokers redpanda:9092

# List topics
kubectl exec -it deploy/redpanda -n taskgpt -- rpk topic list
```

### Dapr Not Working

```bash
# Check Dapr installation
dapr status -k

# Check Dapr components
kubectl get components -n taskgpt

# Check Dapr sidecar injection
kubectl get pods -n taskgpt -o jsonpath='{range .items[*]}{.metadata.name}{"\t"}{.spec.containers[*].name}{"\n"}{end}'
```

## Resource Requirements

### Minimum (Development)
- CPU: 2 cores
- Memory: 4 GB
- Storage: 10 GB

### Recommended (Production)
- CPU: 8 cores
- Memory: 16 GB
- Storage: 50 GB

## Production Considerations

### 1. Secrets Management
- Use External Secrets Operator or Sealed Secrets
- Never commit real credentials to Git
- Rotate secrets regularly

### 2. Database
- Use managed PostgreSQL (Neon, RDS, Cloud SQL)
- Remove local postgres-deployment.yaml
- Use connection pooling (PgBouncer)

### 3. Kafka
- Use Redpanda Cloud or managed Kafka
- Remove local redpanda-deployment.yaml
- Configure proper retention policies

### 4. Monitoring
- Deploy Prometheus + Grafana
- Configure alerting rules
- Set up log aggregation (ELK, Loki)

### 5. Security
- Enable Dapr mTLS
- Use Network Policies
- Configure RBAC
- Enable Pod Security Standards

### 6. High Availability
- Multi-zone deployment
- Pod Disruption Budgets
- Readiness/Liveness probes
- Resource requests/limits

### 7. TLS/SSL
- Use cert-manager for TLS certificates
- Enable HTTPS on ingress
- Configure TLS for all services

## Cleanup

```bash
# Delete all resources
kubectl delete namespace taskgpt

# Or delete individually
kubectl delete -f k8s/ --recursive

# Uninstall Dapr
dapr uninstall -k
```

## Next Steps

1. Test local deployment on Minikube
2. Create Helm chart for templating
3. Set up CI/CD pipeline
4. Deploy to cloud (DOKS/GKE/AKS)
5. Configure monitoring and alerting
6. Implement backup and disaster recovery

## References

- [Kubernetes Documentation](https://kubernetes.io/docs/)
- [Dapr on Kubernetes](https://docs.dapr.io/operations/hosting/kubernetes/)
- [Kubectl Cheat Sheet](https://kubernetes.io/docs/reference/kubectl/cheatsheet/)
