# Phase V Deployment Guide

Complete guide for deploying TaskGPT Phase V with event-driven architecture, microservices, and Dapr integration.

## Table of Contents

1. [Prerequisites](#prerequisites)
2. [Local Development Setup](#local-development-setup)
3. [Minikube Deployment](#minikube-deployment)
4. [Cloud Deployment](#cloud-deployment)
5. [Configuration](#configuration)
6. [Monitoring](#monitoring)
7. [Troubleshooting](#troubleshooting)
8. [Production Checklist](#production-checklist)

## Prerequisites

### Required Tools

| Tool | Version | Purpose |
|------|---------|---------|
| Docker | 24.0+ | Container runtime |
| Docker Compose | 2.20+ | Local orchestration |
| kubectl | 1.28+ | Kubernetes CLI |
| Helm | 3.13+ | Package manager |
| Dapr CLI | 1.12+ | Distributed runtime |
| Minikube | 1.32+ | Local Kubernetes |

### Installation

#### Docker & Docker Compose
```bash
# Linux
curl -fsSL https://get.docker.com | sh
sudo usermod -aG docker $USER

# Verify
docker --version
docker-compose --version
```

#### kubectl
```bash
# Linux
curl -LO "https://dl.k8s.io/release/$(curl -L -s https://dl.k8s.io/release/stable.txt)/bin/linux/amd64/kubectl"
sudo install -o root -g root -m 0755 kubectl /usr/local/bin/kubectl

# Verify
kubectl version --client
```

#### Helm
```bash
curl https://raw.githubusercontent.com/helm/helm/main/scripts/get-helm-3 | bash

# Verify
helm version
```

#### Dapr CLI
```bash
curl -fsSL https://raw.githubusercontent.com/dapr/cli/master/install/install.sh | bash

# Verify
dapr --version
```

#### Minikube
```bash
# Linux
curl -LO https://storage.googleapis.com/minikube/releases/latest/minikube-linux-amd64
sudo install minikube-linux-amd64 /usr/local/bin/minikube

# Verify
minikube version
```

## Local Development Setup

### Option 1: Docker Compose (Recommended for Development)

Quick start for local development without Kubernetes.

#### 1. Clone Repository
```bash
git clone https://github.com/panaversity/AI-todo.git
cd AI-todo/phase-5
```

#### 2. Configure Environment
```bash
# Copy example environment file
cp .env.example .env

# Edit with your values
nano .env
```

Required environment variables:
```bash
DATABASE_URL=postgresql://postgres:postgres@postgres:5432/todo
KAFKA_BOOTSTRAP_SERVERS=redpanda:9092
OPENAI_API_KEY=sk-your-key-here
```

#### 3. Build Images
```bash
docker-compose build
```

#### 4. Start Services
```bash
docker-compose up -d
```

#### 5. Verify Deployment
```bash
# Check running services
docker-compose ps

# View logs
docker-compose logs -f backend

# Test health endpoint
curl http://localhost:8000/health

# Access Redpanda Console
open http://localhost:8080
```

#### 6. Stop Services
```bash
# Stop all services
docker-compose down

# Stop and remove volumes
docker-compose down -v
```

### Option 2: Local Python Development

For backend development without containers.

#### 1. Setup Virtual Environment
```bash
cd phase-5
python -m venv venv
source venv/bin/activate  # Linux/Mac
# or
venv\Scripts\activate  # Windows
```

#### 2. Install Dependencies
```bash
pip install -r requirements.txt
```

#### 3. Start Infrastructure
```bash
# Start only PostgreSQL and Redpanda
docker-compose up -d postgres redpanda
```

#### 4. Run Backend
```bash
export DATABASE_URL="postgresql://postgres:postgres@localhost:5432/todo"
export KAFKA_BOOTSTRAP_SERVERS="localhost:9092"

uvicorn src.main:app --reload --port 8000
```

## Minikube Deployment

Deploy complete Phase V stack to local Kubernetes cluster.

### Step 1: Start Minikube

```bash
# Start with sufficient resources
minikube start --cpus=4 --memory=8192 --disk-size=20g

# Enable required addons
minikube addons enable ingress
minikube addons enable metrics-server

# Verify cluster
kubectl cluster-info
```

### Step 2: Install Dapr

```bash
# Initialize Dapr on Kubernetes
dapr init -k

# Verify Dapr installation
dapr status -k

# Expected output:
# NAME                   NAMESPACE    HEALTHY  STATUS   REPLICAS  VERSION  AGE  CREATED
# dapr-sidecar-injector  dapr-system  True     Running  1         1.12.0   1m   2024-01-05 12:00.00
# dapr-sentry            dapr-system  True     Running  1         1.12.0   1m   2024-01-05 12:00.00
# dapr-operator          dapr-system  True     Running  1         1.12.0   1m   2024-01-05 12:00.00
# dapr-placement         dapr-system  True     Running  1         1.12.0   1m   2024-01-05 12:00.00
```

### Step 3: Build Docker Images

```bash
# Use Minikube's Docker daemon
eval $(minikube docker-env)

# Build all images
cd phase-5

docker build -t todo-backend:latest -f Dockerfile .
docker build -t todo-notification:latest -f docker/Dockerfile.notification .
docker build -t todo-recurring:latest -f docker/Dockerfile.recurring .
docker build -t todo-audit:latest -f docker/Dockerfile.audit .

# Verify images
docker images | grep todo
```

### Step 4: Deploy with Kubernetes Manifests

```bash
# Apply all manifests in order
kubectl apply -f k8s/namespace.yaml
kubectl apply -f k8s/configmap.yaml
kubectl apply -f k8s/secrets.yaml

# Apply Dapr components
kubectl apply -f dapr-components/

# Deploy infrastructure
kubectl apply -f k8s/postgres-deployment.yaml
kubectl apply -f k8s/redpanda-deployment.yaml

# Wait for infrastructure
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

### Step 5: Deploy with Helm (Alternative)

```bash
# Install with default values
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace

# Or with custom values
helm install taskgpt-phase5 ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  -f custom-values.yaml
```

### Step 6: Verify Deployment

```bash
# Check all pods
kubectl get pods -n taskgpt

# Should see:
# NAME                              READY   STATUS    RESTARTS   AGE
# postgres-...                      1/1     Running   0          2m
# redpanda-...                      1/1     Running   0          2m
# todo-backend-...                  2/2     Running   0          1m
# todo-notification-...             2/2     Running   0          1m
# todo-recurring-...                2/2     Running   0          1m
# todo-audit-...                    2/2     Running   0          1m

# Check Dapr components
kubectl get components -n taskgpt

# View logs
kubectl logs -l app=todo-backend -n taskgpt -c backend
```

### Step 7: Access Application

```bash
# Get Minikube IP
minikube ip

# Add to /etc/hosts
echo "$(minikube ip) api.todo.local" | sudo tee -a /etc/hosts

# Test API
curl http://api.todo.local/health

# Or use port-forward
kubectl port-forward -n taskgpt svc/todo-backend 8000:80

# Access at http://localhost:8000
```

### Step 8: Monitor and Debug

```bash
# View all resources
kubectl get all -n taskgpt

# Describe pod
kubectl describe pod <pod-name> -n taskgpt

# View events
kubectl get events -n taskgpt --sort-by='.lastTimestamp'

# Exec into pod
kubectl exec -it <pod-name> -n taskgpt -c backend -- /bin/sh

# Dapr dashboard
dapr dashboard -k -p 9999
# Open http://localhost:9999
```

## Cloud Deployment

Deploy to production Kubernetes cluster (DigitalOcean, GCP, Azure).

### Prerequisites

- Cloud Kubernetes cluster (DOKS/GKE/AKS)
- Container registry access
- Domain name (optional)
- SSL certificate (for production)

### Option 1: DigitalOcean Kubernetes (DOKS)

#### Step 1: Create Cluster

```bash
# Install doctl
snap install doctl
# or
brew install doctl

# Authenticate
doctl auth init

# Create cluster
doctl kubernetes cluster create taskgpt-prod \
  --region nyc1 \
  --node-pool "name=worker-pool;size=s-2vcpu-4gb;count=3" \
  --wait

# Get kubeconfig
doctl kubernetes cluster kubeconfig save taskgpt-prod
```

#### Step 2: Setup Container Registry

```bash
# Create registry
doctl registry create taskgpt

# Login to registry
doctl registry login

# Tag and push images
docker tag todo-backend:latest registry.digitalocean.com/taskgpt/backend:1.0.0
docker push registry.digitalocean.com/taskgpt/backend:1.0.0

# Repeat for all services
```

#### Step 3: Install Dapr

```bash
dapr init -k
```

#### Step 4: Configure Secrets

```bash
# PostgreSQL (use Neon or managed database)
kubectl create secret generic postgres-secrets \
  --from-literal=connectionString="postgresql://user:pass@neon-host:5432/db?sslmode=require" \
  --namespace=taskgpt

# Kafka (use Redpanda Cloud)
kubectl create secret generic kafka-secrets \
  --from-literal=username="your-username" \
  --from-literal=password="your-password" \
  --from-literal=brokers="your-cluster.cloud.redpanda.com:9092" \
  --namespace=taskgpt

# OpenAI
kubectl create secret generic openai-secrets \
  --from-literal=api-key="sk-your-key" \
  --namespace=taskgpt
```

#### Step 5: Deploy with Helm

```bash
# Create production values
cat > production-values.yaml <<EOF
app:
  environment: production

backend:
  replicaCount: 3
  image:
    repository: registry.digitalocean.com/taskgpt/backend
    tag: "1.0.0"
  autoscaling:
    minReplicas: 3
    maxReplicas: 20

postgres:
  enabled: false  # Using Neon

redpanda:
  enabled: false  # Using Redpanda Cloud

ingress:
  hosts:
    - host: api.yourdomain.com
      paths:
        - path: /
          pathType: Prefix
  tls:
    - secretName: todo-tls
      hosts:
        - api.yourdomain.com
EOF

# Deploy
helm install taskgpt-prod ./helm/taskgpt-phase5 \
  --namespace taskgpt \
  --create-namespace \
  -f production-values.yaml
```

#### Step 6: Setup SSL/TLS

```bash
# Install cert-manager
kubectl apply -f https://github.com/cert-manager/cert-manager/releases/download/v1.13.0/cert-manager.yaml

# Create ClusterIssuer
cat <<EOF | kubectl apply -f -
apiVersion: cert-manager.io/v1
kind: ClusterIssuer
metadata:
  name: letsencrypt-prod
spec:
  acme:
    server: https://acme-v02.api.letsencrypt.org/directory
    email: your-email@example.com
    privateKeySecretRef:
      name: letsencrypt-prod
    solvers:
    - http01:
        ingress:
          class: nginx
EOF

# Update ingress annotation
kubectl annotate ingress taskgpt-prod-ingress \
  cert-manager.io/cluster-issuer=letsencrypt-prod \
  -n taskgpt
```

### Option 2: Google Kubernetes Engine (GKE)

```bash
# Install gcloud CLI
curl https://sdk.cloud.google.com | bash

# Authenticate
gcloud auth login

# Create cluster
gcloud container clusters create taskgpt-prod \
  --num-nodes=3 \
  --machine-type=e2-standard-4 \
  --zone=us-central1-a

# Get credentials
gcloud container clusters get-credentials taskgpt-prod

# Follow similar steps as DOKS
```

### Option 3: Azure Kubernetes Service (AKS)

```bash
# Install Azure CLI
curl -sL https://aka.ms/InstallAzureCLIDeb | sudo bash

# Login
az login

# Create resource group
az group create --name taskgpt-rg --location eastus

# Create cluster
az aks create \
  --resource-group taskgpt-rg \
  --name taskgpt-prod \
  --node-count 3 \
  --node-vm-size Standard_D4s_v3 \
  --enable-addons monitoring

# Get credentials
az aks get-credentials --resource-group taskgpt-rg --name taskgpt-prod

# Follow similar steps as DOKS
```

## Configuration

### Environment-Specific Values

#### Development
```yaml
# dev-values.yaml
app:
  environment: development

backend:
  replicaCount: 1
  resources:
    requests:
      memory: "128Mi"
      cpu: "100m"

postgres:
  enabled: true  # Local database

redpanda:
  enabled: true  # Local Kafka
```

#### Staging
```yaml
# staging-values.yaml
app:
  environment: staging

backend:
  replicaCount: 2
  resources:
    requests:
      memory: "256Mi"
      cpu: "250m"

postgres:
  enabled: false  # Managed database

redpanda:
  enabled: false  # Redpanda Cloud
```

#### Production
```yaml
# production-values.yaml
app:
  environment: production

backend:
  replicaCount: 3
  autoscaling:
    enabled: true
    minReplicas: 3
    maxReplicas: 20
  resources:
    requests:
      memory: "512Mi"
      cpu: "500m"
    limits:
      memory: "1Gi"
      cpu: "1000m"

postgres:
  enabled: false

redpanda:
  enabled: false

config:
  dapr:
    enableMtls: true

ingress:
  tls:
    - secretName: todo-tls
      hosts:
        - api.yourdomain.com
```

## Monitoring

### Logs

```bash
# View all backend logs
kubectl logs -l app.kubernetes.io/component=backend -n taskgpt --tail=100 -f

# View specific service
kubectl logs -l app=todo-notification -n taskgpt -f

# View Dapr sidecar logs
kubectl logs <pod-name> -n taskgpt -c daprd
```

### Metrics

```bash
# Pod resource usage
kubectl top pods -n taskgpt

# Node resource usage
kubectl top nodes

# HPA status
kubectl get hpa -n taskgpt
```

### Health Checks

```bash
# Backend health
curl https://api.yourdomain.com/health

# Check all pods
kubectl get pods -n taskgpt

# Describe failing pod
kubectl describe pod <pod-name> -n taskgpt
```

### Dapr Dashboard

```bash
# Open Dapr dashboard
dapr dashboard -k -p 9999

# Access at http://localhost:9999
```

## Troubleshooting

### Common Issues

#### 1. Pod Not Starting

```bash
# Check events
kubectl describe pod <pod-name> -n taskgpt

# Common causes:
# - Image pull errors
# - Resource constraints
# - Configuration errors
```

#### 2. Database Connection Failed

```bash
# Test connectivity
kubectl run -it --rm psql-test --image=postgres:16-alpine -n taskgpt -- \
  psql "postgresql://..." -c "\l"

# Check secret
kubectl get secret postgres-secrets -n taskgpt -o yaml
```

#### 3. Kafka Connection Issues

```bash
# Check Redpanda status
kubectl exec -it deploy/redpanda -n taskgpt -- rpk cluster health

# List topics
kubectl exec -it deploy/redpanda -n taskgpt -- rpk topic list
```

#### 4. Dapr Sidecar Not Injecting

```bash
# Check annotations
kubectl get pod <pod-name> -n taskgpt -o yaml | grep dapr

# Verify Dapr installation
dapr status -k

# Reinstall Dapr
dapr uninstall -k
dapr init -k
```

#### 5. Ingress Not Working

```bash
# Check ingress
kubectl get ingress -n taskgpt
kubectl describe ingress <ingress-name> -n taskgpt

# Check ingress controller
kubectl get pods -n ingress-nginx
```

## Production Checklist

### Before Deployment

- [ ] **Secrets configured** in Kubernetes
- [ ] **Database backups** enabled
- [ ] **Monitoring** setup (Prometheus/Grafana)
- [ ] **Logging** aggregation configured
- [ ] **SSL/TLS certificates** installed
- [ ] **Resource limits** set appropriately
- [ ] **HPA** configured and tested
- [ ] **Dapr mTLS** enabled
- [ ] **Network policies** applied
- [ ] **Pod disruption budgets** configured
- [ ] **DNS records** configured
- [ ] **Load testing** completed

### Post-Deployment

- [ ] **Health checks** passing
- [ ] **Smoke tests** passing
- [ ] **Logs** flowing correctly
- [ ] **Metrics** being collected
- [ ] **Alerts** configured
- [ ] **Backup verification** completed
- [ ] **Disaster recovery** tested
- [ ] **Documentation** updated
- [ ] **Team notified**

### Security

- [ ] **Secrets rotation** scheduled
- [ ] **RBAC** configured
- [ ] **Network policies** enforced
- [ ] **Pod security** standards enabled
- [ ] **Image scanning** in CI/CD
- [ ] **Vulnerability patching** process
- [ ] **Audit logging** enabled
- [ ] **Compliance requirements** met

## Additional Resources

- [Kubernetes Documentation](https://kubernetes.io/docs/)
- [Helm Documentation](https://helm.sh/docs/)
- [Dapr Documentation](https://docs.dapr.io/)
- [Docker Documentation](https://docs.docker.com/)
- [DigitalOcean Kubernetes](https://docs.digitalocean.com/products/kubernetes/)
- [Redpanda Cloud](https://docs.redpanda.com/docs/get-started/cloud/)
- [Neon Database](https://neon.tech/docs/)

## Support

For issues and questions:
- GitHub Issues: https://github.com/panaversity/AI-todo/issues
- Documentation: https://github.com/panaversity/AI-todo/wiki
- Discussions: https://github.com/panaversity/AI-todo/discussions
