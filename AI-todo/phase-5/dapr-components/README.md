# Dapr Components Configuration

This directory contains Dapr component configurations for the Phase V Todo application.

## Components Overview

| Component | File | Purpose |
|-----------|------|---------|
| **Pub/Sub** | `kafka-pubsub.yaml` | Event streaming with Kafka/Redpanda |
| **State Store** | `statestore.yaml` | Conversation and cache state using PostgreSQL |
| **Cron Bindings** | `reminder-cron.yaml` | Scheduled tasks for reminders and recurring tasks |
| **Secrets** | `secrets.yaml` | Secure credential management |

## Quick Start

### Local Development (Minikube)

1. **Install Dapr on Kubernetes:**
   ```bash
   dapr init -k
   ```

2. **Create namespace:**
   ```bash
   kubectl create namespace taskgpt
   ```

3. **Apply Dapr components:**
   ```bash
   kubectl apply -f phase-5/dapr-components/
   ```

4. **Verify components:**
   ```bash
   kubectl get components -n default
   ```

### Configuration for Different Environments

#### Local Development
- Uses local Redpanda/Kafka on `redpanda:9092`
- PostgreSQL connection without secrets
- Local file-based secrets

#### Production (Cloud)
- Redpanda Cloud with SASL_SSL authentication
- Neon PostgreSQL with secrets
- Kubernetes secrets store

## Component Details

### 1. Kafka Pub/Sub (`kafka-pubsub.yaml`)

**Topics:**
- `task-events` - Task CRUD operations
- `reminders` - Scheduled reminder triggers
- `task-updates` - Real-time client synchronization

**Configuration:**
```yaml
# Local
brokers: "redpanda:9092"

# Production
brokers: "YOUR-CLUSTER.cloud.redpanda.com:9092"
authType: "password"
securityProtocol: "SASL_SSL"
```

**Usage in application:**
```python
import httpx

# Publish event
await httpx.post(
    "http://localhost:3500/v1.0/publish/kafka-pubsub/task-events",
    json={"event_type": "created", "task_id": 1}
)

# Subscribe (handled by Dapr automatically via app endpoint)
@app.post("/task-events")
async def handle_task_event(event: dict):
    # Process event
    pass
```

### 2. State Store (`statestore.yaml`)

**Storage:**
- Conversation history
- User session data
- Cached task data

**Usage in application:**
```python
import httpx

# Save state
await httpx.post(
    "http://localhost:3500/v1.0/state/statestore",
    json=[{
        "key": f"conversation-{conv_id}",
        "value": {"messages": messages}
    }]
)

# Get state
response = await httpx.get(
    f"http://localhost:3500/v1.0/state/statestore/conversation-{conv_id}"
)
state = response.json()
```

### 3. Cron Bindings (`reminder-cron.yaml`)

**Schedules:**
- `reminder-cron` - Every 5 minutes (check reminders)
- `hourly-tasks-cron` - Every hour (process recurring tasks)
- `daily-cleanup-cron` - 2 AM daily (cleanup old data)

**Usage in application:**
```python
# Dapr will POST to this endpoint on schedule
@app.post("/reminder-cron")
async def check_reminders():
    # Check for due reminders
    # Send notifications
    return {"status": "ok"}

@app.post("/hourly-tasks-cron")
async def process_recurring_tasks():
    # Generate next occurrences
    return {"status": "ok"}
```

### 4. Secrets (`secrets.yaml`)

**Stored secrets:**
- PostgreSQL connection string
- Kafka credentials
- OpenAI API key

**Create secrets:**
```bash
# PostgreSQL
kubectl create secret generic postgres-secrets \
  --from-literal=connectionString="host=... user=... password=..."

# Kafka
kubectl create secret generic kafka-secrets \
  --from-literal=username="..." \
  --from-literal=password="..."

# OpenAI
kubectl create secret generic openai-secrets \
  --from-literal=api-key="sk-..."
```

**Usage in application:**
```python
import httpx

# Get secret
response = await httpx.get(
    "http://localhost:3500/v1.0/secrets/kubernetes-secrets/openai-api-key"
)
api_key = response.json()["openai-api-key"]
```

## Deployment Annotations

Add these annotations to your Kubernetes deployment:

```yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: todo-backend
spec:
  template:
    metadata:
      annotations:
        dapr.io/enabled: "true"
        dapr.io/app-id: "todo-backend"
        dapr.io/app-port: "8000"
        dapr.io/log-level: "info"
        # Subscribe to topics
        dapr.io/enable-api-logging: "true"
    spec:
      containers:
      - name: backend
        image: todo-backend:latest
        ports:
        - containerPort: 8000
```

## Testing Dapr Components

### Test Pub/Sub
```bash
# Publish a test event
curl -X POST http://localhost:3500/v1.0/publish/kafka-pubsub/task-events \
  -H "Content-Type: application/json" \
  -d '{"event_type": "test", "task_id": 999}'
```

### Test State Store
```bash
# Save state
curl -X POST http://localhost:3500/v1.0/state/statestore \
  -H "Content-Type: application/json" \
  -d '[{"key": "test-key", "value": {"data": "test"}}]'

# Get state
curl http://localhost:3500/v1.0/state/statestore/test-key
```

### Test Secrets
```bash
# Get secret (requires secret to exist)
curl http://localhost:3500/v1.0/secrets/kubernetes-secrets/openai-api-key
```

## Troubleshooting

### Component not loading
```bash
# Check component status
kubectl get components -n default

# Check Dapr logs
kubectl logs -l app=dapr-sidecar-injector -n dapr-system

# Describe component
kubectl describe component kafka-pubsub -n default
```

### Connection issues
```bash
# Test connectivity from pod
kubectl exec -it <pod-name> -- curl http://localhost:3500/v1.0/healthz

# Check Dapr sidecar logs
kubectl logs <pod-name> -c daprd
```

### Secret not found
```bash
# List secrets
kubectl get secrets -n default

# Verify secret content
kubectl get secret postgres-secrets -o yaml
```

## References

- [Dapr Pub/Sub](https://docs.dapr.io/developing-applications/building-blocks/pubsub/)
- [Dapr State Management](https://docs.dapr.io/developing-applications/building-blocks/state-management/)
- [Dapr Bindings](https://docs.dapr.io/developing-applications/building-blocks/bindings/)
- [Dapr Secrets](https://docs.dapr.io/developing-applications/building-blocks/secrets/)
- [Kafka Component](https://docs.dapr.io/reference/components-reference/supported-pubsub/setup-apache-kafka/)
- [PostgreSQL State Store](https://docs.dapr.io/reference/components-reference/supported-state-stores/setup-postgresql/)
