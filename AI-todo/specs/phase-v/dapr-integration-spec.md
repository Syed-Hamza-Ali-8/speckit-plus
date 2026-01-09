# Dapr Integration Specification for Phase V

## Overview
This document specifies the Dapr (Distributed Application Runtime) integration for Phase V, providing building blocks for state management, pub/sub, service invocation, bindings, and secrets management in a Kubernetes environment.

## Architecture Goals
- Abstract infrastructure dependencies behind standard HTTP/gRPC APIs
- Enable loose coupling between services
- Provide built-in resilience patterns (retries, circuit breakers)
- Simplify service-to-service communication
- Secure secret management
- Enable platform portability

## Dapr Building Blocks

### 1. Pub/Sub Building Block (Kafka)
**Purpose**: Event-driven communication between services using Kafka
**Component Type**: `pubsub.kafka`

**Configuration**:
```yaml
# dapr-components/pubsub-kafka.yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: kafka-pubsub
spec:
  type: pubsub.kafka
  version: v1
  metadata:
  - name: brokers
    value: "kafka:9092"  # For local; use Redpanda Cloud URL in production
  - name: consumerGroup
    value: "todo-service-group"
  - name: authRequired
    value: "true"
  - name: saslUsername
    secretKeyRef:
      name: kafka-username
      key: username
  - name: saslPassword
    secretKeyRef:
      name: kafka-password
      key: password
  - name: saslMechanism
    value: "SCRAM-SHA-256"
  - name: maxMessageBytes
    value: "1048576"
  - name: consumeRetryInterval
    value: "100ms"
```

**Usage in Services**:
```python
# Publishing events via Dapr
import httpx

async def publish_task_event(event_type: str, task_data: dict, user_id: str):
    event = {
        "event_type": event_type,
        "task_id": task_data.get("id"),
        "user_id": user_id,
        "timestamp": datetime.utcnow().isoformat(),
        "task_data": task_data
    }

    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"http://localhost:3500/v1.0/publish/kafka-pubsub/task-events",
            json=event
        )
        return response.status_code == 200

# Subscribing to events via Dapr
from fastapi import FastAPI
import uvicorn

app = FastAPI()

@app.post("/reminders")
async def handle_reminder_event(data: dict):
    # Process reminder event
    user_id = data["user_id"]
    task_title = data["title"]
    # Send notification logic here
    return {"status": "processed"}

# Dapr subscription annotation
@app.get("/dapr/subscribe")
async def subscribe():
    return [
        {
            "pubsubname": "kafka-pubsub",
            "topic": "reminders",
            "route": "/reminders"
        }
    ]
```

### 2. State Management Building Block (PostgreSQL)
**Purpose**: Store conversation state and other transient data
**Component Type**: `state.postgresql`

**Configuration**:
```yaml
# dapr-components/state-postgresql.yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: statestore
spec:
  type: state.postgresql
  version: v1
  metadata:
  - name: connectionString
    secretKeyRef:
      name: postgres-connection
      key: connection-string
  - name: actorStateStore
    value: "true"
  - name: concurrency
    value: "first-write"
  - name: keyPrefix
    value: "none"
```

**Usage in Services**:
```python
# Storing conversation state via Dapr
async def save_conversation_state(conversation_id: str, state_data: dict):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"http://localhost:3500/v1.0/state/statestore",
            json=[
                {
                    "key": f"conversation-{conversation_id}",
                    "value": state_data,
                    "options": {
                        "consistency": "strong"
                    }
                }
            ]
        )
        return response.status_code == 200

# Retrieving conversation state via Dapr
async def get_conversation_state(conversation_id: str):
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"http://localhost:3500/v1.0/state/statestore/conversation-{conversation_id}"
        )
        if response.status_code == 200:
            return response.json()
        return None

# Deleting conversation state
async def delete_conversation_state(conversation_id: str):
    async with httpx.AsyncClient() as client:
        response = await client.delete(
            f"http://localhost:3500/v1.0/state/statestore/conversation-{conversation_id}"
        )
        return response.status_code == 200
```

### 3. Service Invocation Building Block
**Purpose**: Enable service-to-service communication with built-in resilience
**Component Type**: Built-in Dapr feature

**Usage in Services**:
```python
# Invoking other services via Dapr
async def call_notification_service(user_id: str, message: str):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"http://localhost:3500/v1.0/invoke/notification-service/method/send-notification",
            json={
                "user_id": user_id,
                "message": message
            }
        )
        return response.json()

# Example of calling the recurring task service
async def trigger_recurring_task_logic(task_data: dict):
    async with httpx.AsyncClient() as client:
        response = await client.post(
            f"http://localhost:3500/v1.0/invoke/recurring-task-service/method/process-completion",
            json=task_data
        )
        return response.json()
```

### 4. Bindings Building Block (Cron)
**Purpose**: Trigger scheduled operations like reminder checks
**Component Type**: `bindings.cron`

**Configuration**:
```yaml
# dapr-components/binding-cron.yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: reminder-cron
spec:
  type: bindings.cron
  version: v1
  metadata:
  - name: schedule
    value: "*/5 * * * *"  # Every 5 minutes
```

**Usage in Services**:
```python
# Handler for cron-triggered events
@app.post("/reminder-cron")
async def check_reminders():
    """
    This endpoint is called by Dapr every 5 minutes
    according to the cron schedule in the binding configuration
    """
    # Check for due tasks and send notifications
    due_tasks = await get_due_tasks_within_window(minutes=5)

    for task in due_tasks:
        # Publish reminder event
        reminder_event = {
            "task_id": task.id,
            "user_id": task.user_id,
            "title": task.title,
            "due_at": task.due_date.isoformat(),
            "remind_at": datetime.utcnow().isoformat(),
            "notification_method": "push"
        }

        # Publish via Dapr pub/sub
        await publish_reminder_event(reminder_event)

    return {"status": "checked", "due_tasks_count": len(due_tasks)}

def publish_reminder_event(reminder_data: dict):
    async with httpx.AsyncClient() as client:
        return await client.post(
            f"http://localhost:3500/v1.0/publish/kafka-pubsub/reminders",
            json=reminder_data
        )
```

### 5. Secret Store Building Block (Kubernetes)
**Purpose**: Securely store and access sensitive information
**Component Type**: `secretstores.kubernetes`

**Configuration**:
```yaml
# dapr-components/secretstore-kubernetes.yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: kubernetes-secrets
spec:
  type: secretstores.kubernetes
  version: v1
  metadata:
  - name: namespace
    value: "default"
```

**Usage in Services**:
```python
# Accessing secrets via Dapr
async def get_secret(secret_name: str, key: str):
    async with httpx.AsyncClient() as client:
        response = await client.get(
            f"http://localhost:3500/v1.0/secrets/kubernetes-secrets/{secret_name}"
        )
        if response.status_code == 200:
            secrets = response.json()
            return secrets.get(key)
        return None

# Example usage
async def get_database_connection_string():
    return await get_secret("postgres-connection", "connection-string")

async def get_openai_api_key():
    return await get_secret("openai-api-key", "api-key")
```

## Dapr Sidecar Configuration

### 1. Backend Service with Dapr Sidecar
```yaml
# deployments/backend-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: backend-service
  labels:
    app: backend-service
spec:
  replicas: 2
  selector:
    matchLabels:
      app: backend-service
  template:
    metadata:
      labels:
        app: backend-service
      annotations:
        dapr.io/enabled: "true"
        dapr.io/app-id: "backend-service"
        dapr.io/app-port: "8000"
        dapr.io/app-protocol: "http"
        dapr.io/app-max-concurrency: "10"
        dapr.io/enable-api-logging: "true"
    spec:
      containers:
      - name: backend
        image: todo-backend:latest
        ports:
        - containerPort: 8000
        env:
        - name: DAPR_HTTP_ENDPOINT
          value: "http://localhost:3500"
        - name: DAPR_GRPC_ENDPOINT
          value: "http://localhost:50001"
---
apiVersion: v1
kind: Service
metadata:
  name: backend-service
spec:
  selector:
    app: backend-service
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8000
  type: ClusterIP
```

### 2. Notification Service with Dapr Sidecar
```yaml
# deployments/notification-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: notification-service
  labels:
    app: notification-service
spec:
  replicas: 1
  selector:
    matchLabels:
      app: notification-service
  template:
    metadata:
      labels:
        app: notification-service
      annotations:
        dapr.io/enabled: "true"
        dapr.io/app-id: "notification-service"
        dapr.io/app-port: "8001"
        dapr.io/enable-api-logging: "true"
    spec:
      containers:
      - name: notification
        image: todo-notification:latest
        ports:
        - containerPort: 8001
---
apiVersion: v1
kind: Service
metadata:
  name: notification-service
spec:
  selector:
    app: notification-service
  ports:
  - protocol: TCP
    port: 80
    targetPort: 8001
  type: ClusterIP
```

## Dapr Operations in Kubernetes

### 1. Installing Dapr on Minikube
```bash
# Install Dapr CLI
wget -q https://raw.githubusercontent.com/dapr/cli/master/install/install.sh -O - | /bin/bash

# Initialize Dapr on Kubernetes
dapr init -k

# Verify installation
kubectl get pods -n dapr-system
```

### 2. Deploying Dapr Components
```bash
# Apply Dapr components
kubectl apply -f dapr-components/

# Verify components
kubectl get components.dapr.io
```

### 3. Deploying Applications with Dapr
```bash
# Deploy all services
kubectl apply -f deployments/

# Check application status
kubectl get pods
kubectl logs <pod-name> -c dapr-sidecar
```

## Integration with Existing Architecture

### 1. MCP Tools Integration with Dapr
```python
# MCP tool using Dapr for state management
async def list_tasks_mcp_tool(user_id: str, status: str = "all"):
    # Get conversation context if needed
    conversation_state = await get_conversation_state(f"user-{user_id}")

    # Use existing database logic to fetch tasks
    tasks = await get_tasks_from_db(user_id, status)

    # Store or update conversation context if needed
    if conversation_state:
        conversation_state["last_action"] = "list_tasks"
        await save_conversation_state(f"user-{user_id}", conversation_state)

    return tasks

# MCP tool using Dapr for pub/sub
async def create_task_mcp_tool(user_id: str, title: str, description: str = None):
    # Create task in database
    task = await create_task_in_db(user_id, title, description)

    # Publish event via Dapr
    event_data = {
        "event_type": "created",
        "task_id": task.id,
        "user_id": user_id,
        "timestamp": datetime.utcnow().isoformat(),
        "task_data": task.dict()
    }

    await publish_task_event(event_data)

    return {"task_id": task.id, "status": "created", "title": title}
```

### 2. Frontend Integration with Dapr Services
```typescript
// Frontend calling backend via Dapr service invocation
async function callBackendAPI(endpoint: string, data: any) {
  const response = await fetch(
    `http://localhost:3500/v1.0/invoke/backend-service/method${endpoint}`,
    {
      method: 'POST',
      headers: {
        'Content-Type': 'application/json'
      },
      body: JSON.stringify(data)
    }
  );
  return response.json();
}

// Example usage
const newTask = await callBackendAPI('/api/chat', {
  user_id: currentUser.id,
  message: 'Add a new task to buy groceries'
});
```

## Security Considerations
- Use Kubernetes secrets for sensitive data
- Enable mTLS for service-to-service communication
- Implement proper RBAC for Dapr components
- Use Dapr's built-in authentication for pub/sub
- Regularly rotate secrets

## Monitoring and Observability
- Enable Dapr API logging
- Use Dapr's built-in metrics (Prometheus)
- Implement distributed tracing (OpenTelemetry)
- Monitor sidecar health and performance
- Set up alerts for Dapr component failures

## Local Development Considerations
- Use Dapr standalone mode for local development
- Configure local Kafka/PostgreSQL for testing
- Use Dapr CLI for local service invocation testing
- Mock external services when needed