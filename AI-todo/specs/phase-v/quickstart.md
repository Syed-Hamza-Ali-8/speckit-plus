# Phase V Quickstart Guide

## Overview
This quickstart guide provides step-by-step instructions to get the Phase V Todo app with advanced features, event-driven architecture, and Dapr integration up and running quickly.

## Prerequisites
- Python 3.13+
- Docker and Docker Compose
- Kubernetes cluster (Minikube for local development)
- Claude Code and Spec-Kit Plus
- Git
- Node.js 18+ (for frontend, if applicable)

## 1. Environment Setup

### 1.1 Install Dependencies
```bash
# Install Python dependencies
pip install uv
uv venv
source .venv/bin/activate  # On Windows: .venv\Scripts\activate

# Install project dependencies
uv pip install -r requirements.txt
```

### 1.2 Set Up Local Kubernetes (Minikube)
```bash
# Install Minikube
curl -Lo minikube https://storage.googleapis.com/minikube/releases/latest/minikube-windows-amd64.exe
# Or use package manager like Chocolatey: choco install minikube kubernetes-cli

# Start Minikube
minikube start --cpus=4 --memory=8192 --disk-size=40g

# Verify Kubernetes is running
kubectl cluster-info
```

### 1.3 Install Dapr
```bash
# Install Dapr CLI
wget -q https://raw.githubusercontent.com/dapr/cli/master/install/install.sh -O - | /bin/bash

# Initialize Dapr on Kubernetes
dapr init -k

# Verify Dapr installation
kubectl get pods -n dapr-system
```

### 1.4 Set Up Local Kafka (Redpanda)
```bash
# Create Redpanda configuration
cat <<EOF > docker-compose.redpanda.yml
version: '3.8'
services:
  redpanda:
    image: redpandadata/redpanda:latest
    command:
      - redpanda start
      - --smp
      - "1"
      - --memory
      - 512M
      - --overprovisioned
      - --kafka-addr
      - PLAINTEXT://0.0.0.0:9092
      - --advertise-kafka-addr
      - PLAINTEXT://localhost:9092
    ports:
      - "9092:9092"
      - "8081:8081"  # Schema Registry
      - "8082:8082"  # REST Proxy
EOF

# Start Redpanda
docker-compose -f docker-compose.redpanda.yml up -d
```

## 2. Configuration

### 2.1 Environment Variables
Create a `.env` file in the project root:

```bash
# Backend Configuration
DATABASE_URL="postgresql://user:password@localhost:5432/todo_db"
BETTER_AUTH_SECRET="your-better-auth-secret-here"
OPENAI_API_KEY="your-openai-api-key"

# Kafka Configuration
KAFKA_BOOTSTRAP_SERVERS="localhost:9092"
KAFKA_SECURITY_PROTOCOL="PLAINTEXT"
KAFKA_CONSUMER_GROUP="todo-service-group"

# Dapr Configuration
DAPR_HTTP_ENDPOINT="http://localhost:3500"
DAPR_GRPC_ENDPOINT="http://localhost:50001"

# Dapr Component Secrets (for local development)
DAPR_KAFKA_USERNAME=""
DAPR_KAFKA_PASSWORD=""
```

### 2.2 Dapr Components
Create Dapr component configurations in the `dapr-components/` directory:

```bash
# Create directory if it doesn't exist
mkdir -p dapr-components
```

**dapr-components/pubsub-kafka.yaml**:
```yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: kafka-pubsub
spec:
  type: pubsub.kafka
  version: v1
  metadata:
  - name: brokers
    value: "localhost:9092"
  - name: consumerGroup
    value: "todo-service-group"
  - name: authRequired
    value: "false"
```

**dapr-components/statestore.yaml**:
```yaml
apiVersion: dapr.io/v1alpha1
kind: Component
metadata:
  name: statestore
spec:
  type: state.redis
  version: v1
  metadata:
  - name: redisHost
    value: localhost:6379
  - name: redisPassword
    value: ""
  - name: actorStateStore
    value: "true"
```

**dapr-components/cron-binding.yaml**:
```yaml
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

Deploy the Dapr components:
```bash
kubectl apply -f dapr-components/
```

## 3. Database Setup

### 3.1 Run Database Migrations
```bash
# Install and run Alembic for database migrations
pip install alembic
alembic init alembic

# Create migration for Phase V models
alembic revision --autogenerate -m "Add Phase V models: recurring tasks, reminders, tags"

# Apply migrations
alembic upgrade head
```

## 4. Run the Services

### 4.1 Start the Backend Service with Dapr
```bash
# Terminal 1: Start the backend service with Dapr sidecar
dapr run --app-id backend-service --app-port 8000 -- uvicorn main:app --reload --port 8000
```

### 4.2 Start the Notification Service
```bash
# Terminal 2: Start the notification service with Dapr sidecar
dapr run --app-id notification-service --app-port 8001 -- uvicorn notification_service:app --reload --port 8001
```

### 4.3 Start the Recurring Task Service
```bash
# Terminal 3: Start the recurring task service with Dapr sidecar
dapr run --app-id recurring-task-service --app-port 8002 -- uvicorn recurring_task_service:app --reload --port 8002
```

### 4.4 Start the WebSocket Service (for real-time sync)
```bash
# Terminal 4: Start the WebSocket service with Dapr sidecar
dapr run --app-id websocket-service --app-port 8003 -- uvicorn websocket_service:app --reload --port 8003
```

## 5. Frontend Setup (if applicable)

### 5.1 Install Frontend Dependencies
```bash
cd frontend
npm install
```

### 5.2 Run Frontend Development Server
```bash
npm run dev
```

## 6. Testing the Setup

### 6.1 Verify Services are Running
```bash
# Check Dapr services
dapr list

# Check Kubernetes pods (if deployed to K8s)
kubectl get pods
```

### 6.2 Test API Endpoints
```bash
# Test basic task creation
curl -X POST http://localhost:3500/v1.0/invoke/backend-service/method/api/user123/tasks \
  -H "Content-Type: application/json" \
  -d '{
    "title": "Test task",
    "description": "This is a test task for Phase V"
  }'

# Test recurring task creation
curl -X POST http://localhost:3500/v1.0/invoke/backend-service/method/api/user123/tasks/recurring \
  -H "Content-Type: application/json" \
  -d '{
    "base_task_title": "Water plants",
    "pattern_type": "daily",
    "start_date": "2026-01-04T09:00:00Z"
  }'
```

## 7. Phase V Features Testing

### 7.1 Test Advanced Features
1. **Recurring Tasks**: Create a recurring task and verify it generates occurrences
2. **Due Dates & Reminders**: Set a due date and check if reminder events are published
3. **Priorities & Tags**: Assign priorities and tags to tasks
4. **Search & Filter**: Test search functionality with keywords and filters
5. **Sort Tasks**: Test sorting by different criteria

### 7.2 Test Event-Driven Architecture
1. Verify events are published to Kafka when tasks are created/updated
2. Check that notification service receives reminder events
3. Confirm recurring task service processes completion events
4. Validate audit service logs all operations

### 7.3 Test Dapr Integration
1. Verify pub/sub works through Dapr
2. Test state management through Dapr
3. Confirm service invocation between services
4. Check cron binding triggers as expected

## 8. Deployment to Minikube

### 8.1 Build Docker Images
```bash
# Build backend service image
docker build -t todo-backend:latest -f Dockerfile.backend .

# Build notification service image
docker build -t todo-notification:latest -f Dockerfile.notification .

# Build recurring task service image
docker build -t todo-recurring:latest -f Dockerfile.recurring .
```

### 8.2 Deploy to Minikube
```bash
# Load images into Minikube
minikube image load todo-backend:latest
minikube image load todo-notification:latest
minikube image load todo-recurring:latest

# Deploy services to Minikube
kubectl apply -f deployments/
```

### 8.3 Verify Deployment
```bash
# Check all pods are running
kubectl get pods

# Check services are available
kubectl get services

# Check Dapr sidecars are injected
kubectl describe pods
```

## 9. Troubleshooting

### 9.1 Common Issues
- **Kafka Connection**: Ensure Redpanda is running and accessible
- **Dapr Sidecar**: Check Dapr is initialized and sidecars are running
- **Database Connection**: Verify connection string and database is accessible
- **Port Conflicts**: Ensure required ports (8000-8003) are available

### 9.2 Useful Commands
```bash
# Check Dapr logs
kubectl logs -n dapr-system -l app=dapr-placement-server

# Check service logs
kubectl logs <pod-name>

# Check Kafka topics
docker exec -it <redpanda-container> rpk topic list

# Check Dapr sidecar health
dapr status -k
```

## 10. Next Steps
1. Implement the remaining MCP tools for advanced features
2. Add comprehensive error handling and validation
3. Implement monitoring and logging
4. Set up CI/CD pipeline
5. Prepare for cloud deployment