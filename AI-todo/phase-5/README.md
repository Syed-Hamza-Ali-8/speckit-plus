# Phase V: Advanced Cloud Deployment

This is the implementation of Phase V of the Todo app evolution, featuring advanced functionality with event-driven architecture and Dapr integration.

## Features

### Advanced Task Management
- **Recurring Tasks**: Create tasks that repeat on daily, weekly, monthly, or yearly schedules
- **Due Dates & Reminders**: Set due dates and receive notifications
- **Priorities & Tags**: Assign priority levels and organize tasks with tags
- **Search & Filter**: Full-text search and advanced filtering capabilities
- **Sort Tasks**: Sort tasks by various criteria (created date, title, due date, priority)

### Event-Driven Architecture
- **Kafka Integration**: Event streaming for task operations, reminders, and updates
- **Event Sourcing**: Complete audit trail of all task operations
- **Real-time Sync**: WebSocket support for real-time task synchronization

### Dapr Integration
- **Pub/Sub**: Kafka abstraction through Dapr
- **State Management**: Conversation and user state management
- **Service Invocation**: Service-to-service communication
- **Bindings**: Cron-based scheduled operations
- **Secrets Management**: Secure credential storage

## Architecture

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Frontend      │    │   Dapr Sidecar  │    │   Backend API   │
│   (React)       │◄──►│   (Service A)   │◄──►│   (FastAPI)     │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│ Notification    │    │   Dapr Sidecar  │    │ Recurring Task  │
│   Service       │◄──►│   (Service B)   │◄──►│   Service       │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│  WebSocket      │    │   Dapr Sidecar  │    │  Audit Service  │
│   Service       │◄──►│   (Service C)   │◄──►│                 │
└─────────────────┘    └─────────────────┘    └─────────────────┘
                                                        │
                                                        ▼
                                        ┌─────────────────────────────┐
                                        │    Kafka Event Stream       │
                                        │  (task-events, reminders,   │
                                        │     task-updates)           │
                                        └─────────────────────────────┘
                                                        │
                                                        ▼
                                        ┌─────────────────────────────┐
                                        │       PostgreSQL DB         │
                                        │  (Tasks, Patterns, Users)   │
                                        └─────────────────────────────┘
```

## Installation

1. **Clone the repository**
   ```bash
   git clone <repository-url>
   cd phase-5
   ```

2. **Set up Python environment**
   ```bash
   python -m venv venv
   source venv/bin/activate  # On Windows: venv\Scripts\activate
   ```

3. **Install dependencies**
   ```bash
   pip install -r requirements.txt
   ```

4. **Set up environment variables**
   ```bash
   cp .env.example .env
   # Edit .env with your configuration
   ```

5. **Set up Dapr**
   ```bash
   dapr init -k  # Initialize Dapr in Kubernetes
   kubectl apply -f dapr-components/
   ```

6. **Set up Kafka (for local development)**
   ```bash
   docker-compose -f docker-compose.redpanda.yml up -d
   ```

## Environment Variables

Create a `.env` file with the following variables:

```env
DATABASE_URL=postgresql://user:password@localhost:5432/todo_phase5
KAFKA_BOOTSTRAP_SERVERS=localhost:9092
KAFKA_SECURITY_PROTOCOL=PLAINTEXT
DAPR_HTTP_ENDPOINT=http://localhost:3500
DAPR_GRPC_ENDPOINT=http://localhost:50001
BETTER_AUTH_SECRET=your-better-auth-secret
OPENAI_API_KEY=your-openai-api-key
```

## Running the Application

### Local Development

1. **Start the backend service with Dapr**
   ```bash
   dapr run --app-id backend-service --app-port 8000 -- uvicorn src.main:app --reload --port 8000
   ```

2. **Start additional services with Dapr**
   ```bash
   # Notification service
   dapr run --app-id notification-service --app-port 8001 -- uvicorn src.notification_service:app --reload --port 8001

   # Recurring task service
   dapr run --app-id recurring-task-service --app-port 8002 -- uvicorn src.recurring_task_service:app --reload --port 8002
   ```

### With Docker

1. **Build the image**
   ```bash
   docker build -t todo-phase5:latest .
   ```

2. **Run with Docker**
   ```bash
   docker run -p 8000:8000 todo-phase5:latest
   ```

## API Endpoints

### Basic Task Operations
- `GET /api/users/{user_id}/tasks` - Get user's tasks
- `POST /api/users/{user_id}/tasks` - Create a new task
- `GET /api/tasks/{task_id}` - Get a specific task
- `PUT /api/tasks/{task_id}` - Update a task
- `DELETE /api/tasks/{task_id}` - Delete a task

### Advanced Features
- `POST /api/recurring-tasks` - Create a recurring task pattern
- `GET /api/users/{user_id}/recurring-tasks` - Get user's recurring patterns
- `PUT /api/tasks/{task_id}/due-date` - Set due date for a task
- `GET /api/users/{user_id}/due-soon` - Get tasks due soon
- `PUT /api/tasks/{task_id}/priority` - Set task priority
- `PUT /api/tasks/{task_id}/tags` - Add tags to a task
- `GET /api/users/{user_id}/tasks/search` - Search tasks

## Dapr Components

The application uses the following Dapr components:

### 1. Pub/Sub (Kafka)
Located in `dapr-components/pubsub-kafka.yaml`

### 2. State Management
Located in `dapr-components/statestore.yaml`

### 3. Cron Bindings
Located in `dapr-components/cron-binding.yaml`

## Event-Driven Architecture

The application publishes events to Kafka topics:

- `task-events`: All task CRUD operations
- `reminders`: Scheduled reminder triggers
- `task-updates`: Real-time synchronization events

## Testing

Run the tests with pytest:

```bash
pytest tests/
```

## Deployment

### Minikube
```bash
# Start Minikube
minikube start

# Deploy Dapr
dapr init -k

# Apply Kubernetes manifests
kubectl apply -f deployments/

# Check deployment
kubectl get pods
```

### Cloud Deployment (DOKS/GKE/AKS)
1. Configure your Kubernetes cluster
2. Install Dapr
3. Deploy the application manifests
4. Configure external Kafka (Redpanda Cloud)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Make your changes
4. Add tests for new functionality
5. Submit a pull request

## License

This project is licensed under the MIT License.