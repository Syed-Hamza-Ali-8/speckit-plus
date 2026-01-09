# Event-Driven Architecture with Kafka Integration Specification

## Overview
This document specifies the event-driven architecture implementation for Phase V, using Apache Kafka (via Redpanda Cloud) to enable decoupled, scalable microservices communication.

## Architecture Goals
- Decouple services to enable independent scaling and development
- Enable real-time processing of task events
- Provide audit trail of all task operations
- Support for scheduled operations (reminders, recurring tasks)
- Enable real-time synchronization across multiple clients

## Kafka Topics Design

### 1. task-events Topic
**Purpose**: Capture all task CRUD operations for audit, analytics, and downstream processing
**Partitions**: 3 (for scalability)
**Retention**: 30 days

**Event Schema**:
```json
{
  "event_type": "created|updated|completed|deleted|priority_changed|due_date_set|tags_added",
  "task_id": 123,
  "user_id": "user123",
  "timestamp": "2026-01-03T10:00:00Z",
  "task_data": {
    "title": "Task title",
    "description": "Task description",
    "completed": false,
    "priority": "medium",
    "due_date": "2026-01-10T09:00:00Z",
    "tags": ["work", "important"]
  },
  "previous_task_data": { /* Previous state for updates */ }
}
```

### 2. reminders Topic
**Purpose**: Schedule and trigger reminder notifications
**Partitions**: 3
**Retention**: 7 days

**Event Schema**:
```json
{
  "task_id": 123,
  "user_id": "user123",
  "title": "Task title",
  "due_at": "2026-01-10T09:00:00Z",
  "remind_at": "2026-01-10T08:30:00Z",
  "notification_method": "email|push|sms",
  "created_at": "2026-01-03T10:00:00Z"
}
```

### 3. task-updates Topic
**Purpose**: Real-time synchronization of task changes across clients
**Partitions**: 3
**Retention**: 1 hour

**Event Schema**:
```json
{
  "task_id": 123,
  "user_id": "user123",
  "operation": "created|updated|completed|deleted",
  "timestamp": "2026-01-03T10:00:00Z",
  "task_data": { /* Full task object */ }
}
```

## Event Producers

### 1. Chat API Service (MCP Tools)
**Responsibility**: Publish events when task operations occur
**Events Published**:
- task-events: When tasks are created, updated, completed, or deleted
- reminders: When tasks with due dates are created
- task-updates: When any task attribute changes

**Implementation**:
```python
# Example event publishing
async def publish_task_event(event_type: str, task: Task, user_id: str):
    event = {
        "event_type": event_type,
        "task_id": task.id,
        "user_id": user_id,
        "timestamp": datetime.utcnow().isoformat(),
        "task_data": task.dict()
    }
    await kafka_producer.send("task-events", value=event)
```

## Event Consumers

### 1. Notification Service
**Responsibility**: Send reminders and notifications to users
**Consumes From**: reminders topic
**Actions**:
- Send email notifications
- Send push notifications
- Send SMS notifications (if configured)

### 2. Recurring Task Service
**Responsibility**: Handle recurring task logic
**Consumes From**: task-events topic
**Actions**:
- When a recurring task is completed, create the next occurrence
- Handle recurrence pattern calculations

### 3. Audit Service
**Responsibility**: Maintain audit logs of all task operations
**Consumes From**: task-events topic
**Actions**:
- Store audit records in audit database
- Generate compliance reports
- Track user activity

### 4. WebSocket Service
**Responsibility**: Real-time task synchronization to clients
**Consumes From**: task-updates topic
**Actions**:
- Push updates to connected clients
- Maintain real-time task lists

## Kafka Configuration

### 1. Connection Settings
```python
KAFKA_BOOTSTRAP_SERVERS = os.getenv("KAFKA_BOOTSTRAP_SERVERS", "localhost:9092")
KAFKA_SECURITY_PROTOCOL = os.getenv("KAFKA_SECURITY_PROTOCOL", "SASL_SSL")
KAFKA_SASL_MECHANISM = os.getenv("KAFKA_SASL_MECHANISM", "SCRAM-SHA-256")
KAFKA_SASL_USERNAME = os.getenv("KAFKA_SASL_USERNAME")
KAFKA_SASL_PASSWORD = os.getenv("KAFKA_SASL_PASSWORD")
```

### 2. Producer Configuration
```python
producer_config = {
    'bootstrap_servers': KAFKA_BOOTSTRAP_SERVERS,
    'security_protocol': KAFKA_SECURITY_PROTOCOL,
    'sasl_mechanism': KAFKA_SASL_MECHANISM,
    'sasl_plain_username': KAFKA_SASL_USERNAME,
    'sasl_plain_password': KAFKA_SASL_PASSWORD,
    'value_serializer': lambda v: json.dumps(v).encode('utf-8'),
    'acks': 'all',  # Ensure durability
    'retries': 3,
    'batch_size': 16000,
    'linger_ms': 5
}
```

### 3. Consumer Configuration
```python
consumer_config = {
    'bootstrap_servers': KAFKA_BOOTSTRAP_SERVERS,
    'security_protocol': KAFKA_SECURITY_PROTOCOL,
    'sasl_mechanism': KAFKA_SASL_MECHANISM,
    'sasl_plain_username': KAFKA_SASL_USERNAME,
    'sasl_plain_password': KAFKA_SASL_PASSWORD,
    'value_deserializer': lambda v: json.loads(v.decode('utf-8')),
    'auto_offset_reset': 'earliest',
    'enable_auto_commit': False,
    'group_id': 'todo-service-group'
}
```

## Service Implementations

### 1. Notification Service
```python
class NotificationService:
    def __init__(self, kafka_consumer):
        self.consumer = kafka_consumer

    async def process_reminder_events(self):
        async for message in self.consumer:
            reminder_data = message.value
            await self.send_notification(reminder_data)
            await self.consumer.commit()

    async def send_notification(self, reminder_data):
        # Send notification via configured method
        user_id = reminder_data['user_id']
        task_title = reminder_data['title']

        # Fetch user preferences for notification method
        user_prefs = await get_user_notification_preferences(user_id)

        if user_prefs.get('email_enabled'):
            await send_email(user_id, f"Reminder: {task_title}")

        if user_prefs.get('push_enabled'):
            await send_push_notification(user_id, f"Reminder: {task_title}")
```

### 2. Recurring Task Service
```python
class RecurringTaskService:
    def __init__(self, kafka_consumer, db_session):
        self.consumer = kafka_consumer
        self.db = db_session

    async def process_task_events(self):
        async for message in self.consumer:
            event = message.value
            if event['event_type'] == 'completed' and event['task_data'].get('is_recurring'):
                await self.create_next_occurrence(event)
            await self.consumer.commit()

    async def create_next_occurrence(self, completed_event):
        task_data = completed_event['task_data']
        pattern = await get_recurring_pattern(task_data['recurring_pattern_id'])

        next_task = create_next_task_from_pattern(task_data, pattern)
        await save_task(next_task)

        # Publish event for the new task
        await publish_task_event('created', next_task, completed_event['user_id'])
```

### 3. Audit Service
```python
class AuditService:
    def __init__(self, kafka_consumer, audit_db):
        self.consumer = kafka_consumer
        self.audit_db = audit_db

    async def process_task_events(self):
        async for message in self.consumer:
            event = message.value
            audit_record = create_audit_record(event)
            await self.audit_db.save(audit_record)
            await self.consumer.commit()

    def create_audit_record(self, event):
        return {
            'user_id': event['user_id'],
            'task_id': event['task_id'],
            'operation': event['event_type'],
            'timestamp': event['timestamp'],
            'details': event['task_data'],
            'ip_address': event.get('ip_address'),
            'user_agent': event.get('user_agent')
        }
```

## Error Handling and Resilience

### 1. Producer Error Handling
- Retry with exponential backoff
- Dead letter queue for failed messages
- Circuit breaker pattern to prevent cascading failures

### 2. Consumer Error Handling
- Process messages individually to prevent blocking
- Use manual offset commits for reliability
- Implement poison pill detection

### 3. Monitoring and Alerting
- Track message production/consumption rates
- Monitor consumer lag
- Alert on failed message processing

## Local Development Setup (Minikube)

### 1. Kafka Deployment
```yaml
# kafka-deployment.yaml
apiVersion: apps/v1
kind: Deployment
metadata:
  name: kafka
spec:
  replicas: 1
  selector:
    matchLabels:
      app: kafka
  template:
    metadata:
      labels:
        app: kafka
    spec:
      containers:
      - name: kafka
        image: redpandadata/redpanda:latest
        command:
        - rpk
        - redpanda
        - start
        - --smp
        - "1"
        - --memory
        - 512M
        - --overprovisioned
        - --kafka-addr
        - PLAINTEXT://0.0.0.0:9092
        - --advertise-kafka-addr
        - PLAINTEXT://kafka:9092
        ports:
        - containerPort: 9092
---
apiVersion: v1
kind: Service
metadata:
  name: kafka
spec:
  selector:
    app: kafka
  ports:
  - port: 9092
    targetPort: 9092
```

## Cloud Deployment (Redpanda Cloud)

### 1. Connection Configuration
- Use Redpanda Cloud cluster credentials
- Implement secure connection with SASL/SCRAM authentication
- Use environment-specific configuration

### 2. Topic Management
- Pre-create topics with appropriate settings
- Set up topic replication and partitioning
- Configure retention policies

## Integration with Existing Architecture

### 1. MCP Tools Integration
- MCP tools publish events when they perform operations
- Events are published after successful database operations
- Transactional outbox pattern to ensure consistency

### 2. Dapr Integration
- Use Dapr Kafka component for pub/sub operations
- Leverage Dapr for connection management
- Benefit from Dapr's resiliency features

## Security Considerations
- Use SASL/SCRAM for authentication
- Enable SSL/TLS encryption
- Implement proper access controls
- Encrypt sensitive data in events
- Audit access to Kafka topics

## Performance Considerations
- Optimize partition count for scalability
- Configure appropriate batch sizes and linger times
- Monitor and tune consumer group performance
- Implement efficient serialization/deserialization