# Phase V: Advanced Cloud Deployment Specification

## Overview
This specification outlines the requirements for Phase V of the Todo app evolution, which involves implementing advanced features and deploying the application with event-driven architecture on production-grade Kubernetes.

## Objective
Implement advanced features and deploy the Todo Chatbot with event-driven architecture on DigitalOcean Kubernetes (DOKS), Google Cloud (GKE), or Azure (AKS) with Kafka on Redpanda Cloud.

## Requirements

### Part A: Advanced Features

#### 1. Advanced Level Functionality
- **Recurring Tasks**
  - Support for creating tasks that repeat on a schedule (daily, weekly, monthly, yearly)
  - Auto-generation of next occurrence when current occurrence is completed
  - Support for recurring patterns with end dates or indefinitely

- **Due Dates & Time Reminders**
  - Ability to set due dates and times for tasks
  - Browser notifications for upcoming due tasks
  - Reminder scheduling system that can send notifications at specified times

#### 2. Intermediate Level Functionality
- **Priorities & Tags/Categories**
  - Assign priority levels (high/medium/low) to tasks
  - Create and assign tags/categories to tasks (work/home/personal)
  - Filter tasks by priority or tags

- **Search & Filter**
  - Search tasks by keyword in title or description
  - Filter tasks by status (all/pending/completed)
  - Filter tasks by priority level
  - Filter tasks by due date ranges

- **Sort Tasks**
  - Sort by due date (ascending/descending)
  - Sort by priority level
  - Sort alphabetically by title

### Part B: Local Deployment
- Deploy to Minikube
- Deploy Dapr on Minikube with full capabilities:
  - Pub/Sub (Publish/Subscribe)
  - State management
  - Bindings (cron)
  - Secrets management
  - Service Invocation

### Part C: Cloud Deployment
- Deploy to DigitalOcean Kubernetes (DOKS)/Google Cloud (GKE)/Azure (AKS)
- Deploy Dapr on cloud platform with full capabilities
- Use Kafka on Redpanda Cloud
- Set up CI/CD pipeline using GitHub Actions
- Configure monitoring and logging

## Technical Architecture

### Event-Driven Architecture with Kafka
- **Kafka Topics**:
  - `task-events`: All task CRUD operations (created, updated, completed, deleted)
  - `reminders`: Scheduled reminder triggers
  - `task-updates`: Real-time client synchronization

- **Event Schema Examples**:
  - **Task Event**: {event_type, task_id, task_data, user_id, timestamp}
  - **Reminder Event**: {task_id, title, due_at, remind_at, user_id}

### Dapr Integration
- **Pub/Sub Building Block**: Kafka abstraction without direct Kafka client code
- **State Management**: Conversation state storage (alternative to direct DB)
- **Service Invocation**: Frontend → Backend communication with retries
- **Bindings**: Cron triggers for scheduled reminders
- **Secrets Management**: Secure storage of API keys and credentials

### API Extensions
- New endpoints for advanced features:
  - GET /api/tasks/search?q={query}
  - GET /api/tasks/filter?priority={level}&tag={tag}&due_date={range}
  - POST /api/tasks/recurring
  - PUT /api/tasks/{id}/due-date
  - PUT /api/tasks/{id}/priority
  - PUT /api/tasks/{id}/tags

## MCP Tools Extensions
- Add new MCP tools for advanced features:
  - `search_tasks`: Search tasks by keyword
  - `filter_tasks`: Filter tasks by criteria
  - `set_task_due_date`: Set due date for a task
  - `set_task_priority`: Set priority for a task
  - `add_task_tags`: Add tags to a task
  - `create_recurring_task`: Create recurring task patterns

## Natural Language Processing Enhancements
The AI chatbot should understand and respond to advanced commands:
- "Create a recurring task to water plants every Monday"
- "Set a due date for the project report task to next Friday"
- "Show me all high priority tasks"
- "Find tasks with the 'work' tag"
- "Sort my tasks by due date"
- "Remind me about the meeting 30 minutes before it starts"

## Deployment Architecture
- Minikube for local deployment
- Production-grade Kubernetes (DOKS/GKE/AKS) for cloud
- Redpanda Cloud for Kafka
- Dapr sidecars for all services
- Neon PostgreSQL for persistent storage
- CI/CD pipeline with GitHub Actions

## Acceptance Criteria
- [ ] All advanced features implemented and functional
- [ ] Event-driven architecture with Kafka working
- [ ] Dapr integration complete with all building blocks
- [ ] Local deployment on Minikube successful
- [ ] Cloud deployment on chosen platform successful
- [ ] CI/CD pipeline configured and operational
- [ ] Monitoring and logging configured
- [ ] All existing functionality from previous phases preserved
- [ ] AI chatbot understands and processes advanced commands

## Non-Functional Requirements
- Scalability: System should handle increased load with horizontal scaling
- Reliability: Event-driven architecture should be resilient to failures
- Performance: Response times should remain acceptable with added complexity
- Security: All communication should be secure, secrets properly managed
- Maintainability: Architecture should be modular and well-documented