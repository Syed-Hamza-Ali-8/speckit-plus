# Phase V API and Service Contracts

## Overview
This document defines the API contracts, service interfaces, and communication protocols for Phase V of the Todo app evolution, including advanced features and event-driven architecture.

## 1. API Contracts

### 1.1 Task Management API

#### 1.1.1 Create Task
```
POST /api/{user_id}/tasks
```
**Headers:**
- Authorization: Bearer {JWT_TOKEN}

**Request Body:**
```json
{
  "title": "string (required, 1-200 chars)",
  "description": "string (optional, max 1000 chars)",
  "priority": "enum (low|medium|high, default: medium)",
  "tags": "array of strings (optional)",
  "due_date": "ISO 8601 datetime string (optional)"
}
```

**Response:**
```json
{
  "id": "integer",
  "user_id": "string",
  "title": "string",
  "description": "string",
  "completed": "boolean",
  "priority": "string",
  "tags": "array of strings",
  "due_date": "ISO 8601 datetime string or null",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

**Status Codes:**
- 201: Created successfully
- 400: Invalid input
- 401: Unauthorized
- 500: Internal server error

#### 1.1.2 Get All Tasks
```
GET /api/{user_id}/tasks
```
**Query Parameters:**
- status: "all|pending|completed" (default: all)
- priority: "low|medium|high" (optional)
- tag: "string" (optional, can be repeated)
- sort_by: "created|title|due_date|priority" (default: created)
- order: "asc|desc" (default: desc)
- due_before: "ISO 8601 datetime string" (optional)
- due_after: "ISO 8601 datetime string" (optional)

**Response:**
```json
[
  {
    "id": "integer",
    "title": "string",
    "description": "string",
    "completed": "boolean",
    "priority": "string",
    "tags": "array of strings",
    "due_date": "ISO 8601 datetime string or null",
    "created_at": "ISO 8601 datetime string",
    "updated_at": "ISO 8601 datetime string"
  }
]
```

#### 1.1.3 Update Task
```
PUT /api/{user_id}/tasks/{task_id}
```
**Request Body:**
```json
{
  "title": "string (optional)",
  "description": "string (optional)",
  "completed": "boolean (optional)",
  "priority": "enum (low|medium|high, optional)",
  "tags": "array of strings (optional)",
  "due_date": "ISO 8601 datetime string (optional)"
}
```

### 1.2 Recurring Task API

#### 1.2.1 Create Recurring Task Pattern
```
POST /api/{user_id}/tasks/recurring
```
**Request Body:**
```json
{
  "base_task_title": "string (required)",
  "base_task_description": "string (optional)",
  "pattern_type": "enum (daily|weekly|monthly|yearly|custom)",
  "interval": "integer (default: 1)",
  "start_date": "ISO 8601 datetime string",
  "end_date": "ISO 8601 datetime string (optional)",
  "weekdays": "array of integers (0-6, for weekly patterns)",
  "days_of_month": "array of integers (1-31, for monthly patterns)"
}
```

**Response:**
```json
{
  "id": "integer",
  "user_id": "string",
  "base_task_title": "string",
  "base_task_description": "string",
  "pattern_type": "string",
  "interval": "integer",
  "start_date": "ISO 8601 datetime string",
  "end_date": "ISO 8601 datetime string or null",
  "weekdays": "array of integers",
  "days_of_month": "array of integers",
  "created_at": "ISO 8601 datetime string",
  "updated_at": "ISO 8601 datetime string"
}
```

#### 1.2.2 Get Recurring Task Patterns
```
GET /api/{user_id}/tasks/recurring
```
**Response:**
```json
[
  {
    "id": "integer",
    "base_task_title": "string",
    "pattern_type": "string",
    "interval": "integer",
    "start_date": "ISO 8601 datetime string",
    "end_date": "ISO 8601 datetime string or null",
    "created_at": "ISO 8601 datetime string"
  }
]
```

### 1.3 Reminder API

#### 1.3.1 Set Task Due Date
```
PUT /api/{user_id}/tasks/{task_id}/due-date
```
**Request Body:**
```json
{
  "due_date": "ISO 8601 datetime string (required)",
  "reminder_times": "array of ISO 8601 datetime strings (optional)"
}
```

#### 1.3.2 Get Due Soon Tasks
```
GET /api/{user_id}/tasks/due-soon
```
**Query Parameters:**
- within_hours: "integer (default: 24)"

**Response:**
```json
[
  {
    "id": "integer",
    "title": "string",
    "due_date": "ISO 8601 datetime string",
    "reminder_times": "array of ISO 8601 datetime strings",
    "is_reminder_sent": "boolean"
  }
]
```

### 1.4 Search & Filter API

#### 1.4.1 Search Tasks
```
GET /api/{user_id}/tasks/search
```
**Query Parameters:**
- q: "search query string (required)"
- status: "all|pending|completed"
- priority: "low|medium|high"
- tag: "string (can be repeated)"
- due_before: "ISO 8601 datetime string"
- due_after: "ISO 8601 datetime string"
- sort_by: "created|title|due_date|priority"
- order: "asc|desc"

**Response:**
```json
{
  "results": [
    {
      "id": "integer",
      "title": "string",
      "description": "string",
      "completed": "boolean",
      "priority": "string",
      "tags": "array of strings",
      "due_date": "ISO 8601 datetime string or null",
      "relevance_score": "float"
    }
  ],
  "total_count": "integer",
  "page": "integer",
  "per_page": "integer"
}
```

## 2. MCP Tools Contracts

### 2.1 Task Management Tools

#### 2.1.1 add_task
**Description:** Create a new task
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "title": {"type": "string", "description": "Task title"},
    "description": {"type": "string", "description": "Task description"},
    "priority": {"type": "string", "enum": ["low", "medium", "high"], "default": "medium"},
    "tags": {"type": "array", "items": {"type": "string"}},
    "due_date": {"type": "string", "format": "date-time", "description": "Due date in ISO 8601 format"}
  },
  "required": ["user_id", "title"]
}
```
**Output Schema:**
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer", "description": "Created task ID"},
    "status": {"type": "string", "enum": ["created"]},
    "title": {"type": "string", "description": "Task title"}
  }
}
```

#### 2.1.2 list_tasks
**Description:** Retrieve tasks from the list
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "status": {"type": "string", "enum": ["all", "pending", "completed"], "default": "all"},
    "priority": {"type": "string", "enum": ["low", "medium", "high"]},
    "tags": {"type": "array", "items": {"type": "string"}},
    "due_before": {"type": "string", "format": "date-time"},
    "due_after": {"type": "string", "format": "date-time"},
    "search_query": {"type": "string", "description": "Full-text search query"}
  },
  "required": ["user_id"]
}
```
**Output Schema:**
```json
{
  "type": "array",
  "items": {
    "type": "object",
    "properties": {
      "id": {"type": "integer"},
      "title": {"type": "string"},
      "completed": {"type": "boolean"},
      "priority": {"type": "string"},
      "due_date": {"type": "string", "format": "date-time"},
      "tags": {"type": "array", "items": {"type": "string"}}
    }
  }
}
```

### 2.2 Advanced Feature Tools

#### 2.2.1 create_recurring_task
**Description:** Create a recurring task pattern
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "base_task_title": {"type": "string", "description": "Template title"},
    "base_task_description": {"type": "string", "description": "Template description"},
    "pattern_type": {"type": "string", "enum": ["daily", "weekly", "monthly", "yearly", "custom"]},
    "interval": {"type": "integer", "default": 1},
    "start_date": {"type": "string", "format": "date-time"},
    "end_date": {"type": "string", "format": "date-time"},
    "weekdays": {"type": "array", "items": {"type": "integer"}},
    "days_of_month": {"type": "array", "items": {"type": "integer"}}
  },
  "required": ["user_id", "base_task_title", "pattern_type", "start_date"]
}
```
**Output Schema:**
```json
{
  "type": "object",
  "properties": {
    "pattern_id": {"type": "integer", "description": "Created pattern ID"},
    "status": {"type": "string", "enum": ["created"]},
    "base_task_title": {"type": "string", "description": "Template title"}
  }
}
```

#### 2.2.2 set_task_due_date
**Description:** Set due date for a task
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "task_id": {"type": "integer", "description": "Task ID"},
    "due_date": {"type": "string", "format": "date-time", "description": "Due date in ISO 8601 format"},
    "reminder_times": {
      "type": "array",
      "items": {"type": "string", "format": "date-time"},
      "description": "Array of reminder times"
    }
  },
  "required": ["user_id", "task_id", "due_date"]
}
```
**Output Schema:**
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer", "description": "Task ID"},
    "status": {"type": "string", "enum": ["due_date_set", "reminder_scheduled"]},
    "due_date": {"type": "string", "format": "date-time"}
  }
}
```

#### 2.2.3 set_task_priority
**Description:** Set priority for a task
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "task_id": {"type": "integer", "description": "Task ID"},
    "priority": {"type": "string", "enum": ["low", "medium", "high"]}
  },
  "required": ["user_id", "task_id", "priority"]
}
```
**Output Schema:**
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer", "description": "Task ID"},
    "status": {"type": "string", "enum": ["priority_updated"]},
    "priority": {"type": "string", "enum": ["low", "medium", "high"]}
  }
}
```

#### 2.2.4 add_task_tags
**Description:** Add tags to a task
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "task_id": {"type": "integer", "description": "Task ID"},
    "tags": {"type": "array", "items": {"type": "string"}, "description": "Array of tags to add"}
  },
  "required": ["user_id", "task_id", "tags"]
}
```
**Output Schema:**
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer", "description": "Task ID"},
    "status": {"type": "string", "enum": ["tags_added"]},
    "tags": {"type": "array", "items": {"type": "string"}}
  }
}
```

#### 2.2.5 search_tasks
**Description:** Search tasks by keyword
**Input Schema:**
```json
{
  "type": "object",
  "properties": {
    "user_id": {"type": "string", "description": "User ID"},
    "query": {"type": "string", "description": "Search query"},
    "filters": {
      "type": "object",
      "properties": {
        "status": {"type": "string", "enum": ["all", "pending", "completed"]},
        "priority": {"type": "string", "enum": ["low", "medium", "high"]},
        "tags": {"type": "array", "items": {"type": "string"}}
      }
    }
  },
  "required": ["user_id", "query"]
}
```
**Output Schema:**
```json
{
  "type": "array",
  "items": {
    "type": "object",
    "properties": {
      "id": {"type": "integer"},
      "title": {"type": "string"},
      "description": {"type": "string"},
      "completed": {"type": "boolean"},
      "priority": {"type": "string"},
      "tags": {"type": "array", "items": {"type": "string"}},
      "relevance_score": {"type": "number"}
    }
  }
}
```

## 3. Kafka Event Contracts

### 3.1 Task Events Schema
```json
{
  "type": "object",
  "properties": {
    "event_type": {
      "type": "string",
      "enum": [
        "created", "updated", "completed", "deleted",
        "priority_changed", "due_date_set", "tags_added",
        "recurring_created", "recurring_updated", "recurring_deleted"
      ]
    },
    "task_id": {"type": "integer"},
    "user_id": {"type": "string"},
    "timestamp": {"type": "string", "format": "date-time"},
    "task_data": {
      "type": "object",
      "properties": {
        "id": {"type": "integer"},
        "title": {"type": "string"},
        "description": {"type": "string"},
        "completed": {"type": "boolean"},
        "priority": {"type": "string"},
        "due_date": {"type": "string", "format": "date-time"},
        "tags": {"type": "array", "items": {"type": "string"}},
        "is_recurring": {"type": "boolean"},
        "recurring_pattern_id": {"type": "integer"}
      }
    },
    "previous_task_data": {
      "type": "object",
      "properties": {
        // Same structure as task_data, optional
      }
    }
  },
  "required": ["event_type", "task_id", "user_id", "timestamp", "task_data"]
}
```

### 3.2 Reminder Events Schema
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer"},
    "user_id": {"type": "string"},
    "title": {"type": "string"},
    "due_at": {"type": "string", "format": "date-time"},
    "remind_at": {"type": "string", "format": "date-time"},
    "notification_method": {"type": "string", "enum": ["email", "push", "sms"]},
    "created_at": {"type": "string", "format": "date-time"}
  },
  "required": ["task_id", "user_id", "title", "due_at", "remind_at", "created_at"]
}
```

### 3.3 Task Update Events Schema
```json
{
  "type": "object",
  "properties": {
    "task_id": {"type": "integer"},
    "user_id": {"type": "string"},
    "operation": {"type": "string", "enum": ["created", "updated", "completed", "deleted"]},
    "timestamp": {"type": "string", "format": "date-time"},
    "task_data": {
      "type": "object",
      "properties": {
        "id": {"type": "integer"},
        "title": {"type": "string"},
        "description": {"type": "string"},
        "completed": {"type": "boolean"},
        "priority": {"type": "string"},
        "due_date": {"type": "string", "format": "date-time"},
        "tags": {"type": "array", "items": {"type": "string"}}
      }
    }
  },
  "required": ["task_id", "user_id", "operation", "timestamp", "task_data"]
}
```

## 4. Dapr Component Contracts

### 4.1 Pub/Sub Contract
**Component Name:** kafka-pubsub
**Topics:**
- task-events
- reminders
- task-updates

**Publish Operation:**
- Method: POST to http://localhost:3500/v1.0/publish/kafka-pubsub/{topic}
- Content-Type: application/json
- Payload: As defined in Kafka Event Contracts

**Subscribe Operation:**
- Endpoint: /dapr/subscribe (returns subscription list)
- Handler: POST to configured route

### 4.2 State Management Contract
**Component Name:** statestore
**Operations:**
- GET: http://localhost:3500/v1.0/state/statestore/{key}
- POST: http://localhost:3500/v1.0/state/statestore
- DELETE: http://localhost:3500/v1.0/state/statestore/{key}

**State Key Patterns:**
- conversation-{conversation_id}
- user-preferences-{user_id}
- session-{session_id}

### 4.3 Service Invocation Contract
**Base URL:** http://localhost:3500/v1.0/invoke/{app-id}/method/{method}
**App IDs:**
- backend-service
- notification-service
- recurring-task-service
- websocket-service

## 5. Error Handling Contracts

### 5.1 HTTP Error Responses
```json
{
  "type": "object",
  "properties": {
    "error": {"type": "string", "description": "Error message"},
    "error_code": {"type": "string", "description": "Machine-readable error code"},
    "details": {"type": "object", "description": "Additional error details"},
    "timestamp": {"type": "string", "format": "date-time"}
  },
  "required": ["error", "error_code"]
}
```

### 5.2 Common Error Codes
- TASK_NOT_FOUND: Task with specified ID does not exist
- USER_UNAUTHORIZED: User is not authorized to perform operation
- INVALID_INPUT: Request data does not match expected schema
- RECURRING_PATTERN_INVALID: Recurring task pattern is invalid
- KAFKA_CONNECTION_ERROR: Unable to publish event to Kafka
- DATABASE_ERROR: Database operation failed