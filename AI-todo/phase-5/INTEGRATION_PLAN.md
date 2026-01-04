# Phase V Integration Plan

## Overview
This document outlines the plan to integrate Phase V advanced features with the existing Phase 2 architecture (frontend, backend, and chatbot).

## Current Architecture Analysis

### Backend (Phase 2)
- **Framework**: FastAPI
- **Database**: SQLModel with PostgreSQL (using UUID primary keys)
- **Authentication**: JWT-based with Better Auth
- **Chatbot**: OpenAI Agents SDK with MCP tools
- **Task Model**: Uses UUID primary keys, has due_date field
- **API Structure**: RESTful with `/tasks` prefix
- **MCP Tools**: add_task, list_tasks, complete_task, delete_task, update_task

### Frontend (Phase 2)
- **Framework**: React/TypeScript with Vite
- **State Management**: RTK Query for API integration
- **Chat Interface**: WebSocket-based with streaming
- **API Services**: TypeScript services for all backend interactions

### Phase V Features
- **Advanced Task Features**: Recurring tasks, priorities, tags, search, filter, sort
- **Event-Driven Architecture**: Kafka integration
- **Dapr Integration**: All building blocks (pub/sub, state, service invocation, bindings, secrets)

## Integration Strategy

### 1. Database Model Alignment
The Phase 2 backend uses UUID primary keys and SQLModel, while Phase 5 uses integer primary keys. We need to align the models.

### 2. API Endpoint Integration
Phase 2 uses `/tasks` prefix, Phase 5 needs to be compatible with this structure.

### 3. MCP Tools Extension
Extend existing MCP tools to support new advanced features.

### 4. Frontend Integration
Update frontend to support new advanced features.

## Implementation Steps

### Step 1: Update Phase 5 Data Models
- Change primary keys from integers to UUIDs to match Phase 2
- Align field names and types with Phase 2
- Maintain advanced features (recurring, priorities, tags, etc.)

### Step 2: Update API Endpoints
- Use `/tasks` prefix to match Phase 2
- Maintain compatibility with existing endpoints
- Add new endpoints for advanced features

### Step 3: Extend MCP Tools
- Add new MCP tools for advanced features
- Update existing tools to support new fields
- Maintain backward compatibility

### Step 4: Update Chatbot Integration
- Update system prompt to include new capabilities
- Train AI to understand advanced feature requests
- Add natural language processing for new features

### Step 5: Frontend Integration
- Update UI components to support new features
- Add forms for recurring tasks, priorities, tags
- Update task lists to show new information

## Detailed Integration Plan

### A. Database Model Integration

#### Phase 2 Task Model (Current):
```python
class Task(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID
    title: str
    description: str | None
    status: TaskStatus
    due_date: date | None
    created_at: datetime
    updated_at: datetime
```

#### Phase 5 Task Model (Updated):
```python
class Task(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID
    title: str
    description: str | None
    status: TaskStatus  # Maintain compatibility with Phase 2
    due_date: date | None  # Use date instead of datetime to match Phase 2
    # New advanced features
    priority: PriorityLevel | None = PriorityLevel.MEDIUM
    tags: List[str] = Field(default=[])  # Store as JSON in DB
    is_recurring: bool = False
    recurring_pattern_id: UUID | None
    is_reminder_sent: bool = False
    reminder_times: List[datetime] = Field(default=[])  # Store as JSON in DB
    created_at: datetime
    updated_at: datetime
    created_by: UUID
    updated_by: UUID
```

### B. API Endpoint Integration

#### Maintain Existing Endpoints:
- `GET /tasks` - List tasks (add support for new filters)
- `POST /tasks` - Create task (support new fields)
- `GET /tasks/{task_id}` - Get task
- `PATCH /tasks/{task_id}` - Update task (support new fields)
- `DELETE /tasks/{task_id}` - Delete task

#### Add New Endpoints:
- `POST /tasks/recurring` - Create recurring task pattern
- `GET /tasks/search` - Search tasks
- `PUT /tasks/{task_id}/priority` - Set task priority
- `PUT /tasks/{task_id}/tags` - Add tags to task
- `PUT /tasks/{task_id}/due-date` - Set due date and reminders

### C. MCP Tools Extension

#### Extend Existing Tools:
- `add_task`: Add priority, tags, recurring, reminder parameters
- `list_tasks`: Add filtering by priority, tags, due date ranges
- `update_task`: Add support for priority, tags, recurring updates

#### Add New Tools:
- `create_recurring_task`: Create recurring task pattern
- `set_task_priority`: Set task priority
- `add_task_tags`: Add tags to task
- `set_task_reminder`: Set task reminder
- `search_tasks`: Search tasks by keyword

### D. Chatbot Integration

#### Update System Prompt:
Add information about new capabilities:
- Creating recurring tasks
- Setting priorities and tags
- Advanced search and filtering
- Setting reminders

#### Natural Language Examples:
- "Create a recurring task to water plants every Monday"
- "Set high priority for the project task"
- "Add 'work' and 'urgent' tags to this task"
- "Find all tasks with 'meeting' in the title"

## Migration Path

### Phase 1: Database Schema Updates
1. Update Phase 5 models to use UUIDs
2. Add new columns to existing task table
3. Create new tables for recurring patterns, tags, etc.

### Phase 2: API Layer Updates
1. Update existing API endpoints to support new fields
2. Add new endpoints for advanced features
3. Maintain backward compatibility

### Phase 3: MCP Tools Updates
1. Extend existing tools with new parameters
2. Create new MCP tools for advanced features
3. Update tool schemas

### Phase 4: Chatbot Updates
1. Update system prompt with new capabilities
2. Train AI on new natural language patterns
3. Test integration

### Phase 5: Frontend Updates
1. Update UI components to show new information
2. Add forms for advanced features
3. Update task lists and filtering

## Compatibility Considerations

### Backward Compatibility
- All existing Phase 2 functionality must continue to work
- Existing API endpoints must maintain the same interface
- Existing MCP tools must continue to work with new parameters as optional

### Forward Compatibility
- New features should be optional
- Default values should be provided for new fields
- Migration scripts for existing data

## Testing Strategy

### Unit Tests
- Test new model fields and validations
- Test new API endpoints
- Test extended MCP tools

### Integration Tests
- Test end-to-end task creation with new features
- Test chatbot interactions with new capabilities
- Test frontend-backend integration

### Migration Tests
- Test migration from Phase 2 to Phase 5
- Test data integrity during migration
- Test API compatibility

## Timeline
- Database Model Updates: 1 day
- API Layer Updates: 2 days
- MCP Tools Updates: 1 day
- Chatbot Updates: 1 day
- Frontend Updates: 2 days
- Testing and Integration: 2 days
- Total: 9 days