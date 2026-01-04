# Phase V Integration Guide

## Overview
This document provides instructions for integrating Phase V advanced features with the existing Phase 2 architecture (frontend, backend, and chatbot).

## Architecture Overview

### Phase 2 Architecture
- **Backend**: FastAPI with SQLModel, UUID primary keys, JWT authentication
- **Frontend**: React/TypeScript with RTK Query
- **Chatbot**: OpenAI Agents SDK with MCP tools
- **API Structure**: RESTful with `/tasks` prefix
- **Task Model**: Uses UUIDs, has due_date field, TaskStatus enum

### Phase V Features
- **Advanced Task Features**: Recurring tasks, priorities, tags, search, filter, sort
- **Event-Driven Architecture**: Kafka integration
- **Dapr Integration**: All building blocks (pub/sub, state, service invocation, bindings, secrets)

## Integration Strategy

### 1. Database Model Alignment
Phase 5 has been updated to use UUID primary keys and align with Phase 2's TaskStatus enum.

### 2. API Endpoint Compatibility
Phase 5 endpoints are designed to be compatible with Phase 2's API structure while adding advanced features.

### 3. MCP Tools Extension
New MCP tools have been added for advanced features while maintaining compatibility with existing tools.

## Integration Steps

### Step 1: Database Migration
1. Update existing Phase 2 database schema to include new Phase 5 fields:
   - priority (TEXT)
   - tags (JSON/TEXT)
   - is_recurring (BOOLEAN)
   - recurring_pattern_id (UUID/FOREIGN KEY)
   - is_reminder_sent (BOOLEAN)
   - reminder_times (JSON/TEXT)

2. Create new tables for advanced features:
   - recurring_task_patterns
   - reminders
   - tags
   - task_history

### Step 2: Backend Integration

#### 2.1 Update Phase 2 Backend
- Add Phase 5 endpoints to existing Phase 2 backend
- Extend existing MCP tools with new parameters
- Add Kafka producer integration
- Add Dapr service integration

#### 2.2 MCP Tools Extension
Update the existing MCP tools in Phase 2 with new parameters:

```python
# In Phase 2 app/mcp/tools.py, extend existing tools:

@staticmethod
async def create_task(
    db: AsyncSession,
    user_id: UUID,
    input_data: CreateTaskInput,
) -> MCPToolResult:
    """Create a new task for a user (Phase 2 + Phase 5 features)."""
    try:
        task_create = TaskCreate(
            title=input_data.title,
            description=input_data.description,
            due_date=input_data.due_date,
            priority=input_data.priority,  # Phase 5: new field
            tags=input_data.tags,          # Phase 5: new field
            is_recurring=input_data.is_recurring,  # Phase 5: new field
        )
        # ... rest of implementation
```

### Step 3: Chatbot Integration

#### 3.1 Update System Prompt
Update the chatbot system prompt in Phase 2 to include new capabilities:

```python
TODO_AGENT_SYSTEM_PROMPT = """You are a helpful todo task management assistant. Your role is to help users manage their tasks through natural language conversation.

You have access to the following tools:
- add_task_tool: Create a new task with title, optional description, due date, priority, tags, and recurring options
- list_tasks_tool: View all tasks, optionally filtered by status (all, pending, completed), priority, tags
- complete_task_tool: Mark a task as completed
- delete_task_tool: Remove a task from the list
- update_task_tool: Modify a task's title, description, due date, priority, or tags

New advanced capabilities:
- create_recurring_task_tool: Create tasks that repeat on a schedule (daily, weekly, monthly, yearly)
- set_task_priority_tool: Set priority level (low, medium, high) for tasks
- add_task_tags_tool: Add tags to organize tasks
- search_tasks_tool: Find tasks by keywords and filters

Guidelines:
1. Be helpful and conversational
2. When creating tasks, extract the title from the user's message
3. When listing tasks, present them in a clear, readable format
4. For recurring tasks, ask for the pattern (daily, weekly, monthly, etc.)
5. For destructive actions (delete), confirm the task details before proceeding
6. If a user's request is ambiguous, ask clarifying questions
7. Always confirm actions after they're completed

Examples of user requests you should handle:
- "Add a task to buy groceries" -> Use add_task_tool with title="Buy groceries"
- "Create a recurring task to water plants every Monday" -> Use create_recurring_task_tool
- "Show my high priority tasks" -> Use list_tasks_tool with priority filter
- "Find tasks with 'meeting' in the title" -> Use search_tasks_tool
- "Add 'work' tag to this task" -> Use add_task_tags_tool
"""
```

#### 3.2 Add New MCP Tools
Add new MCP tools for Phase 5 features in Phase 2:

```python
# In Phase 2 app/mcp/tools.py, add new tools:

@staticmethod
async def create_recurring_task(
    db: AsyncSession,
    user_id: UUID,
    input_data: CreateRecurringTaskInput,
) -> MCPToolResult:
    """Create a recurring task pattern."""
    # Implementation for recurring tasks

@staticmethod
async def set_task_priority(
    db: AsyncSession,
    user_id: UUID,
    input_data: SetTaskPriorityInput,
) -> MCPToolResult:
    """Set priority for a task."""
    # Implementation for setting priority

# ... other new tools
```

### Step 4: Frontend Integration

#### 4.1 Update API Services
Update the frontend API services to support new features:

```typescript
// In phase2/frontend/src/services/taskApi.ts, extend with new endpoints:

export const taskApi = api.injectEndpoints({
  endpoints: (builder) => ({
    // ... existing endpoints

    // New Phase 5 endpoints
    createRecurringTask: builder.mutation<RecurringTaskPattern, RecurringTaskCreate>({
      query: (pattern) => ({
        url: '/recurring-tasks',
        method: 'POST',
        body: pattern,
      }),
      invalidatesTags: [{ type: TAG_TYPES.Task, id: 'LIST' }],
    }),

    setTaskPriority: builder.mutation<Task, { id: string; priority: string }>({
      query: ({ id, priority }) => ({
        url: `/tasks/${id}/priority`,
        method: 'PUT',
        body: { priority },
      }),
      invalidatesTags: (_result, _error, { id }) => [{ type: TAG_TYPES.Task, id }],
    }),

    // ... other new endpoints
  }),
});
```

#### 4.2 Update UI Components
- Add new forms for recurring tasks
- Update task lists to show priorities and tags
- Add filtering and sorting controls
- Update task creation/editing forms with new fields

## Deployment Strategy

### Option 1: Incremental Integration (Recommended)
1. Deploy Phase 5 backend endpoints alongside Phase 2
2. Gradually migrate frontend to use new features
3. Update MCP tools to support new capabilities
4. Enhance chatbot with new features

### Option 2: Complete Migration
1. Update Phase 2 backend to include all Phase 5 features
2. Update frontend to support all new functionality
3. Enhance chatbot with advanced capabilities
4. Deploy event-driven architecture with Kafka/Dapr

## API Compatibility

### Phase 2 Compatible Endpoints
All Phase 5 endpoints maintain compatibility with Phase 2:

```
GET    /tasks                    # Phase 2 compatible + advanced filters
POST   /tasks                    # Phase 2 compatible + new fields
GET    /tasks/{task_id}          # Phase 2 compatible
PATCH  /tasks/{task_id}          # Phase 2 compatible + new fields
DELETE /tasks/{task_id}          # Phase 2 compatible
```

### Phase 5 New Endpoints
```
POST   /recurring-tasks          # Create recurring task patterns
GET    /users/{user_id}/recurring-tasks  # Get user's recurring patterns
PUT    /tasks/{task_id}/priority # Set task priority
PUT    /tasks/{task_id}/tags     # Add/remove tags
GET    /users/{user_id}/tasks/search     # Search tasks
GET    /users/{user_id}/tasks/filter     # Filter tasks
GET    /users/{user_id}/tasks/sort       # Sort tasks
```

## Event-Driven Architecture

### Kafka Integration
Phase 5 includes Kafka event producers for:
- Task creation/update/completion/deletion events
- Reminder scheduling events
- Recurring task generation events

### Dapr Integration
Phase 5 includes Dapr services for:
- Pub/Sub for event communication
- State management for conversation state
- Service invocation between microservices
- Secret management for credentials
- Bindings for scheduled operations

## Testing Integration

### Unit Tests
- Test new model fields and validations
- Test extended API endpoints
- Test enhanced MCP tools

### Integration Tests
- Test end-to-end task management with new features
- Test chatbot interactions with advanced capabilities
- Test frontend-backend integration

### Migration Tests
- Test data migration from Phase 2 to Phase 5
- Test API compatibility
- Test existing functionality remains intact

## Security Considerations

### Authentication
- Maintain JWT-based authentication from Phase 2
- Ensure new endpoints are properly secured
- Validate user permissions for all operations

### Authorization
- Ensure users can only access their own data
- Validate permissions for recurring task patterns
- Secure access to advanced features

## Performance Considerations

### Database Performance
- Add indexes for new fields (priority, tags, etc.)
- Optimize queries for search and filtering
- Consider caching for frequently accessed data

### API Performance
- Implement pagination for large result sets
- Add rate limiting for new endpoints
- Optimize queries with proper filtering

## Monitoring and Observability

### Logging
- Add structured logging for new features
- Log important events and operations
- Monitor system performance

### Metrics
- Track API performance metrics
- Monitor Kafka/Dapr integration
- Measure system health

## Rollback Plan

If integration issues occur:
1. Disable new Phase 5 features while maintaining Phase 2 functionality
2. Rollback database changes if needed
3. Revert API extensions
4. Restore original MCP tools and chatbot functionality

## Next Steps

1. **Database Migration**: Update schema with new fields and tables
2. **Backend Integration**: Add Phase 5 endpoints to Phase 2 backend
3. **MCP Tools**: Extend existing tools with new capabilities
4. **Chatbot**: Update system prompt and add new tools
5. **Frontend**: Add UI support for new features
6. **Testing**: Thoroughly test integration
7. **Deployment**: Deploy with monitoring and rollback plan