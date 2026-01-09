# Advanced Features Specification for Phase V

## Overview
This document details the specifications for the advanced features to be implemented in Phase V of the Todo app evolution.

## 1. Recurring Tasks Feature

### 1.1 Requirements
- Users can create tasks that repeat on a schedule
- Support for various recurrence patterns: daily, weekly, monthly, yearly
- Option to set an end date or repeat indefinitely
- When a recurring task is marked complete, the next occurrence should be automatically created
- Users can modify or delete recurring task patterns

### 1.2 Data Model Changes
```python
class RecurringTaskPattern(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="users.id")
    base_task_id: int = Field(foreign_key="tasks.id")  # Reference to the template task
    pattern_type: str  # daily, weekly, monthly, yearly
    interval: int  # Every N days/weeks/months/years
    start_date: datetime
    end_date: Optional[datetime] = None  # None means indefinitely
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

class Task(SQLModel, table=True):
    # Existing fields...
    is_recurring: bool = False  # Whether this task was generated from a recurring pattern
    recurring_pattern_id: Optional[int] = Field(default=None, foreign_key="recurringtaskpatterns.id")
    parent_task_id: Optional[int] = Field(default=None, foreign_key="tasks.id")  # For recurring tasks, points to template
    next_occurrence_id: Optional[int] = Field(default=None, foreign_key="tasks.id")  # Next occurrence in the chain
```

### 1.3 API Endpoints
- **POST /api/{user_id}/tasks/recurring** - Create a recurring task pattern
- **GET /api/{user_id}/tasks/recurring** - List recurring task patterns
- **PUT /api/{user_id}/tasks/recurring/{id}** - Update a recurring task pattern
- **DELETE /api/{user_id}/tasks/recurring/{id}** - Delete a recurring task pattern

### 1.4 MCP Tools
- `create_recurring_task`: Create a recurring task pattern
- `update_recurring_task`: Update a recurring task pattern
- `delete_recurring_task`: Delete a recurring task pattern
- `list_recurring_tasks`: List recurring task patterns

### 1.5 Natural Language Commands
- "Create a recurring task to water plants every day"
- "Set up a weekly recurring task for team meetings every Monday"
- "Create a monthly recurring task to pay rent on the 1st"
- "I want to exercise every other day"
- "Create a yearly recurring task for my birthday"

## 2. Due Dates & Time Reminders Feature

### 2.1 Requirements
- Users can set due dates and times for tasks
- System should send reminders at specified times before due date
- Support for multiple reminder times per task
- Browser notifications for upcoming due tasks
- Option to snooze reminders

### 2.2 Data Model Changes
```python
class Task(SQLModel, table=True):
    # Existing fields...
    due_date: Optional[datetime] = None
    reminder_times: Optional[List[datetime]] = []  # Array of reminder times
    is_reminder_sent: bool = False
    last_reminder_sent: Optional[datetime] = None

class Reminder(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="users.id")
    task_id: int = Field(foreign_key="tasks.id")
    reminder_time: datetime
    is_sent: bool = False
    sent_at: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

### 2.3 API Endpoints
- **PUT /api/{user_id}/tasks/{id}/due-date** - Set due date for a task
- **PUT /api/{user_id}/tasks/{id}/reminders** - Set reminder times for a task
- **GET /api/{user_id}/tasks/due-soon** - Get tasks due soon (configurable time window)
- **POST /api/{user_id}/tasks/{id}/snooze-reminder** - Snooze a reminder

### 2.4 MCP Tools
- `set_task_due_date`: Set due date for a task
- `set_task_reminder`: Set reminder times for a task
- `get_due_tasks`: Get tasks due within a specified time window
- `snooze_reminder`: Snooze a reminder for a task

### 2.5 Natural Language Commands
- "Set a due date for the project report to next Friday"
- "Remind me about the meeting 30 minutes before it starts"
- "I need to finish this by end of day tomorrow"
- "Send me a reminder for the dentist appointment 2 hours before"
- "Set a due date for my vacation planning to next month"

## 3. Priorities & Tags/Categories Feature

### 3.1 Requirements
- Users can assign priority levels (high/medium/low) to tasks
- Users can create and assign tags/categories to tasks
- Support for multiple tags per task
- Ability to filter and sort by priority and tags

### 3.2 Data Model Changes
```python
from enum import Enum

class PriorityLevel(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

class Task(SQLModel, table=True):
    # Existing fields...
    priority: Optional[PriorityLevel] = PriorityLevel.MEDIUM
    tags: Optional[List[str]] = []  # Array of tag names

class Tag(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="users.id")
    name: str
    color: Optional[str] = "#000000"  # Color for UI representation
    created_at: datetime = Field(default_factory=datetime.utcnow)
```

### 3.3 API Endpoints
- **PUT /api/{user_id}/tasks/{id}/priority** - Set priority for a task
- **PUT /api/{user_id}/tasks/{id}/tags** - Set tags for a task
- **GET /api/{user_id}/tasks?priority={level}** - Filter tasks by priority
- **GET /api/{user_id}/tasks?tags={tag1,tag2}** - Filter tasks by tags
- **GET /api/{user_id}/tags** - Get all tags for user

### 3.4 MCP Tools
- `set_task_priority`: Set priority for a task
- `add_task_tags`: Add tags to a task
- `remove_task_tags`: Remove tags from a task
- `filter_tasks_by_priority`: Filter tasks by priority
- `filter_tasks_by_tags`: Filter tasks by tags

### 3.5 Natural Language Commands
- "Set the priority of the project task to high"
- "Add 'work' and 'urgent' tags to this task"
- "Show me all high priority tasks"
- "Find tasks with the 'personal' tag"
- "I need to prioritize the client meeting task as high"

## 4. Search & Filter Feature

### 4.1 Requirements
- Full-text search in task titles and descriptions
- Advanced filtering by multiple criteria
- Combined search and filter capabilities
- Support for complex queries

### 4.2 API Endpoints
- **GET /api/{user_id}/tasks/search?q={query}&filters** - Search tasks with optional filters
- **GET /api/{user_id}/tasks?status={status}&priority={level}&due_before={date}&tags={tags}** - Advanced filtering

### 4.3 MCP Tools
- `search_tasks`: Search tasks by keyword
- `filter_tasks`: Filter tasks by multiple criteria
- `advanced_search`: Complex search with multiple filters

### 4.4 Natural Language Commands
- "Find tasks containing 'meeting' in the title"
- "Show me all pending work tasks due this week"
- "Search for tasks with 'important' in the description"
- "Find high priority tasks with the 'bug' tag"

## 5. Sort Tasks Feature

### 5.1 Requirements
- Sort tasks by various criteria
- Support for ascending and descending order
- Multi-level sorting (e.g., sort by priority then by due date)

### 5.2 API Endpoints
- **GET /api/{user_id}/tasks?sort_by={field}&order={asc|desc}** - Sort tasks by specified field

### 5.3 MCP Tools
- `sort_tasks`: Sort tasks by specified criteria

### 5.4 Natural Language Commands
- "Sort my tasks by due date"
- "Show tasks by priority with high priority first"
- "Sort tasks alphabetically by title"
- "Order tasks with most urgent first"

## 6. Integration with Existing Features

### 6.1 Combined Functionality
- All advanced features should work in combination
- Users can create high priority recurring tasks with due dates and tags
- Search should work across all task attributes
- Filters should be combinable

### 6.2 Natural Language Processing
- The AI chatbot should understand complex requests combining multiple features
- Example: "Create a high priority recurring task to backup data every day and remind me 1 hour before each occurrence"

## 7. User Experience Considerations
- Intuitive UI for setting up recurring tasks
- Clear indication of recurring tasks in the task list
- Easy access to due date and reminder settings
- Visual indicators for priority levels
- Tag management interface
- Responsive search and filter controls