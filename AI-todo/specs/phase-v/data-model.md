# Phase V Data Model Specification

## Overview
This document defines the data models for Phase V of the Todo app, including all advanced features and supporting entities for event-driven architecture and Dapr integration.

## Entity Relationship Diagram
```
Users (1) -----> (Many) Tasks
Users (1) -----> (Many) RecurringTaskPatterns
Users (1) -----> (Many) Tags
Users (1) -----> (Many) Reminders
Users (1) -----> (Many) Conversations
Conversations (1) -----> (Many) Messages
Tasks (1) -----> (Many) TaskHistory
RecurringTaskPatterns (1) -----> (Many) Tasks (via recurrence)
```

## Core Entities

### 1. User (Managed by Better Auth)
```python
class User(SQLModel, table=True):
    id: str = Field(default=None, primary_key=True)
    email: str = Field(sa_column=Column(String, unique=True, index=True))
    name: str
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    preferences: Optional[dict] = {}  # User preferences for notifications, etc.
```

### 2. Task (Enhanced from previous phases)
```python
from enum import Enum
from typing import List, Optional

class PriorityLevel(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"

class Task(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    title: str = Field(max_length=200)
    description: Optional[str] = Field(default=None, max_length=1000)
    completed: bool = False
    completed_at: Optional[datetime] = None

    # Advanced Features
    priority: Optional[PriorityLevel] = PriorityLevel.MEDIUM
    tags: Optional[List[str]] = []  # Array of tag names
    due_date: Optional[datetime] = None
    is_reminder_sent: bool = False
    reminder_times: Optional[List[datetime]] = []  # Array of reminder times

    # Recurring Task Support
    is_recurring: bool = False
    recurring_pattern_id: Optional[int] = Field(default=None, foreign_key="recurringtaskpattern.id")
    parent_task_id: Optional[int] = Field(default=None, foreign_key="task.id")  # For recurring tasks, points to template
    next_occurrence_id: Optional[int] = Field(default=None, foreign_key="task.id")  # Next occurrence in the chain

    # Metadata
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    created_by: str = Field(foreign_key="user.id")
    updated_by: str = Field(foreign_key="user.id")

    # Relationships
    user: Optional["User"] = Relationship(back_populates="tasks")
    recurring_pattern: Optional["RecurringTaskPattern"] = Relationship(back_populates="tasks")
    parent_task: Optional["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.id"))
    next_occurrence: Optional["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.id"))
    child_tasks: List["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.parent_task_id"))
```

### 3. RecurringTaskPattern
```python
from enum import Enum

class RecurrencePattern(str, Enum):
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"
    YEARLY = "yearly"
    CUSTOM = "custom"

class RecurringTaskPattern(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    base_task_title: str  # Template title for recurring tasks
    base_task_description: Optional[str] = None
    pattern_type: RecurrencePattern
    interval: int = 1  # Every N days/weeks/months/years
    start_date: datetime
    end_date: Optional[datetime] = None  # None means indefinitely
    weekdays: Optional[List[int]] = []  # For weekly patterns (0=Sunday, 1=Monday, etc.)
    days_of_month: Optional[List[int]] = []  # For monthly patterns
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    user: Optional["User"] = Relationship(back_populates="recurring_patterns")
    tasks: List["Task"] = Relationship(back_populates="recurring_pattern")
```

### 4. Tag
```python
class Tag(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    name: str = Field(max_length=50, index=True)
    color: Optional[str] = "#000000"  # Color for UI representation
    description: Optional[str] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    user: Optional["User"] = Relationship(back_populates="tags")
```

### 5. Reminder
```python
class Reminder(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    task_id: int = Field(foreign_key="task.id")
    reminder_time: datetime
    notification_method: str = "push"  # push, email, sms
    is_sent: bool = False
    sent_at: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    user: Optional["User"] = Relationship(back_populates="reminders")
    task: Optional["Task"] = Relationship(back_populates="reminders")
```

### 6. Conversation (Enhanced for Phase V)
```python
class Conversation(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    title: Optional[str] = None  # Auto-generated from first message or user-provided
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    is_active: bool = True

    # Relationships
    user: Optional["User"] = Relationship(back_populates="conversations")
    messages: List["Message"] = Relationship(back_populates="conversation")
```

### 7. Message (Enhanced for Phase V)
```python
class Message(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    conversation_id: int = Field(foreign_key="conversation.id")
    role: str = Field(regex="^(user|assistant)$")  # user or assistant
    content: str
    tool_calls: Optional[List[dict]] = []  # For MCP tool calls
    tool_responses: Optional[List[dict]] = []  # For MCP tool responses
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    user: Optional["User"] = Relationship(back_populates="messages")
    conversation: Optional["Conversation"] = Relationship(back_populates="messages")
```

### 8. TaskHistory (New for Audit Trail)
```python
class TaskHistory(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    task_id: int = Field(foreign_key="task.id")
    user_id: str = Field(foreign_key="user.id")
    action: str  # created, updated, completed, deleted, priority_changed, due_date_set, tags_added, etc.
    previous_state: Optional[dict] = {}  # Previous task state before change
    new_state: Optional[dict] = {}  # New task state after change
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    task: Optional["Task"] = Relationship(back_populates="history")
    user: Optional["User"] = Relationship()
```

### 9. AuditLog (New for Phase V)
```python
class AuditLog(SQLModel, table=True):
    id: int = Field(default=None, primary_key=True)
    user_id: str = Field(foreign_key="user.id")
    action: str  # The action performed
    resource_type: str  # Type of resource (task, recurring_pattern, reminder, etc.)
    resource_id: Optional[int] = None  # ID of the resource
    ip_address: Optional[str] = None
    user_agent: Optional[str] = None
    details: Optional[dict] = {}  # Additional details about the action
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    user: Optional["User"] = Relationship()
```

## Indexes for Performance
```sql
-- Task indexes
CREATE INDEX idx_task_user_id ON tasks(user_id);
CREATE INDEX idx_task_completed ON tasks(completed);
CREATE INDEX idx_task_due_date ON tasks(due_date);
CREATE INDEX idx_task_priority ON tasks(priority);
CREATE INDEX idx_task_created_at ON tasks(created_at);

-- Tag indexes
CREATE INDEX idx_tag_user_id ON tags(user_id);
CREATE INDEX idx_tag_name ON tags(user_name);

-- Reminder indexes
CREATE INDEX idx_reminder_user_id ON reminders(user_id);
CREATE INDEX idx_reminder_task_id ON reminders(task_id);
CREATE INDEX idx_reminder_reminder_time ON reminders(reminder_time);
CREATE INDEX idx_reminder_is_sent ON reminders(is_sent);

-- Conversation indexes
CREATE INDEX idx_conversation_user_id ON conversations(user_id);
CREATE INDEX idx_conversation_updated_at ON conversations(updated_at);
```

## Dapr State Store Keys
- `conversation-{conversation_id}`: Stores conversation state
- `user-preferences-{user_id}`: Stores user preferences
- `session-{session_id}`: Stores session data
- `cache-{resource_type}-{resource_id}`: General caching pattern

## Kafka Message Keys
- `task-{task_id}`: For task-related events
- `user-{user_id}`: For user-specific events
- `reminder-{reminder_id}`: For reminder events