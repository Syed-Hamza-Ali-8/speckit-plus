from datetime import datetime, date
from enum import Enum
from typing import List, Optional
from uuid import UUID, uuid4
from sqlmodel import SQLModel, Field, Relationship
from pydantic import BaseModel


class TaskStatus(str, Enum):
    """Task status values following Phase 2 structure."""
    PENDING = "pending"
    COMPLETED = "completed"
    # Extensible: add new values without migration
    # IN_PROGRESS = "in_progress"
    # BLOCKED = "blocked"
    # ARCHIVED = "archived"


class PriorityLevel(str, Enum):
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class RecurrencePattern(str, Enum):
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"
    YEARLY = "yearly"
    CUSTOM = "custom"


# Base models
class TaskBase(SQLModel):
    title: str = Field(min_length=1, max_length=200)
    description: Optional[str] = Field(default=None, max_length=1000)
    user_id: UUID = Field(foreign_key="user.id")


class TaskCreate(TaskBase):
    priority: Optional[PriorityLevel] = PriorityLevel.MEDIUM
    tags: Optional[List[str]] = []
    due_date: Optional[date] = None  # Use date to match Phase 2
    is_recurring: bool = False
    recurring_pattern_id: Optional[UUID] = None


class TaskUpdate(SQLModel):
    title: Optional[str] = Field(default=None, min_length=1, max_length=200)
    description: Optional[str] = Field(default=None, max_length=1000)
    status: Optional[TaskStatus] = None  # Use Phase 2 TaskStatus
    priority: Optional[PriorityLevel] = None
    tags: Optional[List[str]] = None
    due_date: Optional[date] = None  # Use date to match Phase 2


class Task(TaskBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    status: TaskStatus = Field(default=TaskStatus.PENDING)  # Use Phase 2 TaskStatus
    priority: Optional[PriorityLevel] = PriorityLevel.MEDIUM
    tags: Optional[List[str]] = Field(default=[])  # Store as JSON in DB
    due_date: Optional[date] = Field(default=None)  # Use date to match Phase 2
    is_reminder_sent: bool = False
    reminder_times: Optional[List[datetime]] = Field(default=[])  # Store as JSON in DB
    is_recurring: bool = False
    recurring_pattern_id: Optional[UUID] = Field(default=None, foreign_key="recurringtaskpattern.id")
    parent_task_id: Optional[UUID] = Field(default=None, foreign_key="task.id")
    next_occurrence_id: Optional[UUID] = Field(default=None, foreign_key="task.id")
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
    created_by: UUID = Field(foreign_key="user.id")
    updated_by: UUID = Field(foreign_key="user.id")

    # Relationships
    recurring_pattern: Optional["RecurringTaskPattern"] = Relationship(back_populates="tasks")
    parent_task: Optional["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.id"))
    next_occurrence: Optional["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.id"))
    child_tasks: List["Task"] = Relationship(sa_relationship_kwargs=dict(remote_side="Task.parent_task_id"))


class RecurringTaskPatternBase(SQLModel):
    base_task_title: str = Field(max_length=200)
    base_task_description: Optional[str] = None
    user_id: UUID = Field(foreign_key="user.id")
    pattern_type: RecurrencePattern
    interval: int = 1
    start_date: date  # Use date for consistency
    end_date: Optional[date] = None
    weekdays: Optional[List[int]] = []
    days_of_month: Optional[List[int]] = []


class RecurringTaskPatternCreate(RecurringTaskPatternBase):
    pass


class RecurringTaskPatternUpdate(SQLModel):
    base_task_title: Optional[str] = Field(default=None, max_length=200)
    base_task_description: Optional[str] = None
    pattern_type: Optional[RecurrencePattern] = None
    interval: Optional[int] = None
    start_date: Optional[date] = None
    end_date: Optional[date] = None
    weekdays: Optional[List[int]] = None
    days_of_month: Optional[List[int]] = None


class RecurringTaskPattern(RecurringTaskPatternBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    tasks: List["Task"] = Relationship(back_populates="recurring_pattern")


class TagBase(SQLModel):
    name: str = Field(max_length=50)
    user_id: UUID = Field(foreign_key="user.id")
    color: Optional[str] = "#000000"
    description: Optional[str] = None


class TagCreate(TagBase):
    pass


class TagUpdate(SQLModel):
    name: Optional[str] = Field(default=None, max_length=50)
    color: Optional[str] = None
    description: Optional[str] = None


class Tag(TagBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    tasks: List["Task"] = Relationship(
        back_populates="tags",
        link_model="TaskTagLink"
    )


class ReminderBase(SQLModel):
    user_id: UUID = Field(foreign_key="user.id")
    task_id: UUID = Field(foreign_key="task.id")
    reminder_time: datetime
    notification_method: str = "push"  # push, email, sms


class ReminderCreate(ReminderBase):
    pass


class ReminderUpdate(SQLModel):
    reminder_time: Optional[datetime] = None
    notification_method: Optional[str] = None
    is_sent: Optional[bool] = None


class Reminder(ReminderBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    is_sent: bool = False
    sent_at: Optional[datetime] = None
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    task: Optional["Task"] = Relationship(back_populates="reminders")


class TaskHistoryBase(SQLModel):
    task_id: UUID = Field(foreign_key="task.id")
    user_id: UUID = Field(foreign_key="user.id")
    action: str  # created, updated, completed, deleted, etc.
    previous_state: Optional[dict] = {}
    new_state: Optional[dict] = {}


class TaskHistoryCreate(TaskHistoryBase):
    pass


class TaskHistory(TaskHistoryBase, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    created_at: datetime = Field(default_factory=datetime.utcnow)

    # Relationships
    task: Optional["Task"] = Relationship(back_populates="history")


# Link models for many-to-many relationships
class TaskTagLink(SQLModel, table=True):
    task_id: UUID = Field(foreign_key="task.id", primary_key=True)
    tag_id: UUID = Field(foreign_key="tag.id", primary_key=True)


# Pydantic models for API responses
class TaskRead(TaskBase):
    id: UUID
    status: TaskStatus
    priority: PriorityLevel
    tags: List[str] = []
    due_date: Optional[date] = None
    is_reminder_sent: bool
    is_recurring: bool
    recurring_pattern_id: Optional[UUID] = None
    parent_task_id: Optional[UUID] = None
    next_occurrence_id: Optional[UUID] = None
    created_at: datetime
    updated_at: datetime
    created_by: UUID
    updated_by: UUID

    # Include relationship data if needed
    recurring_pattern: Optional["RecurringTaskPatternRead"] = None


class RecurringTaskPatternRead(RecurringTaskPatternBase):
    id: UUID
    created_at: datetime
    updated_at: datetime


class TagRead(TagBase):
    id: UUID
    created_at: datetime


class ReminderRead(ReminderBase):
    id: UUID
    is_sent: bool
    sent_at: Optional[datetime]
    created_at: datetime


class TaskHistoryRead(TaskHistoryBase):
    id: UUID
    created_at: datetime


# Request/Response models for API endpoints
class TaskSearchRequest(BaseModel):
    query: str
    status: Optional[str] = None  # all, pending, completed
    priority: Optional[PriorityLevel] = None
    tags: Optional[List[str]] = []
    due_before: Optional[date] = None
    due_after: Optional[date] = None
    sort_by: Optional[str] = "created_at"  # created_at, title, due_date, priority
    order: Optional[str] = "desc"  # asc, desc
    page: Optional[int] = 1
    per_page: Optional[int] = 20


class TaskSearchResponse(BaseModel):
    results: List[TaskRead]
    total_count: int
    page: int
    per_page: int