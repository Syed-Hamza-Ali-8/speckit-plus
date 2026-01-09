"""Task model with dual-layer validation (Python + Database)."""

from datetime import date, datetime, timezone
from enum import Enum
from uuid import UUID, uuid4
from typing import List, Optional

from pydantic import field_validator
import sqlalchemy as sa
from sqlalchemy import CheckConstraint, Column, Date, DateTime, Index, func
from sqlalchemy import Enum as SAEnum
from sqlalchemy.dialects.postgresql import JSON
from sqlmodel import Field, SQLModel


class TaskStatus(str, Enum):
    """Task status values.

    Uses native_enum=False for extensibility without migrations.
    To add new statuses, simply add them here - no schema migration needed.
    """

    PENDING = "pending"
    COMPLETED = "completed"
    # Extensible: add new values without migration
    # IN_PROGRESS = "in_progress"
    # BLOCKED = "blocked"
    # ARCHIVED = "archived"


class PriorityLevel(str, Enum):
    """Priority levels for tasks."""
    LOW = "low"
    MEDIUM = "medium"
    HIGH = "high"


class RecurrencePattern(str, Enum):
    """Types of recurrence patterns."""
    DAILY = "daily"
    WEEKLY = "weekly"
    MONTHLY = "monthly"
    YEARLY = "yearly"
    CUSTOM = "custom"


class Task(SQLModel, table=True):
    """Task entity representing a todo item owned by a user."""

    model_config = {"validate_assignment": True}

    __tablename__ = "tasks"
    __table_args__ = (
        CheckConstraint(
            "length(trim(title)) > 0",
            name="ck_tasks_title_not_empty",
        ),
        Index("ix_tasks_user_id", "user_id"),
    )

    id: UUID = Field(
        default_factory=uuid4,
        primary_key=True,
        description="Unique task identifier",
    )
    user_id: UUID = Field(
        nullable=False,
        description="Owner's user ID (FK deferred to Part 2)",
    )
    title: str = Field(
        max_length=200,
        nullable=False,
        description="Task title (1-200 characters)",
    )
    description: str | None = Field(
        default=None,
        max_length=1000,
        description="Optional task description",
    )
    status: TaskStatus = Field(
        default=TaskStatus.PENDING,
        sa_column=Column(
            SAEnum(TaskStatus, native_enum=False, length=50),
            nullable=False,
        ),
        description="Task status",
    )
    due_date: date | None = Field(
        default=None,
        sa_column=Column(Date, nullable=True),
        description="Optional due date for the task",
    )
    # Phase V: Advanced features
    priority: str = Field(
        default="medium",
        sa_column=Column(
            sa.String(20),
            nullable=False,
            server_default="medium"
        ),
        description="Task priority level",
    )
    tags: List[str] = Field(
        default_factory=list,
        sa_column=Column(
            sa.JSON,
            nullable=False,
            default=list,
        ),
        description="List of tags for the task",
    )
    is_recurring: bool = Field(
        default=False,
        description="Whether this task is part of a recurring pattern",
    )
    recurring_pattern_id: Optional[UUID] = Field(
        default=None,
        description="ID of the recurring pattern this task belongs to",
    )
    is_reminder_sent: bool = Field(
        default=False,
        description="Whether a reminder has been sent for this task",
    )
    parent_task_id: Optional[UUID] = Field(
        default=None,
        description="ID of the parent task (for recurring tasks)",
    )
    next_occurrence_id: Optional[UUID] = Field(
        default=None,
        description="ID of the next occurrence (for recurring tasks)",
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),  # ensures DB default in UTC
        ),
        description="Creation timestamp (UTC)",
    )
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),
            onupdate=func.now(),  # auto-update on modification
        ),
        description="Last modification timestamp (UTC)",
    )

    @field_validator("title", mode="before")
    @classmethod
    def title_not_empty(cls, v: str) -> str:
        """Validate title is not empty or whitespace-only."""
        if not v or not v.strip():
            raise ValueError("Title cannot be empty or whitespace-only")
        return v
