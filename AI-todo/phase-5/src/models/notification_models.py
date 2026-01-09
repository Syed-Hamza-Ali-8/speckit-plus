"""Notification models for Phase V Todo App."""
from datetime import datetime
from enum import Enum
from typing import Optional
from uuid import UUID, uuid4
from sqlmodel import SQLModel, Field


class NotificationType(str, Enum):
    """Notification type values."""
    TASK_CREATED = "task_created"
    TASK_UPDATED = "task_updated"
    TASK_DELETED = "task_deleted"
    TASK_COMPLETED = "task_completed"
    TASK_DUE_SOON = "task_due_soon"
    REMINDER = "reminder"


class NotificationBase(SQLModel):
    """Base notification model."""
    user_id: UUID = Field(foreign_key="user.id")
    title: str = Field(max_length=200)
    message: str = Field(max_length=500)
    type: NotificationType
    task_id: Optional[UUID] = Field(default=None, foreign_key="task.id")


class NotificationCreate(NotificationBase):
    """Model for creating notifications."""
    pass


class Notification(NotificationBase, table=True):
    """Notification database model."""
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    is_read: bool = Field(default=False)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    read_at: Optional[datetime] = Field(default=None)


class NotificationUpdate(SQLModel):
    """Model for updating notifications."""
    is_read: Optional[bool] = None
    read_at: Optional[datetime] = None
