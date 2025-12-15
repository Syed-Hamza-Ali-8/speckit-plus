"""Pydantic schemas for notification operations."""

from datetime import datetime
from uuid import UUID

from pydantic import BaseModel, Field


# Notification types enum
NOTIFICATION_TYPES = {"task_due", "task_overdue", "task_completed", "welcome", "system"}


class NotificationResponse(BaseModel):
    """Schema for notification in responses."""

    id: UUID = Field(..., description="Notification ID")
    type: str = Field(..., description="Notification type")
    title: str = Field(..., description="Short title")
    message: str = Field(..., description="Full message")
    is_read: bool = Field(..., description="Read status")
    action_url: str | None = Field(None, description="Deep link URL")
    created_at: datetime = Field(..., description="Creation timestamp")

    model_config = {"from_attributes": True}


class NotificationListResponse(BaseModel):
    """Schema for paginated notification list."""

    notifications: list[NotificationResponse] = Field(..., description="Notification list")
    unread_count: int = Field(..., description="Total unread count")
    total: int = Field(..., description="Total notification count")


class NotificationMarkReadResponse(BaseModel):
    """Schema for mark-as-read response."""

    id: UUID = Field(..., description="Notification ID")
    is_read: bool = Field(..., description="New read status")


class NotificationMarkAllReadResponse(BaseModel):
    """Schema for mark-all-as-read response."""

    marked_count: int = Field(..., description="Number of notifications marked")


class UnreadCountResponse(BaseModel):
    """Schema for unread count response."""

    unread_count: int = Field(..., description="Unread notification count")


class NotificationCreate(BaseModel):
    """Schema for creating a notification (internal use)."""

    user_id: UUID = Field(..., description="Target user ID")
    type: str = Field(..., description="Notification type")
    title: str = Field(..., max_length=100, description="Short title")
    message: str = Field(..., max_length=500, description="Full message")
    action_url: str | None = Field(None, max_length=255, description="Deep link URL")
