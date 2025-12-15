"""Notification model for user notifications."""

from datetime import datetime, timezone
from uuid import UUID, uuid4

from sqlalchemy import Column, DateTime, Index, func
from sqlmodel import Field, SQLModel


class Notification(SQLModel, table=True):
    """Notification for a user.

    Attributes:
        id: Unique notification identifier (UUID).
        user_id: Owner user ID (FK to users).
        type: Notification type (task_due, task_overdue, task_completed, welcome, system).
        title: Short title (max 100 chars).
        message: Full message (max 500 chars).
        is_read: Whether notification has been read.
        action_url: Optional deep link URL.
        created_at: Creation timestamp (UTC).
    """

    __tablename__ = "notifications"
    __table_args__ = (
        Index("ix_notifications_user_created", "user_id", "created_at"),
        Index("ix_notifications_user_read", "user_id", "is_read"),
    )

    id: UUID = Field(
        default_factory=uuid4,
        primary_key=True,
        description="Unique notification identifier",
    )
    user_id: UUID = Field(
        foreign_key="users.id",
        nullable=False,
        index=True,
        description="Owner user ID",
    )
    type: str = Field(
        max_length=50,
        nullable=False,
        description="Notification type",
    )
    title: str = Field(
        max_length=100,
        nullable=False,
        description="Short title",
    )
    message: str = Field(
        max_length=500,
        nullable=False,
        description="Full message text",
    )
    is_read: bool = Field(
        default=False,
        nullable=False,
        description="Read status",
    )
    action_url: str | None = Field(
        default=None,
        max_length=255,
        nullable=True,
        description="Optional deep link URL",
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(
            DateTime(timezone=True),
            nullable=False,
            server_default=func.now(),
        ),
        description="Creation timestamp (UTC)",
    )
