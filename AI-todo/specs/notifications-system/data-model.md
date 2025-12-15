# Data Model — Notifications System

**Date**: 2025-12-15
**Feature**: notifications-system

## Entity Definitions

### Notification

**Table**: `notifications`

| Field | Type | Constraints | Description |
|-------|------|-------------|-------------|
| `id` | UUID | PK | Unique notification identifier |
| `user_id` | UUID | FK → users.id, NOT NULL | Owner of notification |
| `type` | VARCHAR(50) | NOT NULL | Notification type |
| `title` | VARCHAR(100) | NOT NULL | Short title |
| `message` | VARCHAR(500) | NOT NULL | Full message text |
| `is_read` | BOOLEAN | NOT NULL, default false | Read status |
| `action_url` | VARCHAR(255) | NULLABLE | Optional deep link |
| `created_at` | TIMESTAMP(tz) | NOT NULL | Creation timestamp |

**Indexes**:
- `ix_notifications_user_created` on `(user_id, created_at DESC)` - Paginated queries
- `ix_notifications_user_read` on `(user_id, is_read)` - Unread count queries

**Foreign Keys**:
- `user_id` → `users.id` ON DELETE CASCADE

---

## Schema Definitions

### Backend (Python/SQLModel)

#### Notification Model

```python
# app/models/notification.py

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
```

#### Pydantic Schemas

```python
# app/schemas/notification.py

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
```

---

### Frontend (TypeScript)

#### Notification Types

```typescript
// src/types/notification.ts

/**
 * Notification types
 */
export type NotificationType =
  | 'task_due'
  | 'task_overdue'
  | 'task_completed'
  | 'welcome'
  | 'system';

/**
 * Notification from backend
 */
export interface Notification {
  id: string;
  type: NotificationType;
  title: string;
  message: string;
  is_read: boolean;
  action_url: string | null;
  created_at: string;
}

/**
 * Paginated notification list response
 */
export interface NotificationListResponse {
  notifications: Notification[];
  unread_count: number;
  total: number;
}

/**
 * Mark as read response
 */
export interface NotificationMarkReadResponse {
  id: string;
  is_read: boolean;
}

/**
 * Mark all as read response
 */
export interface NotificationMarkAllReadResponse {
  marked_count: number;
}

/**
 * Unread count response
 */
export interface UnreadCountResponse {
  unread_count: number;
}

/**
 * Query params for notification list
 */
export interface NotificationQueryParams {
  limit?: number;
  offset?: number;
  unread_only?: boolean;
}
```

---

## Validation Rules

### Notification Create (Internal)

| Field | Rule | Error Message |
|-------|------|---------------|
| `type` | must be in NOTIFICATION_TYPES | "Invalid notification type" |
| `title` | max 100 chars | "Title too long" |
| `message` | max 500 chars | "Message too long" |
| `action_url` | valid URL format (if provided) | "Invalid URL format" |

---

## State Transitions

### Notification Read Status

```
┌─────────────┐     PATCH /notifications/{id}/read     ┌─────────────┐
│   Unread    │ ────────────────────────────────────► │    Read     │
│  is_read=   │                                        │  is_read=   │
│   false     │                                        │    true     │
└─────────────┘                                        └─────────────┘
                                                             │
                     No reverse transition                   │
                     (notifications stay read)              ─┘
```

---

## Database Migration

### Alembic Migration Script

```python
# alembic/versions/YYYYMMDD_create_notifications_table.py
"""Create notifications table

Revision ID: 006
Revises: 005
Create Date: 2025-12-15

"""

from typing import Sequence, Union

import sqlalchemy as sa
from alembic import op

revision: str = "006"
down_revision: Union[str, None] = "005"
branch_labels: Union[str, Sequence[str], None] = None
depends_on: Union[str, Sequence[str], None] = None


def upgrade() -> None:
    op.create_table(
        "notifications",
        sa.Column("id", sa.UUID(), nullable=False),
        sa.Column("user_id", sa.UUID(), nullable=False),
        sa.Column("type", sa.String(length=50), nullable=False),
        sa.Column("title", sa.String(length=100), nullable=False),
        sa.Column("message", sa.String(length=500), nullable=False),
        sa.Column("is_read", sa.Boolean(), nullable=False, server_default="false"),
        sa.Column("action_url", sa.String(length=255), nullable=True),
        sa.Column(
            "created_at",
            sa.DateTime(timezone=True),
            nullable=False,
            server_default=sa.func.now(),
        ),
        sa.ForeignKeyConstraint(
            ["user_id"],
            ["users.id"],
            ondelete="CASCADE",
        ),
        sa.PrimaryKeyConstraint("id"),
    )
    # Create indexes
    op.create_index(
        "ix_notifications_user_created",
        "notifications",
        ["user_id", sa.text("created_at DESC")],
    )
    op.create_index(
        "ix_notifications_user_read",
        "notifications",
        ["user_id", "is_read"],
    )


def downgrade() -> None:
    op.drop_index("ix_notifications_user_read", table_name="notifications")
    op.drop_index("ix_notifications_user_created", table_name="notifications")
    op.drop_table("notifications")
```

---

## Cascade Behavior

When a user is deleted:

```
User (DELETE)
  └── Notifications (CASCADE DELETE via user_id FK)
```

**Note**: Notifications do NOT have a direct FK to tasks to allow loose coupling. Task-related notifications use `action_url` for navigation.
