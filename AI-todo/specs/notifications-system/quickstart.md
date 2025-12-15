# Quickstart — Notifications System

**Date**: 2025-12-15
**Feature**: notifications-system

## Prerequisites

- Node.js 18+
- Python 3.11+
- Running backend (`uvicorn app.main:app`)
- Running frontend (`npm run dev`)
- Authenticated user (JWT token)
- Previous migrations applied

## Implementation Order

```
Step 1-2: Backend Model & Migration
         ↓
Step 3-4: Backend Service & Endpoints
         ↓
Step 5: Frontend Types & API
         ↓
Step 6: NotificationBell Component
         ↓
Step 7: Notification Dropdown & Items
         ↓
Step 8: Integration & Toast Sync
```

---

## Step 1: Create Notification Model

```python
# app/models/notification.py
from datetime import datetime, timezone
from uuid import UUID, uuid4

from sqlalchemy import Column, DateTime, Index, func
from sqlmodel import Field, SQLModel


class Notification(SQLModel, table=True):
    __tablename__ = "notifications"
    __table_args__ = (
        Index("ix_notifications_user_created", "user_id", "created_at"),
        Index("ix_notifications_user_read", "user_id", "is_read"),
    )

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID = Field(foreign_key="users.id", nullable=False, index=True)
    type: str = Field(max_length=50, nullable=False)
    title: str = Field(max_length=100, nullable=False)
    message: str = Field(max_length=500, nullable=False)
    is_read: bool = Field(default=False, nullable=False)
    action_url: str | None = Field(default=None, max_length=255, nullable=True)
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        sa_column=Column(DateTime(timezone=True), nullable=False, server_default=func.now()),
    )
```

---

## Step 2: Create Migration

```bash
cd phase2/backend
# Create migration file manually or use alembic autogenerate
```

```python
# alembic/versions/20251215_000003_create_notifications.py
def upgrade() -> None:
    op.create_table(
        "notifications",
        sa.Column("id", sa.UUID(), nullable=False),
        sa.Column("user_id", sa.UUID(), nullable=False),
        sa.Column("type", sa.String(50), nullable=False),
        sa.Column("title", sa.String(100), nullable=False),
        sa.Column("message", sa.String(500), nullable=False),
        sa.Column("is_read", sa.Boolean(), nullable=False, server_default="false"),
        sa.Column("action_url", sa.String(255), nullable=True),
        sa.Column("created_at", sa.DateTime(timezone=True), nullable=False, server_default=sa.func.now()),
        sa.ForeignKeyConstraint(["user_id"], ["users.id"], ondelete="CASCADE"),
        sa.PrimaryKeyConstraint("id"),
    )
    op.create_index("ix_notifications_user_created", "notifications", ["user_id", "created_at"])
    op.create_index("ix_notifications_user_read", "notifications", ["user_id", "is_read"])

def downgrade() -> None:
    op.drop_index("ix_notifications_user_read")
    op.drop_index("ix_notifications_user_created")
    op.drop_table("notifications")
```

```bash
uv run alembic upgrade head
```

---

## Step 3: Create Notification Service

```python
# app/services/notification_service.py
from uuid import UUID
from sqlmodel import select, func
from sqlmodel.ext.asyncio.session import AsyncSession

from app.models.notification import Notification

NOTIFICATION_LIMIT = 100  # Max per user


async def get_notifications(
    db: AsyncSession,
    user_id: UUID,
    limit: int = 20,
    offset: int = 0,
    unread_only: bool = False,
) -> tuple[list[Notification], int, int]:
    """Get paginated notifications with counts."""
    query = select(Notification).where(Notification.user_id == user_id)
    if unread_only:
        query = query.where(Notification.is_read == False)
    query = query.order_by(Notification.created_at.desc()).offset(offset).limit(limit)

    result = await db.exec(query)
    notifications = result.all()

    # Get counts
    unread_count = await get_unread_count(db, user_id)
    total = await db.exec(
        select(func.count()).select_from(Notification).where(Notification.user_id == user_id)
    )

    return notifications, unread_count, total.first() or 0


async def get_unread_count(db: AsyncSession, user_id: UUID) -> int:
    """Get unread notification count."""
    result = await db.exec(
        select(func.count())
        .select_from(Notification)
        .where(Notification.user_id == user_id, Notification.is_read == False)
    )
    return result.first() or 0


async def mark_as_read(db: AsyncSession, notification_id: UUID, user_id: UUID) -> Notification | None:
    """Mark single notification as read."""
    result = await db.exec(
        select(Notification).where(Notification.id == notification_id, Notification.user_id == user_id)
    )
    notification = result.first()
    if notification:
        notification.is_read = True
        db.add(notification)
        await db.commit()
        await db.refresh(notification)
    return notification


async def mark_all_as_read(db: AsyncSession, user_id: UUID) -> int:
    """Mark all notifications as read."""
    result = await db.exec(
        select(Notification).where(Notification.user_id == user_id, Notification.is_read == False)
    )
    notifications = result.all()
    count = len(notifications)
    for n in notifications:
        n.is_read = True
        db.add(n)
    await db.commit()
    return count


async def create_notification(
    db: AsyncSession,
    user_id: UUID,
    type: str,
    title: str,
    message: str,
    action_url: str | None = None,
) -> Notification:
    """Create notification and cleanup old ones."""
    notification = Notification(
        user_id=user_id,
        type=type,
        title=title,
        message=message,
        action_url=action_url,
    )
    db.add(notification)

    # Cleanup if over limit
    count_result = await db.exec(
        select(func.count()).select_from(Notification).where(Notification.user_id == user_id)
    )
    count = count_result.first() or 0
    if count >= NOTIFICATION_LIMIT:
        old = await db.exec(
            select(Notification)
            .where(Notification.user_id == user_id)
            .order_by(Notification.created_at.asc())
            .limit(count - NOTIFICATION_LIMIT + 1)
        )
        for old_notif in old.all():
            await db.delete(old_notif)

    await db.commit()
    await db.refresh(notification)
    return notification
```

---

## Step 4: Create Notification Endpoints

```python
# app/api/routes/notifications.py
from uuid import UUID
from fastapi import APIRouter, HTTPException, Query, status

from app.api.deps import CurrentUser, DbSession
from app.schemas.notification import (
    NotificationListResponse,
    NotificationMarkReadResponse,
    NotificationMarkAllReadResponse,
    NotificationResponse,
    UnreadCountResponse,
)
from app.services import notification_service

router = APIRouter(prefix="/notifications", tags=["notifications"])


@router.get("", response_model=NotificationListResponse)
async def get_notifications(
    current_user: CurrentUser,
    db: DbSession,
    limit: int = Query(20, ge=1, le=100),
    offset: int = Query(0, ge=0),
    unread_only: bool = Query(False),
) -> NotificationListResponse:
    """Get paginated notifications."""
    notifications, unread_count, total = await notification_service.get_notifications(
        db, current_user.id, limit, offset, unread_only
    )
    return NotificationListResponse(
        notifications=[NotificationResponse.model_validate(n) for n in notifications],
        unread_count=unread_count,
        total=total,
    )


@router.patch("/{notification_id}/read", response_model=NotificationMarkReadResponse)
async def mark_notification_read(
    notification_id: UUID,
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationMarkReadResponse:
    """Mark notification as read."""
    notification = await notification_service.mark_as_read(db, notification_id, current_user.id)
    if not notification:
        raise HTTPException(status_code=status.HTTP_404_NOT_FOUND, detail="Notification not found")
    return NotificationMarkReadResponse(id=notification.id, is_read=True)


@router.post("/mark-all-read", response_model=NotificationMarkAllReadResponse)
async def mark_all_notifications_read(
    current_user: CurrentUser,
    db: DbSession,
) -> NotificationMarkAllReadResponse:
    """Mark all notifications as read."""
    count = await notification_service.mark_all_as_read(db, current_user.id)
    return NotificationMarkAllReadResponse(marked_count=count)


@router.get("/unread-count", response_model=UnreadCountResponse)
async def get_unread_count(
    current_user: CurrentUser,
    db: DbSession,
) -> UnreadCountResponse:
    """Get unread notification count."""
    count = await notification_service.get_unread_count(db, current_user.id)
    return UnreadCountResponse(unread_count=count)
```

**Register Router in main.py**:
```python
from app.api.routes import notifications
app.include_router(notifications.router, prefix="/api")
```

---

## Step 5: Frontend Types & API

```typescript
// src/types/notification.ts
export type NotificationType = 'task_due' | 'task_overdue' | 'task_completed' | 'welcome' | 'system';

export interface Notification {
  id: string;
  type: NotificationType;
  title: string;
  message: string;
  is_read: boolean;
  action_url: string | null;
  created_at: string;
}

export interface NotificationListResponse {
  notifications: Notification[];
  unread_count: number;
  total: number;
}
```

```typescript
// src/services/notificationApi.ts
import { api, TAG_TYPES } from './api';
import type { NotificationListResponse, UnreadCountResponse } from '@/types/notification';

export const notificationApi = api.injectEndpoints({
  endpoints: (builder) => ({
    getNotifications: builder.query<NotificationListResponse, { limit?: number; offset?: number }>({
      query: ({ limit = 10, offset = 0 }) => `/notifications?limit=${limit}&offset=${offset}`,
      providesTags: ['Notification'],
    }),

    getUnreadCount: builder.query<UnreadCountResponse, void>({
      query: () => '/notifications/unread-count',
      providesTags: ['Notification'],
    }),

    markAsRead: builder.mutation<{ id: string; is_read: boolean }, string>({
      query: (id) => ({ url: `/notifications/${id}/read`, method: 'PATCH' }),
      // Optimistic update
      async onQueryStarted(id, { dispatch, queryFulfilled }) {
        const patchResult = dispatch(
          notificationApi.util.updateQueryData('getNotifications', { limit: 10, offset: 0 }, (draft) => {
            const notification = draft.notifications.find(n => n.id === id);
            if (notification && !notification.is_read) {
              notification.is_read = true;
              draft.unread_count = Math.max(0, draft.unread_count - 1);
            }
          })
        );
        try {
          await queryFulfilled;
        } catch {
          patchResult.undo();
        }
      },
    }),

    markAllAsRead: builder.mutation<{ marked_count: number }, void>({
      query: () => ({ url: '/notifications/mark-all-read', method: 'POST' }),
      invalidatesTags: ['Notification'],
    }),
  }),
});

export const {
  useGetNotificationsQuery,
  useGetUnreadCountQuery,
  useMarkAsReadMutation,
  useMarkAllAsReadMutation,
} = notificationApi;
```

---

## Step 6-8: See Component Implementation in plan.md

---

## Verification Checklist

- [ ] Migration creates notifications table with indexes
- [ ] GET /notifications returns paginated list
- [ ] PATCH /notifications/{id}/read marks as read
- [ ] POST /notifications/mark-all-read clears all
- [ ] GET /notifications/unread-count returns count
- [ ] Bell shows badge with unread count
- [ ] Dropdown opens with notification list
- [ ] Click marks as read (optimistic)
- [ ] Mark all clears badge
- [ ] Toast shows on errors

---

## Quick Commands

```bash
# Backend - run migration
cd phase2/backend && uv run alembic upgrade head

# Backend - test endpoints
curl -X GET http://localhost:8000/api/notifications \
  -H "Authorization: Bearer $TOKEN"

curl -X PATCH http://localhost:8000/api/notifications/$ID/read \
  -H "Authorization: Bearer $TOKEN"

# Frontend - add Notification tag to api.ts
# Add 'Notification' to TAG_TYPES
```
