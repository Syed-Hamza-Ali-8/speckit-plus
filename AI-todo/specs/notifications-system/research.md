# Research — Notifications System

**Date**: 2025-12-15
**Feature**: notifications-system

## Research Summary

This document captures technical decisions made during the planning phase for the Notifications System feature.

---

## Decision 1: Notification Storage Strategy

**Question**: How should notifications be stored and queried efficiently?

**Decision**: Single `notifications` table with composite indexes

**Rationale**:
- Simple single-table design fits notification model
- Composite index on `(user_id, created_at DESC)` for paginated queries
- Composite index on `(user_id, is_read)` for unread count
- SQLModel/SQLAlchemy handles async queries efficiently

**Alternatives Considered**:
| Option | Pros | Cons |
|--------|------|------|
| Single table (chosen) | Simple, efficient for reads | All in one table |
| Separate unread table | Fast unread queries | Sync complexity |
| Redis cache | Very fast reads | Extra infrastructure |

---

## Decision 2: Optimistic Update Pattern

**Question**: How should RTK Query handle optimistic updates for mark-as-read?

**Decision**: Use `onQueryStarted` with manual cache updates and rollback

**Rationale**:
- RTK Query's `onQueryStarted` provides clean pattern for optimistic updates
- `patchResult.undo()` provides automatic rollback on error
- Immer-style draft mutations are safe and performant
- Matches existing patterns in settingsApi.ts

**Implementation Pattern**:
```typescript
async onQueryStarted(id, { dispatch, queryFulfilled }) {
  const patchResult = dispatch(
    notificationApi.util.updateQueryData('getNotifications', undefined, (draft) => {
      const notification = draft.notifications.find(n => n.id === id);
      if (notification) notification.is_read = true;
      draft.unread_count = Math.max(0, draft.unread_count - 1);
    })
  );
  try {
    await queryFulfilled;
  } catch {
    patchResult.undo();
    toast.error('Failed to mark as read');
  }
}
```

---

## Decision 3: Notification Type as Enum vs String

**Question**: Should notification type be a database enum or string field?

**Decision**: String field with validation at schema level

**Rationale**:
- PostgreSQL enum changes require migrations
- String field is more flexible for adding new types
- Pydantic/Zod validation enforces allowed values at API boundary
- Matches existing pattern for task status

**Allowed Types**: `task_due`, `task_overdue`, `task_completed`, `welcome`, `system`

---

## Decision 4: Polling vs WebSocket for Real-Time Updates

**Question**: How should the frontend receive new notifications?

**Decision**: Polling with `refetchOnFocus` and manual refresh (v1)

**Rationale**:
- Simpler implementation for v1
- RTK Query provides `refetchOnFocus` for window focus events
- Manual refresh on relevant actions (task completion, etc.)
- WebSocket can be added in future iteration

**Polling Strategy**:
- Unread count: Refetch on window focus
- Full list: Refetch when dropdown opens
- Manual: Refetch after creating/completing tasks

**Future Enhancement**: WebSocket for Phase III with real-time push

---

## Decision 5: Notification Cleanup Strategy

**Question**: How should old notifications be cleaned up?

**Decision**: Application-level cleanup on create (limit 100 per user)

**Rationale**:
- Simple implementation without cron jobs
- Triggered when creating new notification
- Query deletes oldest if count exceeds 100
- 30-day expiry handled by background job (future)

**Implementation**:
```python
async def create_notification(...):
    # Create notification
    notification = Notification(...)
    db.add(notification)

    # Cleanup old notifications (keep latest 100)
    count = await db.exec(
        select(func.count()).where(Notification.user_id == user_id)
    )
    if count.first() > 100:
        oldest = await db.exec(
            select(Notification)
            .where(Notification.user_id == user_id)
            .order_by(Notification.created_at.asc())
            .limit(count.first() - 100)
        )
        for old in oldest:
            await db.delete(old)
```

---

## Decision 6: Badge Count Endpoint Separation

**Question**: Should unread count be a separate endpoint or part of list response?

**Decision**: Both - separate lightweight endpoint + count in list response

**Rationale**:
- Separate `/notifications/unread-count` is lightweight for polling
- List response includes count for convenience
- Badge can poll count endpoint without loading full list
- Reduces bandwidth for badge-only updates

---

## Decision 7: Notification-Task Relationship

**Question**: How should task-related notifications link to tasks?

**Decision**: Use `action_url` field with task path (e.g., `/tasks?id=<uuid>`)

**Rationale**:
- Loose coupling - notification doesn't depend on task existing
- Works for deleted tasks (graceful handling)
- Flexible for future non-task notifications
- Frontend can parse URL and navigate appropriately

**Not Using**: Foreign key to tasks table (too rigid)

---

## Decision 8: Welcome Notification Trigger

**Question**: When should welcome notification be created?

**Decision**: In `register_user` service function after successful registration

**Rationale**:
- Ensures welcome notification for every new user
- Single point of creation in auth service
- Transactional with user creation

---

## Technical Stack Confirmation

| Component | Technology | Confirmed |
|-----------|------------|-----------|
| Backend Model | SQLModel | ✅ |
| Backend Schemas | Pydantic | ✅ |
| Backend Router | FastAPI | ✅ |
| Frontend State | RTK Query | ✅ |
| Frontend Types | TypeScript | ✅ |
| Frontend Validation | Zod (optional) | ✅ |
| Toasts | Sonner | ✅ |
| Styling | Tailwind + Glass UI | ✅ |

---

## Open Questions (Resolved)

| Question | Resolution |
|----------|------------|
| Real-time updates | Polling for v1, WebSocket in future |
| Notification limit | 100 per user, cleanup on create |
| Type field | String with schema validation |
| Task relationship | Loose coupling via action_url |
