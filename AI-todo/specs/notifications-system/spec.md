# Specification — Notifications System

**Feature**: notifications-system
**Date**: 2025-12-15
**Status**: Draft
**Estimated**: 10 minutes

## Feature Overview

Implement an in-app notifications system with real-time badge updates in the header, notification list with mark-as-read functionality, optimistic UI updates, and integration with the existing toast system.

## User Stories

### US-1: View Notification Count
**As a** logged-in user
**I want to** see an unread notification count badge on the bell icon
**So that** I know when I have new notifications without checking

**Acceptance Criteria:**
- [ ] Bell icon displays badge with unread count
- [ ] Badge only shows when count > 0
- [ ] Badge updates immediately when notifications are read
- [ ] Badge shows "9+" for counts greater than 9

### US-2: View Notifications List
**As a** logged-in user
**I want to** click the bell icon to see my notifications
**So that** I can review what happened in my account

**Acceptance Criteria:**
- [ ] Dropdown panel opens on bell click
- [ ] Shows list of notifications (newest first)
- [ ] Each notification shows: icon, message, timestamp, read status
- [ ] Empty state when no notifications
- [ ] Maximum 10 notifications shown in dropdown
- [ ] "View All" link if more than 10

### US-3: Mark Notification as Read
**As a** logged-in user
**I want to** mark individual notifications as read
**So that** I can track which notifications I've seen

**Acceptance Criteria:**
- [ ] Click notification marks it as read
- [ ] Optimistic update - UI updates immediately
- [ ] Badge count decreases instantly
- [ ] Read notifications appear dimmed/greyed
- [ ] Rollback on server error

### US-4: Mark All as Read
**As a** logged-in user
**I want to** mark all notifications as read at once
**So that** I can clear my notification queue quickly

**Acceptance Criteria:**
- [ ] "Mark all as read" button in dropdown header
- [ ] Clears badge count to 0
- [ ] All notifications marked as read
- [ ] Optimistic update with rollback on error

### US-5: Toast Integration
**As a** logged-in user
**I want to** see toast notifications for important events
**So that** I'm immediately aware of significant changes

**Acceptance Criteria:**
- [ ] New notifications trigger toast popup
- [ ] Toast shows notification message
- [ ] Clicking toast navigates to relevant content (if applicable)
- [ ] Toasts auto-dismiss after 5 seconds

## Functional Specifications

### 1. Notification Types

| Type | Icon | Description | Example |
|------|------|-------------|---------|
| `task_due` | Clock | Task approaching due date | "Task 'Project Report' is due tomorrow" |
| `task_overdue` | AlertTriangle | Task past due date | "Task 'Meeting Notes' is overdue" |
| `task_completed` | CheckCircle | Task marked complete | "You completed 'Weekly Review'" |
| `welcome` | Star | New user welcome | "Welcome to TaskGPT!" |
| `system` | Info | System announcements | "New features available" |

### 2. Notification Model

```typescript
interface Notification {
  id: string;                    // UUID
  user_id: string;               // Owner UUID
  type: NotificationType;        // task_due | task_overdue | task_completed | welcome | system
  title: string;                 // Short title (max 100 chars)
  message: string;               // Full message (max 500 chars)
  is_read: boolean;              // Read status
  action_url?: string;           // Optional deep link (e.g., /tasks/123)
  created_at: string;            // ISO timestamp
}
```

### 3. API Endpoints

#### GET /notifications
Fetch user's notifications (paginated).

**Query Parameters:**
| Param | Type | Default | Description |
|-------|------|---------|-------------|
| `limit` | int | 20 | Max notifications per page |
| `offset` | int | 0 | Pagination offset |
| `unread_only` | bool | false | Filter to unread only |

**Response:**
```json
{
  "notifications": [...],
  "unread_count": 5,
  "total": 42
}
```

#### PATCH /notifications/{id}/read
Mark single notification as read.

**Response:**
```json
{
  "id": "...",
  "is_read": true
}
```

#### POST /notifications/mark-all-read
Mark all notifications as read.

**Response:**
```json
{
  "marked_count": 5
}
```

#### GET /notifications/unread-count
Get unread notification count only (lightweight).

**Response:**
```json
{
  "unread_count": 5
}
```

### 4. Header Bell Component

**States:**
- Default: Bell icon, no badge
- Has unread: Bell icon + red badge with count
- Open: Dropdown with notification list

**Interactions:**
- Click bell → Toggle dropdown
- Click outside → Close dropdown
- Click notification → Mark as read + navigate (if action_url)
- Click "Mark all as read" → Clear all

### 5. Optimistic Updates

```typescript
// On mark as read
1. Immediately update local cache (is_read = true)
2. Decrement unread_count in cache
3. Send PATCH to server
4. On error: rollback cache + show error toast
```

### 6. Toast Integration

| Event | Toast Type | Duration |
|-------|------------|----------|
| New notification | info | 5s |
| Mark all read | success | 3s |
| Error marking read | error | 5s |

## Technical Specifications

### Backend

**New Files:**
| File | Description |
|------|-------------|
| `app/models/notification.py` | Notification SQLModel |
| `app/schemas/notification.py` | Pydantic schemas |
| `app/services/notification_service.py` | Business logic |
| `app/api/routes/notifications.py` | REST endpoints |

**Database Migration:**
- Create `notifications` table
- Index on `user_id` + `created_at` (for efficient queries)
- Index on `user_id` + `is_read` (for unread count)

### Frontend

**New Files:**
| File | Description |
|------|-------------|
| `src/types/notification.ts` | TypeScript types |
| `src/services/notificationApi.ts` | RTK Query endpoints |
| `src/components/notifications/NotificationBell.tsx` | Bell icon + badge |
| `src/components/notifications/NotificationDropdown.tsx` | Dropdown panel |
| `src/components/notifications/NotificationItem.tsx` | Single notification |

**Modified Files:**
| File | Changes |
|------|---------|
| `src/components/layout/Header.tsx` | Replace static bell with NotificationBell |
| `src/services/api.ts` | Add Notification tag type |

### State Management

```typescript
// RTK Query tags for cache invalidation
providesTags: ['Notification']
invalidatesTags: ['Notification']

// Optimistic update pattern
async onQueryStarted(id, { dispatch, queryFulfilled }) {
  const patchResult = dispatch(
    notificationApi.util.updateQueryData('getNotifications', undefined, (draft) => {
      const notification = draft.notifications.find(n => n.id === id);
      if (notification) notification.is_read = true;
      draft.unread_count--;
    })
  );
  try {
    await queryFulfilled;
  } catch {
    patchResult.undo();
  }
}
```

## UI/UX Specifications

### Bell Icon Badge

```
Normal:          With count:      Many:
  🔔                🔔               🔔
                   (3)             (9+)
```

### Notification Dropdown

```
┌─────────────────────────────────┐
│ Notifications    [Mark all read]│
├─────────────────────────────────┤
│ ⚠️ Task "Report" is overdue     │
│    2 hours ago              •   │ ← unread (dot indicator)
├─────────────────────────────────┤
│ ✓ You completed "Meeting"       │
│    Yesterday                    │ ← read (dimmed)
├─────────────────────────────────┤
│ 📋 Task "Review" is due tomorrow│
│    Just now                 •   │
├─────────────────────────────────┤
│         [View All →]            │
└─────────────────────────────────┘
```

### Empty State

```
┌─────────────────────────────────┐
│ Notifications                   │
├─────────────────────────────────┤
│                                 │
│     🔔 No notifications yet     │
│                                 │
│   You're all caught up!         │
│                                 │
└─────────────────────────────────┘
```

## Constraints

- Maximum 100 notifications stored per user (older ones auto-deleted)
- Notifications auto-expire after 30 days
- No real-time push (polling every 60s or on-demand refresh)
- Badge capped at "9+" for display

## Non-Functional Requirements

| Requirement | Target |
|-------------|--------|
| Unread count query | < 50ms |
| Notification list load | < 200ms |
| Optimistic update latency | Instant (< 16ms) |
| Toast display time | 3-5s |

## Security Considerations

- Users can only access their own notifications
- Notification content sanitized (no XSS)
- Rate limit on mark-as-read: 30/min
- No sensitive data in notification messages

## Out of Scope (v1)

- Real-time WebSocket push notifications
- Email notification delivery
- Notification preferences/settings
- Push notifications to mobile/desktop
- Notification categories/filters

## Dependencies

- Existing auth system (JWT + CurrentUser)
- Toast system (Sonner)
- RTK Query for data fetching
- shadcn/ui dropdown components

## Test Cases

### Backend Tests
- [ ] GET /notifications returns user's notifications only
- [ ] PATCH /notifications/{id}/read marks as read
- [ ] POST /notifications/mark-all-read marks all as read
- [ ] Cannot access other user's notifications (403)
- [ ] Pagination works correctly

### Frontend Tests
- [ ] Bell shows badge when unread_count > 0
- [ ] Dropdown opens/closes on click
- [ ] Mark as read updates UI optimistically
- [ ] Error shows rollback and toast
- [ ] Empty state displays correctly

## Implementation Notes

1. **Seeding**: Create welcome notification on user registration
2. **Task notifications**: Hook into task service to create due/overdue notifications
3. **Cleanup job**: Consider cron job to delete old notifications (future)
4. **Performance**: Use `unread_count` endpoint for badge (lighter than full list)

## References

- [RTK Query Optimistic Updates](https://redux-toolkit.js.org/rtk-query/usage/manual-cache-updates#optimistic-updates)
- [Sonner Toast Library](https://sonner.emilkowal.ski/)
- Existing patterns: `settingsApi.ts`, `Header.tsx`
