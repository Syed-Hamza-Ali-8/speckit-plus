# Tasks — Notifications System

**Date**: 2025-12-15
**Feature**: notifications-system
**Plan**: [plan.md](./plan.md)
**Spec**: [spec.md](./spec.md)

## Task Summary

| ID | Task | Est. | Status |
|----|------|------|--------|
| T001 | Backend: Model + Migration + Schemas | 3min | ✅ |
| T002 | Backend: Service Layer + Endpoints | 3min | ✅ |
| T003 | Frontend: Types + RTK Query API | 2min | ✅ |
| T004 | Frontend: NotificationBell Component | 2min | ✅ |
| T005 | Frontend: Dropdown + Item Components | 3min | ✅ |
| T006 | Integration: Header + Welcome Notification | 2min | ✅ |

**Total**: 6 tasks (~15min) - **ALL COMPLETE**

---

## Phase 1: Backend Foundation

**Purpose**: Create the notification data layer and API

- [x] T001 Create Notification model in `phase2/backend/app/models/notification.py`, migration in `phase2/backend/alembic/versions/20251215_000003_create_notifications.py`, and schemas in `phase2/backend/app/schemas/notification.py`

- [x] T002 Implement notification service in `phase2/backend/app/services/notification_service.py`, endpoints in `phase2/backend/app/api/routes/notifications.py`, and register router in `phase2/backend/app/main.py`

**Checkpoint**: Backend ready - test with curl before proceeding to frontend

---

## Phase 2: Frontend Integration

**Purpose**: Add TypeScript types, RTK Query hooks, and UI components

- [x] T003 [P] Create frontend types in `phase2/frontend/src/types/notification.ts`, RTK Query API in `phase2/frontend/src/services/notificationApi.ts`, and add 'Notification' tag to `phase2/frontend/src/services/api.ts`

- [x] T004 [US1,US2] Create NotificationBell component with badge in `phase2/frontend/src/components/notifications/NotificationBell.tsx`

- [x] T005 [US2,US3,US4] Create NotificationDropdown in `phase2/frontend/src/components/notifications/NotificationDropdown.tsx`, NotificationItem in `phase2/frontend/src/components/notifications/NotificationItem.tsx`, and barrel export in `phase2/frontend/src/components/notifications/index.ts`

- [x] T006 [US5] Integrate NotificationBell into `phase2/frontend/src/components/layout/Header.tsx` and add welcome notification in `phase2/backend/app/services/auth_service.py`

**Checkpoint**: Full E2E notification flow works

---

## Task Details

### T001: Backend Model + Migration + Schemas

**Goal**: Create Notification model, migration, and Pydantic schemas

**Files**:
| File | Action |
|------|--------|
| `phase2/backend/app/models/notification.py` | CREATE |
| `phase2/backend/alembic/versions/20251215_000003_create_notifications.py` | CREATE |
| `phase2/backend/app/schemas/notification.py` | CREATE |

**Model Fields**:
```python
class Notification(SQLModel, table=True):
    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID = Field(foreign_key="users.id", nullable=False, index=True)
    type: str = Field(max_length=50, nullable=False)  # task_due, task_overdue, task_completed, welcome, system
    title: str = Field(max_length=100, nullable=False)
    message: str = Field(max_length=500, nullable=False)
    is_read: bool = Field(default=False, nullable=False)
    action_url: str | None = Field(default=None, max_length=255, nullable=True)
    created_at: datetime = Field(default_factory=..., sa_column=Column(DateTime(timezone=True), ...))
```

**Schemas**:
- `NotificationResponse`: Single notification
- `NotificationListResponse`: Paginated list with `notifications`, `unread_count`, `total`
- `NotificationMarkReadResponse`: `id`, `is_read`
- `NotificationMarkAllReadResponse`: `marked_count`
- `UnreadCountResponse`: `unread_count`

**Acceptance**: Model, migration, and schemas ready

---

### T002: Backend Service Layer + Endpoints

**Goal**: Implement notification service functions and REST endpoints

**Files**:
| File | Action |
|------|--------|
| `phase2/backend/app/services/notification_service.py` | CREATE |
| `phase2/backend/app/api/routes/notifications.py` | CREATE |
| `phase2/backend/app/main.py` | MODIFY (register router) |

**Service Functions**:
```python
async def get_notifications(db, user_id, limit=20, offset=0, unread_only=False) -> tuple[list, int, int]
async def get_unread_count(db, user_id) -> int
async def mark_as_read(db, notification_id, user_id) -> Notification | None
async def mark_all_as_read(db, user_id) -> int
async def create_notification(db, user_id, type, title, message, action_url=None) -> Notification
```

**Endpoints**:
- `GET /notifications` - Paginated list (limit, offset, unread_only)
- `PATCH /notifications/{id}/read` - Mark single as read
- `POST /notifications/mark-all-read` - Mark all as read
- `GET /notifications/unread-count` - Lightweight count

**Acceptance**: All 4 endpoints functional with proper auth

---

### T003: Frontend Types + RTK Query API

**Goal**: Add TypeScript types and RTK Query endpoints with optimistic updates

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/types/notification.ts` | CREATE |
| `phase2/frontend/src/services/notificationApi.ts` | CREATE |
| `phase2/frontend/src/services/api.ts` | MODIFY (add Notification tag) |

**Types**:
```typescript
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
```

**RTK Query Endpoints**:
```typescript
useGetNotificationsQuery({ limit, offset })
useGetUnreadCountQuery()
useMarkAsReadMutation()      // With optimistic update + rollback
useMarkAllAsReadMutation()
```

**Acceptance**: Types and API hooks ready for components

---

### T004: NotificationBell Component

**Goal**: Create bell icon with badge that shows unread count

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/components/notifications/NotificationBell.tsx` | CREATE |

**Features**:
- Bell icon (Lucide `Bell`)
- Red badge with unread count
- Badge shows "9+" for counts > 9
- Badge hidden when count = 0
- Click toggles dropdown state
- Uses `useGetUnreadCountQuery()` for count
- Glass UI styling matching Header

**Acceptance**: Bell component with working badge

---

### T005: Dropdown + Item Components

**Goal**: Create notification dropdown panel and item components

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/components/notifications/NotificationDropdown.tsx` | CREATE |
| `phase2/frontend/src/components/notifications/NotificationItem.tsx` | CREATE |
| `phase2/frontend/src/components/notifications/index.ts` | CREATE |

**NotificationDropdown Features**:
- Header: "Notifications" title + "Mark all as read" button
- Scrollable list (max-h with overflow-y-auto)
- Uses `useGetNotificationsQuery({ limit: 10 })`
- Empty state: "No notifications yet - You're all caught up!"
- Click outside closes (passed via `onClose` prop)
- Glass UI styling

**NotificationItem Features**:
- Icon based on type (Clock, AlertTriangle, CheckCircle, Star, Info)
- Title and message
- Relative timestamp using `formatDistanceToNow` from date-fns
- Unread indicator (purple dot)
- Click calls `useMarkAsReadMutation()` + navigates if action_url
- Read items appear dimmed (opacity-60)

**Acceptance**: Dropdown and Item components functional

---

### T006: Integration - Header + Welcome Notification

**Goal**: Wire NotificationBell into Header and add welcome notification on registration

**Files**:
| File | Action |
|------|--------|
| `phase2/frontend/src/components/layout/Header.tsx` | MODIFY |
| `phase2/backend/app/services/auth_service.py` | MODIFY |

**Header Changes**:
- Import `NotificationBell` from `@/components/notifications`
- Replace static bell button with `<NotificationBell />`
- Keep existing styling/position

**Auth Service Changes**:
```python
# In register_user(), after user creation:
from app.services import notification_service

await notification_service.create_notification(
    db,
    user.id,
    "welcome",
    "Welcome to TaskFlow!",
    "Get started by creating your first task.",
    "/tasks"
)
```

**Acceptance**: Full integration complete, welcome notification working

---

## Dependencies & Execution Order

```
T001 → T002 (Backend - sequential)
         ↓
T003 → T004 → T005 → T006 (Frontend - sequential)
```

### Task Dependencies

- **T001**: No dependencies - start immediately
- **T002**: Depends on T001 (model/schemas must exist)
- **T003**: Depends on T002 (backend must be ready)
- **T004**: Depends on T003 (types/API needed)
- **T005**: Depends on T004 (uses NotificationBell state)
- **T006**: Depends on T005 (needs all components)

### Parallel Opportunities

```bash
# T003 is marked [P] - can start as soon as T002 backend tests pass
# Within T005, Dropdown and Item can be built together
```

---

## User Story Coverage

| Task | US1 (Count) | US2 (List) | US3 (Mark Read) | US4 (Mark All) | US5 (Toast) |
|------|-------------|------------|-----------------|----------------|-------------|
| T001 | ✅ | ✅ | ✅ | ✅ | - |
| T002 | ✅ | ✅ | ✅ | ✅ | - |
| T003 | ✅ | ✅ | ✅ | ✅ | ✅ |
| T004 | ✅ | ✅ | - | - | - |
| T005 | - | ✅ | ✅ | ✅ | ✅ |
| T006 | ✅ | ✅ | - | - | ✅ |

---

## Quick Commands

```bash
# Backend - run migration
cd phase2/backend && uv run alembic upgrade head

# Backend - test endpoints
curl -X GET http://localhost:8000/api/notifications \
  -H "Authorization: Bearer $TOKEN"

# Frontend - add date-fns if needed
cd phase2/frontend && npm install date-fns

# Frontend - type check
cd phase2/frontend && npm run type-check
```

---

## Notes

- Tests not included per request (6 atomic tasks only)
- All tasks follow checklist format: `- [ ] [ID] [P?] [Story?] Description with file path`
- Glass UI styling from existing components
- Sonner toast library already available
- RTK Query refetchOnFocus for polling strategy
