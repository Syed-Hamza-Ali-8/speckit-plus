# Implementation Plan: Notifications System

**Branch**: `main` | **Date**: 2025-12-15 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/notifications-system/spec.md`

## Summary

Implement an in-app notifications system with bell icon badge, dropdown list, mark-as-read with optimistic updates, and toast integration. Uses existing RTK Query patterns with Glass UI components.

## Technical Context

**Language/Version**: Python 3.11+ (backend), TypeScript 5.x (frontend)
**Primary Dependencies**: FastAPI, SQLModel, React 18, RTK Query, Sonner
**Storage**: Neon DB (PostgreSQL via SQLModel)
**Testing**: pytest (backend), Vitest (frontend)
**Target Platform**: Web (responsive)
**Project Type**: Web application (frontend + backend)
**Performance Goals**: < 50ms unread count, < 200ms list load
**Constraints**: Max 100 notifications per user, polling (no WebSocket v1)
**Scale/Scope**: Single user notifications, 5 user stories

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Rule | Status | Notes |
|------|--------|-------|
| Spec exists in `/specs/` | ✅ PASS | `specs/notifications-system/spec.md` |
| Phase II technologies only | ✅ PASS | FastAPI, React, SQLModel, JWT |
| Clean Architecture | ✅ PASS | Service layer, schema separation |
| TypeScript strict mode | ✅ PASS | Existing frontend config |
| Type hints (Python) | ✅ PASS | All schemas typed |
| JWT authentication | ✅ PASS | Uses existing CurrentUser dependency |
| Minimum 80% coverage | ⚠️ CHECK | Tests defined in tasks |

## Project Structure

### Documentation (this feature)

```text
specs/notifications-system/
├── spec.md              # Feature requirements
├── plan.md              # This file
├── research.md          # Phase 0 research decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Implementation guide
├── contracts/           # API contracts
│   └── notifications-api.yaml
└── tasks.md             # Implementation tasks (via /sp.tasks)
```

### Source Code (repository root)

```text
phase2/
├── backend/
│   ├── app/
│   │   ├── api/routes/notifications.py   # ADD: Notification endpoints
│   │   ├── schemas/notification.py       # ADD: Pydantic schemas
│   │   ├── services/notification_service.py # ADD: Business logic
│   │   ├── services/auth_service.py      # MODIFY: Welcome notification on register
│   │   └── models/notification.py        # ADD: Notification SQLModel
│   ├── alembic/versions/                 # ADD: Migration for notifications
│   └── tests/
│       └── unit/test_notifications.py    # ADD: Notification tests
│
└── frontend/
    ├── src/
    │   ├── types/notification.ts              # ADD: Notification types
    │   ├── services/notificationApi.ts        # ADD: RTK Query endpoints
    │   ├── services/api.ts                    # MODIFY: Add Notification tag
    │   ├── components/notifications/          # ADD: Notification components
    │   │   ├── NotificationBell.tsx
    │   │   ├── NotificationDropdown.tsx
    │   │   └── NotificationItem.tsx
    │   └── components/layout/Header.tsx       # MODIFY: Use NotificationBell
    └── tests/
        └── NotificationBell.test.tsx          # ADD: Component tests
```

**Structure Decision**: Follows existing patterns from settings feature. Notification components in dedicated folder.

---

## Implementation Steps (8-Step Roadmap)

### Step 1: Backend Model & Migration

**Goal**: Create Notification model and database migration

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/models/notification.py` | CREATE |
| `phase2/backend/alembic/versions/20251215_000003_create_notifications.py` | CREATE |

**Model Fields**:
- `id`: UUID (PK)
- `user_id`: UUID (FK → users.id, CASCADE DELETE)
- `type`: VARCHAR(50) - task_due, task_overdue, task_completed, welcome, system
- `title`: VARCHAR(100)
- `message`: VARCHAR(500)
- `is_read`: BOOLEAN (default false)
- `action_url`: VARCHAR(255) (nullable)
- `created_at`: TIMESTAMP

**Indexes**:
- `(user_id, created_at DESC)` - Paginated queries
- `(user_id, is_read)` - Unread count

**Acceptance Criteria**:
- [ ] Notification model created with all fields
- [ ] Migration runs successfully
- [ ] Indexes created for performance

---

### Step 2: Backend Schemas

**Goal**: Add Pydantic schemas for notification operations

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/schemas/notification.py` | CREATE |

**Schemas**:
- `NotificationResponse`: Single notification
- `NotificationListResponse`: Paginated list with counts
- `NotificationMarkReadResponse`: Mark-as-read response
- `NotificationMarkAllReadResponse`: Bulk mark response
- `UnreadCountResponse`: Lightweight count
- `NotificationCreate`: Internal creation schema

**Acceptance Criteria**:
- [ ] All schemas defined with proper validation
- [ ] Type field validates against allowed values
- [ ] Response schemas use `from_attributes=True`

---

### Step 3: Backend Service Layer

**Goal**: Implement notification business logic

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/services/notification_service.py` | CREATE |

**Functions**:
```python
async def get_notifications(db, user_id, limit, offset, unread_only) -> tuple[list, int, int]
async def get_unread_count(db, user_id) -> int
async def mark_as_read(db, notification_id, user_id) -> Notification | None
async def mark_all_as_read(db, user_id) -> int
async def create_notification(db, user_id, type, title, message, action_url) -> Notification
```

**Cleanup Logic**: Delete oldest when count exceeds 100 per user

**Acceptance Criteria**:
- [ ] All service functions implemented
- [ ] Pagination works correctly
- [ ] Cleanup triggered on create
- [ ] User can only access own notifications

---

### Step 4: Backend Endpoints

**Goal**: Implement REST endpoints for notifications

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/backend/app/api/routes/notifications.py` | CREATE |
| `phase2/backend/app/main.py` | MODIFY: Register router |

**Endpoints**:
- `GET /notifications` - Paginated list
- `PATCH /notifications/{id}/read` - Mark single as read
- `POST /notifications/mark-all-read` - Mark all as read
- `GET /notifications/unread-count` - Lightweight count

**Acceptance Criteria**:
- [ ] All endpoints return correct responses
- [ ] 404 for non-existent notification
- [ ] 403 for accessing other user's notification
- [ ] Query params validated (limit 1-100, offset >= 0)

---

### Step 5: Frontend Types & API Layer

**Goal**: Add TypeScript types and RTK Query endpoints

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/types/notification.ts` | CREATE |
| `phase2/frontend/src/services/notificationApi.ts` | CREATE |
| `phase2/frontend/src/services/api.ts` | MODIFY: Add Notification tag |

**RTK Query Endpoints**:
```typescript
useGetNotificationsQuery({ limit, offset })
useGetUnreadCountQuery()
useMarkAsReadMutation()      // With optimistic update
useMarkAllAsReadMutation()
```

**Optimistic Update**: Mark as read updates cache immediately, rolls back on error

**Acceptance Criteria**:
- [ ] Types match backend schemas
- [ ] RTK hooks export correctly
- [ ] Optimistic update works with rollback
- [ ] Notification tag added for cache invalidation

---

### Step 6: NotificationBell Component

**Goal**: Create bell icon with badge in header

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/notifications/NotificationBell.tsx` | CREATE |
| `phase2/frontend/src/components/layout/Header.tsx` | MODIFY: Replace static bell |

**Features**:
- Bell icon (Lucide `Bell`)
- Red badge with unread count
- Badge shows "9+" for counts > 9
- Badge hidden when count = 0
- Click toggles dropdown
- Fetches unread count on mount

**Acceptance Criteria**:
- [ ] Badge displays correct count
- [ ] Badge capped at "9+"
- [ ] Badge hidden when 0
- [ ] Click opens dropdown
- [ ] Matches existing header styling

---

### Step 7: NotificationDropdown & NotificationItem

**Goal**: Create notification list dropdown and item components

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/notifications/NotificationDropdown.tsx` | CREATE |
| `phase2/frontend/src/components/notifications/NotificationItem.tsx` | CREATE |
| `phase2/frontend/src/components/notifications/index.ts` | CREATE |

**Dropdown Features**:
- Header with "Notifications" title and "Mark all as read" button
- Scrollable notification list (max 10 shown)
- Empty state when no notifications
- Click outside closes dropdown
- Glass UI styling

**Item Features**:
- Icon based on notification type
- Title and message
- Relative timestamp ("2 hours ago")
- Unread indicator (dot)
- Click marks as read
- Navigate to action_url if present

**Acceptance Criteria**:
- [ ] Dropdown opens below bell
- [ ] Max 10 notifications shown
- [ ] Empty state displays correctly
- [ ] Mark all as read clears badge
- [ ] Click item marks as read
- [ ] Relative timestamps display correctly

---

### Step 8: Integration & Toast Sync

**Goal**: Wire everything together and add toast notifications

**Files Changed**:
| File | Change |
|------|--------|
| `phase2/frontend/src/components/notifications/NotificationBell.tsx` | MODIFY: Polish |
| `phase2/backend/app/services/auth_service.py` | MODIFY: Welcome notification |

**Integration Tasks**:
1. Replace static bell in Header with NotificationBell
2. Add welcome notification on user registration
3. Add toast on mark-as-read error
4. Add toast on mark-all success
5. Refetch on window focus (RTK Query)

**Welcome Notification**:
```python
# In register_user after user creation
await notification_service.create_notification(
    db, user.id, "welcome", "Welcome to TaskGPT!",
    "Get started by creating your first task.", "/tasks"
)
```

**Acceptance Criteria**:
- [ ] Header uses NotificationBell
- [ ] New users get welcome notification
- [ ] Toast shows on errors
- [ ] Refetch on window focus

---

## Dependency Graph

```
Step 1 ──► Step 2 ──► Step 3 ──► Step 4 (Backend)
                                    ↓
                              ┌─────┴─────┐
                              ↓           ↓
                          Step 5       (Backend Ready)
                              ↓
                          Step 6
                              ↓
                          Step 7
                              ↓
                          Step 8
```

### Parallel Opportunities

```bash
# After Step 4 completes, frontend work can begin
# Step 5-8 are sequential (frontend)
# Within Step 7, Dropdown and Item can be built in parallel
```

---

## Risk Analysis

| Risk | Mitigation |
|------|------------|
| Unread count query slow | Composite index on (user_id, is_read) |
| Optimistic update race | RTK Query handles with patchResult.undo() |
| Too many notifications | Cleanup on create (max 100) |
| Badge overflow | Cap display at "9+" |

## Complexity Tracking

No constitution violations requiring justification.

---

## Next Steps

1. Run `/sp.tasks` to generate detailed task file
2. Implement Step 1-4 (backend) first
3. Run migration
4. Implement Step 5-8 (frontend)
5. Run `/sp.analyze` for cross-artifact consistency check
