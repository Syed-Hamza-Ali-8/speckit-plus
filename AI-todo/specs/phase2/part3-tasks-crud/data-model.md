# Data Model — Phase 2 Part 3: Task CRUD API

> **Created:** 2025-12-14
> **Status:** Complete

---

## Existing Entity: Task

The Task entity was defined in Part 1 (Database Setup) and extended in Part 2 (Authentication) with user ownership.

### SQLModel Definition

**Location:** `phase2/backend/app/models/task.py`

```python
from datetime import datetime
from enum import Enum
from uuid import UUID, uuid4

from sqlmodel import Field, SQLModel


class TaskStatus(str, Enum):
    """Task completion status."""
    PENDING = "pending"
    COMPLETED = "completed"


class Task(SQLModel, table=True):
    """Task entity with user ownership."""
    __tablename__ = "tasks"

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID = Field(foreign_key="users.id", index=True)
    title: str = Field(max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    status: TaskStatus = Field(default=TaskStatus.PENDING)
    created_at: datetime = Field(default_factory=datetime.utcnow)
    updated_at: datetime = Field(default_factory=datetime.utcnow)
```

### Field Specifications

| Field | Type | Constraints | Index | Description |
|-------|------|-------------|-------|-------------|
| `id` | UUID | PK, auto-generated | Primary | Unique task identifier |
| `user_id` | UUID | FK → users.id, NOT NULL | Yes | Task owner |
| `title` | VARCHAR(200) | NOT NULL, min 1 char | No | Task title |
| `description` | VARCHAR(1000) | NULL allowed | No | Optional description |
| `status` | ENUM | pending/completed | No | Completion status |
| `created_at` | TIMESTAMP | NOT NULL, auto | No | Creation timestamp |
| `updated_at` | TIMESTAMP | NOT NULL, auto | No | Last modification |

### Relationships

```
User (1) ───────< Task (N)
     └─ users.id = tasks.user_id
```

---

## Request Schemas

### TaskCreate (Existing)

**Location:** `phase2/backend/app/schemas/task.py`

```python
class TaskCreate(BaseModel):
    """Schema for task creation request."""
    title: str = Field(min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
```

### TaskUpdate (Existing)

```python
class TaskUpdate(BaseModel):
    """Schema for partial task update."""
    title: str | None = Field(default=None, min_length=1, max_length=200)
    description: str | None = Field(default=None, max_length=1000)
    status: TaskStatus | None = Field(default=None)
    # user_id: NOT included - immutable
```

### TaskQueryParams (New)

```python
from datetime import date

class TaskQueryParams(BaseModel):
    """Query parameters for listing tasks."""
    status: TaskStatus | None = None
    created_after: date | None = None
    created_before: date | None = None
    sort: str = "created_at:desc"
    limit: int = Field(default=20, ge=1, le=100)
    offset: int = Field(default=0, ge=0)
```

---

## Response Schemas

### TaskRead (New - replaces TaskResponse for list)

**Purpose:** Hide `user_id` from API responses

```python
class TaskRead(BaseModel):
    """Response schema for a single task (user_id hidden)."""
    id: UUID
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime
    # user_id: EXCLUDED for security

    model_config = {"from_attributes": True}
```

### PaginatedTaskResponse (New)

**Purpose:** Paginated list response with metadata

```python
class PaginatedTaskResponse(BaseModel):
    """Paginated response for task listing."""
    items: list[TaskRead]
    total: int    # Total count (before pagination)
    limit: int    # Page size
    offset: int   # Current offset
```

### TaskResponse (Existing - keep for single item)

```python
class TaskResponse(BaseModel):
    """Full task response (for POST/PATCH single item)."""
    id: UUID
    user_id: UUID  # Included for single-item responses
    title: str
    description: str | None
    status: TaskStatus
    created_at: datetime
    updated_at: datetime

    model_config = {"from_attributes": True}
```

**Note:** Consider deprecating `TaskResponse` in favor of `TaskRead` for consistency. Decision deferred to implementation.

---

## Schema Mapping

### Endpoint → Schema Mapping

| Endpoint | Request Schema | Response Schema |
|----------|---------------|-----------------|
| `GET /tasks` | Query params | `PaginatedTaskResponse` |
| `POST /tasks` | `TaskCreate` | `TaskRead` |
| `GET /tasks/{id}` | - | `TaskRead` |
| `PATCH /tasks/{id}` | `TaskUpdate` | `TaskRead` |
| `DELETE /tasks/{id}` | - | 204 No Content |

### Query Parameter Validation

| Parameter | Validation | Error Response |
|-----------|------------|----------------|
| `status` | Must be `pending` or `completed` | 422 Unprocessable Entity |
| `created_after` | ISO date format (YYYY-MM-DD) | 422 Unprocessable Entity |
| `created_before` | ISO date format (YYYY-MM-DD) | 422 Unprocessable Entity |
| `sort` | Format `field:direction`, valid field/direction | 400 Bad Request |
| `limit` | 1 ≤ value ≤ 100 | 422 Unprocessable Entity |
| `offset` | value ≥ 0 | 422 Unprocessable Entity |

---

## State Transitions

### Task Status

```
           create
              │
              ▼
        ┌──────────┐
        │ PENDING  │
        └────┬─────┘
             │ update(status=completed)
             ▼
        ┌──────────┐
        │COMPLETED │
        └────┬─────┘
             │ update(status=pending)
             ▼
        ┌──────────┐
        │ PENDING  │
        └──────────┘
```

**Rules:**
- Tasks are created with `PENDING` status
- Status can be toggled freely between `PENDING` and `COMPLETED`
- No validation on status transitions (user can change anytime)

---

## Database Indexes

### Existing Indexes
- `tasks_pkey` - Primary key on `id`
- `ix_tasks_user_id` - Index on `user_id` for ownership queries

### Recommended Indexes for Part 3

```sql
-- Composite index for filtering + sorting (optional, for performance)
CREATE INDEX ix_tasks_user_status_created
ON tasks (user_id, status, created_at DESC);
```

**Note:** Add only if query performance becomes an issue at scale.

---

## Validation Rules Summary

### Create
| Field | Rule | Error |
|-------|------|-------|
| `title` | Required, 1-200 chars | 400 validation error |
| `description` | Optional, max 1000 chars | 400 validation error |

### Update
| Field | Rule | Error |
|-------|------|-------|
| `title` | If provided, 1-200 chars | 400 validation error |
| `description` | If provided, max 1000 chars | 400 validation error |
| `status` | If provided, valid enum | 400 validation error |
| `user_id` | Not in schema - rejected | Ignored |

### Query
| Parameter | Rule | Error |
|-----------|------|-------|
| `status` | Valid enum if provided | 422 validation error |
| `created_after` | Valid ISO date if provided | 422 validation error |
| `created_before` | Valid ISO date if provided | 422 validation error |
| `sort` | Valid `field:direction` format | 400 bad request |
| `limit` | 1-100 | 422 validation error |
| `offset` | ≥ 0 | 422 validation error |
