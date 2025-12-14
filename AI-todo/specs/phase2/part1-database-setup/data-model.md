# Data Model: Phase 2 Part 1 - Database Setup

**Feature**: Database Foundation for Multi-User Todo Application
**Date**: 2025-12-13
**Status**: Approved

---

## Entity: Task

### Overview

The Task entity represents a single todo item owned by a user. It supports status tracking with extensible enum values and automatic timestamp management.

### Schema Definition

```
┌─────────────────────────────────────────────────────────────┐
│                          tasks                               │
├─────────────────────────────────────────────────────────────┤
│ id           : UUID        PK, DEFAULT uuid_generate_v4()   │
│ user_id      : UUID        NOT NULL, INDEX                  │
│ title        : VARCHAR(200) NOT NULL, CHECK(not empty)      │
│ description  : VARCHAR(1000) NULL                           │
│ status       : VARCHAR(50)  NOT NULL, DEFAULT 'pending'     │
│ created_at   : TIMESTAMPTZ  NOT NULL, DEFAULT NOW()         │
│ updated_at   : TIMESTAMPTZ  NOT NULL, DEFAULT NOW()         │
├─────────────────────────────────────────────────────────────┤
│ INDEXES:                                                     │
│   - ix_tasks_user_id ON user_id                             │
│ CONSTRAINTS:                                                 │
│   - ck_tasks_title_not_empty: length(trim(title)) > 0       │
└─────────────────────────────────────────────────────────────┘
```

### Field Specifications

| Field | Type | Constraints | Default | Notes |
|-------|------|-------------|---------|-------|
| `id` | UUID | PRIMARY KEY | `uuid4()` | Server-generated, immutable |
| `user_id` | UUID | NOT NULL, INDEXED | - | FK deferred to Part 2 |
| `title` | VARCHAR(200) | NOT NULL, CHECK | - | Must be non-empty after trim |
| `description` | VARCHAR(1000) | NULLABLE | `NULL` | Optional task details |
| `status` | VARCHAR(50) | NOT NULL | `'pending'` | Non-native enum for extensibility |
| `created_at` | TIMESTAMPTZ | NOT NULL | `NOW()` | UTC, immutable after create |
| `updated_at` | TIMESTAMPTZ | NOT NULL | `NOW()` | UTC, auto-updated on modify |

### Enum: TaskStatus

```python
class TaskStatus(str, Enum):
    PENDING = "pending"
    COMPLETED = "completed"
    # Extensible without migration:
    # IN_PROGRESS = "in_progress"
    # BLOCKED = "blocked"
    # ARCHIVED = "archived"
```

**Storage**: VARCHAR(50) with `native_enum=False`
**Validation**: Python Enum type enforcement + CHECK constraint

---

## Validation Rules

### Python Layer (Pydantic v2)

| Field | Validator | Error Message |
|-------|-----------|---------------|
| `title` | `@field_validator` | "Title cannot be empty or whitespace-only" |
| `title` | `max_length=200` | "String should have at most 200 characters" |
| `description` | `max_length=1000` | "String should have at most 1000 characters" |
| `status` | Enum type | "Input should be 'pending' or 'completed'" |

### Database Layer (CHECK Constraints)

| Constraint | SQL Expression | Purpose |
|------------|----------------|---------|
| `ck_tasks_title_not_empty` | `length(trim(title)) > 0` | Prevent empty/whitespace titles |

---

## Relationships

### Part 1 (Current)
- `user_id` is indexed but has NO foreign key constraint
- FK will be added in Part 2 when User table exists

### Part 2 (Future)
```
tasks.user_id → users.id (CASCADE DELETE)
```

---

## State Transitions

```
                    ┌───────────┐
     create task    │           │
    ─────────────►  │  PENDING  │
                    │           │
                    └─────┬─────┘
                          │
                          │ mark_complete()
                          ▼
                    ┌───────────┐
                    │           │
                    │ COMPLETED │
                    │           │
                    └─────┬─────┘
                          │
                          │ mark_incomplete()
                          ▼
                    ┌───────────┐
                    │           │
                    │  PENDING  │
                    │           │
                    └───────────┘
```

**Notes**:
- Status transitions are bidirectional (can undo completion)
- No intermediate states in Part 1 (extensible for future)
- All transitions update `updated_at` timestamp

---

## Timestamp Behavior

### created_at
- Set automatically on INSERT
- Never modified after creation
- Stored in UTC (TIMESTAMPTZ)

### updated_at
- Set automatically on INSERT
- Updated automatically on every UPDATE
- Stored in UTC (TIMESTAMPTZ)

### Implementation
```python
from datetime import datetime, timezone

# Default factory for both fields
default_factory=lambda: datetime.now(timezone.utc)

# Update trigger pattern (in service layer)
task.updated_at = datetime.now(timezone.utc)
```

---

## Index Strategy

| Index | Column(s) | Type | Rationale |
|-------|-----------|------|-----------|
| `pk_tasks` | `id` | PRIMARY KEY | Unique identifier |
| `ix_tasks_user_id` | `user_id` | B-TREE | Filter tasks by owner (all queries) |

**Future Indexes** (not in Part 1):
- `ix_tasks_status` - If filtering by status becomes common
- `ix_tasks_created_at` - If sorting by date becomes common
- Composite `(user_id, status)` - If filtered status lists are common

---

## Migration Notes

### Initial Migration (Part 1)
1. Create `tasks` table with all columns
2. Create `ix_tasks_user_id` index
3. Create `ck_tasks_title_not_empty` CHECK constraint

### Downgrade
1. Drop `tasks` table (CASCADE handles index and constraint)

### Naming Convention
```python
NAMING_CONVENTION = {
    "ix": "ix_%(column_0_label)s",
    "uq": "uq_%(table_name)s_%(column_0_name)s",
    "ck": "ck_%(table_name)s_%(constraint_name)s",
    "fk": "fk_%(table_name)s_%(column_0_name)s_%(referred_table_name)s",
    "pk": "pk_%(table_name)s",
}
```
