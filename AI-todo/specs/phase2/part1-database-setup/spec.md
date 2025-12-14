# Phase 2 Part 1: Database Setup Specification

**Feature**: Database Foundation for Multi-User Todo Application
**Phase**: 2 (Web Application)
**Part**: 1 of N
**Status**: Approved
**Created**: 2025-12-13
**Last Updated**: 2025-12-13

---

## 1. Intent

Establish a production-ready database foundation for the multi-user Todo web application using SQLModel with Neon Postgres. This spec defines the Task data model with multi-user ownership enforced from inception, ensuring all future Phase 2 parts (Authentication, API, Frontend) build on a solid, tested database layer.

### Business Context

- **Why Now**: Database schema must exist before API endpoints or authentication can be implemented
- **User Impact**: Enables task isolation per user; no user can access another user's tasks
- **System Impact**: Provides the persistence layer for all CRUD operations in Phase 2+

### Core Requirements

1. **Task Model**: Define SQLModel schema for tasks with UUID primary key, user ownership, status tracking, and automatic timestamps
2. **Database Connectivity**: Configure async connection to Neon Postgres with SSL and environment-based configuration
3. **Migration Infrastructure**: Set up Alembic for versioned, reversible schema migrations
4. **Test Infrastructure**: Enable isolated testing with SQLite (unit) and Neon (integration)
5. **Dual-Layer Validation**: Enforce constraints at both Python/SQLModel and database levels

---

## 2. Constraints

### Technology Stack (Phase 2 Mandate)

| Component | Technology | Version |
|-----------|------------|---------|
| ORM | SQLModel | Latest stable |
| Database | Neon Postgres | Serverless |
| Async Driver | asyncpg | Latest stable |
| Migrations | Alembic | Latest stable |
| Runtime | Python | 3.13+ |

### File Location Constraints

| Artifact | Path |
|----------|------|
| Models | `phase2/backend/app/models/` |
| Database Config | `phase2/backend/app/core/database.py` |
| Alembic Config | `phase2/backend/alembic/` |
| Tests | `phase2/backend/tests/` |

### Schema Constraints

| Field | Type | Constraints |
|-------|------|-------------|
| `id` | UUID | Primary key, server-generated default (uuid4) |
| `user_id` | UUID | NOT NULL, indexed (FK enforced in Part 2) |
| `title` | String(200) | NOT NULL, non-empty after trim, CHECK constraint |
| `description` | String(1000) | Nullable |
| `status` | Enum | Non-native enum for extensibility, default `pending` |
| `created_at` | DateTime(tz) | NOT NULL, server-generated UTC |
| `updated_at` | DateTime(tz) | NOT NULL, auto-updated on modification, UTC |

### Enum Design Constraint

- **TaskStatus** enum MUST use `native_enum=False` in SQLAlchemy column definition
- This allows adding new statuses (e.g., `in_progress`, `blocked`, `archived`) without schema migration
- Initial values: `pending`, `completed`

### Validation Constraints (Dual-Layer)

| Constraint | Python/SQLModel | Database |
|------------|-----------------|----------|
| Title non-empty | Pydantic validator | CHECK (title != '' AND title !~ '^\s*$') |
| Title max length | Field(max_length=200) | VARCHAR(200) |
| Description max length | Field(max_length=1000) | VARCHAR(1000) |
| Status valid | Enum type | CHECK constraint |

### Timestamp Constraints

- All timestamps stored in **UTC** (TIMESTAMPTZ)
- Millisecond precision (default DateTime behavior)
- Timezone conversion handled at API/frontend layer, not database

### Connection Constraints

- Async connections via `asyncpg`
- SSL mode: `require` (Neon mandate)
- Connection string from environment variable `DATABASE_URL`
- Test database from environment variable `TEST_DATABASE_URL`

### Testing Constraints

- **Unit tests**: SQLite in-memory (fast, isolated)
- **Integration tests**: Neon only (TEST_DATABASE_URL)
- **User ID in tests**: Random UUIDs via `uuid.uuid4()` (no placeholders)
- Local Postgres/Docker is out of scope for Part 1

### Migration Constraints

- All migrations MUST be reversible (upgrade + downgrade)
- Migration naming: `YYYYMMDD_HHMMSS_<description>.py`
- No seed data in migrations (test data in test fixtures only)

---

## 3. Success Evals

All criteria must pass for this part to be considered complete.

### 3.1 Model Definition (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| M1 | Task model exists at specified path | File exists at `phase2/backend/app/models/task.py` | Pass |
| M2 | Task model has all required fields | Schema inspection shows: id (UUID PK), user_id (UUID NOT NULL), title (VARCHAR 200 NOT NULL), description (VARCHAR 1000 NULL), status (VARCHAR/non-native enum), created_at (TIMESTAMPTZ NOT NULL), updated_at (TIMESTAMPTZ NOT NULL) | 7/7 fields |
| M3 | Status enum defined correctly | Enum contains exactly `pending`, `completed` with `native_enum=False` | Pass |
| M4 | user_id has database index | Index exists on `user_id` column | Pass |
| M5 | Timestamps auto-populate in UTC | Insert record without timestamps; verify both populated with UTC server time | Pass |
| M6 | updated_at auto-updates | Update record; verify updated_at changes, created_at unchanged | Pass |
| M7 | Enum is extensible | Adding new enum value does NOT require schema migration | Pass |

### 3.2 Validation (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| V1 | Python validates empty title | `Task(title="")` raises ValidationError before DB call | Pass |
| V2 | Python validates title length | `Task(title="x"*201)` raises ValidationError before DB call | Pass |
| V3 | DB CHECK rejects empty title | Direct SQL INSERT with empty title fails with constraint violation | Pass |
| V4 | DB CHECK rejects whitespace-only title | Direct SQL INSERT with `"   "` title fails with constraint violation | Pass |
| V5 | Python validates description length | `Task(description="x"*1001)` raises ValidationError | Pass |

### 3.3 Database Connectivity (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| C1 | Async engine factory exists | `get_async_engine()` function in database.py | Pass |
| C2 | Async session factory exists | `get_async_session()` async generator in database.py | Pass |
| C3 | Connection uses SSL | Connection string includes `sslmode=require` | Pass |
| C4 | Connection succeeds to Neon | Integration test connects and runs simple query | Pass |
| C5 | Environment variable configuration | DATABASE_URL and TEST_DATABASE_URL read from env | Pass |

### 3.4 Migration Infrastructure (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| A1 | Alembic initialized | `alembic.ini` and `alembic/` directory exist | Pass |
| A2 | Initial migration created | Migration file creates `tasks` table with all constraints | Pass |
| A3 | Migration is reversible | `alembic downgrade -1` succeeds after upgrade | Pass |
| A4 | Migration applies cleanly | `alembic upgrade head` on fresh DB succeeds | Pass |
| A5 | Migration creates CHECK constraints | Post-migration schema includes title validation CHECK | Pass |

### 3.5 Test Infrastructure (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| T1 | Unit tests use SQLite in-memory | Test config creates SQLite engine | Pass |
| T2 | Integration tests use Neon | Integration test config uses TEST_DATABASE_URL | Pass |
| T3 | Test isolation achieved | Each test runs in transaction, rolled back after | Pass |
| T4 | Unit tests pass | `pytest phase2/backend/tests/unit/` exits 0 | Pass |
| T5 | Integration tests pass | `pytest phase2/backend/tests/integration/` exits 0 | Pass |
| T6 | Model CRUD operations tested | Tests cover: create, read, update, delete Task | 4/4 ops |
| T7 | Python validation tested | Tests verify ValidationError for invalid inputs | Pass |
| T8 | User ID uses random UUIDs | Test fixtures generate user_id via `uuid.uuid4()` | Pass |

### 3.6 Code Quality (SMART)

| ID | Criterion | Measurement | Target |
|----|-----------|-------------|--------|
| Q1 | Type hints on all functions | mypy passes with no errors | Pass |
| Q2 | No hardcoded credentials | grep for passwords/secrets returns empty | Pass |
| Q3 | Test coverage | coverage report shows >= 80% for models | >= 80% |

---

## 4. Non-Goals

The following are explicitly **OUT OF SCOPE** for Part 1:

| Excluded Item | Reason | Addressed In |
|---------------|--------|--------------|
| User model/table | Authentication is separate concern | Part 2 |
| Foreign key constraint on user_id | Requires User table to exist | Part 2 |
| API endpoints | API layer is separate concern | Part 3 |
| Authentication/authorization logic | Auth is separate concern | Part 2 |
| Frontend components | Frontend is separate concern | Part 4+ |
| Business logic/services | Service layer is separate concern | Part 3 |
| Priority field | Feature enhancement | Future Part |
| Due date field | Feature enhancement | Future Part |
| Soft delete (deleted_at) | Feature enhancement | Future Part |
| Tags/categories | Feature enhancement | Future Part |
| Seed data migrations | Test data in fixtures only | N/A |
| Connection pooling configuration | Neon handles serverless pooling | N/A |
| Local Postgres/Docker for testing | Neon-only for integration tests | N/A |
| Database backup/restore procedures | Ops concern | Phase 4 |

### Boundaries

- **DO**: Define data model, configure connection, create migrations, write model tests, implement dual-layer validation
- **DO NOT**: Implement business logic, create API routes, handle authentication, build UI, support local Postgres

---

## Appendix A: Task Model Reference

```python
# Expected schema structure (implementation detail for reference)
from enum import Enum
from uuid import uuid4
from datetime import datetime, timezone
from sqlmodel import SQLModel, Field
from sqlalchemy import Column, Enum as SAEnum, CheckConstraint

class TaskStatus(str, Enum):
    PENDING = "pending"
    COMPLETED = "completed"
    # Extensible: add IN_PROGRESS = "in_progress" etc. without migration

class Task(SQLModel, table=True):
    __tablename__ = "tasks"
    __table_args__ = (
        CheckConstraint("title <> '' AND title !~ '^\\s*$'", name="ck_tasks_title_not_empty"),
    )

    id: UUID = Field(default_factory=uuid4, primary_key=True)
    user_id: UUID = Field(nullable=False, index=True)
    title: str = Field(max_length=200, nullable=False)
    description: str | None = Field(default=None, max_length=1000)
    status: TaskStatus = Field(
        default=TaskStatus.PENDING,
        sa_column=Column(SAEnum(TaskStatus, native_enum=False))
    )
    created_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        nullable=False
    )
    updated_at: datetime = Field(
        default_factory=lambda: datetime.now(timezone.utc),
        nullable=False
    )

    # Pydantic validators for dual-layer validation
    @validator('title')
    def title_not_empty(cls, v):
        if not v or not v.strip():
            raise ValueError('Title cannot be empty or whitespace-only')
        return v
```

## Appendix B: Directory Structure

```
phase2/
└── backend/
    ├── app/
    │   ├── __init__.py
    │   ├── core/
    │   │   ├── __init__.py
    │   │   ├── config.py
    │   │   └── database.py
    │   └── models/
    │       ├── __init__.py
    │       └── task.py
    ├── alembic/
    │   ├── versions/
    │   ├── env.py
    │   └── script.py.mako
    ├── alembic.ini
    ├── tests/
    │   ├── __init__.py
    │   ├── conftest.py
    │   ├── unit/
    │   │   └── test_task_model.py
    │   └── integration/
    │       └── test_task_db.py
    ├── pyproject.toml
    └── .env.example
```

---

**Next Steps**: After spec approval, proceed to `/sp.plan` for architectural decisions, then `/sp.tasks` for implementation breakdown.
