# Implementation Checklist: Phase 2 Part 1 - Database Setup

**Purpose**: Track implementation progress against spec success criteria
**Created**: 2025-12-13
**Spec**: specs/phase2/part1-database-setup/spec.md
**Status**: Ready for Implementation

---

## 1. Model Definition (M1-M7)

- [ ] **M1**: Task model exists at `phase2/backend/app/models/task.py`
- [ ] **M2**: Task model has all 7 required fields:
  - [ ] `id` (UUID, Primary Key)
  - [ ] `user_id` (UUID, NOT NULL)
  - [ ] `title` (VARCHAR 200, NOT NULL)
  - [ ] `description` (VARCHAR 1000, NULL)
  - [ ] `status` (non-native enum)
  - [ ] `created_at` (TIMESTAMPTZ, NOT NULL)
  - [ ] `updated_at` (TIMESTAMPTZ, NOT NULL)
- [ ] **M3**: TaskStatus enum contains exactly `pending`, `completed` with `native_enum=False`
- [ ] **M4**: Index exists on `user_id` column
- [ ] **M5**: Timestamps auto-populate with UTC server time on insert
- [ ] **M6**: `updated_at` auto-updates on modification, `created_at` unchanged
- [ ] **M7**: Enum extensible (adding new value doesn't require schema migration)

---

## 2. Validation (V1-V5)

### Python/SQLModel Layer
- [ ] **V1**: `Task(title="")` raises ValidationError before DB call
- [ ] **V2**: `Task(title="x"*201)` raises ValidationError before DB call
- [ ] **V5**: `Task(description="x"*1001)` raises ValidationError

### Database Layer
- [ ] **V3**: Direct SQL INSERT with empty title fails with constraint violation
- [ ] **V4**: Direct SQL INSERT with whitespace-only title (`"   "`) fails with constraint violation

---

## 3. Database Connectivity (C1-C5)

- [ ] **C1**: `get_async_engine()` function exists in `database.py`
- [ ] **C2**: `get_async_session()` async generator exists in `database.py`
- [ ] **C3**: Connection string includes `sslmode=require`
- [ ] **C4**: Integration test successfully connects to Neon and runs simple query
- [ ] **C5**: `DATABASE_URL` and `TEST_DATABASE_URL` read from environment variables

---

## 4. Migration Infrastructure (A1-A5)

- [ ] **A1**: `alembic.ini` and `alembic/` directory exist
- [ ] **A2**: Initial migration creates `tasks` table with all constraints
- [ ] **A3**: `alembic downgrade -1` succeeds after upgrade (reversible)
- [ ] **A4**: `alembic upgrade head` succeeds on fresh database
- [ ] **A5**: Migration includes CHECK constraint for title validation

---

## 5. Test Infrastructure (T1-T8)

### Configuration
- [ ] **T1**: Unit test config creates SQLite in-memory engine
- [ ] **T2**: Integration test config uses `TEST_DATABASE_URL` (Neon)
- [ ] **T3**: Each test runs in transaction, rolled back after (isolation)

### Test Execution
- [ ] **T4**: `pytest phase2/backend/tests/unit/` exits 0
- [ ] **T5**: `pytest phase2/backend/tests/integration/` exits 0

### Test Coverage
- [ ] **T6**: Tests cover all CRUD operations:
  - [ ] Create Task
  - [ ] Read Task
  - [ ] Update Task
  - [ ] Delete Task
- [ ] **T7**: Tests verify ValidationError for invalid inputs
- [ ] **T8**: Test fixtures generate `user_id` via `uuid.uuid4()` (random UUIDs)

---

## 6. Code Quality (Q1-Q3)

- [ ] **Q1**: `mypy` passes with no errors (type hints on all functions)
- [ ] **Q2**: `grep` for passwords/secrets returns empty (no hardcoded credentials)
- [ ] **Q3**: Coverage report shows >= 80% for models module

---

## Spec Quality Verification

- [x] No implementation details in spec (WHAT only, no HOW)
- [x] All requirements are testable and unambiguous
- [x] Success criteria are SMART (Specific, Measurable, Achievable, Relevant, Time-bound)
- [x] Non-goals clearly documented
- [x] Dual-layer validation strategy defined
- [x] Enum extensibility strategy defined
- [x] UTC timestamp strategy defined

---

## Files to Create

| File | Purpose | Status |
|------|---------|--------|
| `phase2/backend/app/__init__.py` | Package init | [ ] |
| `phase2/backend/app/core/__init__.py` | Core package init | [ ] |
| `phase2/backend/app/core/config.py` | Environment config | [ ] |
| `phase2/backend/app/core/database.py` | DB connection | [ ] |
| `phase2/backend/app/models/__init__.py` | Models package init | [ ] |
| `phase2/backend/app/models/task.py` | Task model | [ ] |
| `phase2/backend/alembic.ini` | Alembic config | [ ] |
| `phase2/backend/alembic/env.py` | Alembic env | [ ] |
| `phase2/backend/alembic/script.py.mako` | Migration template | [ ] |
| `phase2/backend/alembic/versions/` | Migration files | [ ] |
| `phase2/backend/tests/__init__.py` | Tests package init | [ ] |
| `phase2/backend/tests/conftest.py` | Pytest fixtures | [ ] |
| `phase2/backend/tests/unit/__init__.py` | Unit tests init | [ ] |
| `phase2/backend/tests/unit/test_task_model.py` | Model unit tests | [ ] |
| `phase2/backend/tests/integration/__init__.py` | Integration tests init | [ ] |
| `phase2/backend/tests/integration/test_task_db.py` | DB integration tests | [ ] |
| `phase2/backend/pyproject.toml` | Project config | [ ] |
| `phase2/backend/.env.example` | Env template | [ ] |

---

## Sign-off

| Role | Name | Date | Status |
|------|------|------|--------|
| Spec Author | Claude | 2025-12-13 | Complete |
| Reviewer | | | Pending |
| Implementer | | | Pending |

---

**Next Steps**: Run `/sp.plan` → `/sp.tasks` → `/sp.implement`
