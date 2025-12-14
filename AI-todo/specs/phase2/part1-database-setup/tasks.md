# Tasks: Phase 2 Part 1 - Database Setup

**Input**: Design documents from `/specs/phase2/part1-database-setup/`
**Prerequisites**: plan.md (required), spec.md (required), research.md, data-model.md, contracts/database-contracts.md

**Tests**: Tests ARE included as they are explicitly required in the spec (Success Evals T1-T8).

**Organization**: This is an infrastructure feature with no traditional user stories. Tasks are organized by implementation phases from plan.md, with core requirements (Model, Validation, Connectivity, Migrations, Tests) mapped to phases.

## Format: `[ID] [P?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- Include exact file paths in descriptions

## Path Conventions

- **Web app backend**: `phase2/backend/` at repository root
- Source: `phase2/backend/app/`
- Tests: `phase2/backend/tests/`

---

## Phase 1: Setup (Project Structure & Dependencies)

**Purpose**: Create project structure and install dependencies
**Success Criteria**: `uv sync` succeeds, imports work

- [x] T001 Create project directory structure: `phase2/backend/app/`, `phase2/backend/app/core/`, `phase2/backend/app/models/`
- [x] T002 Create `phase2/backend/pyproject.toml` with dependencies: sqlmodel>=0.0.14, sqlalchemy[asyncio]>=2.0.0, asyncpg>=0.29.0, alembic>=1.13.0, pydantic-settings>=2.0.0, python-dotenv>=1.0.0, and dev deps: pytest>=8.0.0, pytest-asyncio>=0.23.0, pytest-cov>=4.0.0, mypy>=1.8.0, aiosqlite>=0.19.0
- [x] T003 [P] Create `phase2/backend/app/__init__.py` (empty package init)
- [x] T004 [P] Create `phase2/backend/app/core/__init__.py` (empty package init)
- [x] T005 [P] Create `phase2/backend/app/models/__init__.py` (empty package init)
- [x] T006 [P] Create `phase2/backend/.env.example` with DATABASE_URL and TEST_DATABASE_URL placeholders
- [ ] T007 Run `uv sync` in `phase2/backend/` to install dependencies (MANUAL: run in your terminal)

**Checkpoint**: Project structure created, dependencies installed, imports work

---

## Phase 2: Configuration Layer

**Purpose**: Set up environment-based configuration
**Success Criteria**: C5 - Environment variables read correctly from .env

- [x] T008 Create `phase2/backend/app/core/config.py` with Settings class using pydantic-settings BaseSettings for DATABASE_URL (required) and TEST_DATABASE_URL (optional)

**Checkpoint**: Settings class loads environment variables correctly

---

## Phase 3: Database Connection

**Purpose**: Create async engine and session factory
**Success Criteria**: C1 (engine factory), C2 (session factory), C3 (SSL), C4 (Neon connection)

- [x] T009 Create `phase2/backend/app/core/database.py` with `get_async_engine()` function using create_async_engine with settings.database_url
- [x] T010 Add `get_async_session()` async generator in `phase2/backend/app/core/database.py` using async_sessionmaker with expire_on_commit=False
- [x] T011 Export database functions in `phase2/backend/app/core/__init__.py`

**Checkpoint**: Async engine and session factory ready for model usage

---

## Phase 4: Task Model

**Purpose**: Define Task entity with TaskStatus enum and dual-layer validation
**Success Criteria**: M1-M7 (model complete), V1-V5 (validation complete)

- [x] T012 Create TaskStatus enum in `phase2/backend/app/models/task.py` with PENDING and COMPLETED values, inheriting from (str, Enum)
- [x] T013 Create Task SQLModel class in `phase2/backend/app/models/task.py` with fields: id (UUID PK with uuid4 default), user_id (UUID NOT NULL indexed), title (str max_length=200 NOT NULL), description (str|None max_length=1000), status (TaskStatus default PENDING with native_enum=False sa_column), created_at (datetime UTC auto-set), updated_at (datetime UTC auto-set)
- [x] T014 Add CheckConstraint for title validation in `phase2/backend/app/models/task.py`: `length(trim(title)) > 0` named `ck_tasks_title_not_empty`
- [x] T015 Add @field_validator for title in `phase2/backend/app/models/task.py` to reject empty or whitespace-only titles
- [x] T016 Export Task and TaskStatus in `phase2/backend/app/models/__init__.py`

**Checkpoint**: Task model complete with dual-layer validation (Python + DB)

---

## Phase 5: Alembic Setup

**Purpose**: Configure Alembic for async migrations
**Success Criteria**: A1 - Alembic initialized

- [x] T017 Initialize Alembic in `phase2/backend/` with async template: `alembic init -t async alembic`
- [x] T018 Configure `phase2/backend/alembic.ini` to use DATABASE_URL from environment
- [x] T019 Update `phase2/backend/alembic/env.py` to import SQLModel metadata and Task model, set target_metadata = SQLModel.metadata
- [x] T020 Add `import sqlmodel.sql.sqltypes` to `phase2/backend/alembic/script.py.mako` template

**Checkpoint**: Alembic configured for async migrations with SQLModel

---

## Phase 6: Initial Migration

**Purpose**: Create and verify tasks table migration
**Success Criteria**: A2 (migration created), A3 (reversible), A4 (applies cleanly), A5 (CHECK constraints)

- [x] T021 Generate initial migration: `alembic revision --autogenerate -m "create_tasks_table"` in `phase2/backend/`
- [x] T022 Verify migration file in `phase2/backend/alembic/versions/` includes: tasks table, all columns, ix_tasks_user_id index, ck_tasks_title_not_empty CHECK constraint
- [x] T023 Test migration upgrade: `alembic upgrade head` against Neon test database (MANUAL)
- [x] T024 Test migration downgrade: `alembic downgrade -1` to verify reversibility (MANUAL)
- [x] T025 Test re-upgrade: `alembic upgrade head` to confirm clean re-application (MANUAL)

**Checkpoint**: Migration is reversible, creates all constraints, applies cleanly

---

## Phase 7: Test Infrastructure

**Purpose**: Set up pytest fixtures for unit and integration tests
**Success Criteria**: T1 (SQLite unit), T2 (Neon integration), T3 (isolation)

- [x] T026 Create `phase2/backend/tests/__init__.py` (empty package init)
- [x] T027 [P] Create `phase2/backend/tests/unit/__init__.py` (empty package init)
- [x] T028 [P] Create `phase2/backend/tests/integration/__init__.py` (empty package init)
- [x] T029 Create `phase2/backend/tests/conftest.py` with pytest-asyncio configuration and fixtures: `sqlite_engine` (in-memory async), `sqlite_session` (isolated session for unit tests), `neon_session` (isolated session for integration tests using TEST_DATABASE_URL)
- [x] T030 Add `sample_task` fixture in `phase2/backend/tests/conftest.py` returning valid Task with random UUID user_id

**Checkpoint**: Test fixtures ready for both SQLite unit and Neon integration tests

---

## Phase 8: Unit Tests (SQLite)

**Purpose**: Test Python validation with SQLite in-memory
**Success Criteria**: T4 (unit tests pass), T6 (CRUD tested), T7 (validation tested), T8 (random UUIDs)

- [x] T031 Create `phase2/backend/tests/unit/test_task_model.py` with test class for Task model validation
- [x] T032 Add test: valid task creation with all fields (title, description, user_id, status)
- [x] T033 Add test: empty title raises ValidationError (V1)
- [x] T034 Add test: whitespace-only title raises ValidationError (V1)
- [x] T035 Add test: title exceeding 200 chars raises ValidationError (V2)
- [x] T036 Add test: description exceeding 1000 chars raises ValidationError (V5)
- [x] T037 Add test: invalid status value raises ValidationError
- [x] T038 Add test: default status is PENDING
- [x] T039 Add test: timestamps auto-populate on creation (M5)
- [x] T040 Add test: TaskStatus enum is extensible without migration (M7) - add new value, verify storage
- [x] T041 Run unit tests: `pytest phase2/backend/tests/unit/ -v` (MANUAL) - 22 passed

**Checkpoint**: All unit tests pass, Python validation verified

---

## Phase 9: Integration Tests (Neon)

**Purpose**: Test database operations with Neon Postgres
**Success Criteria**: T5 (integration tests pass), C4 (Neon connection)

- [x] T042 Create `phase2/backend/tests/integration/test_task_db.py` with test class for database operations
- [x] T043 Add test: create task and retrieve by id (CRUD - Create, Read)
- [x] T044 Add test: update task title and verify updated_at changes, created_at unchanged (CRUD - Update, M6)
- [x] T045 Add test: delete task and verify removal (CRUD - Delete)
- [x] T046 Add test: DB CHECK constraint rejects empty title on direct SQL INSERT (V3)
- [x] T047 Add test: DB CHECK constraint rejects whitespace-only title on direct SQL INSERT (V4)
- [x] T048 Add test: verify ix_tasks_user_id index exists (M4)
- [x] T049 Add test: SSL connection succeeds (C4) - verify connection string has sslmode=require
- [x] T050 Add test: timestamps stored in UTC (TIMESTAMPTZ behavior)
- [x] T051 Run integration tests: `pytest phase2/backend/tests/integration/ -v` (MANUAL) - 10 passed

**Checkpoint**: All integration tests pass, Neon database operations verified

---

## Phase 10: Quality Checks

**Purpose**: Verify type hints, coverage, and security
**Success Criteria**: Q1 (mypy passes), Q2 (no secrets), Q3 (80%+ coverage)

- [ ] T052 Run mypy type checking: `mypy phase2/backend/app/` with no errors (Q1)
- [ ] T053 Run coverage report: `pytest phase2/backend/tests/ --cov=phase2/backend/app --cov-report=term-missing` targeting >=80% (Q3)
- [ ] T054 Verify no hardcoded credentials: grep for password/secret/token in `phase2/backend/app/` returns empty (Q2)
- [ ] T055 Run full test suite: `pytest phase2/backend/tests/ -v` to confirm all tests pass

**Checkpoint**: All quality gates pass - mypy clean, 80%+ coverage, no secrets

---

## Dependencies & Execution Order

### Phase Dependencies

```
Phase 1 (Setup)
    │
    ▼
Phase 2 (Config)
    │
    ▼
Phase 3 (Database)
    │
    ▼
Phase 4 (Model)
    │
    ▼
Phase 5 (Alembic)
    │
    ▼
Phase 6 (Migration)
    │
    ├─────────────────┐
    ▼                 ▼
Phase 7 (Test Infra) │
    │                 │
    ├─────────┬───────┘
    ▼         ▼
Phase 8     Phase 9
(Unit)      (Integration)
    │         │
    └────┬────┘
         ▼
    Phase 10 (Quality)
```

### Within Each Phase

- Tasks without [P] must execute sequentially
- Tasks with [P] can run in parallel (different files, no dependencies)
- Complete phase before moving to next

### Parallel Opportunities

**Phase 1**:
```bash
# Can run T003, T004, T005, T006 in parallel:
Task: "Create phase2/backend/app/__init__.py"
Task: "Create phase2/backend/app/core/__init__.py"
Task: "Create phase2/backend/app/models/__init__.py"
Task: "Create phase2/backend/.env.example"
```

**Phase 7**:
```bash
# Can run T027, T028 in parallel:
Task: "Create phase2/backend/tests/unit/__init__.py"
Task: "Create phase2/backend/tests/integration/__init__.py"
```

---

## Implementation Strategy

### MVP First (Phases 1-6)

1. Complete Phase 1: Setup - Project structure and dependencies
2. Complete Phase 2: Config - Environment configuration
3. Complete Phase 3: Database - Async engine and session
4. Complete Phase 4: Model - Task entity with validation
5. Complete Phase 5: Alembic - Migration setup
6. Complete Phase 6: Migration - Apply and verify schema
7. **STOP and VALIDATE**: Database layer functional

### Full Implementation (Phases 7-10)

8. Complete Phase 7: Test Infrastructure
9. Complete Phase 8: Unit Tests
10. Complete Phase 9: Integration Tests
11. Complete Phase 10: Quality Checks
12. **FINAL VALIDATION**: All success criteria met

### Success Criteria Mapping

| Phase | Success Criteria from Spec |
|-------|---------------------------|
| 1 | Project structure created |
| 2 | C5 (env variables) |
| 3 | C1, C2, C3, C4 (connectivity) |
| 4 | M1-M7 (model), V1-V5 (validation) |
| 5 | A1 (Alembic init) |
| 6 | A2-A5 (migrations) |
| 7 | T1-T3 (test infrastructure) |
| 8 | T4, T6-T8 (unit tests) |
| 9 | T5 (integration tests) |
| 10 | Q1-Q3 (quality) |

---

## Notes

- [P] tasks = different files, no dependencies within the phase
- All timestamps in UTC (TIMESTAMPTZ)
- SSL required for Neon connections
- SQLite for unit tests (speed), Neon for integration tests (parity)
- TaskStatus uses native_enum=False for extensibility
- CHECK constraint uses `length(trim(title)) > 0` for SQLite compatibility
- Commit after each task or logical group
- Verify tests fail before implementation (Red-Green pattern for tests)
