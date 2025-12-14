# Implementation Plan: Phase 2 Part 2 - Authentication

**Branch**: `main` | **Date**: 2025-12-14 | **Spec**: [spec.md](./spec.md)
**Input**: Feature specification from `/specs/phase2/part2-authentication/spec.md`

## Summary

Implement JWT-based authentication with Argon2id password hashing for the Todo API. This includes user registration, login, profile retrieval, and task ownership enforcement. The existing Task model already has `user_id` field; this phase adds the User model, authentication utilities, and protected endpoints.

## Technical Context

**Language/Version**: Python 3.13, UV package manager
**Primary Dependencies**: FastAPI, SQLModel, python-jose[cryptography], argon2-cffi, slowapi
**Storage**: Neon PostgreSQL (existing), Alembic migrations
**Testing**: pytest, pytest-asyncio, httpx (SQLite unit + Neon integration)
**Target Platform**: Linux server (API), Windows dev
**Project Type**: Web application (backend focus for Part 2)
**Performance Goals**: < 500ms p95 for auth endpoints
**Constraints**: 30-min JWT expiry, rate limiting (5/min register, 10/min login)
**Scale/Scope**: Single-tenant MVP, ~100 users initial

## Constitution Check

*GATE: Must pass before Phase 0 research. Re-check after Phase 1 design.*

| Gate | Status | Notes |
|------|--------|-------|
| Phase II Technology Stack | PASS | FastAPI, SQLModel, Neon DB, JWT auth - all permitted |
| Spec-Driven Development | PASS | spec.md exists at `/specs/phase2/part2-authentication/spec.md` |
| Clean Architecture | PASS | Layered: models, services, routes, deps |
| JWT Requirements | PARTIAL | Using HS256 (simpler than RS256); 30-min expiry (shorter than 24hr max) |
| Type Hints | PASS | All functions will include type hints |
| Testing Coverage | PASS | 20 test cases defined, targeting 80%+ coverage |
| Password Policy | PASS | Min 8 chars enforced at schema level |

**Violation Justifications:**
- JWT sub claim only (no scope claim yet) - scopes deferred to Part 3 for role-based access

## Project Structure

### Documentation (this feature)

```text
specs/phase2/part2-authentication/
├── spec.md              # Feature specification
├── plan.md              # This file
├── research.md          # Technology decisions
├── data-model.md        # Entity definitions
├── quickstart.md        # Setup instructions
├── contracts/           # OpenAPI spec
│   └── auth-api.yaml
└── tasks.md             # Implementation tasks (created by /sp.tasks)
```

### Source Code (repository root)

```text
phase2/backend/
├── app/
│   ├── __init__.py
│   ├── main.py                    # FastAPI app entry (to create)
│   ├── core/
│   │   ├── __init__.py
│   │   ├── config.py              # Settings (extend with JWT config)
│   │   ├── database.py            # Async session (existing)
│   │   └── security.py            # NEW: password hashing, JWT utils
│   ├── models/
│   │   ├── __init__.py
│   │   ├── task.py                # Existing task model
│   │   └── user.py                # NEW: User model
│   ├── schemas/
│   │   ├── __init__.py            # NEW
│   │   ├── auth.py                # NEW: auth request/response schemas
│   │   └── task.py                # NEW: task schemas (for consistency)
│   ├── services/
│   │   ├── __init__.py            # NEW
│   │   ├── auth_service.py        # NEW: registration, login logic
│   │   └── task_service.py        # NEW: task CRUD with user filtering
│   ├── api/
│   │   ├── __init__.py            # NEW
│   │   ├── deps.py                # NEW: get_current_user dependency
│   │   └── routes/
│   │       ├── __init__.py        # NEW
│   │       ├── auth.py            # NEW: /auth/* endpoints
│   │       └── tasks.py           # NEW: /tasks/* endpoints
│   └── middleware/
│       ├── __init__.py            # NEW
│       └── rate_limit.py          # NEW: slowapi rate limiting
├── alembic/
│   └── versions/
│       ├── 001_create_tasks_table.py   # Existing
│       └── 002_add_users_table.py      # NEW
├── tests/
│   ├── conftest.py                # NEW: fixtures for both DBs
│   ├── unit/
│   │   ├── __init__.py
│   │   ├── test_security.py       # NEW
│   │   ├── test_auth_service.py   # NEW
│   │   └── test_user_model.py     # NEW
│   └── integration/
│       ├── __init__.py
│       ├── test_auth_endpoints.py # NEW
│       └── test_task_ownership.py # NEW
├── requirements.txt               # Extend with auth deps
└── .env.example                   # JWT_SECRET_KEY placeholder
```

**Structure Decision**: Web application backend structure selected. Frontend is out of scope for Part 2.

## Implementation Phases

### Phase 1: User Model + Migration

**Files:**
- `app/models/user.py` - User SQLModel with email, hashed_password, is_active
- `alembic/versions/002_add_users_table.py` - Create users table, add FK to tasks

**Dependencies:** None (builds on existing infrastructure)

**Deliverables:**
- [ ] User model with UUID primary key
- [ ] Email unique index
- [ ] Alembic migration creates users table
- [ ] Migration adds FK constraint to tasks.user_id

### Phase 2: Auth Utilities (JWT + Argon2)

**Files:**
- `app/core/security.py` - hash_password, verify_password, create_access_token, decode_token
- `app/core/config.py` - Add JWT_SECRET_KEY, ALGORITHM, ACCESS_TOKEN_EXPIRE_MINUTES

**Dependencies:** Phase 1 (User model for type hints)

**Deliverables:**
- [ ] Argon2id password hashing with secure defaults
- [ ] JWT token creation with sub, email, exp, iat claims
- [ ] JWT token decoding with expiry validation
- [ ] Settings extended for JWT configuration

### Phase 3: TaskService with Ownership Filtering

**Files:**
- `app/services/task_service.py` - CRUD operations with user_id filtering
- `app/schemas/task.py` - TaskCreate, TaskUpdate, TaskResponse schemas

**Dependencies:** Phase 1 (Task model), Phase 2 (for auth context)

**Deliverables:**
- [ ] create_task(db, user_id, data) - creates task owned by user
- [ ] get_tasks(db, user_id) - returns only user's tasks
- [ ] get_task(db, task_id, user_id) - returns task if owned by user
- [ ] update_task(db, task_id, user_id, data) - updates if owned
- [ ] delete_task(db, task_id, user_id) - deletes if owned

### Phase 4: Auth Endpoints + Rate Limiting

**Files:**
- `app/api/routes/auth.py` - POST /auth/register, POST /auth/login, GET /auth/me
- `app/services/auth_service.py` - register_user, authenticate_user, get_user_by_id
- `app/schemas/auth.py` - UserCreate, LoginRequest, TokenResponse, UserResponse
- `app/middleware/rate_limit.py` - slowapi limiter configuration

**Dependencies:** Phase 2 (security utils), Phase 1 (User model)

**Deliverables:**
- [ ] POST /auth/register - creates user, returns UserResponse
- [ ] POST /auth/login - validates credentials, returns TokenResponse
- [ ] GET /auth/me - returns current user (protected)
- [ ] Rate limiting: 5/min register, 10/min login

### Phase 5: Dependencies + Middleware

**Files:**
- `app/api/deps.py` - get_current_user, get_current_active_user
- `app/main.py` - FastAPI app with router mounting, CORS, rate limit middleware
- `app/api/routes/tasks.py` - Protected task endpoints

**Dependencies:** Phase 4 (auth endpoints), Phase 3 (task service)

**Deliverables:**
- [ ] get_current_user dependency extracts user from JWT
- [ ] get_current_active_user validates is_active=True
- [ ] FastAPI app with all routers mounted
- [ ] Task endpoints require authentication

### Phase 6: Tests (Unit + Integration)

**Files:**
- `tests/conftest.py` - Fixtures for SQLite and Neon connections
- `tests/unit/test_security.py` - Password hashing, JWT tests
- `tests/unit/test_auth_service.py` - Auth service logic tests
- `tests/unit/test_user_model.py` - User model validation tests
- `tests/integration/test_auth_endpoints.py` - Full auth flow tests
- `tests/integration/test_task_ownership.py` - Task isolation tests

**Dependencies:** All phases complete

**Deliverables:**
- [ ] 20 test cases from spec matrix
- [ ] SQLite for fast unit tests
- [ ] Neon PostgreSQL for integration tests
- [ ] 80%+ code coverage

## Dependency Graph

```text
Phase 1: User Model + Migration
    │
    ▼
Phase 2: Auth Utilities (JWT + Argon2)
    │
    ├───────────────────┐
    ▼                   ▼
Phase 3: TaskService    Phase 4: Auth Endpoints
    │                   │
    └───────┬───────────┘
            ▼
Phase 5: Dependencies + Middleware
            │
            ▼
Phase 6: Tests (Unit + Integration)
```

## Success Criteria

| Criterion | Validation Method |
|-----------|-------------------|
| users table exists in Neon | `alembic upgrade head` succeeds |
| POST /auth/register → 201 | Integration test TC-01 |
| POST /auth/login → JWT token | Integration test TC-06 |
| GET /auth/me requires token | Integration test TC-11 |
| TaskService filters by user_id | Unit test + TC-15, TC-16 |
| 15+ tests passing | `pytest --cov` shows 15+ pass |
| Rate limiting works | Integration test TC-19, TC-20 |

## Risks & Mitigations

| Risk | Impact | Mitigation |
|------|--------|------------|
| Argon2 memory usage on constrained hosts | Medium | Use lower memory_cost in test/dev |
| JWT secret exposure | High | Load from env, never commit |
| Rate limit bypass via proxy | Medium | Document need for X-Forwarded-For config |

## References

- Spec: `specs/phase2/part2-authentication/spec.md`
- ADR: `history/adr/ADR-001-authentication-security-stack.md`
- Constitution: `.specify/memory/constitution.md`
