# Tasks: Phase 2 Part 2 - Authentication

**Input**: Design documents from `/specs/phase2/part2-authentication/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/auth-api.yaml

**Tests**: Included (explicitly requested in user input)

**Organization**: Tasks organized by functional area aligned with plan.md phases

## Format: `[ID] [P?] [Story?] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[US#]**: User story mapping (US1=Register, US2=Login, US3=Profile, US4=TaskOwnership)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `phase2/backend/app/`
- **Tests**: `phase2/backend/tests/`
- **Migrations**: `phase2/backend/alembic/versions/`

---

## Phase 1: Setup (Dependencies & Configuration)

**Purpose**: Install dependencies and configure environment

- [x] T001 Install auth dependencies: `uv add argon2-cffi python-jose[cryptography] python-multipart slowapi aiosqlite` in phase2/backend/
- [x] T002 [P] Add JWT_SECRET_KEY to phase2/backend/.env.example
- [x] T003 [P] Extend Settings with JWT config in phase2/backend/app/core/config.py

**Checkpoint**: `uv sync` succeeds, config loads JWT_SECRET_KEY from env

---

## Phase 2: Foundational (Models & Migration)

**Purpose**: User model and database schema - BLOCKS all auth features

**⚠️ CRITICAL**: No auth endpoints can be implemented until this phase is complete

- [x] T004 Create User SQLModel in phase2/backend/app/models/user.py (id:UUID, email:unique, hashed_password, is_active, created_at)
- [x] T005 Export User from phase2/backend/app/models/__init__.py
- [x] T006 Create Alembic migration 002_add_users_table.py in phase2/backend/alembic/versions/
- [x] T007 Run `alembic upgrade head` to apply migration to Neon

**Checkpoint**: `users` table exists in Neon with ix_users_email index, mypy passes on models

---

## Phase 3: User Story 1 - Registration (Priority: P1) 🎯 MVP

**Goal**: Users can register with email and password (FR-01)

**Independent Test**: `curl -X POST /auth/register -d '{"email":"test@example.com","password":"password123"}'` → 201

### Implementation for User Story 1

- [x] T008 [P] [US1] Create password hashing functions (hash_password, verify_password) in phase2/backend/app/core/security.py
- [x] T009 [P] [US1] Create auth schemas (UserCreate, UserResponse) in phase2/backend/app/schemas/auth.py
- [x] T010 [US1] Create auth_service.py with register_user() in phase2/backend/app/services/auth_service.py
- [x] T011 [US1] Create POST /auth/register endpoint in phase2/backend/app/api/routes/auth.py
- [x] T012 [US1] Add rate limiting (5/min) to register endpoint in phase2/backend/app/middleware/rate_limit.py

**Checkpoint**: Registration works - user created in DB with hashed password

---

## Phase 4: User Story 2 - Login (Priority: P1)

**Goal**: Users can login and receive JWT access token (FR-02)

**Independent Test**: `curl -X POST /auth/login -d '{"email":"test@example.com","password":"password123"}'` → 200 + JWT

### Implementation for User Story 2

- [x] T013 [P] [US2] Create JWT utilities (create_access_token, decode_token) in phase2/backend/app/core/security.py
- [x] T014 [P] [US2] Add LoginRequest, TokenResponse schemas in phase2/backend/app/schemas/auth.py
- [x] T015 [US2] Add authenticate_user() to phase2/backend/app/services/auth_service.py
- [x] T016 [US2] Create POST /auth/login endpoint in phase2/backend/app/api/routes/auth.py
- [x] T017 [US2] Add rate limiting (10/min) to login endpoint

**Checkpoint**: Login works - returns valid JWT token with sub, email, exp, iat claims

---

## Phase 5: User Story 3 - Profile (Priority: P1)

**Goal**: Users can retrieve their profile via protected endpoint (FR-03)

**Independent Test**: `curl /auth/me -H "Authorization: Bearer $TOKEN"` → 200 + UserResponse

### Implementation for User Story 3

- [x] T018 [US3] Create get_current_user dependency in phase2/backend/app/api/deps.py
- [x] T019 [US3] Create get_current_active_user dependency (checks is_active) in phase2/backend/app/api/deps.py
- [x] T020 [US3] Create GET /auth/me endpoint in phase2/backend/app/api/routes/auth.py
- [x] T021 [US3] Add oauth2_scheme (Bearer token) to deps.py

**Checkpoint**: /auth/me returns current user, rejects requests without valid token

---

## Phase 6: User Story 4 - Task Ownership (Priority: P1)

**Goal**: Tasks are scoped to authenticated user (FR-04, FR-05)

**Independent Test**: User A cannot see/modify User B's tasks

### Implementation for User Story 4

- [x] T022 [P] [US4] Create task schemas (TaskCreate, TaskUpdate, TaskResponse) in phase2/backend/app/schemas/task.py
- [x] T023 [US4] Create TaskService with user_id filtering in phase2/backend/app/services/task_service.py
- [x] T024 [US4] Implement get_tasks(db, user_id) - returns only user's tasks
- [x] T025 [US4] Implement get_task(db, task_id, user_id) - returns None if not owned
- [x] T026 [US4] Implement create_task(db, user_id, data) - sets user_id on task
- [x] T027 [US4] Implement update_task(db, task_id, user_id, data) - validates ownership
- [x] T028 [US4] Implement delete_task(db, task_id, user_id) - validates ownership
- [x] T029 [US4] Create protected task endpoints in phase2/backend/app/api/routes/tasks.py

**Checkpoint**: Task CRUD works, users only see/modify their own tasks

---

## Phase 7: App Assembly & Middleware

**Purpose**: Wire everything together in FastAPI app

- [x] T030 Create FastAPI app with lifespan in phase2/backend/app/main.py
- [x] T031 Mount auth router at /auth prefix
- [x] T032 Mount tasks router at /tasks prefix
- [x] T033 Add slowapi rate limit middleware and error handler
- [x] T034 Configure CORS middleware (if needed)

**Checkpoint**: `uvicorn app.main:app --reload` starts, Swagger UI shows all endpoints

---

## Phase 8: Unit Tests

**Purpose**: Fast tests with SQLite for core logic

- [x] T035 [P] Create test fixtures in phase2/backend/tests/conftest.py (SQLite async engine, test client)
- [x] T036 [P] Test password hash/verify roundtrip in phase2/backend/tests/unit/test_security.py
- [x] T037 [P] Test JWT create/decode in phase2/backend/tests/unit/test_security.py
- [x] T038 [P] Test expired token rejection in phase2/backend/tests/unit/test_security.py
- [x] T039 [P] Test invalid token rejection in phase2/backend/tests/unit/test_security.py
- [x] T040 [P] Test User model validation in phase2/backend/tests/unit/test_user_model.py
- [x] T041 [P] Test auth_service.register_user() in phase2/backend/tests/unit/test_auth_service.py
- [x] T042 [P] Test auth_service.authenticate_user() in phase2/backend/tests/unit/test_auth_service.py
- [x] T043 [P] Test TaskService filtering in phase2/backend/tests/unit/test_task_service.py

**Checkpoint**: `pytest tests/unit/ -v` - all 9+ tests pass

---

## Phase 9: Integration Tests

**Purpose**: E2E tests with Neon PostgreSQL

- [x] T044 Add Neon test fixtures to phase2/backend/tests/conftest.py
- [x] T045 [P] Test TC-01: Valid registration → 201 in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T046 [P] Test TC-02: Duplicate email → 409 in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T047 [P] Test TC-06: Valid login → JWT in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T048 [P] Test TC-07: Wrong password → 401 in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T049 [P] Test TC-10: Valid token → user in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T050 [P] Test TC-11: No token → 401 in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T051 Test TC-15: User A lists only A's tasks in phase2/backend/tests/integration/test_task_ownership.py
- [x] T052 Test TC-16: User A gets B's task → 404 in phase2/backend/tests/integration/test_task_ownership.py
- [x] T053 Test TC-19: Rate limit register → 429 in phase2/backend/tests/integration/test_auth_endpoints.py
- [x] T054 Test TC-20: Rate limit login → 429 in phase2/backend/tests/integration/test_auth_endpoints.py

**Checkpoint**: `pytest tests/integration/ -v` - all 10+ tests pass

---

## Phase 10: Polish & Validation

**Purpose**: Final verification and documentation

- [ ] T055 Run full test suite: `pytest --cov=app --cov-report=term-missing`
- [ ] T056 Verify 80%+ code coverage
- [ ] T057 Test Alembic downgrade: `alembic downgrade -1`
- [ ] T058 Test Alembic upgrade: `alembic upgrade head`
- [ ] T059 Run mypy type checking: `mypy app/`
- [ ] T060 Verify Swagger UI at /docs shows all endpoints with auth
- [ ] T061 Manual E2E test: register → login → /me → create task → list tasks

**Checkpoint**: All acceptance criteria from spec.md verified

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: Setup
    │
    ▼
Phase 2: Foundational (User Model + Migration) ← BLOCKS ALL
    │
    ├──────────────────┬──────────────────┐
    ▼                  ▼                  ▼
Phase 3: Register   Phase 4: Login    (wait)
    │                  │
    └────────┬─────────┘
             ▼
Phase 5: Profile (needs JWT from Phase 4)
             │
             ▼
Phase 6: Task Ownership (needs auth from Phase 5)
             │
             ▼
Phase 7: App Assembly
             │
             ▼
Phase 8: Unit Tests ←──┬──→ Phase 9: Integration Tests
                       │
                       ▼
              Phase 10: Polish
```

### User Story Dependencies

| Story | Depends On | Can Parallelize With |
|-------|------------|---------------------|
| US1 (Register) | Phase 2 | US2 (different functions) |
| US2 (Login) | Phase 2 | US1 (different endpoints) |
| US3 (Profile) | US2 (JWT) | - |
| US4 (Tasks) | US3 (get_current_user) | - |

### Parallel Opportunities

**Within Phase 1 (Setup):**
- T002, T003 can run in parallel

**Within Phase 3-4 (Register + Login):**
- T008, T009 can run in parallel
- T013, T014 can run in parallel

**Within Phase 8 (Unit Tests):**
- All test tasks (T035-T043) can run in parallel

**Within Phase 9 (Integration Tests):**
- T045-T050 can run in parallel

---

## Parallel Example: Unit Tests

```bash
# Launch all unit tests in parallel:
Task: "Test password hash/verify in tests/unit/test_security.py"
Task: "Test JWT create/decode in tests/unit/test_security.py"
Task: "Test User model validation in tests/unit/test_user_model.py"
Task: "Test auth_service functions in tests/unit/test_auth_service.py"
Task: "Test TaskService filtering in tests/unit/test_task_service.py"
```

---

## Implementation Strategy

### MVP First (Registration + Login + Profile)

1. Complete Phase 1: Setup
2. Complete Phase 2: Foundational (User model + migration)
3. Complete Phase 3: Registration
4. Complete Phase 4: Login
5. Complete Phase 5: Profile
6. **STOP and VALIDATE**: Test register → login → /me flow
7. Deploy/demo basic auth working

### Full Feature Delivery

1. Complete MVP (Phases 1-5)
2. Complete Phase 6: Task Ownership
3. Complete Phase 7: App Assembly
4. Complete Phase 8-9: Tests
5. Complete Phase 10: Polish
6. All 20 test cases pass, 80%+ coverage

---

## Task Summary

| Phase | Tasks | Parallel |
|-------|-------|----------|
| 1. Setup | 3 | 2 |
| 2. Foundational | 4 | 0 |
| 3. Register (US1) | 5 | 2 |
| 4. Login (US2) | 5 | 2 |
| 5. Profile (US3) | 4 | 0 |
| 6. Task Ownership (US4) | 8 | 1 |
| 7. App Assembly | 5 | 0 |
| 8. Unit Tests | 9 | 8 |
| 9. Integration Tests | 11 | 6 |
| 10. Polish | 7 | 0 |
| **Total** | **61** | **21** |

---

## Notes

- [P] tasks = different files, no dependencies on incomplete tasks
- [US#] label maps task to user story for traceability
- Commit after each task or logical group
- Stop at any checkpoint to validate incrementally
- JWT_SECRET_KEY must be 32+ chars, loaded from environment
- Rate limiting uses in-memory storage (slowapi default)
