# Implementation Checklist: Phase 2 Part 2 - Authentication

**Feature**: User Authentication & Task Ownership
**Date**: 2025-12-14

---

## Pre-Implementation

- [ ] Review spec.md and plan.md
- [ ] Verify Neon database connection works
- [ ] Confirm Phase 1 migration (001) is applied
- [ ] Generate JWT_SECRET_KEY for .env

---

## Phase 1: User Model + Migration

### Files to Create
- [ ] `app/models/user.py` - User SQLModel

### Files to Modify
- [ ] `app/models/__init__.py` - Export User model

### Migration
- [ ] `alembic/versions/002_add_users_table.py` - Create users table + FK

### Verification
- [ ] `alembic upgrade head` succeeds
- [ ] `users` table exists in Neon
- [ ] `ix_users_email` index exists
- [ ] `fk_tasks_user_id` constraint exists

---

## Phase 2: Auth Utilities (JWT + Argon2)

### Files to Create
- [ ] `app/core/security.py` - Password hashing + JWT utils

### Files to Modify
- [ ] `app/core/config.py` - Add JWT settings

### Functions to Implement
- [ ] `hash_password(password: str) -> str`
- [ ] `verify_password(password: str, hash: str) -> bool`
- [ ] `create_access_token(data: dict) -> str`
- [ ] `decode_access_token(token: str) -> dict | None`

### Verification
- [ ] Password hashing produces valid Argon2id hash
- [ ] Password verification works for correct/incorrect passwords
- [ ] JWT encodes with correct claims (sub, email, exp, iat)
- [ ] JWT decodes and validates expiry

---

## Phase 3: TaskService with Ownership Filtering

### Files to Create
- [ ] `app/services/__init__.py`
- [ ] `app/services/task_service.py`
- [ ] `app/schemas/__init__.py`
- [ ] `app/schemas/task.py`

### Functions to Implement
- [ ] `create_task(db, user_id, data) -> Task`
- [ ] `get_tasks(db, user_id) -> list[Task]`
- [ ] `get_task(db, task_id, user_id) -> Task | None`
- [ ] `update_task(db, task_id, user_id, data) -> Task | None`
- [ ] `delete_task(db, task_id, user_id) -> bool`

### Verification
- [ ] Tasks created with correct user_id
- [ ] get_tasks returns only user's tasks
- [ ] get_task returns None for other user's tasks
- [ ] update_task fails for other user's tasks
- [ ] delete_task fails for other user's tasks

---

## Phase 4: Auth Endpoints + Rate Limiting

### Files to Create
- [ ] `app/services/auth_service.py`
- [ ] `app/schemas/auth.py`
- [ ] `app/api/__init__.py`
- [ ] `app/api/routes/__init__.py`
- [ ] `app/api/routes/auth.py`
- [ ] `app/middleware/__init__.py`
- [ ] `app/middleware/rate_limit.py`

### Endpoints to Implement
- [ ] `POST /auth/register` - 201 on success, 409 on duplicate
- [ ] `POST /auth/login` - 200 with JWT, 401 on bad credentials
- [ ] `GET /auth/me` - 200 with user, 401 without token

### Rate Limiting
- [ ] Register: 5 requests/minute per IP
- [ ] Login: 10 requests/minute per IP
- [ ] Returns 429 when exceeded

### Verification
- [ ] Register creates user in database
- [ ] Register returns UserResponse (no password)
- [ ] Login validates password correctly
- [ ] Login returns valid JWT
- [ ] /auth/me rejects requests without token
- [ ] /auth/me returns current user with valid token

---

## Phase 5: Dependencies + Middleware

### Files to Create
- [ ] `app/api/deps.py`
- [ ] `app/api/routes/tasks.py`
- [ ] `app/main.py`

### Dependencies to Implement
- [ ] `get_current_user(token) -> User`
- [ ] `get_current_active_user(user) -> User`

### FastAPI App Setup
- [ ] Mount auth router at `/auth`
- [ ] Mount tasks router at `/tasks`
- [ ] Add rate limit middleware
- [ ] Configure CORS (if needed)

### Task Endpoints (Protected)
- [ ] `GET /tasks` - List user's tasks
- [ ] `POST /tasks` - Create task
- [ ] `GET /tasks/{id}` - Get task (owned only)
- [ ] `PATCH /tasks/{id}` - Update task (owned only)
- [ ] `DELETE /tasks/{id}` - Delete task (owned only)

### Verification
- [ ] All task endpoints require authentication
- [ ] User A cannot see User B's tasks
- [ ] User A cannot modify User B's tasks
- [ ] Swagger UI shows auth requirements

---

## Phase 6: Tests

### Files to Create
- [ ] `tests/conftest.py` - Fixtures
- [ ] `tests/unit/test_security.py`
- [ ] `tests/unit/test_auth_service.py`
- [ ] `tests/unit/test_user_model.py`
- [ ] `tests/integration/test_auth_endpoints.py`
- [ ] `tests/integration/test_task_ownership.py`

### Unit Tests (SQLite)
- [ ] TC-03: Invalid email format → 400
- [ ] TC-04: Password < 8 chars → 400
- [ ] TC-05: Password > 128 chars → 400
- [ ] TC-11: No token → 401
- [ ] TC-12: Expired token → 401
- [ ] TC-13: Invalid token → 401

### Integration Tests (Neon)
- [ ] TC-01: Valid register → 201
- [ ] TC-02: Duplicate email → 409
- [ ] TC-06: Correct login → 200 + JWT
- [ ] TC-07: Wrong password → 401
- [ ] TC-08: Non-existent email → 401
- [ ] TC-09: Inactive user → 401
- [ ] TC-10: Valid token → 200 + User
- [ ] TC-14: User A creates task → user_id = A
- [ ] TC-15: User A lists tasks → only A's tasks
- [ ] TC-16: User A gets B's task → 404
- [ ] TC-17: User A updates B's task → 404
- [ ] TC-18: User A deletes B's task → 404
- [ ] TC-19: 6th register in 1 min → 429
- [ ] TC-20: 11th login in 1 min → 429

### Coverage
- [ ] Run `pytest --cov=app --cov-report=term-missing`
- [ ] Verify ≥80% coverage

---

## Post-Implementation

### Documentation
- [ ] Swagger UI accessible at /docs
- [ ] ReDoc accessible at /redoc
- [ ] OpenAPI JSON matches contracts/auth-api.yaml

### Security Review
- [ ] JWT_SECRET_KEY not in codebase
- [ ] Passwords never logged or returned
- [ ] Rate limiting active on auth endpoints

### Final Verification
- [ ] All 20 test cases pass
- [ ] `alembic upgrade head` + `alembic downgrade -1` works
- [ ] Fresh user can register → login → create task → view task

---

## Sign-Off

| Reviewer | Date | Status |
|----------|------|--------|
| Developer | | Pending |
| Code Review | | Pending |

---

**References**:
- Spec: `specs/phase2/part2-authentication/spec.md`
- Plan: `specs/phase2/part2-authentication/plan.md`
- ADR: `history/adr/ADR-001-authentication-security-stack.md`
