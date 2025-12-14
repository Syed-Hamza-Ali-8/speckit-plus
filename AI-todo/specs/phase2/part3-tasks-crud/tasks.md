# Tasks: Phase 2 Part 3 - Task CRUD API

**Input**: Design documents from `/specs/phase2/part3-tasks-crud/`
**Prerequisites**: plan.md, spec.md, research.md, data-model.md, contracts/tasks-api.yaml

**Tests**: Integration tests are included as requested in the user input (Task 8).

**Organization**: Tasks organized sequentially (schemas → service → router → tests) as this is an extension of existing code.

## Format: `[ID] [P?] [Story] Description`

- **[P]**: Can run in parallel (different files, no dependencies)
- **[Story]**: Not applicable for this feature (single cohesive API enhancement)
- Include exact file paths in descriptions

## Path Conventions

- **Backend**: `phase2/backend/app/`
- **Tests**: `phase2/backend/tests/`

---

## Phase 1: Schemas + Pydantic Models

**Purpose**: Add new response schemas for paginated list and hiding user_id

**Checkpoint**: `mypy phase2/backend/app/schemas/task.py` passes

- [x] T001 Add `TaskRead` schema (excludes user_id) in `phase2/backend/app/schemas/task.py`
- [x] T002 Add `PaginatedTaskResponse` schema (items, total, limit, offset) in `phase2/backend/app/schemas/task.py`
- [x] T003 Add imports for `date` type in `phase2/backend/app/schemas/task.py`

**Deliverables**:
- `TaskRead`: id, title, description, status, created_at, updated_at (NO user_id)
- `PaginatedTaskResponse`: items (list[TaskRead]), total, limit, offset
- Existing `TaskCreate`, `TaskUpdate`, `TaskResponse` preserved for backward compatibility

---

## Phase 2: Extend TaskService CRUD

**Purpose**: Add filtering, sorting, and pagination to get_tasks()

**Checkpoint**: Service methods can be called with new parameters

- [x] T004 Add `parse_sort()` helper function to validate sort string in `phase2/backend/app/services/task_service.py`
- [x] T005 Extend `get_tasks()` signature to accept filter params (status, created_after, created_before) in `phase2/backend/app/services/task_service.py`
- [x] T006 Extend `get_tasks()` to accept sort params (sort_field, sort_direction) in `phase2/backend/app/services/task_service.py`
- [x] T007 Extend `get_tasks()` to accept pagination (limit, offset) in `phase2/backend/app/services/task_service.py`
- [x] T008 Modify `get_tasks()` to return tuple (tasks, total_count) in `phase2/backend/app/services/task_service.py`
- [x] T009 Add count query using `func.count()` before applying LIMIT/OFFSET in `phase2/backend/app/services/task_service.py`

**Deliverables**:
- `get_tasks(db, user_id, status?, created_after?, created_before?, sort_field, sort_direction, limit, offset) -> tuple[list[Task], int]`
- `parse_sort(sort: str) -> tuple[str, str]` with validation for allowed fields/directions

---

## Phase 3: Tasks Router Setup + Endpoints

**Purpose**: Update router with query parameters and new response schemas

**Checkpoint**: `GET /tasks` returns 401 without auth, `PaginatedTaskResponse` with auth

- [x] T010 Add `Request` import and `date` import to `phase2/backend/app/api/routes/tasks.py`
- [x] T011 Import `TaskRead`, `PaginatedTaskResponse` in `phase2/backend/app/api/routes/tasks.py`
- [x] T012 Add `ALLOWED_SORT_FIELDS` constant in `phase2/backend/app/api/routes/tasks.py`
- [x] T013 Add `parse_sort()` helper with 400 error handling in `phase2/backend/app/api/routes/tasks.py`
- [x] T014 Update `list_tasks()` endpoint to accept Query parameters (status, created_after, created_before, sort, limit, offset) in `phase2/backend/app/api/routes/tasks.py`
- [x] T015 Update `list_tasks()` to return `PaginatedTaskResponse` in `phase2/backend/app/api/routes/tasks.py`
- [x] T016 Update `create_task()` to return `TaskRead` instead of `TaskResponse` in `phase2/backend/app/api/routes/tasks.py`
- [x] T017 Update `get_task()` to return `TaskRead` instead of `TaskResponse` in `phase2/backend/app/api/routes/tasks.py`
- [x] T018 Update `update_task()` to return `TaskRead` instead of `TaskResponse` in `phase2/backend/app/api/routes/tasks.py`

**Deliverables**:
- `GET /tasks?status=pending&sort=created_at:desc&limit=20&offset=0` → `PaginatedTaskResponse`
- `POST /tasks` → 201 `TaskRead`
- `GET /tasks/{id}` → `TaskRead`
- `PATCH /tasks/{id}` → `TaskRead`
- `DELETE /tasks/{id}` → 204 (no change)

---

## Phase 4: Rate Limiting

**Purpose**: Add per-user rate limiting on write endpoints

**Checkpoint**: 31st POST in 1 hour returns 429

- [x] T019 Add `get_user_key()` function to extract user ID from request in `phase2/backend/app/middleware/rate_limit.py`
- [x] T020 Update limiter instantiation to use `get_user_key` in `phase2/backend/app/middleware/rate_limit.py`
- [x] T021 Import limiter in `phase2/backend/app/api/routes/tasks.py`
- [x] T022 Add `@limiter.limit("30/hour")` decorator to `create_task()` in `phase2/backend/app/api/routes/tasks.py`
- [x] T023 Add `@limiter.limit("60/hour")` decorator to `update_task()` in `phase2/backend/app/api/routes/tasks.py`
- [x] T024 Add `@limiter.limit("30/hour")` decorator to `delete_task()` in `phase2/backend/app/api/routes/tasks.py`
- [x] T025 Add `@limiter.limit("100/hour")` decorator to `list_tasks()` in `phase2/backend/app/api/routes/tasks.py`

**Deliverables**:
- Rate limits: 30 POST/hr, 60 PATCH/hr, 30 DELETE/hr, 100 GET list/hr
- User-based limiting (by JWT user ID, fallback to IP)

---

## Phase 5: Integration Tests

**Purpose**: E2E tests for CRUD, pagination, auth, ownership, rate limits

**Checkpoint**: `pytest phase2/backend/tests/integration/test_tasks_crud.py -v` → ALL PASS

- [x] T026 [P] Create test file with fixtures (test client, auth tokens, test users) in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T027 [P] Add TC-01: test_list_tasks_empty in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T028 [P] Add TC-02: test_list_tasks_with_data in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T029 [P] Add TC-03: test_list_tasks_filter_by_status in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T030 [P] Add TC-04: test_list_tasks_filter_by_date in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T031 [P] Add TC-05: test_list_tasks_sort_ascending in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T032 [P] Add TC-06: test_list_tasks_pagination in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T033 [P] Add TC-07: test_list_tasks_no_auth in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T034 [P] Add TC-08: test_create_task_valid in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T035 [P] Add TC-09: test_create_task_empty_title in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T036 [P] Add TC-10: test_create_task_title_too_long in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T037 [P] Add TC-11: test_create_task_no_auth in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T038 [P] Add TC-12: test_get_task_own in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T039 [P] Add TC-13: test_get_task_nonexistent in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T040 [P] Add TC-14: test_get_task_other_user (expect 404) in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T041 [P] Add TC-15: test_get_task_invalid_uuid in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T042 [P] Add TC-16: test_update_task_title_only in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T043 [P] Add TC-17: test_update_task_status_only in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T044 [P] Add TC-18: test_update_task_multiple_fields in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T045 [P] Add TC-19: test_update_task_empty_title in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T046 [P] Add TC-20: test_update_task_other_user (expect 404) in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T047 [P] Add TC-22: test_delete_task_own in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T048 [P] Add TC-23: test_delete_task_nonexistent in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T049 [P] Add TC-24: test_delete_task_other_user (expect 404) in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T050 Add TC-25: test_rate_limit_create (mock or skip if slow) in `phase2/backend/tests/integration/test_tasks_crud.py`
- [x] T051 Add test_response_excludes_user_id in `phase2/backend/tests/integration/test_tasks_crud.py`

**Deliverables**:
- 25+ test cases covering spec.md Section 4.1 test matrix
- All tests pass with SQLite (unit) and Neon PostgreSQL (integration)
- 80%+ code coverage

---

## Phase 6: Polish & Verification

**Purpose**: Final verification and cleanup

**Checkpoint**: All acceptance criteria from spec.md Section 4.4 pass

- [ ] T052 Verify response schemas exclude `user_id` in all endpoints
- [ ] T053 Run full test suite: `pytest phase2/backend/tests/ -v --cov=app`
- [ ] T054 Verify OpenAPI docs at `/docs` show correct schemas
- [ ] T055 Manual curl test: `GET /tasks?status=pending&sort=created_at:desc&limit=5`
- [ ] T056 Manual curl test: `POST /tasks` → verify no `user_id` in response
- [ ] T057 Manual curl test: Cross-user access → verify 404 (not 403)

---

## Dependencies & Execution Order

### Phase Dependencies

```text
Phase 1: Schemas ──────────────────────┐
                                       │
                                       ▼
Phase 2: TaskService ──────────────────┐
                                       │
                                       ▼
Phase 3: Router + Endpoints ───────────┐
                                       │
                                       ▼
Phase 4: Rate Limiting ────────────────┐
                                       │
                                       ▼
Phase 5: Integration Tests ────────────┐
                                       │
                                       ▼
Phase 6: Polish & Verification
```

### Within Each Phase

- T001-T003: Can run sequentially (same file)
- T004-T009: Must be sequential (building on same function)
- T010-T018: Must be sequential (same file, dependent changes)
- T019-T025: T019-T020 first, then T021-T025
- T026-T051: T026 first (fixtures), then T027-T051 can be parallel [P]
- T052-T057: Can run in parallel [P]

### Parallel Opportunities

```bash
# Phase 5 tests (after T026 fixtures):
Task: "Add TC-01: test_list_tasks_empty"
Task: "Add TC-02: test_list_tasks_with_data"
Task: "Add TC-03: test_list_tasks_filter_by_status"
# ... all TC tests can be written in parallel
```

---

## Implementation Strategy

### Sequential Execution (Recommended)

1. Complete Phase 1: Schemas → Checkpoint: mypy passes
2. Complete Phase 2: Service → Checkpoint: can call new get_tasks()
3. Complete Phase 3: Router → Checkpoint: curl returns PaginatedTaskResponse
4. Complete Phase 4: Rate Limiting → Checkpoint: rate limits enforced
5. Complete Phase 5: Tests → Checkpoint: 25+ tests pass
6. Complete Phase 6: Polish → **DONE**

### MVP Validation Points

- **After Phase 3**: Basic pagination works, can demo to stakeholders
- **After Phase 4**: Rate limiting active, security requirement met
- **After Phase 5**: Full test coverage, production ready

---

## Files Modified Summary

| File | Tasks | Lines Changed |
|------|-------|---------------|
| `app/schemas/task.py` | T001-T003 | ~30 |
| `app/services/task_service.py` | T004-T009 | ~40 |
| `app/api/routes/tasks.py` | T010-T018, T021-T025 | ~60 |
| `app/middleware/rate_limit.py` | T019-T020 | ~10 |
| `tests/integration/test_tasks_crud.py` | T026-T051 | ~300 |

**Total**: 57 tasks, ~440 lines of code

---

## Notes

- [P] tasks = different files, no dependencies (test cases after fixtures)
- Preserve `TaskResponse` for backward compatibility (internal use)
- Return 404 (not 403) for ownership violations per NFR-02
- Rate limiting uses user ID from JWT, fallback to IP
- Commit after each phase completion
- Stop at any checkpoint to validate progress
