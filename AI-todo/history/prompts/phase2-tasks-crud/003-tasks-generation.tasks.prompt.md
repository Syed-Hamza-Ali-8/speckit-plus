---
id: 003
title: Task CRUD API Tasks Generation
stage: tasks
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-tasks-crud
branch: main
user: developer
command: /sp.tasks
labels: ["api", "crud", "tasks", "implementation", "testing"]
links:
  spec: specs/phase2/part3-tasks-crud/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - specs/phase2/part3-tasks-crud/tasks.md
tests:
  - null
---

## Prompt

## Task 1: Task Schemas (30m)
**Files:** phase2/backend/app/schemas/task.py
**Description:** TaskCreate, TaskRead (hide user_id), TaskUpdate, PaginatedTaskResponse
**Checkpoint:** mypy app/schemas/task.py passes

## Task 2: Extend TaskService CRUD (45m)
**Files:** phase2/backend/app/services/task_service.py
**Description:** create_task(), update_task(), delete_task() with current_user.id
**Checkpoint:** Manual test service methods

## Task 3: Tasks Router Setup (30m)
**Files:** phase2/backend/app/routers/tasks.py
**Description:** Basic router + include_router in main.py
**Checkpoint:** GET /tasks returns 401 (auth required)

## Task 4: GET /tasks List + Pagination (45m)
**Files:** phase2/backend/app/routers/tasks.py
**Description:** Query params (limit, offset, status, sort), TaskService.get_tasks()
**Checkpoint:** curl "GET /tasks?limit=5" → PaginatedTaskResponse

## Task 5: POST /tasks Create (30m)
**Files:** phase2/backend/app/routers/tasks.py
**Description:** TaskCreate → TaskService.create_task() → 201 TaskRead
**Checkpoint:** curl POST /tasks → 201 created task

## Task 6: GET/PATCH/DELETE Single Task (45m)
**Files:** phase2/backend/app/routers/tasks.py
**Description:** get_task_or_404(), update_task(), delete_task()
**Checkpoint:** PATCH /tasks/{id} → 200, DELETE → 204

## Task 7: Query Filtering + Sorting (45m)
**Files:** phase2/backend/app/routers/tasks.py
**Description:** ?status=pending&sort=created_at:desc&created_after=2025-01-01
**Checkpoint:** curl filtering returns correct tasks

## Task 8: Tests + Rate Limiting (90m)
**Files:** tests/integration/test_tasks_crud.py
**Description:** 20+ E2E tests (CRUD, pagination, auth, ownership, rate limits)
**Checkpoint:** pytest tests/integration/ -v → ALL PASS

## Response snapshot

Generated comprehensive `tasks.md` at `specs/phase2/part3-tasks-crud/tasks.md`:

**Task Summary:**
- **Total Tasks:** 57 tasks across 6 phases
- **Phase 1 (Schemas):** 3 tasks - TaskRead, PaginatedTaskResponse
- **Phase 2 (Service):** 6 tasks - Extend get_tasks() with filtering/sorting/pagination
- **Phase 3 (Router):** 9 tasks - Query params, response schema updates
- **Phase 4 (Rate Limiting):** 7 tasks - User-based rate limits
- **Phase 5 (Tests):** 26 tasks - Full test matrix coverage (TC-01 to TC-25)
- **Phase 6 (Polish):** 6 tasks - Verification and manual testing

**Parallel Opportunities:**
- Phase 5 tests (T027-T051) can run in parallel after fixtures (T026)
- Phase 6 verification tasks can run in parallel

**Files Modified:**
- `app/schemas/task.py` - ~30 lines
- `app/services/task_service.py` - ~40 lines
- `app/api/routes/tasks.py` - ~60 lines
- `app/middleware/rate_limit.py` - ~10 lines
- `tests/integration/test_tasks_crud.py` - ~300 lines (new)

**Total estimated:** ~440 lines of code

## Outcome

- Impact: Created actionable task list for Task CRUD API implementation
- Tests: 26 test tasks covering all 25 test cases from spec.md
- Files: 1 file created (tasks.md)
- Next prompts: /sp.implement to execute tasks
- Reflection: Tasks aligned with user's 8-task breakdown, expanded into 57 granular tasks for LLM execution

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): User-provided task breakdown integrated
- Next experiment: Verify sequential execution order is optimal for single-developer workflow
