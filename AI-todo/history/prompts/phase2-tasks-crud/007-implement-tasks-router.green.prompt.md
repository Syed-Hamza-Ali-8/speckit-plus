---
id: 007
title: Implement Tasks Router Endpoints
stage: green
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-tasks-crud
branch: main
user: developer
command: /sp.implement
labels: ["api", "router", "endpoints", "pagination", "fastapi"]
links:
  spec: specs/phase2/part3-tasks-crud/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - phase2/backend/app/api/routes/tasks.py
  - specs/phase2/part3-tasks-crud/tasks.md
tests:
  - null
---

## Prompt

Task 3: Tasks Router Setup

## Response snapshot

Implemented Phase 3: Tasks Router Setup + Endpoints (T010-T018):

**T010 - Imports:**
- Added `from datetime import date`
- Added `Query` to fastapi imports

**T011 - Schema imports:**
- Added `TaskRead`, `PaginatedTaskResponse` to imports
- Kept `TaskCreate`, `TaskUpdate` for request validation

**T012 - ALLOWED_SORT_FIELDS:**
- Added constant: `{"created_at", "updated_at", "title", "status"}`

**T013 - parse_sort() helper:**
- Validates "field:direction" format
- Raises HTTPException 400 for invalid format/field/direction
- Returns tuple of (field, direction)

**T014 - list_tasks() Query params:**
- `status_filter` (alias="status") - TaskStatus | None
- `created_after` - date | None
- `created_before` - date | None
- `sort` - default "created_at:desc"
- `limit` - default 20, ge=1, le=100
- `offset` - default 0, ge=0

**T015 - list_tasks() returns PaginatedTaskResponse:**
- Calls task_service.get_tasks() with all params
- Returns PaginatedTaskResponse with items, total, limit, offset

**T016-T018 - Updated return types:**
- create_task() → TaskRead (was TaskResponse)
- get_task() → TaskRead (was TaskResponse)
- update_task() → TaskRead (was TaskResponse)
- delete_task() → 204 No Content (unchanged)

**Key changes:**
- Response schemas now exclude user_id (security)
- Pagination enabled on list endpoint
- Query parameter validation with helpful error messages

## Outcome

- Impact: Phase 3 complete - all endpoints updated with new schemas and pagination
- Tests: N/A (implementation phase - tests in Phase 5)
- Files: 2 files modified (tasks.py rewritten ~175 lines, tasks.md checkboxes)
- Next prompts: Task 4 (Rate Limiting) or continue with /sp.implement
- Reflection: Clean rewrite of router; used Query() with alias for status to avoid Python keyword conflict

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A (checkpoint: curl should return PaginatedTaskResponse)
- Prompt variant (if applicable): Single task execution
- Next experiment: Start server and test with curl to verify pagination works
