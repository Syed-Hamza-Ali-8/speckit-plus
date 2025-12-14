---
id: 006
title: Implement TaskService CRUD Extensions
stage: green
date: 2025-12-14
surface: agent
model: claude-sonnet-4-5-20250929
feature: phase2-tasks-crud
branch: main
user: developer
command: /sp.implement
labels: ["api", "service", "crud", "pagination", "filtering"]
links:
  spec: specs/phase2/part3-tasks-crud/spec.md
  ticket: null
  adr: null
  pr: null
files:
  - phase2/backend/app/services/task_service.py
  - specs/phase2/part3-tasks-crud/tasks.md
tests:
  - null
---

## Prompt

Task 2: Extend TaskService CRUD

## Response snapshot

Implemented Phase 2: Extend TaskService CRUD (T004-T009):

**T004 - parse_sort() helper:**
- Added `parse_sort(sort: str) -> tuple[str, str]` function
- Validates format is "field:direction"
- Validates field is in ALLOWED_SORT_FIELDS: {created_at, updated_at, title, status}
- Validates direction is in ALLOWED_SORT_DIRECTIONS: {asc, desc}
- Raises ValueError with descriptive message on invalid input

**T005 - Filter parameters:**
- Extended `get_tasks()` signature with: status, created_after, created_before
- All filters are optional (default: None)
- Filters applied conditionally to base query

**T006 - Sort parameters:**
- Added sort_field (default: "created_at") and sort_direction (default: "desc")
- Dynamic column sorting using `getattr(Task, sort_field)`
- Ascending/descending based on direction

**T007 - Pagination:**
- Added limit (default: 20) and offset (default: 0) parameters
- Applied via `.offset(offset).limit(limit)`

**T008 - Return tuple:**
- Changed return type from `list[Task]` to `tuple[list[Task], int]`
- Returns (tasks, total_count)

**T009 - Count query:**
- Added `func.count()` import from sqlalchemy
- Count query: `select(func.count()).select_from(base_query.subquery())`
- Total calculated before pagination applied

**New imports:**
- `from datetime import date`
- `from sqlalchemy import func`

## Outcome

- Impact: Phase 2 complete - service layer ready for router updates
- Tests: N/A (implementation phase - tests in Phase 5)
- Files: 2 files modified (task_service.py +60 lines, tasks.md checkboxes)
- Next prompts: Task 3 (Tasks Router Setup + Endpoints)
- Reflection: Clean extension of existing service; all filtering/sorting/pagination in single query

## Evaluation notes (flywheel)

- Failure modes observed: None
- Graders run and results (PASS/FAIL): N/A
- Prompt variant (if applicable): Single task execution
- Next experiment: Test service with different filter/sort combinations
